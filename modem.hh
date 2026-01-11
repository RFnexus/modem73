#pragma once

#include <vector>
#include <cstring>
#include <cmath>
#include <functional>
#include <atomic>


#include "phy/common.hh"
#include "schmidl_cox.hh"
#include "bip_buffer.hh"
#include "theil_sen.hh"
#include "blockdc.hh"
#include "hilbert.hh"
#include "phasor.hh"
#include "delay.hh"
#include "polar_encoder.hh"
#include "polar_list_decoder.hh"
#include "hadamard_decoder.hh"


template<typename T>
class BufferWritePCM {
public:
    BufferWritePCM(int rate, int bits, int channels) 
        : rate_(rate), bits_(bits), channels_(channels) {}
    
    void write(const T* buffer, int count, int ch = 1) {
        for (int i = 0; i < count; ++i) {
            // 2 channels, only take real part for mono output
            if (ch == 2) {
                samples_.push_back(buffer[i * 2]);  // real
            } else {
                samples_.push_back(buffer[i]);
            }
        }
    }
    
    void silence(int count) {
        for (int i = 0; i < count; ++i) {
            samples_.push_back(T(0));
        }
    }
    
    const std::vector<T>& samples() const { return samples_; }
    std::vector<T>& samples() { return samples_; }
    void clear() { samples_.clear(); }
    
    int rate() const { return rate_; }
    int bits() const { return bits_; }
    int channels() const { return channels_; }
    
private:
    std::vector<T> samples_;
    int rate_, bits_, channels_;
};

// Modem configuration
struct ModemConfig {
    int sample_rate = 48000;
    int center_freq = 1500;
    int64_t call_sign = 0;
    int oper_mode = 0;
    
    static int64_t encode_callsign(const char* str) {
        int64_t acc = 0;
        for (char c = *str++; c; c = *str++) {
            acc *= 40;
            if (c == '/')
                acc += 3;
            else if (c >= '0' && c <= '9')
                acc += c - '0' + 4;
            else if (c >= 'a' && c <= 'z')
                acc += c - 'a' + 14;
            else if (c >= 'A' && c <= 'Z')
                acc += c - 'A' + 14;
            else if (c != ' ')
                return -1;
        }
        return acc;
    }
    
  static int encode_mode(const char* modulation, const char* code_rate, bool short_frame) {
        int mode = 0;
        
        if (!strcmp(modulation, "BPSK"))
            mode |= 0 << 4;
        else if (!strcmp(modulation, "QPSK"))
            mode |= 1 << 4;
        else if (!strcmp(modulation, "8PSK"))
            mode |= 2 << 4;
        else if (!strcmp(modulation, "QAM16"))
            mode |= 3 << 4;
        else if (!strcmp(modulation, "QAM64"))
            mode |= 4 << 4;
        else if (!strcmp(modulation, "QAM256"))
            mode |= 5 << 4;
        else if (!strcmp(modulation, "QAM1024"))
            mode |= 6 << 4;
        else if (!strcmp(modulation, "QAM4096"))
            mode |= 7 << 4;
        else
            return -1;
        
        if (!strcmp(code_rate, "1/2"))
            mode |= 0 << 1;
        else if (!strcmp(code_rate, "2/3"))
            mode |= 1 << 1;
        else if (!strcmp(code_rate, "3/4"))
            mode |= 2 << 1;
        else if (!strcmp(code_rate, "5/6"))
            mode |= 3 << 1;
        else if (!strcmp(code_rate, "1/4"))
            mode |= 4 << 1;
        else
            return -1;
        
        if (!short_frame)
            mode |= 1;
        
        return mode;
    }
};

// Encoder
template<typename value, typename cmplx, int rate>
class ModemEncoder : public Common {
public:
    typedef int8_t code_type;
    static const int guard_len = rate / 300;
    static const int symbol_len = guard_len * 40;
    
    ModemEncoder() {}
    
    // encode our data to audio samples
    std::vector<value> encode(const uint8_t* input_data, size_t input_len, 
                               int freq_off, int64_t call_sign, int oper_mode) {
        BufferWritePCM<value> pcm(rate, 32, 1);
        
        if (!setup(oper_mode)) {
            std::cerr << "Encoder: invalid mode" << std::endl;
            return {};
        }
        
        int offset = (freq_off * symbol_len) / rate;
        tone_off = offset - tone_count / 2;
        
        guard_interval_weights();
        meta_data((call_sign << 8) | oper_mode);
        
        // leading noise
        CODE::MLS noise(mls2_poly);
        for (int j = 0; j < 1; ++j) {
            for (int i = 0; i < tone_count; ++i)
                tone[i] = nrz(noise());
            symbol(&pcm, -3);
        }
        
        // Copy input data (pad if necessary)
        std::memset(data, 0, data_max);
        std::memcpy(data, input_data, std::min(input_len, (size_t)data_bytes));
        
        // Scramble
        CODE::Xorshift32 scrambler;
        for (int i = 0; i < data_bytes; ++i)
            data[i] ^= scrambler();
        
        // Schmidl-Cox preamble
        CODE::MLS seq0(mls0_poly, mls0_seed);
        for (int i = 0; i < tone_count; ++i)
            tone[i] = nrz(seq0());
        symbol(&pcm, -2);
        symbol(&pcm, -1);
        
        // Encode payload
        for (int i = 0; i < data_bits; ++i)
            mesg[i] = nrz(CODE::get_le_bit(data, i));
        
        crc1.reset();
        for (int i = 0; i < data_bytes; ++i)
            crc1(data[i]);
        for (int i = 0; i < 32; ++i)
            mesg[i + data_bits] = nrz((crc1() >> i) & 1);
        
        polar_encoder(code, mesg, frozen_bits, code_order);
        shuffle(perm, code, code_order);
        
        // Generate symbols
        CODE::MLS seq1(mls1_poly);
        for (int j = 0, k = 0, m = 0; j < symbol_count + 1; ++j) {
            seed_off = (block_skew * j + first_seed) % block_length;
            for (int i = 0; i < tone_count; ++i) {
                if (i % block_length == seed_off) {
                    tone[i] = nrz(seq1());
                } else if (j) {
                    int bits = mod_bits;
                    if (mod_bits == 3 && k % 32 == 30) bits = 2;
                    if (mod_bits == 6 && k % 64 == 60) bits = 4;
                    if (mod_bits == 10 && k % 128 == 120) bits = 8;
                    if (mod_bits == 12 && k % 128 == 120) bits = 8;
                    tone[i] = map_bits(perm + k, bits);
                    k += bits;
                } else {
                    tone[i] = map_bits(meta + m++, 1);
                }
            }
            symbol(&pcm, j);
        }
        


        for (int i = 0; i < guard_len; ++i)
            guard[i] *= 1 - weight[i];
        pcm.write(reinterpret_cast<value*>(guard), guard_len, 2);
        

        return std::move(pcm.samples());

    }
    
    int get_payload_size(int oper_mode) {
        if (!setup(oper_mode)) return 0;
        return data_bytes;
    }
    
private:
    DSP::FastFourierTransform<symbol_len, cmplx, -1> fwd;
    DSP::FastFourierTransform<symbol_len, cmplx, 1> bwd;
    CODE::PolarEncoder<code_type> polar_encoder;
    code_type code[bits_max], perm[bits_max], mesg[bits_max], meta[data_tones];
    cmplx fdom[symbol_len];
    cmplx tdom[symbol_len];
    cmplx test[symbol_len];
    cmplx kern[symbol_len];
    cmplx guard[guard_len];
    cmplx tone[tone_count];
    cmplx temp[tone_count];
    value weight[guard_len];
    value papr[symbols_max];
    
    static int bin(int carrier) {
        return (carrier + symbol_len) % symbol_len;
    }
    static int nrz(bool bit) {
        return 1 - 2 * bit;
    }
    
    cmplx map_bits(code_type* b, int bits) {
        switch (bits) {
        case 1: return PhaseShiftKeying<2, cmplx, code_type>::map(b);
        case 2: return PhaseShiftKeying<4, cmplx, code_type>::map(b);
        case 3: return PhaseShiftKeying<8, cmplx, code_type>::map(b);
        case 4: return QuadratureAmplitudeModulation<16, cmplx, code_type>::map(b);
        case 6: return QuadratureAmplitudeModulation<64, cmplx, code_type>::map(b);
        case 8: return QuadratureAmplitudeModulation<256, cmplx, code_type>::map(b);
        case 10: return QuadratureAmplitudeModulation<1024, cmplx, code_type>::map(b);
        case 12: return QuadratureAmplitudeModulation<4096, cmplx, code_type>::map(b);
        }
        return 0;
    }
    
    void shuffle(code_type* dest, const code_type* src, int order) {
        if (order == 8) {
            CODE::XorShiftMask<int, 8, 1, 1, 2, 1> seq;
            dest[0] = src[0];
            for (int i = 1; i < 256; ++i) dest[i] = src[seq()];
        } else if (order == 11) {
            CODE::XorShiftMask<int, 11, 1, 3, 4, 1> seq;
            dest[0] = src[0];
            for (int i = 1; i < 2048; ++i) dest[i] = src[seq()];
        } else if (order == 12) {
            CODE::XorShiftMask<int, 12, 1, 1, 4, 1> seq;
            dest[0] = src[0];
            for (int i = 1; i < 4096; ++i) dest[i] = src[seq()];
        } else if (order == 13) {
            CODE::XorShiftMask<int, 13, 1, 1, 9, 1> seq;
            dest[0] = src[0];
            for (int i = 1; i < 8192; ++i) dest[i] = src[seq()];
        } else if (order == 14) {
            CODE::XorShiftMask<int, 14, 1, 5, 10, 1> seq;
            dest[0] = src[0];
            for (int i = 1; i < 16384; ++i) dest[i] = src[seq()];
        } else if (order == 15) {
            CODE::XorShiftMask<int, 15, 1, 1, 3, 1> seq;
            dest[0] = src[0];
            for (int i = 1; i < 32768; ++i) dest[i] = src[seq()];
        } else if (order == 16) {
            CODE::XorShiftMask<int, 16, 1, 1, 14, 1> seq;
            dest[0] = src[0];
            for (int i = 1; i < 65536; ++i) dest[i] = src[seq()];
        }
    }
    
    void guard_interval_weights() {
        for (int i = 0; i < guard_len / 4; ++i)
            weight[i] = 0;
        for (int i = guard_len / 4; i < guard_len / 4 + guard_len / 2; ++i) {
            value x = value(i - guard_len / 4) / value(guard_len / 2 - 1);
            weight[i] = value(0.5) * (value(1) - std::cos(DSP::Const<value>::Pi() * x));
        }
        for (int i = guard_len / 4 + guard_len / 2; i < guard_len; ++i)
            weight[i] = 1;
    }
    
    void clipping_and_filtering(value scale) {
        for (int i = 0; i < symbol_len; ++i) {
            value pwr = norm(tdom[i]);
            if (pwr > value(1))
                tdom[i] /= sqrt(pwr);
        }
        fwd(fdom, tdom);
        for (int i = 0; i < symbol_len; ++i) {
            int j = bin(i + tone_off);
            if (i >= tone_count)
                fdom[j] = 0;
            else
                fdom[j] *= 1 / (scale * symbol_len);
        }
        bwd(tdom, fdom);
        for (int i = 0; i < symbol_len; ++i)
            tdom[i] *= scale;
        auto clamp = [](value v) { return v < value(-1) ? value(-1) : v > value(1) ? value(1) : v; };
        for (int i = 0; i < symbol_len; ++i)
            tdom[i] = cmplx(clamp(tdom[i].real()), clamp(tdom[i].imag()));
    }
    
    void symbol(BufferWritePCM<value>* pcm, int symbol_number) {
        value scale = value(0.5) / std::sqrt(value(tone_count));
        if (symbol_number < 0) {
            for (int i = 0; i < symbol_len; ++i)
                fdom[i] = 0;
            for (int i = 0; i < tone_count; ++i)
                fdom[bin(i + tone_off)] = tone[i];
            bwd(tdom, fdom);
            for (int i = 0; i < symbol_len; ++i)
                tdom[i] *= scale;
        } else {
            value best_papr = 1000;
            for (int seed_value = 0; seed_value < 128; ++seed_value) {
                for (int i = 0; i < tone_count; ++i)
                    temp[i] = tone[i];
                hadamard_encoder(seed, seed_value);
                for (int i = 0; i < seed_tones; ++i)
                    temp[i * block_length + seed_off] *= seed[i];
                if (seed_value) {
                    CODE::MLS seq(mls2_poly, seed_value);
                    for (int i = 0; i < tone_count; ++i)
                        if (i % block_length != seed_off)
                            temp[i] *= nrz(seq());
                }
                for (int i = 0; i < symbol_len; ++i)
                    fdom[i] = 0;
                for (int i = 0; i < tone_count; ++i)
                    fdom[bin(i + tone_off)] = temp[i];
                bwd(test, fdom);
                for (int i = 0; i < symbol_len; ++i)
                    test[i] *= scale;
                value peak = 0, mean = 0;
                for (int i = 0; i < symbol_len; ++i) {
                    value power(norm(test[i]));
                    peak = std::max(peak, power);
                    mean += power;
                }
                mean /= symbol_len;
                value test_papr(peak / mean);
                if (test_papr < best_papr) {
                    best_papr = test_papr;
                    papr[symbol_number] = test_papr;
                    for (int i = 0; i < symbol_len; ++i)
                        tdom[i] = test[i];
                    if (test_papr < 5)
                        break;
                }
            }
        }
        clipping_and_filtering(scale);
        if (symbol_number != -1) {
            for (int i = 0; i < guard_len; ++i)
                guard[i] = DSP::lerp(guard[i], tdom[i + symbol_len - guard_len], weight[i]);
            pcm->write(reinterpret_cast<value*>(guard), guard_len, 2);
        }
        for (int i = 0; i < guard_len; ++i)
            guard[i] = tdom[i];
        pcm->write(reinterpret_cast<value*>(tdom), symbol_len, 2);
    }
    
    void meta_data(uint64_t md) {
        for (int i = 0; i < 56; ++i)
            mesg[i] = nrz((md >> i) & 1);
        crc0.reset();
        crc0(md << 8);
        for (int i = 0; i < 16; ++i)
            mesg[i + 56] = nrz((crc0() >> i) & 1);
        polar_encoder(code, mesg, frozen_256_72, 8);
        shuffle(meta, code, 8);
    }
};

// Decoder
template<typename value, typename cmplx, int rate>
class ModemDecoder : public Common {
public:
    typedef int16_t code_type;
    typedef SIMD<code_type, 32> mesg_type;
    typedef DSP::Const<value> Const;
    static const int guard_len = rate / 300;
    static const int symbol_len = guard_len * 40;
    static const int filter_len = 129;
    static const int extended_len = symbol_len + guard_len;
    static const int buffer_len = 5 * extended_len;
    static const int search_pos = extended_len;
    static const int tone_off_const = -tone_count / 2;
    
    using FrameCallback = std::function<void(const uint8_t*, size_t)>;
    
    ModemDecoder() {
        // init fdom_mls before correlator uses it
        init_mls0_seq();
        correlator_ptr = new SchmidlCox<value, cmplx, search_pos, symbol_len, guard_len>(fdom_mls);
        blockdc.samples(filter_len);
    }
    
    ~ModemDecoder() {
        delete correlator_ptr;
        delete seq1_ptr;
    }
    

    void process(const value* samples, size_t count, FrameCallback callback) {
        for (size_t i = 0; i < count; ++i) {
            process_sample(samples[i], callback);
        }
    }
    
    // Reset decoder state
    void reset() {
        state_ = State::SEARCHING;
        sample_count_ = 0;
        symbol_index_ = 0;
        samples_needed_ = 0;
        k_ = 0;
    }
    
    // Get average SNR from last successful decode
    value get_last_snr() const { return last_avg_snr_; }
    
private:
    enum class State {
        SEARCHING,           // looking for preamble
        COLLECTING_SYMBOLS,  // Collecting data symbols
    };
    
    // Arrays used by correlator
    cmplx fdom_mls[symbol_len];
    cmplx fdom[symbol_len], tdom[symbol_len];
    
    DSP::FastFourierTransform<symbol_len, cmplx, -1> fwd;
    DSP::BlockDC<value, value> blockdc;
    DSP::Hilbert<cmplx, filter_len> hilbert;
    DSP::BipBuffer<cmplx, buffer_len> input_hist;
    DSP::TheilSenEstimator<value, tone_count> tse;
    SchmidlCox<value, cmplx, search_pos, symbol_len, guard_len>* correlator_ptr = nullptr;
    CODE::HadamardDecoder<7> hadamard_decoder;
    CODE::PolarListDecoder<mesg_type, code_max> polar_decoder;
    DSP::Phasor<cmplx> osc;
    
    mesg_type mesg[bits_max];
    code_type code[bits_max], perm[bits_max];
    cmplx demod[tone_count], chan[tone_count], tone[tone_count];
    value index[tone_count], phase[tone_count];
    value snr[symbols_max];
    value cfo_rad;
    int symbol_pos;
    value last_avg_snr_ = 0;  
    
    State state_ = State::SEARCHING;
    size_t sample_count_ = 0;
    int symbol_index_ = 0;
    int samples_needed_ = 0;
    int k_ = 0;
    const cmplx* buf_ = nullptr;
    CODE::MLS* seq1_ptr = nullptr;
    
    static int bin(int carrier) {
        return (carrier + symbol_len) % symbol_len;
    }
    
    static value nrz(bool bit) {
        return 1 - 2 * bit;
    }
    
    static cmplx demod_or_erase(cmplx curr, cmplx prev) {
        if (norm(prev) > 0) {
            cmplx d = curr / prev;
            if (norm(d) < 4)
                return d;
        }
        return 0;
    }
    
    void init_mls0_seq() {
        CODE::MLS seq0(mls0_poly, mls0_seed);
        value cur = 0, prv = 0;
        for (int i = 0; i < tone_count; ++i, prv = cur)
            fdom_mls[bin(i + tone_off_const)] = prv * (cur = nrz(seq0()));
    }
    
    cmplx map_bits(code_type* b, int bits) {
        switch (bits) {
        case 1: return PhaseShiftKeying<2, cmplx, code_type>::map(b);
        case 2: return PhaseShiftKeying<4, cmplx, code_type>::map(b);
        case 3: return PhaseShiftKeying<8, cmplx, code_type>::map(b);
        case 4: return QuadratureAmplitudeModulation<16, cmplx, code_type>::map(b);
        case 6: return QuadratureAmplitudeModulation<64, cmplx, code_type>::map(b);
        case 8: return QuadratureAmplitudeModulation<256, cmplx, code_type>::map(b);
        case 10: return QuadratureAmplitudeModulation<1024, cmplx, code_type>::map(b);
        case 12: return QuadratureAmplitudeModulation<4096, cmplx, code_type>::map(b);
        }
        return 0;
    }
    
    void demap_soft(code_type* b, cmplx c, value precision, int bits) {
        switch (bits) {
        case 1: return PhaseShiftKeying<2, cmplx, code_type>::soft(b, c, precision);
        case 2: return PhaseShiftKeying<4, cmplx, code_type>::soft(b, c, precision);
        case 3: return PhaseShiftKeying<8, cmplx, code_type>::soft(b, c, precision);
        case 4: return QuadratureAmplitudeModulation<16, cmplx, code_type>::soft(b, c, precision);
        case 6: return QuadratureAmplitudeModulation<64, cmplx, code_type>::soft(b, c, precision);
        case 8: return QuadratureAmplitudeModulation<256, cmplx, code_type>::soft(b, c, precision);
        case 10: return QuadratureAmplitudeModulation<1024, cmplx, code_type>::soft(b, c, precision);
        case 12: return QuadratureAmplitudeModulation<4096, cmplx, code_type>::soft(b, c, precision);
        }
    }
    
    void demap_hard(code_type* b, cmplx c, int bits) {
        switch (bits) {
        case 1: return PhaseShiftKeying<2, cmplx, code_type>::hard(b, c);
        case 2: return PhaseShiftKeying<4, cmplx, code_type>::hard(b, c);
        case 3: return PhaseShiftKeying<8, cmplx, code_type>::hard(b, c);
        case 4: return QuadratureAmplitudeModulation<16, cmplx, code_type>::hard(b, c);
        case 6: return QuadratureAmplitudeModulation<64, cmplx, code_type>::hard(b, c);
        case 8: return QuadratureAmplitudeModulation<256, cmplx, code_type>::hard(b, c);
        case 10: return QuadratureAmplitudeModulation<1024, cmplx, code_type>::hard(b, c);
        case 12: return QuadratureAmplitudeModulation<4096, cmplx, code_type>::hard(b, c);
        }
    }
    
    void shuffle(code_type* dest, const code_type* src, int order) {
        if (order == 8) {
            CODE::XorShiftMask<int, 8, 1, 1, 2, 1> seq;
            dest[0] = src[0];
            for (int i = 1; i < 256; ++i) dest[seq()] = src[i];
        } else if (order == 11) {
            CODE::XorShiftMask<int, 11, 1, 3, 4, 1> seq;
            dest[0] = src[0];
            for (int i = 1; i < 2048; ++i) dest[seq()] = src[i];
        } else if (order == 12) {
            CODE::XorShiftMask<int, 12, 1, 1, 4, 1> seq;
            dest[0] = src[0];
            for (int i = 1; i < 4096; ++i) dest[seq()] = src[i];
        } else if (order == 13) {
            CODE::XorShiftMask<int, 13, 1, 1, 9, 1> seq;
            dest[0] = src[0];
            for (int i = 1; i < 8192; ++i) dest[seq()] = src[i];
        } else if (order == 14) {
            CODE::XorShiftMask<int, 14, 1, 5, 10, 1> seq;
            dest[0] = src[0];
            for (int i = 1; i < 16384; ++i) dest[seq()] = src[i];
        } else if (order == 15) {
            CODE::XorShiftMask<int, 15, 1, 1, 3, 1> seq;
            dest[0] = src[0];
            for (int i = 1; i < 32768; ++i) dest[seq()] = src[i];
        } else if (order == 16) {
            CODE::XorShiftMask<int, 16, 1, 1, 14, 1> seq;
            dest[0] = src[0];
            for (int i = 1; i < 65536; ++i) dest[seq()] = src[i];
        }
    }
    
    static void base40_decoder(char* str, int64_t val, int len) {
        for (int i = len - 1; i >= 0; --i, val /= 40)
            str[i] = "   /0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ"[val % 40];
    }
    
    int64_t meta_data() {
        shuffle(code, perm, 8);
        polar_decoder(nullptr, mesg, code, frozen_256_72, 8);
        int best = -1;
        for (int k = 0; k < mesg_type::SIZE; ++k) {
            crc0.reset();
            for (int i = 0; i < 72; ++i)
                crc0(mesg[i].v[k] < 0);
            if (crc0() == 0) {
                best = k;
                break;
            }
        }
        if (best < 0)
            return -1;
        uint64_t md = 0;
        for (int i = 0; i < 56; ++i)
            md |= uint64_t(mesg[i].v[best] < 0) << i;
        return md;
    }
    
    void process_sample(value sample, FrameCallback callback) {
        // Convert to complex via Hilbert transform
        cmplx tmp = hilbert(blockdc(sample));
        buf_ = input_hist(tmp);
        ++sample_count_;
        
        switch (state_) {
        case State::SEARCHING:
            if ((*correlator_ptr)(buf_)) {
                // Sync found
                symbol_pos = correlator_ptr->symbol_pos;
                cfo_rad = correlator_ptr->cfo_rad;
                
                std::cerr << "Decoder: Sync found at sample " << sample_count_ << std::endl;
                std::cerr << "Decoder: CFO = " << cfo_rad * (rate / Const::TwoPi()) << " Hz" << std::endl;
                
                // Initialize seq1 for the whole frame
                delete seq1_ptr;
                seq1_ptr = new CODE::MLS(mls1_poly);
                
                // Process preamble and start collecting symbols
                if (process_preamble()) {
                    state_ = State::COLLECTING_SYMBOLS;
                    symbol_index_ = 1;  // Symbol 0 (meta) already processed
                    // Need to advance past preamble: symbol_pos + symbol_len + extended_len
                    // Plus extended_len for the first data symbol
                    samples_needed_ = symbol_pos + symbol_len + 2 * extended_len;
                }
            }
            break;
            
        case State::COLLECTING_SYMBOLS:
            // Keep feeding correlator to maintain buffer
            (*correlator_ptr)(buf_);
            samples_needed_--;
            
            if (samples_needed_ <= 0) {
                // Process this symbol
                if (!process_symbol(symbol_index_)) {
                    // Error, go back to searching
                    state_ = State::SEARCHING;
                    break;
                }
                
                symbol_index_++;
                
                if (symbol_index_ > symbol_count) {
                    // All symbols collected
                    decode_frame(callback);
                    state_ = State::SEARCHING;
                } else {
                    samples_needed_ = extended_len;
                }
            }
            break;
        }
    }
    
    bool process_preamble() {
        // Process Schmidl-Cox preamble symbols
        osc.omega(-cfo_rad);
        
        // First preamble symbol
        for (int i = 0; i < symbol_len; ++i)
            tdom[i] = buf_[i + symbol_pos] * osc();
        fwd(fdom, tdom);
        for (int i = 0; i < tone_count; ++i)
            tone[i] = fdom[bin(i + tone_off_const)];
        
        // Second preamble symbol
        for (int i = 0; i < symbol_len; ++i)
            tdom[i] = buf_[i + symbol_pos + symbol_len] * osc();
        for (int i = 0; i < guard_len; ++i)
            osc();
        fwd(fdom, tdom);
        for (int i = 0; i < tone_count; ++i)
            chan[i] = fdom[bin(i + tone_off_const)];
        
        // Estimate SFO
        for (int i = 0; i < tone_count; ++i) {
            index[i] = tone_off_const + i;
            phase[i] = arg(demod_or_erase(chan[i], tone[i]));
        }
        tse.compute(index, phase, tone_count);
        
        std::cerr << "Decoder: SFO = " << -1000000 * tse.slope() / Const::TwoPi() << " ppm" << std::endl;
        
        // Correct channel estimate
        for (int i = 0; i < tone_count; ++i)
            tone[i] *= DSP::polar<value>(1, tse(i + tone_off_const));
        for (int i = 0; i < tone_count; ++i)
            chan[i] = DSP::lerp(chan[i], tone[i], value(0.5));
        
        // Remove preamble sequence
        CODE::MLS seq0(mls0_poly, mls0_seed);
        for (int i = 0; i < tone_count; ++i)
            chan[i] *= nrz(seq0());
        
        // Process meta symbol (symbol 0)
        for (int i = 0; i < symbol_len; ++i)
            tdom[i] = buf_[i + symbol_pos + symbol_len + extended_len] * osc();
        for (int i = 0; i < guard_len; ++i)
            osc();
        fwd(fdom, tdom);
        
        // Decode meta symbol
        seed_off = first_seed;
        auto clamp = [](int v) { return v < -127 ? -127 : v > 127 ? 127 : v; };
        
        for (int i = 0; i < tone_count; ++i)
            tone[i] = fdom[bin(i + tone_off_const)];
        for (int i = seed_off; i < tone_count; i += block_length)
            tone[i] *= nrz((*seq1_ptr)());
        for (int i = 0; i < tone_count; ++i)
            demod[i] = demod_or_erase(tone[i], chan[i]);
        
        // Decode seed for meta symbol
        for (int i = 0; i < seed_tones; ++i)
            seed[i] = clamp(std::nearbyint(127 * demod[i * block_length + seed_off].real()));
        int seed_value = hadamard_decoder(seed);
        if (seed_value < 0) {
            std::cerr << "Decoder: Seed value damaged in meta" << std::endl;
            return false;
        }
        
        hadamard_encoder(seed, seed_value);
        for (int i = 0; i < seed_tones; ++i) {
            tone[block_length * i + seed_off] *= seed[i];
            demod[block_length * i + seed_off] *= seed[i];
        }
        
        // Phase correction
        for (int i = 0; i < seed_tones; ++i) {
            index[i] = tone_off_const + block_length * i + seed_off;
            phase[i] = arg(demod[block_length * i + seed_off]);
        }
        tse.compute(index, phase, seed_tones);
        for (int i = 0; i < tone_count; ++i)
            demod[i] *= DSP::polar<value>(1, -tse(i + tone_off_const));
        for (int i = 0; i < tone_count; ++i)
            chan[i] *= DSP::polar<value>(1, tse(i + tone_off_const));
        
        if (seed_value) {
            CODE::MLS seq(mls2_poly, seed_value);
            for (int i = 0; i < tone_count; ++i)
                if (i % block_length != seed_off)
                    demod[i] *= nrz(seq());
        }
        
        // SNR estimation and demapping for meta symbol (mod_bits = 1 for meta)
        value sp = 0, np = 0;
        for (int i = 0, l = 0; i < tone_count; ++i) {
            cmplx hard(1, 0);
            if (i % block_length != seed_off) {
                demap_hard(perm + l, demod[i], 1);
                hard = map_bits(perm + l, 1);
                l += 1;
            }
            cmplx error = demod[i] - hard;
            sp += norm(hard);
            np += norm(error);
        }
        value precision = sp / np;
        precision = std::min(precision, value(1023));
        
        // std::cerr << "Decoder: Meta symbol SNR = " << 10 * std::log10(precision) << " dB" << std::endl;
        
        // Soft demap meta symbol
        int k = 0;
        for (int i = 0; i < tone_count; ++i) {
            if (i % block_length != seed_off) {
                demap_soft(perm + k, demod[i], precision, 1);
                k += 1;
            }
        }
        
        // Update channel for meta symbol pilots
        for (int i = seed_off; i < tone_count; i += block_length)
            chan[i] = DSP::lerp(chan[i], tone[i], value(0.5));
        
        // Decode meta data
        int64_t meta_info = meta_data();
        if (meta_info < 0) {
            std::cerr << "Decoder: Preamble decoding error" << std::endl;
            return false;
        }
        
        int64_t call = meta_info >> 8;
        if (call == 0 || call >= 262144000000000L) {
            std::cerr << "Decoder: Invalid call sign" << std::endl;
            return false;
        }
        
        char call_sign[10];
        base40_decoder(call_sign, call, 9);
        call_sign[9] = 0;
        std::cerr << "Decoder: Call sign: " << call_sign << std::endl;
        
        int mode = meta_info & 255;
        if (!setup(mode)) {
            std::cerr << "Decoder: Invalid mode" << std::endl;
            return false;
        }
        
        std::cerr << "Decoder: Mode " << oper_mode << ", " << symbol_count << " data symbols, mod_bits=" << mod_bits << ", code_order=" << code_order << ", data_bytes=" << data_bytes << std::endl;
        
        // Reset for data collection
        k_ = 0;
        snr[0] = 100;  
        
        return true;
    }
    
    bool process_symbol(int j) {
        seed_off = (block_skew * j + first_seed) % block_length;
        auto clamp = [](int v) { return v < -127 ? -127 : v > 127 ? 127 : v; };
        
        // FFT the current symbol
        for (int i = 0; i < symbol_len; ++i)
            tdom[i] = buf_[i] * osc();
        for (int i = 0; i < guard_len; ++i)
            osc();
        fwd(fdom, tdom);
        
        for (int i = 0; i < tone_count; ++i)
            tone[i] = fdom[bin(i + tone_off_const)];
        
        // Remove pilot sequence
        for (int i = seed_off; i < tone_count; i += block_length)
            tone[i] *= nrz((*seq1_ptr)());
        
        for (int i = 0; i < tone_count; ++i)
            demod[i] = demod_or_erase(tone[i], chan[i]);
        
        // Decode seed
        for (int i = 0; i < seed_tones; ++i)
            seed[i] = clamp(std::nearbyint(127 * demod[i * block_length + seed_off].real()));
        int seed_value = hadamard_decoder(seed);
        if (seed_value < 0) {
            std::cerr << "Decoder: Seed damaged at symbol " << j << std::endl;
            return false;
        }
        
        hadamard_encoder(seed, seed_value);
        for (int i = 0; i < seed_tones; ++i) {
            tone[block_length * i + seed_off] *= seed[i];
            demod[block_length * i + seed_off] *= seed[i];
        }
        
        // Phase correction
        for (int i = 0; i < seed_tones; ++i) {
            index[i] = tone_off_const + block_length * i + seed_off;
            phase[i] = arg(demod[block_length * i + seed_off]);
        }
        tse.compute(index, phase, seed_tones);
        for (int i = 0; i < tone_count; ++i)
            demod[i] *= DSP::polar<value>(1, -tse(i + tone_off_const));
        for (int i = 0; i < tone_count; ++i)
            chan[i] *= DSP::polar<value>(1, tse(i + tone_off_const));
        
        if (seed_value) {
            CODE::MLS seq(mls2_poly, seed_value);
            for (int i = 0; i < tone_count; ++i)
                if (i % block_length != seed_off)
                    demod[i] *= nrz(seq());
        }
        
        // SNR estimation and soft demapping
        value sp = 0, np = 0;
        for (int i = 0, l = k_; i < tone_count; ++i) {
            cmplx hard(1, 0);
            if (i % block_length != seed_off) {
                int bits = mod_bits;
                if (mod_bits == 3 && l % 32 == 30) bits = 2;
                if (mod_bits == 6 && l % 64 == 60) bits = 4;
                if (mod_bits == 10 && l % 128 == 120) bits = 8;
                if (mod_bits == 12 && l % 128 == 120) bits = 8;
                demap_hard(perm + l, demod[i], bits);
                hard = map_bits(perm + l, bits);
                l += bits;
            }
            cmplx error = demod[i] - hard;
            sp += norm(hard);
            np += norm(error);
        }
        
        value precision = sp / np;
        snr[j] = precision;
        precision = std::min(precision, value(1023));
        




        std::cerr << "Decoder: Symbol " << j << " SNR = " << 10 * std::log10(snr[j]) << " dB, k=" << k_ << std::endl;


        for (int i = 0; i < tone_count; ++i) {
            if (i % block_length != seed_off) {
                int bits = mod_bits;
                if (mod_bits == 3 && k_ % 32 == 30) bits = 2;
                if (mod_bits == 6 && k_ % 64 == 60) bits = 4;
                if (mod_bits == 10 && k_ % 128 == 120) bits = 8;
                if (mod_bits == 12 && k_ % 128 == 120) bits = 8;
                demap_soft(perm + k_, demod[i], precision, bits);
                k_ += bits;
            }
        }
        


        for (int i = seed_off; i < tone_count; i += block_length)
            chan[i] = DSP::lerp(chan[i], tone[i], value(0.5));


        
        return true;
    }
    
    void decode_frame(FrameCallback callback) {
        std::cerr << "Decoder: Decoding frame, k_=" << k_ << " bits collected" << std::endl;
        std::cerr << "Decoder: Expected code_order=" << code_order << " (code length=" << (1 << code_order) << ")" << std::endl;
        
        int crc_bits = data_bits + 32;
        shuffle(code, perm, code_order);
        polar_decoder(nullptr, mesg, code, frozen_bits, code_order);
        
        int best = -1;
        for (int k = 0; k < mesg_type::SIZE; ++k) {
            crc1.reset();
            for (int i = 0; i < crc_bits; ++i)
                crc1(mesg[i].v[k] < 0);
            if (crc1() == 0) {
                best = k;
                break;
            }
        }
        
        if (best < 0) {
            std::cerr << "Decoder: CRC failed" << std::endl;
            return;
        }
        
        // calculate average SNR from data symbols
        value total_snr = 0;
        int snr_count = 0;
        for (int i = 1; i < symbol_index_; ++i) {  // skip symbol 0
            if (snr[i] > 0) {
                total_snr += snr[i];
                snr_count++;
            }
        }
        if (snr_count > 0) {
            last_avg_snr_ = 10 * std::log10(total_snr / snr_count);
        }
        
        // Extract data
        for (int i = 0; i < data_bits; ++i)
            CODE::set_le_bit(data, i, mesg[i].v[best] < 0);
        
        // Descramble
        CODE::Xorshift32 scrambler;
        for (int i = 0; i < data_bytes; ++i)
            data[i] ^= scrambler();
        
        std::cerr << "Decoder: Frame decoded " << data_bytes << " bytes, SNR=" << last_avg_snr_ << " dB" << std::endl;
        

        callback(data, data_bytes);
    }
};


using Encoder48k = ModemEncoder<float, DSP::Complex<float>, 48000>;
using Decoder48k = ModemDecoder<float, DSP::Complex<float>, 48000>;
