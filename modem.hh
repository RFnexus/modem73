#pragma once

#include <vector>
#include <cstring>
#include <cmath>
#include <functional>
#include <atomic>
#include <algorithm>


#include "phy/common.hh"
#include "schmidl_cox.hh"

#ifndef CHAN_INTERP_ALPHA
#define CHAN_INTERP_ALPHA 0.5
#endif
#ifndef CHAN_ALPHA_ADAPT
#define CHAN_ALPHA_ADAPT 1
#endif
#ifndef CHAN_ALPHA_KNEE
#define CHAN_ALPHA_KNEE 2.0
#endif
#ifndef CHAN_ALPHA_FLOOR
#define CHAN_ALPHA_FLOOR 0.25
#endif
// Impulse-noise blanker in front of the rx filters
// set to 0 to disable
#ifndef IMPULSE_BLANKER
#define IMPULSE_BLANKER 1
#endif
#ifndef RETRY_TWOSIDED
#define RETRY_TWOSIDED 0
#endif
#ifndef CHAN_PILOT_HIST
#define CHAN_PILOT_HIST 0
#endif
#ifndef PER_TONE_PRECISION
#define PER_TONE_PRECISION 1
#endif
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
    
    // frame_size: 0=short, 1=normal, 2=long (bit 7; doubles a normal frame)
    static int encode_mode(const char* modulation, const char* code_rate, int frame_size) {
        int mode = 0;

        if (frame_size < 0 || frame_size > 2)
            return -1;
        
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
        else if (!strcmp(code_rate, "1/2x2"))
            mode |= 5 << 1;
        else if (!strcmp(code_rate, "1/4x2"))
            mode |= 6 << 1;
        else
            return -1;

        int rate_val = (mode >> 1) & 7;
        if (rate_val == 5 || rate_val == 6) {
            if (frame_size == 2 && ((mode >> 4) & 7) >= 4)
                return -1;
            if (frame_size == 1 && ((mode >> 4) & 7) >= 5)
                return -1;
        }

        if (frame_size >= 1)
            mode |= 1;
        if (frame_size == 2) {
            // TODO
            //
            if (((mode >> 4) & 7) >= 6)
                return -1;
            mode |= 128;
        }

        return mode;
    }

    static const char* frame_size_name(int frame_size) {
        return frame_size == 0 ? "short" : frame_size == 2 ? "long" : "normal";
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
                               int freq_off, int64_t call_sign, int oper_mode,
                               bool postamble = false) {
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
                    // repeat2: second half of the frame re-sends the codeword
                    int idx = repeat2 ? (k & ((1 << code_order) - 1)) : k;
                    tone[i] = map_bits(perm + idx, bits);
                    k += bits;
                } else {
                    tone[i] = map_bits(meta + m++, 1);
                }
            }
            symbol(&pcm, j);
        }

        if (postamble) {

            // a second sync anchor (s&c pair + meta symbol)
            
            meta_data(((uint64_t)postamble_call << 8) | (uint64_t)(oper_mode & 255));
            CODE::MLS seq0p(mls0_poly, mls0_seed);
            for (int i = 0; i < tone_count; ++i)
                tone[i] = nrz(seq0p());
            symbol(&pcm, -2);
            symbol(&pcm, -1);
            CODE::MLS seq1p(mls1_poly);
            seed_off = first_seed;
            for (int i = 0, m = 0; i < tone_count; ++i) {
                if (i % block_length == seed_off)
                    tone[i] = nrz(seq1p());
                else
                    tone[i] = map_bits(meta + m++, 1);
            }
            symbol(&pcm, 0);
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
    
    // Constellation callback - called after each symbol is demodulated
    // Parameters: pointer to demodulated symbols, count, modulation bits
    std::function<void(const cmplx*, int, int)> constellation_callback;
    
    ModemDecoder() {
        // init fdom_mls before correlator uses it
        init_mls0_seq();
        correlator_ptr = new SchmidlCox<value, cmplx, search_pos, symbol_len, guard_len>(fdom_mls);
        blockdc.samples(filter_len);
        configure_frontend(1500, true);
        ring_.resize(ring_len);
    }

    void configure_frontend(int center_freq, bool enable_filter) {
        bpf_enabled_ = enable_filter;
        // signal spans center +-1200 Hz; +-150 Hz margin for mistuning
        value f1 = std::max(100, center_freq - 1350);
        value f2 = std::min(rate / 2 - 100, center_freq + 1350);
        for (int i = 0; i < bpf_len; ++i) {
            int k = i - bpf_len / 2;
            value lp2 = k == 0 ? 2 * f2 / rate
                : std::sin(2 * Const::Pi() * f2 * k / rate) / (Const::Pi() * k);
            value lp1 = k == 0 ? 2 * f1 / rate
                : std::sin(2 * Const::Pi() * f1 * k / rate) / (Const::Pi() * k);
            value w = value(0.54) - value(0.46) * std::cos(2 * Const::Pi() * i / (bpf_len - 1));
            bpf_taps_[i] = (lp2 - lp1) * w;
        }
        std::memset(bpf_hist_, 0, sizeof(bpf_hist_));
        bpf_pos_ = 0;
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

    // Get current modulation bits
    int get_mod_bits() const { return mod_bits; }

    bool in_frame() const { return state_ != State::SEARCHING; }

    // Get last per-frame BER
    value get_last_ber() const { return last_ber_; }

    // Get smoothed BER via EMA
    value get_ber_ema() const { return ber_ema_; }

    void reset_ber() {
        last_ber_ = -1;
        ber_ema_ = -1;
    }

    // decode statistics
    int stats_sync_count = 0;      // corelator
    int stats_preamble_errors = 0; // preamble decoding failed
    int stats_symbol_errors = 0;   // erasure budget
    int stats_crc_errors = 0;      // polar CRC failed
    int stats_erased_symbols = 0;  // symbols erased (seed damage), frame continued
    int stats_retry_success = 0;
    int stats_sticky_syncs = 0;
    int stats_postamble_rescues = 0;
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
    CODE::PolarEncoder<int8_t> ber_encoder;
    int8_t ber_mesg[bits_max], ber_code[bits_max];
    DSP::Phasor<cmplx> osc;
    
    mesg_type mesg[bits_max];
    code_type code[bits_max], perm[bits_max];
    cmplx demod[tone_count], chan[tone_count], tone[tone_count];
    cmplx pilot_obs_[tone_count];
    int pilot_age_[tone_count];
    cmplx saved_demod[symbols_max * tone_count];
    int saved_seed_off[symbols_max];
#if RETRY_TWOSIDED
    cmplx saved_tone_[symbols_max * tone_count];
    cmplx saved_fresh_[symbols_max * tone_count];
    value saved_rot_[symbols_max * tone_count];
    int saved_seed_value_[symbols_max];
#endif
    int fwd_perm_table[bits_max];
    value index[tone_count], phase[tone_count];
    value snr[symbols_max];
    bool erased_[symbols_max];
    int erased_count_ = 0;
    int max_erased_ = 0;
    value cfo_rad;
    int symbol_pos;
    value last_avg_snr_ = 0;
    value last_ber_ = -1;
    value ber_ema_ = -1;
    int last_good_mode_ = -1;

    State state_ = State::SEARCHING;
    size_t sample_count_ = 0;
    int symbol_index_ = 0;
    int samples_needed_ = 0;
    int k_ = 0;
    const cmplx* buf_ = nullptr;
    CODE::MLS* seq1_ptr = nullptr;

    code_type perm_save_[bits_max];
    int saved_k_ = 0;
    int sym_k_start_[symbols_max];
    int sym_k_end_[symbols_max];

    std::vector<cmplx> frame_raw_;
    int frame_sym_start_[symbols_max];
    int frame_symbol_pos_ = 0;
    bool replaying_ = false;
    value forced_alpha_ = 0;

    static const size_t ring_len = 640000;
    std::vector<cmplx> ring_;
    size_t ring_count_ = 0;
    size_t last_decode_abs_ = 0;
    int postamble_mode_ = -1;
    bool rescuing_ = false;
    bool preamble_primed_ = false;

    static const int bpf_len = 257;
    value bpf_taps_[bpf_len];
    value bpf_hist_[bpf_len];
    int bpf_pos_ = 0;
    bool bpf_enabled_ = true;
    value blank_env_ = 0;
    value blank_fast_ = 0;
    int blank_low_ = 0;

    value bandpass(value x) {
        bpf_hist_[bpf_pos_] = x;
        value acc = 0;
        int idx = bpf_pos_;
        for (int i = 0; i < bpf_len; ++i) {
            acc += bpf_taps_[i] * bpf_hist_[idx];
            if (--idx < 0)
                idx = bpf_len - 1;
        }
        if (++bpf_pos_ >= bpf_len)
            bpf_pos_ = 0;
        return acc;
    }
    
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
#if IMPULSE_BLANKER
        {
            value mag = std::abs(sample);
            blank_fast_ += (mag - blank_fast_) * value(1.0 / 64);
            if (blank_env_ < value(1e-9))
                blank_env_ = mag;
            if (blank_env_ < blank_fast_ * value(1.0 / 6)) {
                if (++blank_low_ >= 256) {
                    blank_env_ = blank_fast_;
                    blank_low_ = 0;
                }
            } else {
                blank_low_ = 0;
            }
            if (mag > 8 * blank_env_)
                sample = 0;
            else if (mag > 6 * blank_env_)
                sample *= 6 * blank_env_ / mag;
            blank_env_ += (std::min(mag, 3 * blank_env_) - blank_env_)
                        * value(1.0 / 4096);
        }
#endif
        if (bpf_enabled_)
            sample = bandpass(sample);
        // Convert to complex via Hilbert transform
        cmplx tmp = hilbert(blockdc(sample));
        buf_ = input_hist(tmp);
        ring_[ring_count_ % ring_len] = tmp;
        ++ring_count_;
        ++sample_count_;
        
        switch (state_) {
        case State::SEARCHING:
            if ((*correlator_ptr)(buf_)) {
                // Sync found
                ++stats_sync_count;
                symbol_pos = correlator_ptr->symbol_pos;
                cfo_rad = correlator_ptr->cfo_rad;

                frame_raw_.assign(buf_, buf_ + buffer_len);
                frame_symbol_pos_ = symbol_pos;
                
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
                } else if (postamble_mode_ >= 0) {
                    if (postamble_rescue(callback))
                        ++stats_postamble_rescues;
                    postamble_mode_ = -1;
                    reset();
                } else {
                    ++stats_preamble_errors;
                    reset();
                }
            }
            break;
            
        case State::COLLECTING_SYMBOLS:
            // Keep feeding correlator to maintain buffer
            (*correlator_ptr)(buf_);
            samples_needed_--;

            if (frame_raw_.size() < (size_t)(symbols_max + 8) * extended_len + buffer_len)
                frame_raw_.push_back(tmp);

            if (samples_needed_ <= 0) {
                // Process this symbol
                frame_sym_start_[symbol_index_] = (int)frame_raw_.size() - buffer_len;
                if (!process_symbol(symbol_index_)) {
                    // Error, go back to searching
                    ++stats_symbol_errors;
                    reset();
                    break;
                }

                symbol_index_++;

                if (symbol_index_ > symbol_count) {
                    saved_k_ = k_;
                    std::memcpy(perm_save_, perm, saved_k_ * sizeof(code_type));
                    if (!decode_frame(callback) &&
                        (retry_erasures(callback) || retry_twosided(callback) ||
                         retry_decode(callback)))
                        ++stats_retry_success;
                    reset();
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

#if CHAN_PILOT_HIST
        for (int i = 0; i < tone_count; ++i) {
            pilot_obs_[i] = chan[i];
            pilot_age_[i] = 1;
        }
#endif

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
#if CHAN_PILOT_HIST
        for (int i = 0; i < tone_count; ++i)
            pilot_obs_[i] *= DSP::polar<value>(1, tse(i + tone_off_const));
#endif

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
#if CHAN_PILOT_HIST
        for (int i = seed_off; i < tone_count; i += block_length) {
            pilot_obs_[i] = tone[i];
            pilot_age_[i] = 0;
        }
#endif
        
        // Decode meta data
        int64_t meta_info = meta_data();
        int mode;
        if (meta_info < 0) {
            if (last_good_mode_ < 0 || (precision < value(0.4) && !rescuing_)) {
                std::cerr << "Decoder: Preamble decoding error" << std::endl;
                return false;
            }
            mode = last_good_mode_;
            if (!replaying_)
                ++stats_sticky_syncs;
            std::cerr << "Decoder: Meta symbol damaged, trying last good mode " << mode << std::endl;
        } else {
            int64_t call = meta_info >> 8;
            if (call == postamble_call) {
                if (!replaying_ && !rescuing_)
                    postamble_mode_ = (int)(meta_info & 255);
                if (rescuing_)
                    preamble_primed_ = true;
                std::cerr << "Decoder: Postamble detected" << std::endl;
                return false;
            }
            if (call == 0 || call >= 262144000000000L) {
                std::cerr << "Decoder: Invalid call sign" << std::endl;
                return false;
            }

            char call_sign[10];
            base40_decoder(call_sign, call, 9);
            call_sign[9] = 0;
            std::cerr << "Decoder: Call sign: " << call_sign << std::endl;

            mode = meta_info & 255;
        }
        if (!setup(mode)) {
            std::cerr << "Decoder: Invalid mode" << std::endl;
            return false;
        }
        
        std::cerr << "Decoder: Mode " << oper_mode << ", " << symbol_count << " data symbols, mod_bits=" << mod_bits << ", code_order=" << code_order << ", data_bytes=" << data_bytes << std::endl;
        
        k_ = 0;
        snr[0] = precision;

        // Erasure budget: 3/4 of the code's redundancy fraction, beyond which
        // the polar decoder has no realistic chance and we resync instead.
        std::memset(erased_, 0, sizeof(erased_));
        erased_count_ = 0;
        int code_len = (1 << code_order) * (repeat2 ? 2 : 1);
        max_erased_ = (symbol_count * (code_len - data_bits - 32) * 3) / (code_len * 4);

        return true;
    }
    
    void build_fwd_perm(int* table, int order) {
        int len = 1 << order;
        table[0] = 0;
        if (order == 11) {
            CODE::XorShiftMask<int, 11, 1, 3, 4, 1> seq;
            for (int i = 1; i < len; ++i) table[i] = seq();
        } else if (order == 12) {
            CODE::XorShiftMask<int, 12, 1, 1, 4, 1> seq;
            for (int i = 1; i < len; ++i) table[i] = seq();
        } else if (order == 13) {
            CODE::XorShiftMask<int, 13, 1, 1, 9, 1> seq;
            for (int i = 1; i < len; ++i) table[i] = seq();
        } else if (order == 14) {
            CODE::XorShiftMask<int, 14, 1, 5, 10, 1> seq;
            for (int i = 1; i < len; ++i) table[i] = seq();
        } else if (order == 15) {
            CODE::XorShiftMask<int, 15, 1, 1, 3, 1> seq;
            for (int i = 1; i < len; ++i) table[i] = seq();
        } else if (order == 16) {
            CODE::XorShiftMask<int, 16, 1, 1, 14, 1> seq;
            for (int i = 1; i < len; ++i) table[i] = seq();
        }
    }

    bool process_symbol(int j) {
        seed_off = (block_skew * j + first_seed) % block_length;
        sym_k_start_[j] = k_;
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
            value tp = 0, cp = 0;
            for (int i = 0; i < tone_count; ++i) {
                tp += norm(tone[i]);
                cp += norm(chan[i]);
            }
            if (cp > 0 && tp > 4 * cp) {
                value s = std::sqrt(tp / cp);
                for (int i = 0; i < seed_tones; ++i) {
                    cmplx d = demod_or_erase(tone[i * block_length + seed_off],
                                             s * chan[i * block_length + seed_off]);
                    seed[i] = clamp(std::nearbyint(127 * d.real()));
                }
                seed_value = hadamard_decoder(seed);
                if (seed_value >= 0) {
                    std::cerr << "Decoder: Fade release at symbol " << j
                              << ", chan rescaled x" << s << std::endl;
                    for (int i = 0; i < tone_count; ++i)
                        chan[i] *= s;
#if CHAN_PILOT_HIST
                    for (int i = 0; i < tone_count; ++i)
                        pilot_obs_[i] *= s;
#endif
                    for (int i = 0; i < tone_count; ++i)
                        demod[i] = demod_or_erase(tone[i], chan[i]);
                }
            }
        }
        if (seed_value < 0) {
            if (++erased_count_ > max_erased_) {
                std::cerr << "Decoder: Seed damaged at symbol " << j << ", erasure budget (" << max_erased_ << ") exhausted" << std::endl;
                return false;
            }
            std::cerr << "Decoder: Seed damaged at symbol " << j << ", erasing symbol" << std::endl;
            if (!replaying_)
                ++stats_erased_symbols;
            erased_[j] = true;
            snr[j] = 0;
            saved_seed_off[j] = seed_off;
            // Without the seed value the data tones cannot be descrambled and
            // the pilots cannot key phase/channel updates: emit zero LLRs so
            // the polar decoder treats this symbol as punctured, and leave
            // chan untouched.
            for (int i = 0; i < tone_count; ++i) {
                if (i % block_length != seed_off) {
                    int bits = mod_bits;
                    if (mod_bits == 3 && k_ % 32 == 30) bits = 2;
                    if (mod_bits == 6 && k_ % 64 == 60) bits = 4;
                    if (mod_bits == 10 && k_ % 128 == 120) bits = 8;
                    if (mod_bits == 12 && k_ % 128 == 120) bits = 8;
                    for (int b = 0; b < bits; ++b)
                        perm[k_ + b] = 0;
                    k_ += bits;
                }
            }
#if CHAN_PILOT_HIST
            for (int i = 0; i < tone_count; ++i)
                ++pilot_age_[i];
#endif
            sym_k_end_[j] = k_;
            return true;
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
        for (int i = 0; i < tone_count; ++i) {
            chan[i] *= DSP::polar<value>(1, tse(i + tone_off_const));
#if RETRY_TWOSIDED
            saved_rot_[j * tone_count + i] = tse(i + tone_off_const);
#endif
        }
#if CHAN_PILOT_HIST
        for (int i = 0; i < tone_count; ++i) {
            pilot_obs_[i] *= DSP::polar<value>(1, tse(i + tone_off_const));
            ++pilot_age_[i];
        }
#endif

        if (value(CHAN_INTERP_ALPHA) > 0) {
#if CHAN_PILOT_HIST
            value hist_scale = 0;
            {
                cmplx num(0, 0);
                value da = 0, db = 0;
                for (int i = seed_off; i < tone_count; i += block_length) {
                    num += tone[i] * conj(pilot_obs_[i]);
                    da += norm(tone[i]);
                    db += norm(pilot_obs_[i]);
                }
                if (da > 0 && db > 0) {
                    value c = std::sqrt(norm(num) / (da * db));
                    if (c > value(0.75))
                        hist_scale = (c - value(0.75)) * 4;
                    hist_scale *= hist_scale;
                }
            }
#endif
            value prev_prec = 0, chan_mean = 0;
            if (CHAN_ALPHA_ADAPT) {
                prev_prec = snr[j - 1] > 0 ? snr[j - 1] : value(1);
                for (int i = 0; i < tone_count; ++i)
                    chan_mean += norm(chan[i]);
                chan_mean /= tone_count;
            }
            int last_pilot = seed_off + (seed_tones - 1) * block_length;
            for (int i = 0; i < tone_count; ++i) {
                cmplx fresh;
                if (i <= seed_off) {
                    fresh = tone[seed_off];
                } else if (i >= last_pilot) {
                    fresh = tone[last_pilot];
                } else {
                    int k = (i - seed_off) / block_length;
                    int p = seed_off + k * block_length;
                    value t = value(i - p) / value(block_length);
                    fresh = DSP::lerp(tone[p], tone[p + block_length], t);
                }
#if RETRY_TWOSIDED
                saved_fresh_[j * tone_count + i] = fresh;
#endif
#if CHAN_PILOT_HIST
                static const value hist_w[5] = {
                    value(0), value(0.5), value(0.35), value(0.22), value(0.12)
                };
                if (hist_scale > 0 && i % block_length != seed_off && pilot_age_[i] < 5)
                    fresh = DSP::lerp(fresh, pilot_obs_[i], hist_scale * hist_w[pilot_age_[i]]);
#endif
                value alpha = value(CHAN_INTERP_ALPHA);
                if (replaying_ && forced_alpha_ > 0) {
                    alpha = forced_alpha_;
                } else if (CHAN_ALPHA_ADAPT) {
                    value obs = std::max(norm(chan[i]), norm(fresh));
                    value pi = chan_mean > 0
                        ? prev_prec * obs / chan_mean : prev_prec;
                    alpha = pi / (pi + value(CHAN_ALPHA_KNEE));
                    if (alpha < value(CHAN_ALPHA_FLOOR))
                        alpha = value(CHAN_ALPHA_FLOOR);
                }
                chan[i] = DSP::lerp(chan[i], fresh, alpha);
            }
#if CHAN_PILOT_HIST
            for (int i = seed_off; i < tone_count; i += block_length) {
                pilot_obs_[i] = tone[i];
                pilot_age_[i] = 0;
            }
#endif
            for (int i = 0; i < tone_count; ++i)
                if (i % block_length != seed_off)
                    demod[i] = demod_or_erase(tone[i], chan[i]);
        }

        if (seed_value) {
            CODE::MLS seq(mls2_poly, seed_value);
            for (int i = 0; i < tone_count; ++i)
                if (i % block_length != seed_off)
                    demod[i] *= nrz(seq());
        }

        // Save demod for post-decode corrected SNR
        std::memcpy(&saved_demod[j * tone_count], demod, tone_count * sizeof(cmplx));
        saved_seed_off[j] = seed_off;
#if RETRY_TWOSIDED
        std::memcpy(&saved_tone_[j * tone_count], tone, tone_count * sizeof(cmplx));
        saved_seed_value_[j] = seed_value;
#endif

        // Notify constellation callback with fully-corrected demodulated symbols
        if (constellation_callback && !replaying_) {
            constellation_callback(demod, tone_count, mod_bits);
        }

        // SNR estimation from data tones only excluding seed/pilot tones
        value sp = 0, np = 0;
        for (int i = 0, l = k_; i < tone_count; ++i) {
            if (i % block_length != seed_off) {
                int bits = mod_bits;
                if (mod_bits == 3 && l % 32 == 30) bits = 2;
                if (mod_bits == 6 && l % 64 == 60) bits = 4;
                if (mod_bits == 10 && l % 128 == 120) bits = 8;
                if (mod_bits == 12 && l % 128 == 120) bits = 8;
                demap_hard(perm + l, demod[i], bits);
                cmplx hard = map_bits(perm + l, bits);
                cmplx error = demod[i] - hard;
                sp += norm(hard);
                np += norm(error);
                l += bits;
            }
        }
        
        value precision = sp / np;
        snr[j] = precision;
        precision = std::min(precision, value(1023));
        




        std::cerr << "Decoder: Symbol " << j << " SNR = " << 10 * std::log10(snr[j]) << " dB, k=" << k_ << std::endl;


        // Per-tone confidence: after equalization, noise on a tone scales
        // as 1/|chan|^2, so tones in selective-fading notches must demap
        // with proportionally less confidence than strong tones.
        value chan_pwr_mean = 0;
        if (PER_TONE_PRECISION) {
            for (int i = 0; i < tone_count; ++i)
                chan_pwr_mean += norm(chan[i]);
            chan_pwr_mean /= tone_count;
        }

        for (int i = 0; i < tone_count; ++i) {
            if (i % block_length != seed_off) {
                int bits = mod_bits;
                if (mod_bits == 3 && k_ % 32 == 30) bits = 2;
                if (mod_bits == 6 && k_ % 64 == 60) bits = 4;
                if (mod_bits == 10 && k_ % 128 == 120) bits = 8;
                if (mod_bits == 12 && k_ % 128 == 120) bits = 8;
                value prec = precision;
                if (PER_TONE_PRECISION && chan_pwr_mean > 0)
                    prec = std::min(precision * norm(chan[i]) / chan_pwr_mean, value(1023));
                demap_soft(perm + k_, demod[i], prec, bits);
                k_ += bits;
            }
        }

        // legacy pilot-only channel update
        if (value(CHAN_INTERP_ALPHA) <= 0)
            for (int i = seed_off; i < tone_count; i += block_length)
                chan[i] = DSP::lerp(chan[i], tone[i], value(0.5));

        sym_k_end_[j] = k_;
        return true;
    }
    
    bool retry_erasures(FrameCallback callback) {
        int idx[symbols_max], n = 0;
        for (int j = 1; j <= symbol_count; ++j)
            if (!erased_[j])
                idx[n++] = j;
        if (n < 4)
            return false;
        std::sort(idx, idx + n, [this](int a, int b) { return snr[a] < snr[b]; });
        value median = snr[idx[n / 2]];
        int weak = 0;
        while (weak < n / 2 && snr[idx[weak]] * value(1.5) <= median)
            ++weak;
        weak = std::min(weak, max_erased_ - erased_count_);
        bool ok = false;
        bool was_replaying = replaying_;
        replaying_ = true;
        bool erased_before[symbols_max];
        std::memcpy(erased_before, erased_, sizeof(erased_before));
        for (int m = weak; m >= 1; m /= 2) {
            std::memcpy(perm, perm_save_, saved_k_ * sizeof(code_type));
            std::memcpy(erased_, erased_before, sizeof(erased_before));
            for (int t = 0; t < m; ++t) {
                int j = idx[t];
                erased_[j] = true;
                for (int i = sym_k_start_[j]; i < sym_k_end_[j]; ++i)
                    perm[i] = 0;
            }
            std::cerr << "Decoder: retrying decode with " << m << " weakest symbols erased" << std::endl;
            if (decode_frame(callback)) {
                ok = true;
                break;
            }
        }
        if (!ok)
            std::memcpy(erased_, erased_before, sizeof(erased_before));
        replaying_ = was_replaying;
        return ok;
    }

    bool retry_twosided(FrameCallback callback) {
#if !RETRY_TWOSIDED
        (void)callback;
        return false;
#else
        if (value(CHAN_INTERP_ALPHA) <= 0)
            return false;
        std::cerr << "Decoder: retrying decode with two-sided channel estimate" << std::endl;
        std::memcpy(perm, perm_save_, saved_k_ * sizeof(code_type));
        cmplx chan2[tone_count], dem[tone_count];
        for (int j = 1; j <= symbol_count; ++j) {
            if (erased_[j])
                continue;
            for (int i = 0; i < tone_count; ++i) {
                cmplx acc = value(0.5) * saved_fresh_[j * tone_count + i];
                value wsum = value(0.5);
                if (j > 1 && !erased_[j - 1]) {
                    acc = acc + value(0.25) * (saved_fresh_[(j - 1) * tone_count + i]
                        * DSP::polar<value>(1, saved_rot_[j * tone_count + i]));
                    wsum += value(0.25);
                }
                if (j < symbol_count && !erased_[j + 1]) {
                    acc = acc + value(0.25) * (saved_fresh_[(j + 1) * tone_count + i]
                        * DSP::polar<value>(1, -saved_rot_[(j + 1) * tone_count + i]));
                    wsum += value(0.25);
                }
                chan2[i] = (value(1) / wsum) * acc;
            }
            int soff = saved_seed_off[j];
            for (int i = 0; i < tone_count; ++i)
                dem[i] = demod_or_erase(saved_tone_[j * tone_count + i], chan2[i]);
            if (saved_seed_value_[j]) {
                CODE::MLS seq(mls2_poly, saved_seed_value_[j]);
                for (int i = 0; i < tone_count; ++i)
                    if (i % block_length != soff)
                        dem[i] *= nrz(seq());
            }
            value sp = 0, np = 0, chan_pwr_mean = 0;
            for (int i = 0, l = sym_k_start_[j]; i < tone_count; ++i) {
                if (i % block_length != soff) {
                    int bits = mod_bits;
                    if (mod_bits == 3 && l % 32 == 30) bits = 2;
                    if (mod_bits == 6 && l % 64 == 60) bits = 4;
                    if (mod_bits == 10 && l % 128 == 120) bits = 8;
                    if (mod_bits == 12 && l % 128 == 120) bits = 8;
                    demap_hard(perm + l, dem[i], bits);
                    cmplx hard = map_bits(perm + l, bits);
                    sp += norm(hard);
                    np += norm(dem[i] - hard);
                    l += bits;
                }
                chan_pwr_mean += norm(chan2[i]);
            }
            chan_pwr_mean /= tone_count;
            value precision = std::min(sp / (np + value(1e-9)), value(1023));
            for (int i = 0, l = sym_k_start_[j]; i < tone_count; ++i) {
                if (i % block_length != soff) {
                    int bits = mod_bits;
                    if (mod_bits == 3 && l % 32 == 30) bits = 2;
                    if (mod_bits == 6 && l % 64 == 60) bits = 4;
                    if (mod_bits == 10 && l % 128 == 120) bits = 8;
                    if (mod_bits == 12 && l % 128 == 120) bits = 8;
                    value prec = precision;
                    if (PER_TONE_PRECISION && chan_pwr_mean > 0)
                        prec = std::min(precision * norm(chan2[i]) / chan_pwr_mean, value(1023));
                    demap_soft(perm + l, dem[i], prec, bits);
                    l += bits;
                }
            }
            std::memcpy(&saved_demod[j * tone_count], dem, tone_count * sizeof(cmplx));
        }
        bool was_replaying = replaying_;
        replaying_ = true;
        std::memcpy(perm_save_, perm, saved_k_ * sizeof(code_type));
        bool ok = decode_frame(callback) || retry_erasures(callback);
        replaying_ = was_replaying;
        return ok;
#endif
    }

    bool postamble_rescue(FrameCallback callback) {
        int mode = postamble_mode_;
        if (!setup(mode))
            return false;
        int64_t anchor = (int64_t)ring_count_ - (int64_t)buffer_len + symbol_pos;
        int64_t dist = 2 * symbol_len + guard_len
                     + (int64_t)(symbol_count + 1) * extended_len;
        int64_t begin = anchor - dist;
        if (begin < 0 || begin < (int64_t)ring_count_ - (int64_t)ring_len) {
            std::cerr << "Decoder: Postamble rescue: frame no longer buffered" << std::endl;
            return false;
        }
        if (last_decode_abs_ > 0 &&
            (int64_t)ring_count_ - (int64_t)last_decode_abs_ < dist + 2 * extended_len) {
            std::cerr << "Decoder: Postamble for already-decoded frame, skipping" << std::endl;
            return false;
        }
        std::cerr << "Decoder: Postamble rescue, mode " << mode << ", "
                  << symbol_count << " data symbols" << std::endl;

        int64_t total = dist + 2 * symbol_len + extended_len;
        frame_raw_.resize((size_t)total);
        for (int64_t i = 0; i < total; ++i)
            frame_raw_[(size_t)i] = ring_[(size_t)((begin + i) % (int64_t)ring_len)];

        int save_pos = symbol_pos;
        int save_good = last_good_mode_;
        rescuing_ = true;
        replaying_ = true;
        last_good_mode_ = mode;
        bool ok = false;

        symbol_pos = 0;
        buf_ = frame_raw_.data();
        delete seq1_ptr;
        seq1_ptr = new CODE::MLS(mls1_poly);
        if (process_preamble()) {
            bool bad = false;
            for (int j = 1; j <= symbol_count; ++j) {
                buf_ = frame_raw_.data() + 2 * symbol_len + guard_len
                     + (size_t)j * extended_len;
                if (!process_symbol(j)) { bad = true; break; }
            }
            if (!bad) {
                saved_k_ = k_;
                std::memcpy(perm_save_, perm, saved_k_ * sizeof(code_type));
                ok = decode_frame(callback) || retry_erasures(callback)
                  || retry_twosided(callback);
            }
        }

        if (!ok) {
            symbol_pos = (int)dist;
            buf_ = frame_raw_.data();
            delete seq1_ptr;
            seq1_ptr = new CODE::MLS(mls1_poly);
            preamble_primed_ = false;
            process_preamble();
            if (preamble_primed_ && setup(mode)) {
                k_ = 0;
                snr[0] = value(4);
                std::memset(erased_, 0, sizeof(erased_));
                erased_count_ = 0;
                int code_len = (1 << code_order) * (repeat2 ? 2 : 1);
                max_erased_ = (symbol_count * (code_len - data_bits - 32) * 3) / (code_len * 4);
                forced_alpha_ = 1;
                bool bad = false;
                for (int j = 1; j <= symbol_count; ++j) {
                    buf_ = frame_raw_.data() + 2 * symbol_len + guard_len
                         + (size_t)j * extended_len;
                    if (!process_symbol(j)) { bad = true; break; }
                }
                forced_alpha_ = 0;
                if (!bad) {
                    saved_k_ = k_;
                    std::memcpy(perm_save_, perm, saved_k_ * sizeof(code_type));
                    ok = decode_frame(callback) || retry_erasures(callback)
                      || retry_twosided(callback);
                }
            }
        }

        rescuing_ = false;
        replaying_ = false;
        symbol_pos = save_pos;
        if (!ok)
            last_good_mode_ = save_good;
        return ok;
    }

    bool retry_decode(FrameCallback callback) {
        struct Attempt { int dt; value alpha; };
        static const Attempt attempts[] = {
            { 0, value(1) },
            { 0, value(0.3) },
            { -40, value(0) },
            { 40, value(0) },
        };
        int save_pos = symbol_pos;
        int last = symbol_count;
        replaying_ = true;
        bool ok = false;
        for (const auto& a : attempts) {
            if (frame_symbol_pos_ + a.dt < 0)
                continue;
            if ((size_t)(frame_sym_start_[last] + a.dt + symbol_len) > frame_raw_.size())
                continue;
            forced_alpha_ = a.alpha;
            symbol_pos = frame_symbol_pos_ + a.dt;
            buf_ = frame_raw_.data();
            delete seq1_ptr;
            seq1_ptr = new CODE::MLS(mls1_poly);
            if (!process_preamble())
                continue;
            bool bad = false;
            for (int j = 1; j <= symbol_count; ++j) {
                buf_ = frame_raw_.data() + frame_sym_start_[j] + a.dt;
                if (!process_symbol(j)) { bad = true; break; }
            }
            if (bad)
                continue;
            saved_k_ = k_;
            std::memcpy(perm_save_, perm, saved_k_ * sizeof(code_type));
            if (decode_frame(callback) || retry_erasures(callback)) { ok = true; break; }
        }
        forced_alpha_ = 0;
        replaying_ = false;
        symbol_pos = save_pos;
        return ok;
    }

    bool decode_frame(FrameCallback callback) {
        std::cerr << "Decoder: Decoding frame, k_=" << k_ << " bits collected" << std::endl;
        std::cerr << "Decoder: Expected code_order=" << code_order << " (code length=" << (1 << code_order) << ")" << std::endl;
        
        int crc_bits = data_bits + 32;
        if (repeat2) {
            // chase combining: sum the llrs of the two codeword copies
            int len = 1 << code_order;
            for (int i = 0; i < len; ++i) {
                int32_t s = (int32_t)perm[i] + (int32_t)perm[i + len];
                perm[i] = (code_type)(s > 32767 ? 32767 : s < -32767 ? -32767 : s);
            }
        }
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
            if (!replaying_)
                ++stats_crc_errors;
            return false;
        }

        last_good_mode_ = oper_mode;
        last_decode_abs_ = ring_count_;

        // Fallback: average per-symbol SNR 
        value total_snr = 0;
        int snr_count = 0;
        for (int i = 1; i < symbol_index_; ++i) {
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

        for (int i = 0; i < data_bits + 32; ++i)
            ber_mesg[i] = (mesg[i].v[best] < 0) ? -1 : 1;
        ber_encoder(ber_code, ber_mesg, frozen_bits, code_order);





        // use known-correct codeword as referenc
        build_fwd_perm(fwd_perm_table, code_order);
        int cw_mask = (1 << code_order) - 1;
        value corr_sp = 0, corr_np = 0;
        int bit_errors = 0, counted_bits = 0;
        int bk = 0;
        for (int sj = 1; sj <= symbol_count; ++sj) {
            int soff = saved_seed_off[sj];
            for (int i = 0; i < tone_count; ++i) {
                if (i % block_length != soff) {
                    int bits = mod_bits;
                    if (mod_bits == 3 && bk % 32 == 30) bits = 2;
                    if (mod_bits == 6 && bk % 64 == 60) bits = 4;
                    if (mod_bits == 10 && bk % 128 == 120) bits = 8;
                    if (mod_bits == 12 && bk % 128 == 120) bits = 8;
                    if (!erased_[sj]) {
                        code_type ideal_bits[mod_max];
                        for (int b = 0; b < bits; ++b) {
                            int ci = fwd_perm_table[(bk + b) & cw_mask];
                            ideal_bits[b] = ber_code[ci];
                            if ((code[ci] < 0) != (ber_code[ci] < 0))
                                ++bit_errors;
                        }
                        counted_bits += bits;
                        cmplx ideal = map_bits(ideal_bits, bits);
                        cmplx error = saved_demod[sj * tone_count + i] - ideal;
                        corr_sp += norm(ideal);
                        corr_np += norm(error);
                    }
                    bk += bits;
                }
            }
        }
        last_ber_ = counted_bits > 0 ? value(bit_errors) / value(counted_bits) : 0;
        if (ber_ema_ < 0)
            ber_ema_ = last_ber_;
        else
            ber_ema_ = value(0.3) * last_ber_ + value(0.7) * ber_ema_;
        if (corr_np > 0)
            last_avg_snr_ = 10 * std::log10(corr_sp / corr_np);

        std::cerr << "Decoder: Frame decoded " << data_bytes << " bytes, SNR=" << last_avg_snr_ << " dB, BER=" << (last_ber_ * 100) << "%, erased symbols=" << erased_count_ << std::endl;

        callback(data, data_bytes);
        return true;
    }
};


using Encoder48k = ModemEncoder<float, DSP::Complex<float>, 48000>;
using Decoder48k = ModemDecoder<float, DSP::Complex<float>, 48000>;
