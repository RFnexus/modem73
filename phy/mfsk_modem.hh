#pragma once

#include <vector>
#include <cmath>
#include <functional>
#include <cstring>
#include <algorithm>
#include <cstdint>
#include <iostream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

enum class MFSKMode {
    MFSK_8   = 0,   // 8 tones, 250 Hz BW, 3 bits/sym, rate 1/2
    MFSK_16  = 1,   // 16 tones, 500 Hz BW, 4 bits/sym, rate 1/2
    MFSK_32  = 2,   // 32 tones, 1000 Hz BW, 5 bits/sym, rate 1/2
    MFSK_32R = 3    // 32 tones, 1000 Hz BW, 5 bits/sym, rate 3/4
};



static const char* MFSK_MODE_NAMES[] = {"MFSK-8", "MFSK-16", "MFSK-32", "MFSK-32R"};





struct MFSKParams {
    static constexpr int SAMPLE_RATE = 48000;
    static constexpr int SYMBOL_LEN = 1536;
    static constexpr float TONE_SPACING = 31.25f;
    static constexpr int PREAMBLE_SYMBOLS = 8;
    static constexpr int SYNC_SYMBOLS = 2;
    static constexpr int OVERHEAD_SYMBOLS = PREAMBLE_SYMBOLS + SYNC_SYMBOLS;
    static constexpr int DATA_SYMBOLS = 128;
    static constexpr int FRAME_SYMBOLS = OVERHEAD_SYMBOLS + DATA_SYMBOLS;
    static constexpr int GUARD_SAMPLES = 32;
    static constexpr int SEARCH_STEP = SYMBOL_LEN / 4;

    static constexpr int CONV_K = 7;
    static constexpr int CONV_STATES = 64;
    static constexpr int CONV_TAIL = 6;
    static constexpr uint8_t CONV_G0 = 0x79;
    static constexpr uint8_t CONV_G1 = 0x5B;



    static int num_tones(MFSKMode mode) {
        static const int t[] = {8, 16, 32, 32};
        return t[(int)mode];
    }

    static int bits_per_symbol(MFSKMode mode) {
        static const int b[] = {3, 4, 5, 5};
        return b[(int)mode];
    }

    static bool is_rate34(MFSKMode mode) {
        return mode == MFSKMode::MFSK_32R;
    }

    static int coded_capacity(MFSKMode mode) {
        return DATA_SYMBOLS * bits_per_symbol(mode);
    }

    static int data_bytes(MFSKMode mode) {
        int coded = coded_capacity(mode);
        if (is_rate34(mode))
            return (coded * 3 / 4 - CONV_TAIL) / 8;
        return (coded / 2 - CONV_TAIL) / 8;
    }

    static int max_payload(MFSKMode mode) {
        return data_bytes(mode) - 4;
    }

    static int frame_capacity(MFSKMode mode) {
        return data_bytes(mode) - 2;
    }

    static int base_bin(MFSKMode mode, int center_freq) {
        int center_bin = (center_freq * SYMBOL_LEN + SAMPLE_RATE / 2) / SAMPLE_RATE;
        return center_bin - num_tones(mode) / 2;
    }

    static float frame_duration() {
        return FRAME_SYMBOLS * (float)SYMBOL_LEN / SAMPLE_RATE;
    }

    static int bitrate(MFSKMode mode) {
        return (int)(max_payload(mode) * 8.0f / frame_duration());
    }
};


namespace mfsk_detail {

inline uint16_t crc16_ccitt(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
    }
    return crc;
}



inline int gray_encode(int n) { return n ^ (n >> 1); }





inline int gray_decode(int n) {
    int mask = n;
    while (mask) { mask >>= 1; n ^= mask; }
    return n;
}


inline int bit_reverse(int x, int bits) {
    int result = 0;
    for (int i = 0; i < bits; i++) { result = (result << 1) | (x & 1); x >>= 1; }
    return result;
}

inline float goertzel_mag2(const float* samples, int N, int bin) {
    float w = 2.0f * (float)M_PI * bin / N;
    float coeff = 2.0f * cosf(w);
    float s1 = 0.0f, s2 = 0.0f;
    for (int i = 0; i < N; i++) {
        float s0 = samples[i] + coeff * s1 - s2;
        s2 = s1;
        s1 = s0;
    }
    return s1 * s1 + s2 * s2 - coeff * s1 * s2;
}

inline int parity8(uint8_t x) {
    x ^= x >> 4; x ^= x >> 2; x ^= x >> 1;
    return x & 1;
}

inline void interleave(int* data, int n) {
    int bits = 0;
    while ((1 << bits) < n) bits++;
    std::vector<int> tmp(data, data + n);
    for (int i = 0; i < n; i++)
        data[bit_reverse(i, bits)] = tmp[i];
}

inline void interleave_vectors(std::vector<std::vector<float>>& data) {
    int n = data.size();
    int bits = 0;
    while ((1 << bits) < n) bits++;
    std::vector<std::vector<float>> tmp(data);
    for (int i = 0; i < n; i++)
        data[bit_reverse(i, bits)] = std::move(tmp[i]);
}

inline std::vector<int> conv_encode(const uint8_t* data, int data_bytes) {
    int data_bits = data_bytes * 8;
    int total_in = data_bits + MFSKParams::CONV_TAIL;
    std::vector<int> coded;
    coded.reserve(total_in * 2);
    uint8_t state = 0;
    for (int i = 0; i < total_in; i++) {
        int bit = (i < data_bits) ? (data[i / 8] >> (7 - (i % 8))) & 1 : 0;
        uint8_t inp = ((uint8_t)bit << 6) | state;
        coded.push_back(parity8(inp & MFSKParams::CONV_G0));
        coded.push_back(parity8(inp & MFSKParams::CONV_G1));
        state = (((uint8_t)bit << 5) | (state >> 1)) & 0x3F;
    }
    return coded;
}

class ViterbiDecoder {
public:
    static constexpr int STATES = MFSKParams::CONV_STATES;
    static constexpr int MAX_STEPS = 512;

    void reset() {
        for (int s = 0; s < STATES; s++) metric_[s] = -1e30f;
        metric_[0] = 0.0f;
        len_ = 0;
    }

    void step(float s0, float s1) {
        float new_metric[STATES];
        for (int s = 0; s < STATES; s++) new_metric[s] = -1e30f;
        for (int prev = 0; prev < STATES; prev++) {
            if (metric_[prev] < -1e29f) continue;
            for (int bit = 0; bit < 2; bit++) {
                uint8_t inp = ((uint8_t)bit << 6) | (uint8_t)prev;
                int c0 = parity8(inp & MFSKParams::CONV_G0);
                int c1 = parity8(inp & MFSKParams::CONV_G1);
                float branch = s0 * (1 - 2 * c0) + s1 * (1 - 2 * c1);
                int next = (((uint8_t)bit << 5) | ((uint8_t)prev >> 1)) & 0x3F;
                float candidate = metric_[prev] + branch;
                if (candidate > new_metric[next]) {
                    new_metric[next] = candidate;
                    survivor_[len_][next] = ((uint8_t)prev << 1) | (uint8_t)bit;
                }
            }
        }
        memcpy(metric_, new_metric, sizeof(metric_));
        len_++;
    }

    std::vector<uint8_t> finish(int data_bits) {
        int best = 0;
        for (int s = 1; s < STATES; s++)
            if (metric_[s] > metric_[best]) best = s;
        std::vector<int> bits(len_);
        int state = best;
        for (int t = len_ - 1; t >= 0; t--) {
            bits[t] = survivor_[t][state] & 1;
            state = survivor_[t][state] >> 1;
        }
        int n_bytes = data_bits / 8;
        std::vector<uint8_t> result(n_bytes, 0);
        for (int i = 0; i < data_bits && i < len_; i++)
            result[i / 8] |= bits[i] << (7 - (i % 8));
        return result;
    }

private:
    float metric_[STATES];
    uint8_t survivor_[MAX_STEPS][STATES];
    int len_ = 0;
};

inline void soft_demap(const float* energies, int n_tones, int bps, float* soft_bits) {
    for (int j = 0; j < bps; j++) {
        float e0 = 0, e1 = 0;
        int bit_pos = bps - 1 - j;
        for (int t = 0; t < n_tones; t++) {
            int sym_val = gray_decode(t);
            if ((sym_val >> bit_pos) & 1)
                e1 += energies[t];
            else
                e0 += energies[t];
        }
        soft_bits[j] = (e0 - e1) / (e0 + e1 + 1e-20f);
    }
}

inline std::vector<int> puncture_34(const std::vector<int>& coded) {
    std::vector<int> out;
    out.reserve(coded.size() * 2 / 3 + 4);
    for (size_t i = 0; i + 5 < coded.size(); i += 6) {
        out.push_back(coded[i]);
        out.push_back(coded[i+1]);
        out.push_back(coded[i+2]);
        out.push_back(coded[i+5]);
    }
    size_t rem = (coded.size() / 6) * 6;
    for (size_t i = rem; i < coded.size(); i++) out.push_back(coded[i]);
    return out;
}

inline std::vector<float> depuncture_34(const float* soft, int n_soft) {
    std::vector<float> out;
    out.reserve(n_soft * 3 / 2 + 8);
    int si = 0;
    while (si + 3 < n_soft) {
        out.push_back(soft[si++]);
        out.push_back(soft[si++]);
        out.push_back(soft[si++]);
        out.push_back(0.0f);
        out.push_back(0.0f);
        out.push_back(soft[si++]);
    }





    while (si < n_soft) { out.push_back(soft[si++]); out.push_back(0.0f); }
    return out;
}

inline std::vector<int> bits_to_gray_symbols(const std::vector<int>& bits, int bps) {
    std::vector<int> symbols;
    for (int i = 0; i + bps <= (int)bits.size(); i += bps) {
        int sym = 0;
        for (int b = 0; b < bps; b++)
            sym = (sym << 1) | bits[i + b];
        symbols.push_back(gray_encode(sym));
    }
    return symbols;
}

} // namespace mfsk_detail


class MFSKEncoder {
public:
    std::vector<float> encode(const uint8_t* data, size_t len,
                              int center_freq, MFSKMode mode) {
        using namespace mfsk_detail;

        int n_tones = MFSKParams::num_tones(mode);
        int bps = MFSKParams::bits_per_symbol(mode);
        int dbytes = MFSKParams::data_bytes(mode);
        int base = MFSKParams::base_bin(mode, center_freq);

        int payload_len = dbytes - 2;
        std::vector<uint8_t> frame(payload_len, 0);
        memcpy(frame.data(), data, std::min(len, (size_t)payload_len));

        uint16_t crc = crc16_ccitt(frame.data(), frame.size());
        frame.push_back(crc >> 8);
        frame.push_back(crc & 0xFF);

        auto coded_bits = conv_encode(frame.data(), dbytes);
        if (MFSKParams::is_rate34(mode))
            coded_bits = puncture_34(coded_bits);

        int capacity = MFSKParams::DATA_SYMBOLS * bps;
        while ((int)coded_bits.size() < capacity)
            coded_bits.push_back(0);

        auto symbols = bits_to_gray_symbols(coded_bits, bps);
        symbols.resize(MFSKParams::DATA_SYMBOLS, 0);
        interleave(symbols.data(), symbols.size());

        std::vector<int> frame_tones;
        frame_tones.reserve(MFSKParams::FRAME_SYMBOLS);
        for (int i = 0; i < MFSKParams::PREAMBLE_SYMBOLS; i++)
            frame_tones.push_back(i % 2 == 0 ? 0 : n_tones - 1);
        frame_tones.push_back(n_tones / 4);
        frame_tones.push_back(3 * n_tones / 4);
        frame_tones.insert(frame_tones.end(), symbols.begin(), symbols.end());

        return generate_audio(frame_tones, base);
    }

    int get_payload_size(MFSKMode mode) {
        return MFSKParams::frame_capacity(mode);
    }

private:
    float phase_ = 0.0f;

    std::vector<float> generate_audio(const std::vector<int>& tones, int base_bin) {
        const int N = MFSKParams::SYMBOL_LEN;
        const int G = MFSKParams::GUARD_SAMPLES;
        const float amp = 0.8f;
        std::vector<float> audio;
        audio.reserve(tones.size() * N);
        phase_ = 0.0f;
        for (int tone : tones) {
            float freq = (base_bin + tone) * MFSKParams::TONE_SPACING;
            float phase_inc = 2.0f * (float)M_PI * freq / MFSKParams::SAMPLE_RATE;
            for (int i = 0; i < N; i++) {
                float env = 1.0f;
                if (i < G)
                    env = 0.5f * (1.0f - cosf((float)M_PI * i / G));
                else if (i >= N - G)
                    env = 0.5f * (1.0f + cosf((float)M_PI * (i - N + G) / G));
                audio.push_back(amp * env * sinf(phase_));
                phase_ += phase_inc;
            }
            phase_ = fmodf(phase_, 2.0f * (float)M_PI);
        }
        return audio;
    }
};


class MFSKDecoder {
public:
    using FrameCallback = std::function<void(const uint8_t*, size_t)>;

    MFSKDecoder() {}
    MFSKDecoder(MFSKMode mode, int center_freq = 1500) { configure(mode, center_freq); }

    void configure(MFSKMode mode, int center_freq) {
        mode_ = mode;
        n_tones_ = MFSKParams::num_tones(mode);
        bps_ = MFSKParams::bits_per_symbol(mode);
        base_bin_ = MFSKParams::base_bin(mode, center_freq);
        sync1_tone_ = n_tones_ / 4;
        sync3_tone_ = 3 * n_tones_ / 4;
        reset();
    }

    void process(const float* samples, size_t count, FrameCallback callback) {
        buf_.insert(buf_.end(), samples, samples + count);

        while (true) {
            if (buf_.size() - buf_pos_ < (size_t)MFSKParams::SYMBOL_LEN)
                break;

            const float* window = buf_.data() + buf_pos_;

            switch (state_) {
            case State::SEARCHING: {
                float e0  = mfsk_detail::goertzel_mag2(window, MFSKParams::SYMBOL_LEN, base_bin_);
                float en  = mfsk_detail::goertzel_mag2(window, MFSKParams::SYMBOL_LEN, base_bin_ + n_tones_ - 1);
                float eq1 = mfsk_detail::goertzel_mag2(window, MFSKParams::SYMBOL_LEN, base_bin_ + sync1_tone_);
                float eq3 = mfsk_detail::goertzel_mag2(window, MFSKParams::SYMBOL_LEN, base_bin_ + sync3_tone_);

                int p = step_count_ % 4;
                update_tracker(p, e0, en, eq1, eq3);

                if (trackers_[p].tstate == TState::READY) {
                    ++stats_sync_count;

                    freq_offset_ = 0;
                    float best_afc_e = 0;
                    for (int foff = -3; foff <= 3; foff++) {
                        float e = mfsk_detail::goertzel_mag2(window, MFSKParams::SYMBOL_LEN,
                                    base_bin_ + foff + sync3_tone_);
                        if (e > best_afc_e) {
                            best_afc_e = e;
                            freq_offset_ = foff;
                        }
                    }

                    int best_off = 0;
                    float best_e = 0;
                    int sync_bin = base_bin_ + freq_offset_ + sync3_tone_;
                    int half_step = MFSKParams::SEARCH_STEP / 2;
                    for (int off = -half_step; off <= half_step; off += 16) {
                        int64_t pos = (int64_t)buf_pos_ + off;
                        if (pos < 0 || pos + MFSKParams::SYMBOL_LEN > (int64_t)buf_.size())
                            continue;
                        float e = mfsk_detail::goertzel_mag2(
                            buf_.data() + pos, MFSKParams::SYMBOL_LEN, sync_bin);
                        if (e > best_e) {
                            best_e = e;
                            best_off = off;
                        }
                    }
                    if (best_off >= 0)
                        buf_pos_ += best_off;
                    else
                        buf_pos_ -= (size_t)(-best_off);

                    std::cerr << "MFSK: Sync (phase " << p
                              << " t=" << best_off
                              << " f=" << freq_offset_ << ")" << std::endl;

                    buf_pos_ += MFSKParams::SYMBOL_LEN;
                    state_ = State::COLLECTING;
                    collect_count_ = 0;
                    collected_.clear();
                    collected_.reserve(MFSKParams::DATA_SYMBOLS);
                    reset_trackers();
                    continue;
                }

                step_count_++;
                buf_pos_ += MFSKParams::SEARCH_STEP;
                break;
            }

            case State::COLLECTING: {
                if (collect_count_ == 0)
                    collect_start_pos_ = buf_pos_;

                if (collect_count_ > 0 && (collect_count_ % 16) == 0) {
                    float center_ratio = 0, best_ratio = 0;
                    int best_adj = 0;
                    for (int adj = -32; adj <= 32; adj += 8) {
                        int64_t pos = (int64_t)buf_pos_ + adj;
                        if (pos < 0 || pos + MFSKParams::SYMBOL_LEN > (int64_t)buf_.size())
                            continue;
                        const float* w = buf_.data() + pos;
                        float max_e = 0, total_e = 0;
                        for (int t = 0; t < n_tones_; t++) {
                            float e = mfsk_detail::goertzel_mag2(w, MFSKParams::SYMBOL_LEN, base_bin_ + freq_offset_ + t);
                            total_e += e;
                            if (e > max_e) max_e = e;
                        }
                        float ratio = max_e / (total_e + 1e-20f);
                        if (adj == 0) center_ratio = ratio;
                        if (ratio > best_ratio) { best_ratio = ratio; best_adj = adj; }
                    }
                    if (best_adj != 0 && best_ratio > center_ratio * 1.02f) {
                        if (best_adj >= 0) buf_pos_ += best_adj;
                        else buf_pos_ -= (size_t)(-best_adj);
                        window = buf_.data() + buf_pos_;
                    }
                }

                std::vector<float> energies(n_tones_);
                for (int t = 0; t < n_tones_; t++)
                    energies[t] = mfsk_detail::goertzel_mag2(window, MFSKParams::SYMBOL_LEN,
                                                              base_bin_ + freq_offset_ + t);
                collected_.push_back(std::move(energies));
                collect_count_++;
                buf_pos_ += MFSKParams::SYMBOL_LEN;

                if (collect_count_ >= MFSKParams::DATA_SYMBOLS) {
                    bool decoded = try_decode_auto(callback);
                    if (!decoded) {
                        for (int retry_off : {8, -8, 16, -16}) {
                            if (recompute_with_offset(retry_off) && try_decode_auto(callback)) {
                                decoded = true;
                                break;
                            }
                        }
                    }
                    if (!decoded) ++stats_crc_errors;
                    state_ = State::SEARCHING;
                    step_count_ = 0;
                }
                break;
            }
            }
        }

        if (buf_pos_ > 8192 && state_ == State::SEARCHING) {
            buf_.erase(buf_.begin(), buf_.begin() + buf_pos_);
            buf_pos_ = 0;
        }
    }

    void reset() {
        buf_.clear();
        buf_pos_ = 0;
        state_ = State::SEARCHING;
        step_count_ = 0;
        collect_count_ = 0;
        freq_offset_ = 0;
        collected_.clear();
        reset_trackers();
    }

    float get_last_snr() const { return last_snr_; }
    float get_last_ber() const { return last_ber_; }
    float get_ber_ema() const { return ber_ema_; }

    int stats_sync_count = 0;
    int stats_preamble_errors = 0;
    int stats_crc_errors = 0;

    void reset_stats() {
        stats_sync_count = 0;
        stats_preamble_errors = 0;
        stats_crc_errors = 0;
        last_snr_ = 0;
        last_ber_ = -1;
        ber_ema_ = -1;
    }

private:
    MFSKMode mode_ = MFSKMode::MFSK_16;
    int n_tones_ = 16;
    int bps_ = 4;
    int base_bin_ = 40;
    int freq_offset_ = 0;
    int sync1_tone_ = 4;
    int sync3_tone_ = 12;

    std::vector<float> buf_;
    size_t buf_pos_ = 0;

    enum class State { SEARCHING, COLLECTING };
    State state_ = State::SEARCHING;
    int step_count_ = 0;

    int collect_count_ = 0;
    size_t collect_start_pos_ = 0;
    std::vector<std::vector<float>> collected_;

    float last_snr_ = 0;
    float last_ber_ = -1;
    float ber_ema_ = -1;

    enum class TState { PREAMBLE, SYNC1, SYNC2, READY };
    struct Tracker { TState tstate = TState::PREAMBLE; int count = 0; int last_tone = -1; };
    Tracker trackers_[4];

    void reset_trackers() {
        for (auto& t : trackers_) { t.tstate = TState::PREAMBLE; t.count = 0; t.last_tone = -1; }
    }





    void update_tracker(int p, float e0, float en, float eq1, float eq3) { 
        auto& t = trackers_[p];
        float total = e0 + en + eq1 + eq3 + 1e-20f;

        switch (t.tstate) {
        case TState::PREAMBLE: {
            bool low_dom  = (e0 / total > 0.4f);
            bool high_dom = (en / total > 0.4f);
            if (low_dom && !high_dom) {
                t.count = (t.last_tone == 1) ? t.count + 1 : 1;
                t.last_tone = 0;
            } else if (high_dom && !low_dom) {
                t.count = (t.last_tone == 0) ? t.count + 1 : 1;
                t.last_tone = 1;
            } else {
                t.count = 0; t.last_tone = -1;
            }
            if (t.count >= MFSKParams::PREAMBLE_SYMBOLS)
                t.tstate = TState::SYNC1;
            break;
        }
        case TState::SYNC1:
            if (eq1 / total > 0.25f) {
                t.tstate = TState::SYNC2;
            } else {
                bool low_dom  = (e0 / total > 0.4f);
                bool high_dom = (en / total > 0.4f);
                if ((low_dom && t.last_tone == 1) || (high_dom && t.last_tone == 0))
                    t.last_tone = low_dom ? 0 : 1;
                else { ++stats_preamble_errors; t.tstate = TState::PREAMBLE; t.count = 0; t.last_tone = -1; }
            }
            break;
        case TState::SYNC2:
            if (eq3 / total > 0.25f)
                t.tstate = TState::READY;
            else { ++stats_preamble_errors; t.tstate = TState::PREAMBLE; t.count = 0; t.last_tone = -1; }
            break;
        case TState::READY:
            break;
        }
    }

    bool try_decode_auto(FrameCallback callback) {
        if (try_decode(callback, mode_)) return true;
        if (n_tones_ == 32) {
            MFSKMode alt = MFSKParams::is_rate34(mode_) ? MFSKMode::MFSK_32 : MFSKMode::MFSK_32R;
            if (try_decode(callback, alt)) return true;
        }
        return false;
    }

    bool recompute_with_offset(int offset) {
        collected_.clear();
        collected_.reserve(MFSKParams::DATA_SYMBOLS);
        int64_t pos = (int64_t)collect_start_pos_ + offset;
        if (pos < 0) return false;

        for (int i = 0; i < MFSKParams::DATA_SYMBOLS; i++) {
            if ((size_t)pos + MFSKParams::SYMBOL_LEN > buf_.size()) return false;
            const float* w = buf_.data() + pos;
            std::vector<float> energies(n_tones_);
            for (int t = 0; t < n_tones_; t++)
                energies[t] = mfsk_detail::goertzel_mag2(w, MFSKParams::SYMBOL_LEN, base_bin_ + freq_offset_ + t);
            collected_.push_back(std::move(energies));
            pos += MFSKParams::SYMBOL_LEN;
        }
        return true;
    }

    bool try_decode(FrameCallback callback, MFSKMode decode_mode) {
        using namespace mfsk_detail;

        auto deinterleaved = collected_;
        interleave_vectors(deinterleaved);

        int total_soft = MFSKParams::DATA_SYMBOLS * bps_;
        std::vector<float> soft_wire(total_soft);
        float soft_buf[12];
        for (int i = 0; i < MFSKParams::DATA_SYMBOLS; i++) {
            soft_demap(deinterleaved[i].data(), n_tones_, bps_, soft_buf);
            for (int b = 0; b < bps_; b++)
                soft_wire[i * bps_ + b] = soft_buf[b];
        }

        std::vector<float> soft_full;
        const float* soft_ptr;
        int soft_len;
        if (MFSKParams::is_rate34(decode_mode)) {
            soft_full = depuncture_34(soft_wire.data(), total_soft);
            soft_ptr = soft_full.data();
            soft_len = (int)soft_full.size();
        } else {
            soft_ptr = soft_wire.data();
            soft_len = total_soft;
        }

        int dbytes = MFSKParams::data_bytes(decode_mode);
        int data_bits = dbytes * 8;
        int n_steps = data_bits + MFSKParams::CONV_TAIL;

        ViterbiDecoder viterbi;
        viterbi.reset();
        for (int s = 0; s < n_steps && s * 2 + 1 < soft_len; s++)
            viterbi.step(soft_ptr[s * 2], soft_ptr[s * 2 + 1]);

        auto bytes = viterbi.finish(data_bits);
        bytes.resize(dbytes, 0);

        uint16_t computed = crc16_ccitt(bytes.data(), dbytes - 2);
        uint16_t received = ((uint16_t)bytes[dbytes - 2] << 8) | bytes[dbytes - 1];
        if (computed != received) return false;

        auto re_coded = conv_encode(bytes.data(), dbytes);
        if (MFSKParams::is_rate34(decode_mode))
            re_coded = puncture_34(re_coded);
        int re_capacity = MFSKParams::DATA_SYMBOLS * bps_;
        while ((int)re_coded.size() < re_capacity) re_coded.push_back(0);
        auto expected_tones = bits_to_gray_symbols(re_coded, bps_);
        expected_tones.resize(MFSKParams::DATA_SYMBOLS, 0);

        float signal_e = 0, noise_e = 0;
        for (int i = 0; i < MFSKParams::DATA_SYMBOLS; i++) {
            int expected = expected_tones[i];
            float total_e = 0;
            for (int t = 0; t < n_tones_; t++) total_e += deinterleaved[i][t];
            signal_e += deinterleaved[i][expected];
            noise_e += (total_e - deinterleaved[i][expected]);
        }

        if (noise_e > 1e-10f) {
            float sig = signal_e / MFSKParams::DATA_SYMBOLS;
            float noi = noise_e / (MFSKParams::DATA_SYMBOLS * (n_tones_ - 1));
            last_snr_ = 10.0f * log10f(sig / noi);
        } else {
            last_snr_ = 50.0f;
        }

        std::cerr << "MFSK: Decoded SNR=" << (int)last_snr_ << "dB" << std::endl;
        callback(bytes.data(), dbytes - 2);
        return true;
    }
};
