#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

class ToneDCD {
public:
    static constexpr int BLOCK_MS = 10;
    static constexpr int CHIP_BLOCKS = 2;
    static constexpr int CODE_LEN = 11;
    static constexpr int SEQ_BLOCKS = CODE_LEN * CHIP_BLOCKS;
    static constexpr int FSK_DEV = 75;
    static constexpr int ID_SYMBOLS = 12;
    static constexpr int ID_BITS = 16;
    static constexpr int SCORE_MIN_PCT = 55;
    static constexpr int RUN_BLOCKS = 15;
    static constexpr float PURITY_MIN = 0.45f;
    static constexpr float ENERGY_FLOOR = 1e-7f;

    static const int* code() {
        static const int c[CODE_LEN] = {1, 1, 1, 0, 0, 0, 1, 0, 0, 1, 0};
        return c;
    }

    static uint8_t crc8(uint32_t v, int bits) {
        uint8_t crc = 0xFF;
        for (int i = bits - 1; i >= 0; i--) {
            uint8_t in = (uint8_t)((v >> i) & 1);
            uint8_t fb = (uint8_t)(((crc >> 7) & 1) ^ in);
            crc = (uint8_t)((crc << 1) ^ (fb ? 0x07 : 0));
        }
        return crc;
    }

    ToneDCD(int center_freq, int sample_rate)
        : rate_(sample_rate), block_(sample_rate * BLOCK_MS / 1000) {
        buf_.resize(block_);
        configure(center_freq);
    }

    void configure(int center_freq) {
        freq_ = center_freq;
        for (int d = 0; d < NDELTA; d++) {
            int off = (d - NDELTA / 2) * 50;
            co1_[d] = gcoeff(freq_ + FSK_DEV + off);
            co0_[d] = gcoeff(freq_ - FSK_DEV + off);
            coc_[d] = gcoeff(freq_ + off);
            for (int s = 0; s < 4; s++)
                coid_[d][s] = gcoeff(freq_ + (s * 100 - 150) + off);
        }
        reset();
    }

    void reset() {
        fill_ = 0;
        ring_fill_ = 0;
        run_ = 0;
        sig_latch_ = false;
        id_need_ = 0;
        id_buf_.clear();
        hist_.clear();
        id_latch_ = false;
        id_fail_latch_ = false;
        last_score_ = 0;
        for (int d = 0; d < NDELTA; d++)
            for (int i = 0; i < SEQ_BLOCKS; i++)
                soft_[d][i] = 0.0f;
    }

    void process(const float* samples, int n) {
        for (int i = 0; i < n; i++) {
            if (id_need_ > 0) {
                id_buf_.push_back(samples[i]);
                if (--id_need_ == 0)
                    demod_id();
            }
            hist_.push_back(samples[i]);
            if ((int)hist_.size() > 3 * block_)
                hist_.erase(hist_.begin(), hist_.begin() + block_);
            buf_[fill_++] = samples[i];
            if (fill_ == block_) {
                step_block();
                fill_ = 0;
            }
        }
    }

    bool consume_signature() {
        bool v = sig_latch_;
        sig_latch_ = false;
        return v;
    }

    bool consume_station_id(uint16_t* id) {
        if (!id_latch_) return false;
        id_latch_ = false;
        *id = id_value_;
        return true;
    }

    bool consume_id_failure() {
        bool v = id_fail_latch_;
        id_fail_latch_ = false;
        return v;
    }

    bool tone_run_active() const { return run_ >= RUN_BLOCKS; }
    int last_score() const { return last_score_; }

    static std::vector<float> signature_lead(int freq_hz, int num_samples,
                                             float amplitude, int sample_rate,
                                             uint16_t station_id = 0) {
        std::vector<float> out(std::max(0, num_samples));
        if (out.empty()) return out;
        int chip = sample_rate * BLOCK_MS * CHIP_BLOCKS / 1000;
        int sym = sample_rate * BLOCK_MS / 1000;
        int keyed = chip * CODE_LEN + sym * ID_SYMBOLS;
        int gramp = sample_rate / 100;
        bool key = num_samples >= keyed;
        uint32_t word = ((uint32_t)station_id << 8) | crc8(station_id, ID_BITS);
        float phase = 0.0f;
        for (int i = 0; i < num_samples; i++) {
            int f = freq_hz;
            if (key && i < chip * CODE_LEN) {
                f = code()[i / chip] ? freq_hz + FSK_DEV : freq_hz - FSK_DEV;
            } else if (key && i < keyed) {
                int s = (i - chip * CODE_LEN) / sym;
                int val = (int)((word >> (2 * (ID_SYMBOLS - 1 - s))) & 3);
                f = freq_hz + (val * 100 - 150);
            }
            phase += 2.0f * (float)M_PI * f / sample_rate;
            if (phase > 2.0f * (float)M_PI) phase -= 2.0f * (float)M_PI;
            float env = 1.0f;
            if (i < gramp) env = (float)i / gramp;
            if (i > num_samples - gramp)
                env = std::min(env, (float)(num_samples - i) / gramp);
            out[i] = amplitude * env * std::sin(phase);
        }
        return out;
    }

private:
    static constexpr int NDELTA = 5;

    float gcoeff(int f) const {
        return 2.0f * std::cos(2.0f * (float)M_PI * f / rate_);
    }

    float gpower(float coeff) const {
        float s1 = 0.0f, s2 = 0.0f;
        for (int i = 0; i < block_; i++) {
            float s = buf_[i] + coeff * s1 - s2;
            s2 = s1;
            s1 = s;
        }
        return s1 * s1 + s2 * s2 - coeff * s1 * s2;
    }

    float gpower_at(const float* p, int n, float coeff) const {
        float s1 = 0.0f, s2 = 0.0f;
        for (int i = 0; i < n; i++) {
            float s = p[i] + coeff * s1 - s2;
            s2 = s1;
            s1 = s;
        }
        return s1 * s1 + s2 * s2 - coeff * s1 * s2;
    }

    void demod_id() {
        int step = block_ / 4;
        int span = (int)id_buf_.size() - ID_SYMBOLS * block_;
        struct Cand { float margin; uint32_t word; };
        Cand top3[3] = {{-1.0f, 0}, {-1.0f, 0}, {-1.0f, 0}};
        for (int d = 0; d < NDELTA; d++) {
            for (int off = 0; off <= span; off += step) {
                float margin = 0.0f;
                uint32_t word = 0;
                for (int s = 0; s < ID_SYMBOLS; s++) {
                    const float* p = id_buf_.data() + off + s * block_;
                    float best = -1.0f, second = -1.0f;
                    int sym = 0;
                    for (int k = 0; k < 4; k++) {
                        float pw = gpower_at(p, block_, coid_[d][k]);
                        if (pw > best) {
                            second = best;
                            best = pw;
                            sym = k;
                        } else if (pw > second) {
                            second = pw;
                        }
                    }
                    margin += best - second;
                    word = (word << 2) | (uint32_t)sym;
                }
                if (margin > top3[0].margin) {
                    top3[2] = top3[1];
                    top3[1] = top3[0];
                    top3[0] = {margin, word};
                } else if (margin > top3[1].margin) {
                    top3[2] = top3[1];
                    top3[1] = {margin, word};
                } else if (margin > top3[2].margin) {
                    top3[2] = {margin, word};
                }
            }
        }
        id_buf_.clear();
        for (int i = 0; i < 3; i++) {
            uint16_t id = (uint16_t)(top3[i].word >> 8);
            uint8_t crc = (uint8_t)(top3[i].word & 0xFF);
            if (top3[i].margin >= 0.0f && crc == crc8(id, ID_BITS)) {
                id_value_ = id;
                id_latch_ = true;
                return;
            }
        }
        id_fail_latch_ = true;
    }

    void step_block() {
        float energy = 0.0f;
        for (int i = 0; i < block_; i++) energy += buf_[i] * buf_[i];
        bool live = energy / block_ > ENERGY_FLOOR;

        float best_center = 0.0f;
        int best_score = -100;
        int best_d = NDELTA / 2;
        for (int d = 0; d < NDELTA; d++) {
            float p1 = live ? gpower(co1_[d]) : 0.0f;
            float p0 = live ? gpower(co0_[d]) : 0.0f;
            float soft = (p1 - p0) / (p1 + p0 + 1e-12f);
            if (!live) soft = 0.0f;
            for (int i = 0; i < SEQ_BLOCKS - 1; i++)
                soft_[d][i] = soft_[d][i + 1];
            soft_[d][SEQ_BLOCKS - 1] = soft;
            best_center = std::max(best_center, live ? gpower(coc_[d]) : 0.0f);
            if (ring_fill_ >= SEQ_BLOCKS - 1) {
                float acc = 0.0f;
                for (int i = 0; i < SEQ_BLOCKS; i++) {
                    int expect = code()[i / CHIP_BLOCKS] ? 1 : -1;
                    acc += soft_[d][i] * expect;
                }
                int pct = (int)std::lround(100.0f * acc / SEQ_BLOCKS);
                if (pct > best_score) { best_score = pct; best_d = d; }
            }
        }
        if (ring_fill_ < SEQ_BLOCKS) {
            ring_fill_++;
            last_score_ = ring_fill_ < SEQ_BLOCKS ? 0 : std::max(0, best_score);
        } else {
            last_score_ = best_score;
        }
        if (ring_fill_ >= SEQ_BLOCKS && best_score >= SCORE_MIN_PCT &&
            id_need_ == 0 && id_buf_.empty() && !sig_latch_) {
            sig_latch_ = true;
            id_delta_ = best_d;
            id_buf_.assign(hist_.end() - std::min((size_t)(2 * block_),
                                                  hist_.size()),
                           hist_.end());
            id_need_ = (ID_SYMBOLS + 3) * block_;
        }

        float ptone = std::max(best_center,
                               std::max(live ? gpower(co1_[NDELTA / 2]) : 0.0f,
                                        live ? gpower(co0_[NDELTA / 2]) : 0.0f));
        float purity = ptone / (0.5f * block_ * energy + 1e-12f);
        bool on = live && purity > PURITY_MIN;
        run_ = on ? run_ + 1 : 0;
    }

    int rate_;
    int block_;
    int freq_ = 1500;
    float co1_[NDELTA] = {};
    float co0_[NDELTA] = {};
    float coc_[NDELTA] = {};
    float coid_[NDELTA][4] = {};
    std::vector<float> buf_;
    int fill_ = 0;
    float soft_[NDELTA][SEQ_BLOCKS] = {};
    int ring_fill_ = 0;
    int run_ = 0;
    int last_score_ = 0;
    int id_need_ = 0;
    int id_delta_ = 0;
    std::vector<float> id_buf_;
    std::vector<float> hist_;
    uint16_t id_value_ = 0;
    bool id_latch_ = false;
    bool id_fail_latch_ = false;
    bool sig_latch_ = false;
};
