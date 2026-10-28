#pragma once

#include <algorithm>
#include <cstdint>
#include <random>

struct CsmaPreset {
    const char* name;
    int quiet_ms;
    int cw;
    int slot_ms;
    int burst;
    int dither;
    bool lead_tone;
};
static const CsmaPreset CSMA_PRESETS[2][4] = {
    {
        {"BENCH",    0,  3, 500, 3, 0,    true},
        {"RELAXED",  0,  8, 500, 3, 300,  true},
        {"MODERATE", 0, 12, 500, 2, 800,  true},
        {"BUSY",     0, 16, 500, 2, 1500, true},
    },
    {
        {"BENCH",    0,  2, 200, 4, 0,   true},
        {"RELAXED",  0,  4, 200, 4, 200, true},
        {"MODERATE", 0,  6, 200, 3, 300, true},
        {"BUSY",     0, 10, 200, 2, 500, true},
    },
};
static const char* CSMA_BAND_NAMES[2] = {"HF", "VHF/UHF"};
static constexpr int CSMA_PRESET_COUNT = 4;

struct CsmaConfig {
    float threshold_db = -30.0f;
    bool sync_only = false;
    int poll_ms = 25;
    int quiet_ms = 1500;
    int cw = 8;
    int slot_ms = 500;
    bool responder = false;
    int responder_quiet_ms = 300;
    int responder_dither_ms = 0;
    int deaf_limit_ms = 5000;
    int busy_limit_ms = 60000;
    int idle_credit_ms = 0;
    int cold_channel_ms = 10000;
    int dcd_detect_ms = 780;
    int contenders = -1;
    int rank = -1;
    int rank_n = 0;
    int extra_delay_ms = 0;
    bool redraw_on_busy = false;
};

class CsmaGate {
public:
    enum class Verdict { WAIT, TRANSMIT };
    enum class Reason { NONE, CLEAR, RESPONDER, BUSY_OVERRIDE, NO_AUDIO };
    static constexpr int RANKED_SLOT_MS = 1500;
    static constexpr int SETTLED_MS = 2500;

    CsmaGate(const CsmaConfig& cfg, uint32_t seed) : cfg_(cfg), gen_(seed) {
        int slot = std::max(1, cfg_.slot_ms);
        int window = std::max(2, cfg_.cw) * slot;
        if (cfg_.sync_only) {
            int det = std::max(1, cfg_.dcd_detect_ms);
            if (cfg_.contenders >= 0) {
                int slots = std::min(16, std::max(6, 3 * (cfg_.contenders + 1)));
                window = std::max(slots * (det + 150), 4 * slot);
            } else {
                window = std::max(16 * (det + 150), 4 * slot);
            }
        }
        if (cfg_.idle_credit_ms >= cfg_.cold_channel_ms)
            window = std::max(window / 4, 4 * slot);
        window_ = window;
        if (cfg_.sync_only && !cfg_.responder && cfg_.rank >= 0) {
            rank_slot_ = RANKED_SLOT_MS;
            window_ = std::max(1, cfg_.rank_n) * rank_slot_;
            quiet_needed_ = cfg_.quiet_ms;
            contention_ms_ = cfg_.rank * rank_slot_;
        } else if (cfg_.responder) {
            quiet_needed_ = std::min(cfg_.quiet_ms, cfg_.responder_quiet_ms);
            contention_ms_ = cfg_.responder_dither_ms +
                slot * std::uniform_int_distribution<int>(0, 3)(gen_) +
                std::uniform_int_distribution<int>(0, slot - 1)(gen_);
        } else {
            quiet_needed_ = cfg_.quiet_ms;
            contention_ms_ = draw_contention();
        }
        contention_drawn_ = contention_ms_;
        idle_ms_ = credit(cfg_.idle_credit_ms);
    }

    Verdict step(float level_db, bool capture_alive, bool tx_allowed,
                 int idle_since_ms = 0) {
        if (!capture_alive) {
            idle_ms_ = 0;
            deaf_ms_ += cfg_.poll_ms;
            after_busy_ = false;
            lockout_seen_ = false;
            if (deaf_ms_ >= cfg_.deaf_limit_ms) {
                reason_ = Reason::NO_AUDIO;
                return Verdict::TRANSMIT;
            }
            return Verdict::WAIT;
        }
        deaf_ms_ = 0;
        if (!tx_allowed || (!cfg_.sync_only && level_db > cfg_.threshold_db)) {
            idle_ms_ = 0;
            busy_ms_ += cfg_.poll_ms;
            after_busy_ = true;
            if (!tx_allowed)
                lockout_seen_ = true;
            if (busy_ms_ >= cfg_.busy_limit_ms) {
                reason_ = Reason::BUSY_OVERRIDE;
                return Verdict::TRANSMIT;
            }
            return Verdict::WAIT;
        }
        if (after_busy_) {
            if (cfg_.sync_only && !cfg_.responder) {
                if (rank_slot_ > 0) {
                    contention_ms_ = cfg_.rank * rank_slot_;
                    contention_drawn_ = contention_ms_;
                } else if (cfg_.redraw_on_busy) {
                    contention_ms_ = draw_contention();
                    contention_drawn_ = contention_ms_;
                }
            }
            if (lockout_seen_)
                idle_ms_ = credit(idle_since_ms);
            after_busy_ = false;
            lockout_seen_ = false;
        }
        busy_ms_ = 0;
        idle_ms_ += cfg_.poll_ms;
        if (idle_ms_ < quiet_needed_)
            return Verdict::WAIT;
        if (contention_ms_ > 0) {
            contention_ms_ -= cfg_.poll_ms;
            return Verdict::WAIT;
        }
        reason_ = cfg_.responder ? Reason::RESPONDER : Reason::CLEAR;
        return Verdict::TRANSMIT;
    }

    void set_rank(int rank, int n) {
        cfg_.rank = rank;
        cfg_.rank_n = n;
        if (rank < 0) {
            rank_slot_ = 0;
            return;
        }
        rank_slot_ = RANKED_SLOT_MS;
        window_ = std::max(1, cfg_.rank_n) * rank_slot_;
    }

    Reason reason() const { return reason_; }
    bool quiet_met() const { return idle_ms_ >= quiet_needed_; }
    int window_ms() const { return window_; }
    int quiet_needed_ms() const { return quiet_needed_; }
    int contention_drawn_ms() const { return contention_drawn_; }
    int contention_left_ms() const { return std::max(0, contention_ms_); }
    int idle_ms() const { return idle_ms_; }
    int busy_ms() const { return busy_ms_; }
    int deaf_ms() const { return deaf_ms_; }

private:
    int draw_contention() {
        int slot = std::max(1, cfg_.slot_ms);
        int slots = std::max(2, window_ / slot);
        return cfg_.extra_delay_ms + slot *
            std::uniform_int_distribution<int>(0, slots - 1)(gen_) +
            std::uniform_int_distribution<int>(0, slot - 1)(gen_);
    }

    int credit(int idle_since_ms) const {
        int c = std::max(0, idle_since_ms);
        if (rank_slot_ > 0 || c >= SETTLED_MS)
            return std::min(c, quiet_needed_);
        return std::min(c, std::min(quiet_needed_, cfg_.responder_quiet_ms));
    }

    CsmaConfig cfg_;
    std::mt19937 gen_;
    int window_ = 0;
    int rank_slot_ = 0;
    int quiet_needed_ = 0;
    int contention_ms_ = 0;
    int contention_drawn_ = 0;
    int idle_ms_ = 0;
    int deaf_ms_ = 0;
    int busy_ms_ = 0;
    bool after_busy_ = false;
    bool lockout_seen_ = false;
    Reason reason_ = Reason::NONE;
};
