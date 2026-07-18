#pragma once

#include <algorithm>
#include <cstdint>
#include <random>

struct CsmaConfig {
    float threshold_db = -30.0f;
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
};

class CsmaGate {
public:
    enum class Verdict { WAIT, TRANSMIT };
    enum class Reason { NONE, CLEAR, RESPONDER, BUSY_OVERRIDE, NO_AUDIO };

    CsmaGate(const CsmaConfig& cfg, uint32_t seed) : cfg_(cfg) {
        int slot = std::max(1, cfg_.slot_ms);
        int window = std::max(2, cfg_.cw) * slot;
        if (cfg_.idle_credit_ms >= cfg_.cold_channel_ms)
            window = std::max(window / 4, 4 * slot);
        window_ = window;
        std::mt19937 gen(seed);
        if (cfg_.responder) {
            quiet_needed_ = std::min(cfg_.quiet_ms, cfg_.responder_quiet_ms);
            contention_ms_ = cfg_.responder_dither_ms +
                slot * std::uniform_int_distribution<int>(0, 3)(gen);
        } else {
            int slots = std::max(2, window / slot);
            quiet_needed_ = cfg_.quiet_ms;
            contention_ms_ = slot *
                std::uniform_int_distribution<int>(0, slots - 1)(gen);
        }
        contention_drawn_ = contention_ms_;
        idle_ms_ = std::min(std::max(0, cfg_.idle_credit_ms), quiet_needed_);
    }

    Verdict step(float level_db, bool capture_alive, bool tx_allowed) {
        if (!capture_alive) {
            idle_ms_ = 0;
            deaf_ms_ += cfg_.poll_ms;
            if (deaf_ms_ >= cfg_.deaf_limit_ms) {
                reason_ = Reason::NO_AUDIO;
                return Verdict::TRANSMIT;
            }
            return Verdict::WAIT;
        }
        deaf_ms_ = 0;
        if (!tx_allowed || level_db > cfg_.threshold_db) {
            idle_ms_ = 0;
            busy_ms_ += cfg_.poll_ms;
            if (busy_ms_ >= cfg_.busy_limit_ms) {
                reason_ = Reason::BUSY_OVERRIDE;
                return Verdict::TRANSMIT;
            }
            return Verdict::WAIT;
        }
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
    CsmaConfig cfg_;
    int window_ = 0;
    int quiet_needed_ = 0;
    int contention_ms_ = 0;
    int contention_drawn_ = 0;
    int idle_ms_ = 0;
    int deaf_ms_ = 0;
    int busy_ms_ = 0;
    Reason reason_ = Reason::NONE;
};
