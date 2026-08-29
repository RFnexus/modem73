#pragma once

#include <cstdint>
#include <string>

struct RxFrameInfo {
    uint64_t seq = 0;
    double time = 0.0;
    int64_t steady_ms = 0;
    float snr = 0.0f;
    float ber_pct = -1.0f;
    float level_db = 0.0f;
    int size = 0;
    bool reassembled = false;
    std::string modem;
    std::string mode;
    int oper_mode = -1;
    std::string modulation;
    std::string code_rate;
    std::string frame_size;
    int robust_mode = -1;
    int mfsk_mode = -1;
    std::string callsign;
    std::string callsign_source = "none";
};
