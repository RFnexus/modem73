#pragma once

#include <cstring>
#include <string>

struct ProbeStep {
    const char* label;
    const char* family;
    float min_snr_db;
};

static const ProbeStep PROBE_LADDER[] = {
    {"QAM4096 5/6", "ofdm", 35.0f},
    {"QAM1024 3/4", "ofdm", 29.0f},
    {"QAM256 3/4", "ofdm", 24.0f},
    {"QAM64 3/4", "ofdm", 19.0f},
    {"QAM64 1/2", "ofdm", 16.0f},
    {"QAM16 3/4", "ofdm", 14.0f},
    {"QAM16 1/2", "ofdm", 11.0f},
    {"8PSK 1/2", "ofdm", 9.0f},
    {"QPSK 3/4", "ofdm", 8.0f},
    {"QPSK 1/2", "ofdm", 5.0f},
    {"BPSK 1/2", "ofdm", 2.0f},
    {"QPSK 1/4x2", "ofdm", -1.0f},
    {"RDM-1200", "robust", 5.0f},
    {"RDM-600", "robust", 1.0f},
    {"RDM-300", "robust", -2.0f},
    {"MFSK-32R", "mfsk", -7.0f},
    {"MFSK-8", "mfsk", -9.0f},
};

constexpr float PROBE_MARGIN_DB = 3.0f;

inline int probe_family_rank(const char* family) {
    if (!std::strcmp(family, "ofdm")) return 0;
    if (!std::strcmp(family, "robust")) return 1;
    return 2;
}

inline std::string probe_suggest_mode(const std::string& family, float snr_db) {
    int rank = probe_family_rank(family.c_str());
    float usable = snr_db - PROBE_MARGIN_DB;
    const char* slowest = nullptr;
    for (const auto& s : PROBE_LADDER) {
        if (probe_family_rank(s.family) != rank) continue;
        if (s.min_snr_db <= usable) return s.label;
        slowest = s.label;
    }
    if (!slowest) return "(no suggestion)";
    const char* next = rank == 0 ? ", try ROBUST" : rank == 1 ? ", try MFSK" : "";
    return std::string(slowest) + " (marginal" + next + ")";
}
