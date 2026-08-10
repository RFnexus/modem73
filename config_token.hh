#pragma once

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace ConfigToken {

static constexpr int FORMAT_VERSION = 1;
static constexpr int VERSION_BITS = 3;
static constexpr int LENGTH_BITS = 9;
static constexpr int CRC_BITS = 8;
static const char ALPHABET[] = "0123456789ABCDEFGHJKMNPQRSTVWXYZ";

struct Profile {
    int modem_type = 0;
    int mfsk_mode = 1;
    int robust_mode = 0;
    int modulation = 1;
    int code_rate = 0;
    int frame_size = 1;
    int postamble = 0;
    int center_freq = 1500;
    int csma_enabled = 1;
    int csma_mode = 0;
    int csma_band = 0;
    int fast_floor = 1;
    int lead_tone = 1;
    int threshold_db = -30;
    int quiet_ms = 0;
    int cw = 8;
    int slot_ms = 500;
    int burst = 2;
    int dither_ms = 250;
    int p_persistence = 128;
    int fragmentation = 0;
    int tx_blanking = 0;
    int rx_ofdm = 1;
    int rx_robust = 1;
    int rx_mfsk = 1;
    int vox_freq = 1200;
    int vox_lead_ms = 150;
    int vox_tail_ms = 100;
};

struct Field {
    const char* name;
    int bits;
    int (*get)(const Profile&);
    void (*set)(Profile&, int);
};

static const Field SCHEMA[] = {
    {"Modem", 2, [](const Profile& p) { return p.modem_type; },
     [](Profile& p, int v) { p.modem_type = v; }},
    {"MFSK mode", 2, [](const Profile& p) { return p.mfsk_mode; },
     [](Profile& p, int v) { p.mfsk_mode = v; }},
    {"Robust mode", 4, [](const Profile& p) { return p.robust_mode; },
     [](Profile& p, int v) { p.robust_mode = v; }},
    {"Modulation", 3, [](const Profile& p) { return p.modulation; },
     [](Profile& p, int v) { p.modulation = v; }},
    {"Code rate", 3, [](const Profile& p) { return p.code_rate; },
     [](Profile& p, int v) { p.code_rate = v; }},
    {"Frame size", 2, [](const Profile& p) { return p.frame_size; },
     [](Profile& p, int v) { p.frame_size = v; }},
    {"Postamble", 1, [](const Profile& p) { return p.postamble; },
     [](Profile& p, int v) { p.postamble = v; }},
    {"Center freq", 6, [](const Profile& p) { return p.center_freq / 50; },
     [](Profile& p, int v) { p.center_freq = v * 50; }},
    {"CSMA", 1, [](const Profile& p) { return p.csma_enabled; },
     [](Profile& p, int v) { p.csma_enabled = v; }},
    {"CSMA mode", 2, [](const Profile& p) { return p.csma_mode; },
     [](Profile& p, int v) { p.csma_mode = v; }},
    {"Band", 1, [](const Profile& p) { return p.csma_band; },
     [](Profile& p, int v) { p.csma_band = v; }},
    {"Fast floor", 1, [](const Profile& p) { return p.fast_floor; },
     [](Profile& p, int v) { p.fast_floor = v; }},
    {"Lead tone", 1, [](const Profile& p) { return p.lead_tone; },
     [](Profile& p, int v) { p.lead_tone = v; }},
    {"Threshold", 6, [](const Profile& p) { return p.threshold_db + 63; },
     [](Profile& p, int v) { p.threshold_db = v - 63; }},
    {"Quiet", 5, [](const Profile& p) { return p.quiet_ms / 100; },
     [](Profile& p, int v) { p.quiet_ms = v * 100; }},
    {"Window", 4, [](const Profile& p) { return p.cw; },
     [](Profile& p, int v) { p.cw = v; }},
    {"Slot", 5, [](const Profile& p) { return p.slot_ms / 50; },
     [](Profile& p, int v) { p.slot_ms = v * 50; }},
    {"Burst", 3, [](const Profile& p) { return p.burst; },
     [](Profile& p, int v) { p.burst = v; }},
    {"Reply offset", 4, [](const Profile& p) { return p.dither_ms / 50; },
     [](Profile& p, int v) { p.dither_ms = v * 50; }},
    {"Persistence", 8, [](const Profile& p) { return p.p_persistence; },
     [](Profile& p, int v) { p.p_persistence = v; }},
    {"Fragmentation", 1, [](const Profile& p) { return p.fragmentation; },
     [](Profile& p, int v) { p.fragmentation = v; }},
    {"TX blanking", 1, [](const Profile& p) { return p.tx_blanking; },
     [](Profile& p, int v) { p.tx_blanking = v; }},
    {"RX OFDM", 1, [](const Profile& p) { return p.rx_ofdm; },
     [](Profile& p, int v) { p.rx_ofdm = v; }},
    {"RX ROBUST", 1, [](const Profile& p) { return p.rx_robust; },
     [](Profile& p, int v) { p.rx_robust = v; }},
    {"RX MFSK", 1, [](const Profile& p) { return p.rx_mfsk; },
     [](Profile& p, int v) { p.rx_mfsk = v; }},
    {"VOX freq", 6, [](const Profile& p) { return p.vox_freq / 50; },
     [](Profile& p, int v) { p.vox_freq = v * 50; }},
    {"VOX lead", 5, [](const Profile& p) { return p.vox_lead_ms / 10; },
     [](Profile& p, int v) { p.vox_lead_ms = v * 10; }},
    {"VOX tail", 5, [](const Profile& p) { return p.vox_tail_ms / 10; },
     [](Profile& p, int v) { p.vox_tail_ms = v * 10; }},
};

static constexpr int SCHEMA_COUNT = (int)(sizeof(SCHEMA) / sizeof(SCHEMA[0]));

inline uint8_t crc8(const std::vector<uint8_t>& d) {
    uint8_t c = 0xFF;
    for (uint8_t byte : d) {
        c ^= byte;
        for (int b = 0; b < 8; ++b)
            c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1);
    }
    return c;
}

class BitWriter {
public:
    void put(uint32_t v, int w) {
        for (int i = w - 1; i >= 0; --i)
            bits_.push_back((v >> i) & 1);
    }
    int size() const { return (int)bits_.size(); }
    int at(int i) const { return bits_[i]; }
    std::vector<uint8_t> bytes() const {
        std::vector<uint8_t> out((bits_.size() + 7) / 8, 0);
        for (size_t i = 0; i < bits_.size(); ++i)
            if (bits_[i])
                out[i / 8] |= (uint8_t)(0x80 >> (i % 8));
        return out;
    }

private:
    std::vector<uint8_t> bits_;
};

inline int symbol_value(char c) {
    if (c >= 'a' && c <= 'z')
        c = (char)(c - 'a' + 'A');
    if (c == 'I' || c == 'L')
        c = '1';
    if (c == 'O')
        c = '0';
    for (int i = 0; i < 32; ++i)
        if (ALPHABET[i] == c)
            return i;
    return -1;
}

inline std::string encode(const Profile& p) {
    BitWriter payload;
    for (int i = 0; i < SCHEMA_COUNT; ++i) {
        int maxv = (1 << SCHEMA[i].bits) - 1;
        payload.put((uint32_t)std::clamp(SCHEMA[i].get(p), 0, maxv), SCHEMA[i].bits);
    }
    BitWriter all;
    all.put(FORMAT_VERSION, VERSION_BITS);
    all.put((uint32_t)payload.size(), LENGTH_BITS);
    all.put(crc8(payload.bytes()), CRC_BITS);
    for (int i = 0; i < payload.size(); ++i)
        all.put((uint32_t)payload.at(i), 1);

    std::string out;
    int total = all.size();
    for (int i = 0; i < total; i += 5) {
        int sym = 0;
        for (int b = 0; b < 5; ++b)
            sym = (sym << 1) | (i + b < total ? all.at(i + b) : 0);
        if (!out.empty() && (out.size() + 1) % 5 == 0)
            out += '-';
        out += ALPHABET[sym];
    }
    return "M73-" + out;
}

struct Result {
    int applied = 0;
    int extra_bits = 0;
};

inline bool decode(const std::string& text, Profile* io, std::string* err,
                   Result* res = nullptr) {
    std::string s;
    for (char c : text) {
        if (c == '-' || c == ' ' || c == '\t' || c == '\r' || c == '\n')
            continue;
        s += c;
    }
    if (s.size() >= 3) {
        std::string head = s.substr(0, 3);
        for (char& c : head)
            c = (char)toupper((unsigned char)c);
        if (head == "M73")
            s = s.substr(3);
    }
    if (s.empty()) {
        *err = "no token found";
        return false;
    }
    std::vector<int> bits;
    for (char c : s) {
        int sym = symbol_value(c);
        if (sym < 0) {
            *err = std::string("bad character '") + c + "' in token";
            return false;
        }
        for (int b = 4; b >= 0; --b)
            bits.push_back((sym >> b) & 1);
    }
    int header = VERSION_BITS + LENGTH_BITS + CRC_BITS;
    if ((int)bits.size() < header) {
        *err = "token is too short";
        return false;
    }
    int pos = 0;
    auto take = [&](int w) {
        int v = 0;
        for (int i = 0; i < w; ++i)
            v = (v << 1) | bits[pos++];
        return v;
    };
    int ver = take(VERSION_BITS);
    if (ver != FORMAT_VERSION) {
        *err = "token format v" + std::to_string(ver) + ", this build reads v" +
               std::to_string(FORMAT_VERSION);
        return false;
    }
    int payload_bits = take(LENGTH_BITS);
    int want = take(CRC_BITS);
    if (payload_bits <= 0 || pos + payload_bits > (int)bits.size()) {
        *err = "token is truncated";
        return false;
    }
    BitWriter payload;
    for (int i = 0; i < payload_bits; ++i)
        payload.put((uint32_t)bits[pos + i], 1);
    if (crc8(payload.bytes()) != want) {
        *err = "checksum failed, check for a typo";
        return false;
    }
    for (int i = pos + payload_bits; i < (int)bits.size(); ++i) {
        if (bits[i]) {
            *err = "checksum failed, check for a typo";
            return false;
        }
    }

    int p = 0, n = 0;
    for (int i = 0; i < SCHEMA_COUNT && p + SCHEMA[i].bits <= payload_bits; ++i) {
        int v = 0;
        for (int b = 0; b < SCHEMA[i].bits; ++b)
            v = (v << 1) | payload.at(p + b);
        p += SCHEMA[i].bits;
        SCHEMA[i].set(*io, v);
        n++;
    }
    if (res) {
        res->applied = n;
        res->extra_bits = payload_bits - p;
    }
    return true;
}

}
