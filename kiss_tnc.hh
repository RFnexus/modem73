#pragma once

#include <cstdint>
#include <vector>
#include <queue>
#include <map>
#include <mutex>
#include <atomic>
#include <chrono>
#include <functional>
#include <string>
#include <sstream>
#include <iomanip>
#include <iostream>

// KISS protocol
namespace KISS {
    constexpr uint8_t FEND  = 0xC0;
    constexpr uint8_t FESC  = 0xDB;
    constexpr uint8_t TFEND = 0xDC;
    constexpr uint8_t TFESC = 0xDD;
    
    // KISS commands
    constexpr uint8_t CMD_DATA     = 0x00;
    constexpr uint8_t CMD_TXDELAY  = 0x01;
    constexpr uint8_t CMD_P        = 0x02;
    constexpr uint8_t CMD_SLOTTIME = 0x03;
    constexpr uint8_t CMD_TXTAIL   = 0x04;
    constexpr uint8_t CMD_FULLDUPLEX = 0x05;
    constexpr uint8_t CMD_SETHW    = 0x06;
    constexpr uint8_t CMD_RETURN   = 0xFF;
}


enum class PTTType {
    NONE = 0,
    RIGCTL = 1,
    VOX = 2,
    COM = 3
};

struct TNCConfig {
    // Network settings
    std::string bind_address = "0.0.0.0";
    int port = 8001;
    
    // Audio settings
    std::string audio_input_device = "default";
    std::string audio_output_device = "default";
    int sample_rate = 48000;
    
    // Modem settings
    int center_freq = 1500;
    std::string callsign = "N0CALL";
    std::string modulation = "QPSK";
    std::string code_rate = "1/2";
    bool short_frame = false;  
    
    // PTT settings
    PTTType ptt_type = PTTType::RIGCTL;  
    
    // Rigctl settings 
    std::string rigctl_host = "localhost";
    int rigctl_port = 4532;
    
    // VOX settings 
    int vox_tone_freq = 1200;    // Hz - tone frequency for VOX trigger
    int vox_lead_ms = 550;       // ms - tone before OFDM data
    int vox_tail_ms = 500;       // ms - tone after OFDM data
    
    // COM/Serial PTT settings 
    std::string com_port = "/dev/ttyUSB0"; 
    int com_ptt_line = 1;        // 0=DTR, 1=RTS, 2=BOTH
    bool com_invert_dtr = false;
    bool com_invert_rts = false;
    
    // PTT timing 
    int ptt_delay_ms = 50;       // Delay after PTT before TX
    int ptt_tail_ms = 50;        // Delay after TX before PTT release
    
    // Operational settings
    int tx_delay_ms = 500;       // TXDelay 
    bool full_duplex = false;
    int slot_time_ms = 500;      // CSMA slot time
    int p_persistence = 128;     // 0-255 (128 defualt 50%)
    
    // CSMA settings
    bool csma_enabled = true;
    float carrier_threshold_db = -30.0f;
    int carrier_sense_ms = 100;
    int max_backoff_slots = 10;
    
    // Fragmentation settings
    bool fragmentation_enabled = false;
    
    // Settings file path
    std::string config_file = "";
};


class KISSParser {
public:
    using FrameCallback = std::function<void(uint8_t port, uint8_t cmd, const std::vector<uint8_t>&)>;
    
    KISSParser(FrameCallback callback) : callback_(callback) {}
    
    void process(const uint8_t* data, size_t len) {
        for (size_t i = 0; i < len; ++i) {
            process_byte(data[i]);
        }
    }
    
    static std::vector<uint8_t> wrap(const std::vector<uint8_t>& data, uint8_t port = 0) {
        std::vector<uint8_t> frame;
        frame.push_back(KISS::FEND);
        frame.push_back((port << 4) | KISS::CMD_DATA);
        
        for (uint8_t byte : data) {
            if (byte == KISS::FEND) {
                frame.push_back(KISS::FESC);
                frame.push_back(KISS::TFEND);
            } else if (byte == KISS::FESC) {
                frame.push_back(KISS::FESC);
                frame.push_back(KISS::TFESC);
            } else {
                frame.push_back(byte);
            }
        }
        
        frame.push_back(KISS::FEND);
        return frame;
    }
    
private:
    void process_byte(uint8_t byte) {
        if (byte == KISS::FEND) {
            if (in_frame_ && buffer_.size() > 0) {
                uint8_t cmd_byte = buffer_[0];
                uint8_t port = (cmd_byte >> 4) & 0x0F;
                uint8_t cmd = cmd_byte & 0x0F;
                std::vector<uint8_t> payload(buffer_.begin() + 1, buffer_.end());
                callback_(port, cmd, payload);
            }
            in_frame_ = true;
            buffer_.clear();
            escape_ = false;
        } else if (in_frame_) {
            if (escape_) {
                if (byte == KISS::TFEND) {
                    buffer_.push_back(KISS::FEND);
                } else if (byte == KISS::TFESC) {
                    buffer_.push_back(KISS::FESC);
                } else {
                    buffer_.push_back(byte);
                }
                escape_ = false;
            } else if (byte == KISS::FESC) {
                escape_ = true;
            } else {
                buffer_.push_back(byte);
            }
        }
    }
    
    FrameCallback callback_;
    std::vector<uint8_t> buffer_;
    bool in_frame_ = false;
    bool escape_ = false;
};


template<typename T>
class PacketQueue {
public:
    void push(T item) {
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.push(std::move(item));
    }
    
    bool pop(T& item) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.empty()) return false;
        item = std::move(queue_.front());
        queue_.pop();
        return true;
    }
    
    bool empty() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.empty();
    }
    
    size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.size();
    }
    
    void clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        while (!queue_.empty()) queue_.pop();
    }
    
private:
    mutable std::mutex mutex_;
    std::queue<T> queue_;
};


inline void hex_dump(const char* prefix, const uint8_t* data, size_t len) {
    std::cerr << prefix << " (" << len << " bytes):" << std::endl;
    for (size_t i = 0; i < len; i += 16) {
        std::cerr << "  " << std::hex << std::setfill('0') << std::setw(4) << i << ": ";
        
        for (size_t j = 0; j < 16 && i + j < len; ++j) {
            std::cerr << std::setw(2) << (int)data[i + j] << " ";
        }
        for (size_t j = len - i; j < 16 && i + j >= len; ++j) {
            std::cerr << "   ";
        }
        
        std::cerr << " |";
        for (size_t j = 0; j < 16 && i + j < len; ++j) {
            char c = data[i + j];
            std::cerr << (c >= 32 && c < 127 ? c : '.');
        }
        std::cerr << "|" << std::endl;
    }
    std::cerr << std::dec;
}

inline std::string packet_visualize(const uint8_t* data, size_t len, bool is_tx, bool frag_enabled) {
    std::ostringstream oss;
    
    if (len == 0) {
        oss << "  [EMPTY PACKET]";
        return oss.str();
    }
    
    oss << "\n  ┌─────────────────────────────────────────────────────────────┐\n";
    oss << "  │ " << (is_tx ? "TX" : "RX") << " PACKET: " << len << " bytes";
    oss << std::string(47 - std::to_string(len).length(), ' ') << "│\n";
    oss << "  ├─────────────────────────────────────────────────────────────┤\n";
    
    size_t offset = 0;
    
    // Check for fragment by magic byte
    if (frag_enabled && len >= 5 && data[0] == 0xF3) {
        uint16_t pkt_id = (data[1] << 8) | data[2];
        uint8_t seq = data[3];
        uint8_t flags = data[4];
        
        oss << "  │ FRAG HDR [5 bytes]  Magic: 0xF3                             │\n";
        oss << "  │   Packet ID: " << std::setw(5) << pkt_id;
        oss << "   Seq: " << std::setw(3) << (int)seq;
        oss << "   Flags: ";
        
        std::string flag_str;
        if (flags & 0x02) flag_str += "FIRST ";
        if (flags & 0x01) flag_str += "MORE";
        if (flag_str.empty()) flag_str = "LAST";
        oss << std::left << std::setw(12) << flag_str << std::right << "   │\n";
        
        offset = 5;
        oss << "  ├─────────────────────────────────────────────────────────────┤\n";
    }
    
    size_t payload_len = len - offset;
    oss << "  │ PAYLOAD [" << payload_len << " bytes]";
    oss << std::string(49 - std::to_string(payload_len).length(), ' ') << "│\n";
    
    size_t preview_len = std::min(payload_len, (size_t)32);
    if (preview_len > 0) {
        oss << "  │   ";
        for (size_t i = 0; i < preview_len; i++) {
            oss << std::hex << std::setfill('0') << std::setw(2) << (int)data[offset + i];
            if (i < preview_len - 1) oss << " ";
        }
        if (payload_len > 32) oss << "...";
        size_t used = preview_len * 3 - 1 + (payload_len > 32 ? 3 : 0);
        if (used < 57) oss << std::string(57 - used, ' ');
        oss << std::dec << " │\n";
        
        oss << "  │   ";
        for (size_t i = 0; i < preview_len; i++) {
            char c = data[offset + i];
            oss << (c >= 32 && c < 127 ? c : '.');
        }
        if (payload_len > 32) oss << "...";
        size_t ascii_used = preview_len + (payload_len > 32 ? 3 : 0);
        if (ascii_used < 57) oss << std::string(57 - ascii_used, ' ');
        oss << " │\n";
    }
    
    oss << "  └─────────────────────────────────────────────────────────────┘";
    
    return oss.str();
}

inline std::string kiss_frame_visualize(const uint8_t* data, size_t len) {
    std::ostringstream oss;
    
    if (len == 0) {
        oss << "  [EMPTY KISS FRAME]";
        return oss.str();
    }
    
    oss << "\n  ┌─────────────────────────────────────────────────────────────┐\n";
    oss << "  │ KISS FRAME: " << len << " bytes";
    oss << std::string(45 - std::to_string(len).length(), ' ') << "│\n";
    oss << "  ├─────────────────────────────────────────────────────────────┤\n";
    
    if (len >= 1) {
        uint8_t cmd_byte = data[0];
        uint8_t port = (cmd_byte >> 4) & 0x0F;
        uint8_t cmd = cmd_byte & 0x0F;
        
        oss << "  │ CMD BYTE: 0x" << std::hex << std::setfill('0') << std::setw(2) << (int)cmd_byte << std::dec;
        oss << "  Port: " << (int)port << "  Cmd: ";
        
        std::string cmd_name;
        switch (cmd) {
            case 0x00: cmd_name = "DATA"; break;
            case 0x01: cmd_name = "TXDELAY"; break;
            case 0x02: cmd_name = "P"; break;
            case 0x03: cmd_name = "SLOTTIME"; break;
            case 0x04: cmd_name = "TXTAIL"; break;
            case 0x05: cmd_name = "FULLDUPLEX"; break;
            case 0x06: cmd_name = "SETHW"; break;
            case 0x0F: cmd_name = "RETURN"; break;
            default: cmd_name = "UNKNOWN"; break;
        }
        oss << std::left << std::setw(10) << cmd_name << std::right;
        oss << "              │\n";
    }
    
    if (len > 1) {
        oss << "  ├─────────────────────────────────────────────────────────────┤\n";
        size_t payload_len = len - 1;
        oss << "  │ PAYLOAD [" << payload_len << " bytes]";
        oss << std::string(49 - std::to_string(payload_len).length(), ' ') << "│\n";
        
        size_t preview_len = std::min(payload_len, (size_t)24);
        oss << "  │   ";
        for (size_t i = 0; i < preview_len; i++) {
            oss << std::hex << std::setfill('0') << std::setw(2) << (int)data[1 + i];
            if (i < preview_len - 1) oss << " ";
        }
        if (payload_len > 24) oss << " ...";
        oss << std::dec;
        size_t used = preview_len * 3 - 1 + (payload_len > 24 ? 4 : 0);
        if (used < 57) oss << std::string(57 - used, ' ');
        oss << " │\n";
    }
    
    oss << "  └─────────────────────────────────────────────────────────────┘";
    
    return oss.str();
}

// Length-prefix framing 
// This handles OFDM frame padding where the payload is encoded within the 2 byte prefix
inline std::vector<uint8_t> frame_with_length(const std::vector<uint8_t>& data) {
    std::vector<uint8_t> framed;
    uint16_t len = data.size();
    framed.push_back((len >> 8) & 0xFF);  // high byte
    framed.push_back(len & 0xFF);          // low byte
    framed.insert(framed.end(), data.begin(), data.end());
    return framed;
}

inline std::vector<uint8_t> unframe_length(const uint8_t* data, size_t total_len) {
    if (total_len < 2) return {};
    uint16_t payload_len = (data[0] << 8) | data[1];
    if (payload_len > total_len - 2) {
        std::cerr << "Warning: length prefix " << payload_len 
                  << " exceeds available data " << (total_len - 2) << std::endl;
        payload_len = total_len - 2;
    }
    return std::vector<uint8_t>(data + 2, data + 2 + payload_len);
}

namespace Frag {
    constexpr uint8_t MAGIC = 0xF3;
    constexpr size_t HEADER_SIZE = 5;
    constexpr uint8_t FLAG_MORE_FRAGMENTS = 0x01;
    constexpr uint8_t FLAG_FIRST_FRAGMENT = 0x02;
    constexpr int REASSEMBLY_TIMEOUT_MS = 30000;
    constexpr size_t MAX_PENDING_PACKETS = 64;
}

class Fragmenter {
public:
    Fragmenter() : next_packet_id_(0) {}
    
    std::vector<std::vector<uint8_t>> fragment(const std::vector<uint8_t>& data, size_t max_payload) {
        std::vector<std::vector<uint8_t>> fragments;
        
        if (max_payload <= Frag::HEADER_SIZE) {
            return fragments;
        }
        
        size_t data_per_frag = max_payload - Frag::HEADER_SIZE;
        size_t num_frags = (data.size() + data_per_frag - 1) / data_per_frag;
        if (num_frags > 255) {
            num_frags = 255;
        }
        
        uint16_t packet_id = next_packet_id_++;
        
        for (size_t i = 0; i < num_frags; i++) {
            size_t offset = i * data_per_frag;
            size_t chunk_size = std::min(data_per_frag, data.size() - offset);
            
            std::vector<uint8_t> frag;
            frag.reserve(Frag::HEADER_SIZE + chunk_size);
            
            frag.push_back(Frag::MAGIC);
            frag.push_back((packet_id >> 8) & 0xFF);
            frag.push_back(packet_id & 0xFF);
            frag.push_back(static_cast<uint8_t>(i));
            
            uint8_t flags = 0;
            if (i == 0) flags |= Frag::FLAG_FIRST_FRAGMENT;
            if (i < num_frags - 1) flags |= Frag::FLAG_MORE_FRAGMENTS;
            frag.push_back(flags);
            
            frag.insert(frag.end(), data.begin() + offset, data.begin() + offset + chunk_size);
            fragments.push_back(std::move(frag));
        }
        
        return fragments;
    }
    
    bool needs_fragmentation(size_t data_size, size_t max_payload) const {
        return data_size > (max_payload - Frag::HEADER_SIZE);
    }
    
private:
    std::atomic<uint16_t> next_packet_id_;
};

class Reassembler {
public:
    Reassembler() = default;
    
    std::vector<uint8_t> process(const std::vector<uint8_t>& fragment) {
        if (fragment.size() < Frag::HEADER_SIZE) {
            return {};
        }
        
        if (fragment[0] != Frag::MAGIC) {
            return {};
        }
        
        uint16_t packet_id = (fragment[1] << 8) | fragment[2];
        uint8_t seq = fragment[3];
        uint8_t flags = fragment[4];
        
        std::vector<uint8_t> payload(fragment.begin() + Frag::HEADER_SIZE, fragment.end());
        
        std::lock_guard<std::mutex> lock(mutex_);
        
        cleanup_stale();
        
        auto& pkt = pending_[packet_id];
        if (pkt.fragments.empty()) {
            pkt.first_seen = std::chrono::steady_clock::now();
        }
        
        pkt.fragments[seq] = std::move(payload);
        
        if (flags & Frag::FLAG_FIRST_FRAGMENT) {
            pkt.has_first = true;
        }
        
        if (!(flags & Frag::FLAG_MORE_FRAGMENTS)) {
            pkt.last_seq = seq;
            pkt.has_last = true;
        }
        
        if (pkt.has_first && pkt.has_last) {
            bool complete = true;
            for (uint8_t i = 0; i <= pkt.last_seq; i++) {
                if (pkt.fragments.find(i) == pkt.fragments.end()) {
                    complete = false;
                    break;
                }
            }
            
            if (complete) {
                std::vector<uint8_t> reassembled;
                for (uint8_t i = 0; i <= pkt.last_seq; i++) {
                    auto& frag_data = pkt.fragments[i];
                    reassembled.insert(reassembled.end(), frag_data.begin(), frag_data.end());
                }
                pending_.erase(packet_id);
                return reassembled;
            }
        }
        
        return {};
    }
    
    bool is_fragment(const std::vector<uint8_t>& data) const {
        if (data.size() < Frag::HEADER_SIZE) return false;
        return data[0] == Frag::MAGIC;
    }
    
    void reset() {
        std::lock_guard<std::mutex> lock(mutex_);
        pending_.clear();
    }
    
private:
    struct PendingPacket {
        std::map<uint8_t, std::vector<uint8_t>> fragments;
        std::chrono::steady_clock::time_point first_seen;
        uint8_t last_seq = 0;
        bool has_first = false;
        bool has_last = false;
    };
    
    void cleanup_stale() {
        auto now = std::chrono::steady_clock::now();
        for (auto it = pending_.begin(); it != pending_.end();) {
            auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - it->second.first_seen).count();
            if (age > Frag::REASSEMBLY_TIMEOUT_MS) {
                it = pending_.erase(it);
            } else {
                ++it;
            }
        }
        
        while (pending_.size() > Frag::MAX_PENDING_PACKETS) {
            auto oldest = pending_.begin();
            for (auto it = pending_.begin(); it != pending_.end(); ++it) {
                if (it->second.first_seen < oldest->second.first_seen) {
                    oldest = it;
                }
            }
            pending_.erase(oldest);
        }
    }
    
    std::map<uint16_t, PendingPacket> pending_;
    mutable std::mutex mutex_;
};
