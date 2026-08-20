#pragma once

#ifndef MODEM73_VERSION
#define MODEM73_VERSION "dev"
#endif

#include <string>
#include <vector>
#include <deque>
#include <mutex>
#include <atomic>
#include <thread>
#include <chrono>
#include <functional>
#include <sstream>
#include <iomanip>
#include <cstring>
#include <cctype>
#include "config_token.hh"
#include <cmath>
#include <complex>
#include <array>
#include <cstdio>
#include <cstdlib>
#include <algorithm>
#include <random>
#include <unistd.h>
#include <fcntl.h>

#include "kiss_tnc.hh"
#include "csma.hh"
#include "phy/mfsk_modem.hh"
#include "phy/robust_modem.hh"
#include "perf_log.hh"
#ifdef WITH_CM108
#include "cm108_ptt.hh"
#endif

constexpr size_t MAX_LOG_ENTRIES = 500;
constexpr int RANKED_QUIET_DISPLAY_MS = 1000;

const std::vector<std::string> MODEM_TYPE_OPTIONS = {"OFDM", "MFSK", "ROBUST"};
const std::vector<std::string> ROBUST_MODE_OPTIONS = {"RDM-1200", "RDM-800", "RDM-600", "RDM-300", "RDMN-300", "RDMN-150", "RDM-QB"};
const std::vector<std::string> ROBUST_MTU_OPTIONS = {"510 B", "170 B (short)", "30 B (micro)"};

// robust_mode ints: 0-4 full-frame, 5-9 short-frame, 10/11 RDM-800/-800S,
// 12 RDM-QB (micro burst, single size).
// The UI shows a base-mode selector (index into ROBUST_MODE_OPTIONS above,
// display order) plus a frame-size toggle; these map between the two.
inline int robust_base_index(int mode) {
    if (mode == 12) return 6;                       // RDM-QB micro
    if (mode >= 10) return 1;                       // RDM-800 family
    int fam = mode % 5;                             // 0-4 family order
    return fam == 0 ? 0 : fam + 1;                  // shifted past RDM-800
}
inline int robust_mode_of(int base, bool short_frame) {
    if (base == 6) return 12;                       // RDM-QB is one size only
    if (base == 1) return short_frame ? 11 : 10;
    int fam = base == 0 ? 0 : base - 1;
    return fam + (short_frame ? 5 : 0);
}
inline int robust_mtu_index(int mode) {
    return mode == 12 ? 2 : RobustParams::is_short((RobustMode)mode) ? 1 : 0;
}

struct AltMode {
    const char* label;
    int modem_type;
    int modulation;
    int code_rate;
    int frame_size;
    int robust_mode;
    int mfsk_mode;
};
static const AltMode ALT_MODES[] = {
    {"BPSK 1/2 N",   0, 0, 0, 1, 0, 1},
    {"QPSK 1/2 N",   0, 1, 0, 1, 0, 1},
    {"QPSK 1/2x2 N", 0, 1, 5, 1, 0, 1},
    {"QPSK 1/4x2 N", 0, 1, 6, 1, 0, 1},
    {"QAM16 3/4 N",  0, 3, 2, 1, 0, 1},
    {"RDM-1200",     2, 1, 0, 1, 0, 1},
    {"RDM-600",      2, 1, 0, 1, 1, 1},
    {"RDM-300",      2, 1, 0, 1, 2, 1},
    {"RDMN-300",     2, 1, 0, 1, 3, 1},
    {"MFSK-16",      1, 1, 0, 1, 0, 1},
    {"RDM-1200S",    2, 1, 0, 1, 5, 1},
    {"RDMN-300S",    2, 1, 0, 1, 8, 1},
    {"RDM-800",      2, 1, 0, 1, 10, 1},
};
constexpr int ALT_MODE_COUNT = 13;
const std::vector<std::string> MFSK_MODE_OPTIONS = {"MFSK-8", "MFSK-16", "MFSK-32", "MFSK-32R"};

const std::vector<std::string> MODULATION_OPTIONS = {
    "BPSK", "QPSK", "8PSK", "QAM16", "QAM64", "QAM256", "QAM1024", "QAM4096"
};

const std::vector<std::string> CODE_RATE_OPTIONS = {
    "1/2", "2/3", "3/4", "5/6", "1/4", "1/2x2", "1/4x2"
};

const std::vector<std::string> RIG_MODE_OPTIONS = {
    "USB", "LSB", "CW", "CWR", "RTTY", "AM", "FM", "PKTUSB", "PKTLSB"
};

constexpr int RIG_STEP_COUNT = 7;
const long long RIG_STEP_HZ[RIG_STEP_COUNT] = {10, 100, 1000, 5000, 10000, 100000, 1000000};
const char* const RIG_STEP_LABELS[RIG_STEP_COUNT] = {
    "10 Hz", "100 Hz", "1 kHz", "5 kHz", "10 kHz", "100 kHz", "1 MHz"
};

struct RigMeterDef {
    const char* label;
    const char* level;
    float min;
    float max;
};
constexpr int RIG_METER_COUNT = 5;
const RigMeterDef RIG_METERS[RIG_METER_COUNT] = {
    {"S-Meter", "STRENGTH",            -54.0f, 60.0f},
    {"SWR",     "SWR",                   1.0f,  5.0f},
    {"Power",   "RFPOWER_METER_WATTS",   0.0f, 100.0f},
    {"ALC",     "ALC",                   0.0f,  1.0f},
    {"Temp",    "TEMP_METER",            0.0f, 100.0f},
};
constexpr int RIG_METER_SWR = 1;
constexpr float SWR_WARN_THRESHOLD = 2.5f;

constexpr int UTILS_ACTION_COUNT = 10 + ALT_MODE_COUNT;



extern std::atomic<bool> g_running;


struct TNCUIState {
    std::string callsign = "N0CALL";
    int modem_type_index = 0;
    int mfsk_mode_index = 1;   // 0=MFSK-8, 1=MFSK-16, 2=MFSK-32, 3=MFSK-32R
    int robust_mode_index = 0;
    bool utils_testing_open = false;
    PerfLogger* perf_logger = nullptr;
    int alt_mode_mask = 0;
    int modulation_index = 1;  // default QPSK N 1/2
    int code_rate_index = 0;
    int frame_size = 1;        // 0=short, 1=normal, 2=long, 3=micro qpsk 1/2 only curent

    // TODO
    bool micro_allowed() const {
        return modulation_index == 1 && code_rate_index == 0;
    }
    void clamp_micro() {
        if (frame_size == 3 && !micro_allowed())
            frame_size = 1;
    }
    int center_freq = 1500;
    bool postamble = false;

    bool ofdm_rx_enabled = true;
    bool robust_rx_enabled = true;
    bool mfsk_rx_enabled = true;

    bool csma_enabled = true;
    bool csma_sync_only = false;
    bool csma_fast_floor = true;
    bool csma_ranked = false;
    int beacon_interval_s = 45;
    float carrier_threshold_db = -30.0f;
    int slot_time_ms = 500;
    int csma_quiet_ms = 0;
    int csma_cw = 8;
    int p_persistence = 128;
    bool tx_lead_tone = true;
    int csma_responder_dither = 250;
    int csma_burst = 2;
    int csma_band = 0;
    bool csma_advanced_open = false;
    
    // Audio settings 
    std::string audio_input_device = "default";
    std::string audio_output_device = "default";
    std::vector<std::string> available_input_devices;
    std::vector<std::string> input_device_descriptions;
    std::vector<std::string> available_output_devices;
    std::vector<std::string> output_device_descriptions;
    int audio_input_index = 0;
    int audio_output_index = 0;
    
    // Network
    int port = 8001;
    int control_port = 8073;
    std::string bind_address = "0.0.0.0";
    std::string control_bind_address = "127.0.0.1";

    // PTT 
    int ptt_type_index = 1;  // 0=NONE, 1=RIGCTL, 2=VOX
    
    // Rigctl settings (PTT type 1)
    std::string rigctl_host = "localhost";
    int rigctl_port = 4532;
    std::atomic<bool> rigctl_connected{false};
    std::atomic<bool> audio_connected{true};  // Track audio device health

    std::function<std::string(const std::string&)> on_rigctl_command;
    std::function<float()> on_alc_tune;
    std::atomic<bool> rig_poll_enabled{false};
    std::atomic<bool> rig_refresh_requested{false};
    std::atomic<long long> rig_freq_hz{0};
    std::atomic<float> rig_power_level{-1.0f};
    std::atomic<int> rig_tuner_on{-1};
    std::atomic<int> rig_tuner_supported{-1};
    std::atomic<bool> rig_data_valid{false};
    std::atomic<int64_t> rig_last_update_ms{0};
    int64_t rig_last_poll_ms = 0;
    std::array<std::atomic<float>, RIG_METER_COUNT> rig_meter_values;
    // worst SWR of the last TX burst is latched here when it crosses
    // SWR_WARN_THRESHOLD; a later burst that stays below clears it
    std::atomic<float> swr_warn_value{0.0f};
    float swr_burst_max = 0.0f;   // rig poll thread only
    bool swr_prev_ptt = false;    // rig poll thread only
    std::atomic<float> tx_drive{1.0f};
    std::atomic<bool> alc_tune_running{false};
    std::atomic<float> channel_occupancy{0.0f};
    std::atomic<bool> dcd_active{false};
    // 0 idle, 1 deferring on RX lockout, 2 waiting quiet, 3 contending
    std::atomic<int> csma_phase{0};
    std::atomic<int> csma_wait_ms{0};
    std::atomic<int> csma_wait_need{0};
    std::atomic<int> csma_rank{-1};
    std::atomic<int> csma_rank_n{0};
    std::atomic<int> csma_window_ms{0};
    std::mutex rig_mode_mutex;
    std::string rig_mode;
    
    // VOX settings (PTT type 2)
    int vox_tone_freq = 1200;   // Hz
    int vox_lead_ms = 150;      // ms
    int vox_tail_ms = 100;      // ms
    
    // COM/Serial PTT settings (PTT type 3)
    std::string com_port = "/dev/ttyUSB0";
    int com_ptt_line = 1;       // 0=DTR, 1=RTS, 2=BOTH
    bool com_invert_dtr = false;
    bool com_invert_rts = false;
    
#ifdef WITH_CM108
    // CM108 PTT settings (PTT type 4)
    int cm108_gpio = 3;  // GPIO pin to use for PTT, default 3
    std::string cm108_device;  // empty = first compatible device, else serial or USB path
#endif

    int mtu_bytes = 0;
    int bitrate_bps = 0;
    float airtime_seconds = 0.0f;
    int random_data_size = 0;
    bool fragmentation_enabled = false;
    bool tx_blanking_enabled = true;
    int tx_delay_ms = 500;
    
    // stats
    std::atomic<float> total_tx_time{0.0f};  
    

    std::string config_file;
    std::string presets_file;
    
    // Presets 
    struct Preset {
        std::string name;
        // Modem type
        int modem_type_index = 0;
        int mfsk_mode_index = 1;   // 0=MFSK-8, 1=MFSK-16, 2=MFSK-32, 3=MFSK-32R
        int robust_mode_index = 0;
        bool postamble = false;
        // OFDM modem
        int modulation_index;
        int code_rate_index;
        int frame_size;        // 0=short, 1=normal, 2=long, 3=micro (QPSK 1/2 only)
        int center_freq;
        // CSMA
        bool csma_enabled;
        float carrier_threshold_db;
        int slot_time_ms;
        int p_persistence;
        // PTT
        int ptt_type_index;
        int vox_tone_freq;
        int vox_lead_ms;
        int vox_tail_ms;
        // COM PTT
        std::string com_port;
        int com_ptt_line;
        bool com_invert_dtr;
        bool com_invert_rts;
    };
    static constexpr int MAX_PRESETS = 10;
    static constexpr int MAX_FREQ_PRESETS = 12;

    struct FreqPreset {
        long long hz;
        std::string label;
    };
    std::vector<FreqPreset> freq_presets;
    std::vector<Preset> presets;
    int selected_preset = -1;
    int loaded_preset_index = -1;  
    
    std::atomic<bool> ptt_on{false};
    std::atomic<bool> receiving{false};
    std::atomic<bool> transmitting{false};
    std::atomic<int> client_count{0};
    std::atomic<int> tx_queue_size{0};
    std::atomic<float> last_rx_snr{0.0f};
    std::atomic<float> carrier_level_db{-100.0f};
    std::atomic<int> rx_frame_count{0};
    std::atomic<int> tx_frame_count{0};
    std::atomic<int> rx_error_count{0};
    std::atomic<float> last_rx_ber{-1.0f};
    
    // Decode statistics
    std::atomic<int> sync_count{0};
    std::atomic<int> preamble_errors{0};
    std::atomic<int> symbol_errors{0};
    std::atomic<int> erased_symbols{0};
    std::atomic<int> crc_errors{0};
    std::atomic<bool> stats_reset_requested{false};
    
    // Signal visualization
    static constexpr int LEVEL_HISTORY_SIZE = 60;
    std::mutex level_mutex;
    float level_history[LEVEL_HISTORY_SIZE];
    bool level_dcd[LEVEL_HISTORY_SIZE] = {false};
    bool level_tone[LEVEL_HISTORY_SIZE] = {false};
    int level_history_pos = 0;
    std::atomic<bool> decoding_active{false};
    
    // SNR history
    static constexpr int SNR_HISTORY_SIZE = 32;
    std::mutex snr_mutex;
    float snr_history[SNR_HISTORY_SIZE];
    int snr_history_pos = 0;
    int snr_history_count = 0;  

    static constexpr int CONSTELLATION_SIZE = 320;  // tone_count from modem
    static constexpr int CONSTELLATION_GRID = 51;   // density grid size 
    
    std::mutex constellation_mutex;
    std::array<std::complex<float>, CONSTELLATION_SIZE> constellation_points;
    std::array<int, CONSTELLATION_GRID * CONSTELLATION_GRID> constellation_density;
    int constellation_mod_bits = 2;  // Current modulation bits
    std::atomic<bool> constellation_valid{false};
    std::atomic<int64_t> constellation_update_time{0};
    
    void update_constellation(const std::complex<float>* points, int count, int mod_bits, int seed_off = -1) {
        std::lock_guard<std::mutex> lock(constellation_mutex);
        
        // copy data tones only
        static const int BLOCK_LEN = 5;  // from Common::block_length
        int n = 0;
        for (int i = 0; i < count && n < CONSTELLATION_SIZE; ++i) {
            if (seed_off >= 0 && (i % BLOCK_LEN) == seed_off) continue;
            constellation_points[n++] = points[i];
        }
        
        // Build density map
        constellation_density.fill(0);
        
        // Scale factor matched to actual constellation extents + headroom for noise
        float scale;
        switch (mod_bits) {
            case 1:  scale = 1.5f; break;  // BPSK  (extent 1.00)
            case 2:  scale = 1.3f; break;  // QPSK  (extent 0.71)
            case 3:  scale = 1.5f; break;  // 8PSK  (extent 0.92)
            case 4:  scale = 1.7f; break;  // QAM16 (extent 0.95)
            case 6:  scale = 2.0f; break;  // QAM64 (extent 1.08)
            case 8:  scale = 2.3f; break;  // QAM256 (extent 1.15)
            case 10: scale = 2.5f; break;  // QAM1024 (extent 1.19)
            case 12: scale = 2.5f; break;  // QAM4096 (extent 1.21)
            default: scale = 1.5f; break;
        }
        
        int half = CONSTELLATION_GRID / 2;
        for (int i = 0; i < n; ++i) {
            float re = constellation_points[i].real();
            float im = constellation_points[i].imag();
            
            // Map to grid coordinates
            int gx = half + (int)(re * half / scale);
            int gy = half - (int)(im * half / scale);  // Flip Y for display
            
            // Clamp to grid bounds
            gx = std::max(0, std::min(CONSTELLATION_GRID - 1, gx));
            gy = std::max(0, std::min(CONSTELLATION_GRID - 1, gy));
            
            constellation_density[gy * CONSTELLATION_GRID + gx]++;
        }
        
        constellation_mod_bits = mod_bits;
        constellation_valid = true;
        constellation_update_time = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    }

    static constexpr int WF_DECIM = 8;   // 48 kHz ->   6 kHz display spans 0-3 kHz
    static constexpr int WF_RING = 1024;
    std::atomic<bool> scope_active{false};
    std::mutex wf_mutex;
    float wf_ring[WF_RING] = {};
    int wf_wpos = 0;
    uint32_t wf_written = 0;
    std::atomic<int64_t> wf_sig_ms{0};
    float wf_acc = 0.0f;
    int wf_acc_n = 0;

    void push_scope_audio(const float* samples, int n) {
        std::lock_guard<std::mutex> lock(wf_mutex);
        for (int i = 0; i < n; i++) {
            wf_acc += samples[i];
            if (++wf_acc_n == WF_DECIM) {
                wf_ring[wf_wpos] = wf_acc * (1.0f / WF_DECIM);
                wf_wpos = (wf_wpos + 1) % WF_RING;
                wf_written++;
                wf_acc = 0.0f;
                wf_acc_n = 0;
            }
        }
    }

    struct PacketInfo {
        bool is_tx;
        int size;
        float snr;
        float ber;  // pre-FEC BER as percentage, -1 if unavailable
        std::chrono::steady_clock::time_point timestamp;
        std::string mode;
        std::string callsign;
    };
    static constexpr int MAX_RECENT_PACKETS = 8;
    std::mutex packets_mutex;
    std::deque<PacketInfo> recent_packets;
    
    // Chat test
    struct ChatMessage {
        bool is_tx;
        std::string callsign;
        std::string text;
        std::chrono::steady_clock::time_point timestamp;
    };
    static constexpr int MAX_CHAT_MESSAGES = 50;
    std::mutex chat_mutex;
    std::deque<ChatMessage> chat_messages;
    
    void add_chat_message(bool is_tx, const std::string& call, const std::string& text) {
        std::lock_guard<std::mutex> lock(chat_mutex);
        chat_messages.push_back({is_tx, call, text, std::chrono::steady_clock::now()});
        if (chat_messages.size() > MAX_CHAT_MESSAGES) {
            chat_messages.pop_front();
        }
    }
    
    std::vector<ChatMessage> get_chat_messages() {
        std::lock_guard<std::mutex> lock(chat_mutex);
        return std::vector<ChatMessage>(chat_messages.begin(), chat_messages.end());
    }
    

    std::function<void(const std::vector<uint8_t>&)> on_send_data;
    
    TNCUIState() {
        for (int i = 0; i < LEVEL_HISTORY_SIZE; i++) {
            level_history[i] = -100.0f;
        }
        for (int i = 0; i < SNR_HISTORY_SIZE; i++) {
            snr_history[i] = 0.0f;
        }
        for (auto& v : rig_meter_values) {
            v = NAN;
        }
        update_modem_info();
    }
    
    // TEMP modem tables
    void update_modem_info() {
        if (modem_type_index == 2) {
            RobustMode rmode = (RobustMode)robust_mode_index;
            mtu_bytes = RobustParams::data_bytes(rmode) - 2;
            bitrate_bps = RobustParams::bitrate(rmode);
            airtime_seconds = RobustParams::frame_duration(rmode);
            if (random_data_size == 0 || (!fragmentation_enabled && random_data_size > mtu_bytes))
                random_data_size = mtu_bytes;
            return;
        }
        // MFSK mode
        if (modem_type_index == 1) {
            MFSKMode mmode = (MFSKMode)mfsk_mode_index;
            mtu_bytes = MFSKParams::max_payload(mmode);
            bitrate_bps = MFSKParams::bitrate(mmode);
            airtime_seconds = MFSKParams::frame_duration();
            if (random_data_size == 0 || (!fragmentation_enabled && random_data_size > mtu_bytes))
                random_data_size = mtu_bytes;
            return;
        }

        // Modulations: BPSK=0, QPSK=1, 8PSK=2, QAM16=3, QAM64=4, QAM256=5, QAM1024=6, QAM4096=7
        // Code rates: 1/2=0, 2/3=1, 3/4=2, 5/6=3, 1/4=4
        // Columns: [1/2, 2/3, 3/4, 5/6, 1/4]
        static const int payload_short[8][5] = {
            {128, 171, 192, 213, 64},      // BPSK
            {128, 171, 192, 213, 64},      // QPSK
            {512, 684, 768, 852, 256},     // 8PSK
            {256, 342, 384, 426, 128},     // QAM16
            {1024, 1368, 1536, 1704, 512}, // QAM64
            {1024, 1368, 1536, 1704, 512}, // QAM256
            {2048, 2736, 3072, 3408, 1024}, // QAM1024
            {2048, 2736, 3072, 3408, 1024}, // QAM4096
        };
        
        static const int payload_normal[8][5] = {
            {256, 342, 384, 426, 128},      // BPSK
            {512, 684, 768, 852, 256},      // QPSK
            {1024, 1368, 1536, 1704, 512},  // 8PSK
            {1024, 1368, 1536, 1704, 512},  // QAM16
            {2048, 2736, 3072, 3408, 1024}, // QAM64
            {2048, 2736, 3072, 3408, 1024}, // QAM256
            {4096, 5472, 6144, 6816, 2048}, // QAM1024
            {4096, 5472, 6144, 6816, 2048}, // QAM4096
        };
        
        // Bitrate tables in bps (columns: 1/2, 2/3, 3/4, 5/6, 1/4)
        static const int bitrate_short[8][5] = {
            {700, 900, 1000, 1100, 300},      // BPSK
            {1100, 1400, 1600, 1800, 500},    // QPSK
            {2100, 2900, 3200, 3600, 1100},   // 8PSK
            {2100, 2900, 3200, 3600, 1000},   // QAM16
            {4300, 5700, 6400, 7100, 2200},   // QAM64
            {5400, 7300, 8200, 9100, 2700},   // QAM256
            {7500, 10000, 11200, 12500, 3700}, // QAM1024
            {8600, 11400, 12800, 14200, 4300}, // QAM4096
        };
        
        static const int bitrate_normal[8][5] = {
            {800, 1100, 1200, 1300, 400},     // BPSK
            {1600, 2100, 2400, 2600, 800},    // QPSK
            {2400, 3200, 3600, 4000, 1200},   // 8PSK
            {3200, 4200, 4700, 5200, 1600},   // QAM16
            {4800, 6400, 7200, 8000, 2400},   // QAM64
            {6300, 8400, 9500, 10500, 3200},  // QAM256
            {8300, 11000, 12400, 13800, 4100}, // QAM1024
            {9600, 12800, 14400, 16000, 4800}, // QAM4096
        };
        
        // Long frames double a normal frame
        static const int payload_long[8][5] = {
            {512, 684, 768, 852, 256},      // BPSK
            {1024, 1368, 1536, 1704, 512},  // QPSK
            {2048, 2736, 3072, 3408, 1024}, // 8PSK
            {2048, 2736, 3072, 3408, 1024}, // QAM16
            {4096, 5472, 6144, 6816, 2048}, // QAM64
            {4096, 5472, 6144, 6816, 2048}, // QAM256
            {0, 0, 0, 0, 0},                // QAM1024
            {0, 0, 0, 0, 0},                // QAM4096
        };

        static const int bitrate_long[8][5] = {
            {856, 1144, 1285, 1425, 428},     // BPSK
            {1713, 2288, 2569, 2850, 856},    // QPSK
            {2551, 3408, 3826, 4245, 1275},   // 8PSK
            {3425, 4576, 5138, 5700, 1713},   // QAM16
            {5101, 6815, 7652, 8489, 2551},   // QAM64
            {6851, 9152, 10276, 11400, 3425}, // QAM256
            {0, 0, 0, 0, 0},                  // QAM1024
            {0, 0, 0, 0, 0},                  // QAM4096
        };

        static const int duration_short[8] = {1500, 1000, 1900, 1000, 1900, 1500, 2200, 1900};
        static const int duration_normal[8] = {2600, 2600, 3400, 2600, 3400, 2600, 4000, 3400};
        static const int duration_long[8] = {4800, 4800, 6400, 4800, 6400, 4800, 0, 0};

        int mod = modulation_index;
        int rate = code_rate_index;

        if (mod < 0 || mod > 7) mod = 1;
        if (rate < 0 || rate > 6) rate = 0;

        if (frame_size == 3) {
            // QB QPSK quickburst
            bool valid = mod == 1 && rate == 0;
            airtime_seconds = 0.59f;
            mtu_bytes = valid ? 32 - 2 : 0;
            bitrate_bps = valid ? (int)(32 * 8 / airtime_seconds) : 0;
        } else if (rate == 5) {
            static const int payload_rep_short[8]  = {128, 256, 512, 512, 1024, 1024, 2048, 2048};
            static const int payload_rep_normal[8] = {256, 512, 1024, 1024, 2048, 0, 0, 0};
            static const int duration_rep_short[8]  = {2600, 1500, 3400, 1500, 3400, 2600, 4000, 3400};
            static const int duration_rep_normal[8] = {4800, 4800, 6400, 4800, 6400, 0, 0, 0};
            int pl = frame_size == 0 ? payload_rep_short[mod]
                   : frame_size == 1 ? payload_rep_normal[mod] : 0;
            int du = frame_size == 0 ? duration_rep_short[mod]
                   : frame_size == 1 ? duration_rep_normal[mod] : 0;
            mtu_bytes = pl > 0 ? pl - 2 : 0;
            airtime_seconds = du / 1000.0f;
            bitrate_bps = du > 0 ? (int)(pl * 8000.0f / du) : 0;
        } else if (rate == 6) {
            static const int payload_rep2_short[8]  = {64, 64, 256, 128, 512, 512, 1024, 1024};
            static const int payload_rep2_normal[8] = {128, 256, 512, 512, 1024, 0, 0, 0};
            static const int duration_rep2_short[8]  = {2733, 1640, 3553, 1640, 3553, 2733, 4100, 3553};
            static const int duration_rep2_normal[8] = {4920, 4920, 6560, 4920, 6560, 0, 0, 0};
            int pl = frame_size == 0 ? payload_rep2_short[mod]
                   : frame_size == 1 ? payload_rep2_normal[mod] : 0;
            int du = frame_size == 0 ? duration_rep2_short[mod]
                   : frame_size == 1 ? duration_rep2_normal[mod] : 0;
            mtu_bytes = pl > 0 ? pl - 2 : 0;
            airtime_seconds = du / 1000.0f;
            bitrate_bps = du > 0 ? (int)(pl * 8000.0f / du) : 0;
        } else if (frame_size == 0) {
            mtu_bytes = payload_short[mod][rate] - 2;
            bitrate_bps = bitrate_short[mod][rate];
            airtime_seconds = duration_short[mod] / 1000.0f;
        } else if (frame_size == 2) {
            mtu_bytes = payload_long[mod][rate] > 0 ? payload_long[mod][rate] - 2 : 0;
            bitrate_bps = bitrate_long[mod][rate];
            airtime_seconds = duration_long[mod] / 1000.0f;
        } else {
            mtu_bytes = payload_normal[mod][rate] - 2;
            bitrate_bps = bitrate_normal[mod][rate];
            airtime_seconds = duration_normal[mod] / 1000.0f;
        }
        
        // Initialize random_data_size if not set, clamp to MTU only if fragmentation disabled
        if (random_data_size == 0) {
            random_data_size = mtu_bytes;
        } else if (!fragmentation_enabled && random_data_size > mtu_bytes) {
            random_data_size = mtu_bytes;
        }
    }
    
    void update_level(float db, bool dcd = false, bool tone = false) {
        carrier_level_db = db;
        std::lock_guard<std::mutex> lock(level_mutex);
        level_history[level_history_pos] = db;
        level_dcd[level_history_pos] = dcd;
        level_tone[level_history_pos] = tone;
        level_history_pos = (level_history_pos + 1) % LEVEL_HISTORY_SIZE;
    }
    
    void update_snr(float snr) {
        std::lock_guard<std::mutex> lock(snr_mutex);
        snr_history[snr_history_pos] = snr;
        snr_history_pos = (snr_history_pos + 1) % SNR_HISTORY_SIZE;
        if (snr_history_count < SNR_HISTORY_SIZE) snr_history_count++;
    }
    
    std::vector<float> get_snr_history() {
        std::lock_guard<std::mutex> lock(snr_mutex);
        std::vector<float> result;
        if (snr_history_count == 0) return result;
        int start = (snr_history_pos - snr_history_count + SNR_HISTORY_SIZE) % SNR_HISTORY_SIZE;
        for (int i = 0; i < snr_history_count; i++) {
            result.push_back(snr_history[(start + i) % SNR_HISTORY_SIZE]);
        }
        return result;
    }
    
    void add_packet(bool is_tx, int size, float snr = 0.0f, float ber = -1.0f,
                    const std::string& mode = "", const std::string& callsign = "") {
        {
            std::lock_guard<std::mutex> lock(packets_mutex);
            recent_packets.push_back({is_tx, size, snr, ber,
                                      std::chrono::steady_clock::now(), mode, callsign});
            if (recent_packets.size() > MAX_RECENT_PACKETS) {
                recent_packets.pop_front();
            }
        }


        if (!is_tx && snr > 0.0f) {
            update_snr(snr);
        }



    }
    
    std::vector<PacketInfo> get_recent_packets() {
        std::lock_guard<std::mutex> lock(packets_mutex);
        return std::vector<PacketInfo>(recent_packets.begin(), recent_packets.end());
    }
    
    std::vector<float> get_level_history() {
        std::lock_guard<std::mutex> lock(level_mutex);
        std::vector<float> result(LEVEL_HISTORY_SIZE);
        for (int i = 0; i < LEVEL_HISTORY_SIZE; i++) {
            result[i] = level_history[(level_history_pos + i) % LEVEL_HISTORY_SIZE];
        }
        return result;
    }

    std::vector<uint8_t> get_level_dcd_history() {
        std::lock_guard<std::mutex> lock(level_mutex);
        std::vector<uint8_t> result(LEVEL_HISTORY_SIZE);
        for (int i = 0; i < LEVEL_HISTORY_SIZE; i++) {
            result[i] = level_dcd[(level_history_pos + i) % LEVEL_HISTORY_SIZE];
        }
        return result;
    }

    std::vector<uint8_t> get_level_tone_history() {
        std::lock_guard<std::mutex> lock(level_mutex);
        std::vector<uint8_t> result(LEVEL_HISTORY_SIZE);
        for (int i = 0; i < LEVEL_HISTORY_SIZE; i++) {
            result[i] = level_tone[(level_history_pos + i) % LEVEL_HISTORY_SIZE];
        }
        return result;
    }
    
    // Save settings
    bool save_settings() {
        if (config_file.empty()) return false;

        std::string tmp = config_file + ".tmp";
        FILE* f = fopen(tmp.c_str(), "w");
        if (!f) return false;

        fprintf(f, "# MODEM73 Settings\n");
        fprintf(f, "callsign=%s\n", callsign.c_str());
        fprintf(f, "modem_type=%d\n", modem_type_index);
        fprintf(f, "mfsk_mode=%d\n", mfsk_mode_index);
        fprintf(f, "modulation=%d\n", modulation_index);
        fprintf(f, "code_rate=%d\n", code_rate_index);
        fprintf(f, "short_frame=%d\n", frame_size == 0 ? 1 : 0);
        fprintf(f, "frame_size=%d\n", frame_size);
        fprintf(f, "center_freq=%d\n", center_freq);
        fprintf(f, "postamble=%d\n", postamble ? 1 : 0);
        fprintf(f, "robust_mode=%d\n", robust_mode_index);
        fprintf(f, "tx_drive=%.2f\n", tx_drive.load());
        fprintf(f, "alt_mode_mask=%d\n", alt_mode_mask);
        for (const auto& fp : freq_presets)
            fprintf(f, "freq_preset=%lld|%s\n", fp.hz, fp.label.c_str());
        fprintf(f, "csma_enabled=%d\n", csma_enabled ? 1 : 0);
        fprintf(f, "csma_sync_only=%d\n", csma_sync_only ? 1 : 0);
        fprintf(f, "csma_fast_floor=%d\n", csma_fast_floor ? 1 : 0);
        fprintf(f, "csma_ranked=%d\n", csma_ranked ? 1 : 0);
        fprintf(f, "beacon_interval_s=%d\n", beacon_interval_s);
        fprintf(f, "carrier_threshold_db=%.1f\n", carrier_threshold_db);
        fprintf(f, "slot_time_ms=%d\n", slot_time_ms);
        fprintf(f, "csma_quiet_ms=%d\n", csma_quiet_ms);
        fprintf(f, "csma_cw=%d\n", csma_cw);
        fprintf(f, "p_persistence=%d\n", p_persistence);
        fprintf(f, "tx_lead_tone=%d\n", tx_lead_tone ? 1 : 0);
        fprintf(f, "csma_responder_dither=%d\n", csma_responder_dither);
        fprintf(f, "csma_burst=%d\n", csma_burst);
        fprintf(f, "csma_band=%d\n", csma_band);
        fprintf(f, "fragmentation_enabled=%d\n", fragmentation_enabled ? 1 : 0);
        fprintf(f, "tx_blanking_enabled=%d\n", tx_blanking_enabled ? 1 : 0);
        fprintf(f, "ofdm_rx_enabled=%d\n", ofdm_rx_enabled ? 1 : 0);
        fprintf(f, "robust_rx_enabled=%d\n", robust_rx_enabled ? 1 : 0);
        fprintf(f, "mfsk_rx_enabled=%d\n", mfsk_rx_enabled ? 1 : 0);
        fprintf(f, "# Audio/PTT\n");
        fprintf(f, "audio_input=%s\n", audio_input_device.c_str());
        fprintf(f, "audio_output=%s\n", audio_output_device.c_str());
        fprintf(f, "ptt_type=%d\n", ptt_type_index);
        fprintf(f, "vox_tone_freq=%d\n", vox_tone_freq);
        fprintf(f, "vox_lead_ms=%d\n", vox_lead_ms);
        fprintf(f, "vox_tail_ms=%d\n", vox_tail_ms);
        fprintf(f, "tx_delay_ms=%d\n", tx_delay_ms);
        fprintf(f, "# COM PTT\n");
        fprintf(f, "com_port=%s\n", com_port.c_str());
        fprintf(f, "com_ptt_line=%d\n", com_ptt_line);
        fprintf(f, "com_invert_dtr=%d\n", com_invert_dtr ? 1 : 0);
        fprintf(f, "com_invert_rts=%d\n", com_invert_rts ? 1 : 0);
#ifdef WITH_CM108
        fprintf(f, "# CM108 PTT\n");
        fprintf(f, "cm108_gpio=%d\n", cm108_gpio);
        fprintf(f, "cm108_device=%s\n", cm108_device.c_str());
#endif
        fprintf(f, "# Network\n");
        fprintf(f, "port=%d\n", port);
        fprintf(f, "control_port=%d\n", control_port);
        fprintf(f, "bind_address=%s\n", bind_address.c_str());
        fprintf(f, "control_bind_address=%s\n", control_bind_address.c_str());
        fprintf(f, "# Utils\n");
        fprintf(f, "random_data_size=%d\n", random_data_size);
        fprintf(f, "utils_testing=%d\n", utils_testing_open ? 1 : 0);

        if (fclose(f) != 0 || rename(tmp.c_str(), config_file.c_str()) != 0) {
            remove(tmp.c_str());
            return false;
        }
        return true;
    }
    
    // Load settings 
    bool load_settings() {
        if (config_file.empty()) return false;
        
        FILE* f = fopen(config_file.c_str(), "r");
        if (!f) return false;
        
        freq_presets.clear();

        char line[512];
        while (fgets(line, sizeof(line), f)) {
            if (line[0] == '#') continue;

            char key[64], value[384];
            if (sscanf(line, "%63[^=]=%383[^\n]", key, value) == 2) {
                if (strcmp(key, "freq_preset") == 0) {
                    const char* bar = strchr(value, '|');
                    if (bar && (int)freq_presets.size() < MAX_FREQ_PRESETS) {
                        long long hz = atoll(std::string(value, bar - value).c_str());
                        if (hz > 0 && bar[1])
                            freq_presets.push_back({hz, std::string(bar + 1)});
                    }
                }
                else if (strcmp(key, "callsign") == 0) callsign = value;
                else if (strcmp(key, "modem_type") == 0) {
                    int v = atoi(value);
                    if (v >= 0 && v <= 2) modem_type_index = v;
                }
                else if (strcmp(key, "mfsk_mode") == 0) {
                    int v = atoi(value);
                    if (v >= 0 && v <= 3) mfsk_mode_index = v;
                }
                else if (strcmp(key, "robust_mode") == 0) {
                    int v = atoi(value);
                    if (v >= 0 && v < ROBUST_MODE_COUNT) robust_mode_index = v;
                }
                else if (strcmp(key, "tx_drive") == 0) {
                    float v = strtof(value, nullptr);
                    if (v >= 0.05f && v <= 1.0f) tx_drive = v;
                }
                else if (strcmp(key, "alt_mode_mask") == 0)
                    alt_mode_mask = atoi(value) & ((1 << ALT_MODE_COUNT) - 1);
                else if (strcmp(key, "modulation") == 0) {
                    int v = atoi(value);
                    if (v >= 0 && v <= 7) modulation_index = v;
                }
                else if (strcmp(key, "code_rate") == 0) {
                    int v = atoi(value);
                    if (v >= 0 && v <= 6) code_rate_index = v;
                }
                else if (strcmp(key, "short_frame") == 0) frame_size = atoi(value) != 0 ? 0 : 1;
                else if (strcmp(key, "frame_size") == 0) {
                    int v = atoi(value);
                    if (v >= 0 && v <= 3) frame_size = v;
                }
                else if (strcmp(key, "center_freq") == 0) center_freq = 1500;
                else if (strcmp(key, "postamble") == 0) postamble = atoi(value) != 0;
                else if (strcmp(key, "csma_enabled") == 0) csma_enabled = atoi(value) != 0;
                else if (strcmp(key, "csma_sync_only") == 0) csma_sync_only = atoi(value) != 0;
                else if (strcmp(key, "csma_fast_floor") == 0) csma_fast_floor = atoi(value) != 0;
                else if (strcmp(key, "csma_ranked") == 0) csma_ranked = atoi(value) != 0;
                else if (strcmp(key, "beacon_interval_s") == 0) {
                    int v = atoi(value);
                    if (v >= 45 && v <= 90) beacon_interval_s = v;
                }
                else if (strcmp(key, "carrier_threshold_db") == 0) carrier_threshold_db = atof(value);
                else if (strcmp(key, "slot_time_ms") == 0) slot_time_ms = atoi(value);
                else if (strcmp(key, "csma_quiet_ms") == 0) csma_quiet_ms = atoi(value);
                else if (strcmp(key, "csma_cw") == 0) csma_cw = atoi(value);
                else if (strcmp(key, "p_persistence") == 0) p_persistence = atoi(value);
                else if (strcmp(key, "tx_lead_tone") == 0) tx_lead_tone = atoi(value) != 0;
                else if (strcmp(key, "csma_responder_dither") == 0) csma_responder_dither = atoi(value);
                else if (strcmp(key, "csma_burst") == 0) csma_burst = atoi(value);
                else if (strcmp(key, "csma_band") == 0) csma_band = atoi(value) != 0 ? 1 : 0;
                else if (strcmp(key, "fragmentation_enabled") == 0) fragmentation_enabled = atoi(value) != 0;
                else if (strcmp(key, "tx_blanking_enabled") == 0) tx_blanking_enabled = atoi(value) != 0;
                else if (strcmp(key, "ofdm_rx_enabled") == 0) ofdm_rx_enabled = atoi(value) != 0;
                else if (strcmp(key, "robust_rx_enabled") == 0) robust_rx_enabled = atoi(value) != 0;
                else if (strcmp(key, "mfsk_rx_enabled") == 0) mfsk_rx_enabled = atoi(value) != 0;
                else if (strcmp(key, "audio_input") == 0) audio_input_device = value;
                else if (strcmp(key, "audio_output") == 0) audio_output_device = value;
                else if (strcmp(key, "audio_device") == 0) {
                    audio_input_device = value;
                    audio_output_device = value;
                }
                else if (strcmp(key, "ptt_type") == 0) {
                    int v = atoi(value);
                    if (v >= 0 && v < (int)PTT_TYPE_OPTIONS.size()) ptt_type_index = v;
                }
                else if (strcmp(key, "vox_tone_freq") == 0) {
                    int v = atoi(value);
                    if (v >= 300 && v <= 3000) vox_tone_freq = v;
                }
                else if (strcmp(key, "tx_delay_ms") == 0) {
                    int v = atoi(value);
                    if (v >= 250 && v <= 2500) tx_delay_ms = v;
                }
                else if (strcmp(key, "vox_lead_ms") == 0) {
                    int v = atoi(value);
                    if (v >= 50 && v <= 2000) vox_lead_ms = v;
                }
                else if (strcmp(key, "vox_tail_ms") == 0) {
                    int v = atoi(value);
                    if (v >= 50 && v <= 2000) vox_tail_ms = v;
                }
                else if (strcmp(key, "com_port") == 0) com_port = value;
                else if (strcmp(key, "com_ptt_line") == 0) {
                    int v = atoi(value);
                    if (v >= 0 && v < (int)PTT_LINE_OPTIONS.size()) com_ptt_line = v;
                }
                else if (strcmp(key, "com_invert_dtr") == 0) com_invert_dtr = atoi(value) != 0;
                else if (strcmp(key, "com_invert_rts") == 0) com_invert_rts = atoi(value) != 0;
#ifdef WITH_CM108
                else if (strcmp(key, "cm108_gpio") == 0) {
                    int v = atoi(value);
                    if (v >= 1 && v <= 4) cm108_gpio = v;
                }
                else if (strcmp(key, "cm108_device") == 0) cm108_device = value;
#endif
                else if (strcmp(key, "port") == 0) {
                    int v = atoi(value);
                    if (v >= 1 && v <= 65535) port = v;
                }
                else if (strcmp(key, "control_port") == 0) {
                    int v = atoi(value);
                    if (v >= 1 && v <= 65535) control_port = v;
                }
                else if (strcmp(key, "bind_address") == 0) bind_address = value;
                else if (strcmp(key, "control_bind_address") == 0) control_bind_address = value;
                else if (strcmp(key, "random_data_size") == 0) {
                    int v = atoi(value);
                    if (v >= 0 && v <= 65535) random_data_size = v;
                }
                else if (strcmp(key, "utils_testing") == 0) utils_testing_open = atoi(value) != 0;
            }
        }
        
        fclose(f);
        clamp_micro();
        update_modem_info();
        return true;
    }


    bool save_presets() {
        if (presets_file.empty()) return false;
        
        std::string tmp = presets_file + ".tmp";
        FILE* f = fopen(tmp.c_str(), "w");
        if (!f) return false;

        fprintf(f, "# MODEM73 Presets \n");
        for (const auto& p : presets) {
            // 1=short, 0=normal, 2=long, 3=micro
            fprintf(f, "preset=%s,%d,%d,%d,%d,%d,%.1f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
                    p.name.c_str(),
                    p.modulation_index,
                    p.code_rate_index,
                    p.frame_size == 0 ? 1 : p.frame_size >= 2 ? p.frame_size : 0,
                    p.center_freq,
                    p.csma_enabled ? 1 : 0,
                    p.carrier_threshold_db,
                    p.slot_time_ms,
                    p.p_persistence,
                    p.ptt_type_index,
                    p.vox_tone_freq,
                    p.vox_lead_ms,
                    p.vox_tail_ms,
                    p.modem_type_index,
                    p.mfsk_mode_index,
                    p.robust_mode_index,
                    p.postamble ? 1 : 0);
        }

        if (fclose(f) != 0 || rename(tmp.c_str(), presets_file.c_str()) != 0) {
            remove(tmp.c_str());
            return false;
        }
        return true;
    }
    
    // Load presets 
    bool load_presets() {
        if (presets_file.empty()) return false;
        
        FILE* f = fopen(presets_file.c_str(), "r");
        if (!f) return false;
        
        presets.clear();
        
        char line[512];
        while (fgets(line, sizeof(line), f)) {
            if (line[0] == '#') continue;
            if (strncmp(line, "preset=", 7) != 0) continue;
            
            char name[64];
            int mod, rate, sf, freq, csma, slot, persist;
            int ptt_type = 1, vox_freq = 1200, vox_lead = 150, vox_tail = 100;
            int modem_type = 0, mfsk_mode = 1;
            int robust_mode = 0, postamble = 0;
            float thresh;

            int n = sscanf(line + 7, "%63[^,],%d,%d,%d,%d,%d,%f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
                       name, &mod, &rate, &sf, &freq, &csma, &thresh, &slot, &persist,
                       &ptt_type, &vox_freq, &vox_lead, &vox_tail,
                       &modem_type, &mfsk_mode, &robust_mode, &postamble);

            if (n >= 9 && (int)presets.size() < MAX_PRESETS) {
                auto clampi = [](int v, int lo, int hi) {
                    return v < lo ? lo : v > hi ? hi : v;
                };
                Preset p;
                p.name = name;
                p.modulation_index = clampi(mod, 0, (int)MODULATION_OPTIONS.size() - 1);
                p.code_rate_index = clampi(rate, 0, (int)CODE_RATE_OPTIONS.size() - 1);
                p.frame_size = sf == 1 ? 0 : (sf == 2 || sf == 3) ? sf : 1;
                if (p.frame_size == 3 &&
                    !(p.modulation_index == 1 && p.code_rate_index == 0))
                    p.frame_size = 1;
                p.center_freq = 1500;
                p.csma_enabled = csma != 0;
                p.carrier_threshold_db = thresh;
                p.slot_time_ms = slot;
                p.p_persistence = persist;

                p.ptt_type_index = (n >= 10) ? clampi(ptt_type, 0, (int)PTT_TYPE_OPTIONS.size() - 1) : 1;
                p.vox_tone_freq = (n >= 11 && vox_freq >= 300 && vox_freq <= 3000) ? vox_freq : 1200;
                p.vox_lead_ms = (n >= 12) ? clampi(vox_lead, 50, 2000) : 150;
                p.vox_tail_ms = (n >= 13) ? clampi(vox_tail, 50, 2000) : 100;


                p.modem_type_index = (n >= 14) ? clampi(modem_type, 0, (int)MODEM_TYPE_OPTIONS.size() - 1) : 0;
                p.mfsk_mode_index = (n >= 15) ? clampi(mfsk_mode, 0, (int)MFSK_MODE_OPTIONS.size() - 1) : 1;
                p.robust_mode_index = (n >= 16 && robust_mode >= 0 &&
                                       robust_mode < ROBUST_MODE_COUNT) ? robust_mode : 0;
                p.postamble = (n >= 17) && postamble != 0;
                presets.push_back(p);
            }
        }
        
        fclose(f);
        if (!presets.empty()) {
            selected_preset = 0;
        }
        return true;
    }
    



    bool create_preset(const std::string& name) {
        if (presets.size() >= MAX_PRESETS) return false;
        if (name.empty()) return false;
        
        Preset p;
        p.name = name;
        p.modem_type_index = modem_type_index;
        p.mfsk_mode_index = mfsk_mode_index;
        p.robust_mode_index = robust_mode_index;
        p.postamble = postamble;
        p.modulation_index = modulation_index;
        p.code_rate_index = code_rate_index;
        p.frame_size = frame_size;
        p.center_freq = center_freq;
        p.csma_enabled = csma_enabled;
        p.carrier_threshold_db = carrier_threshold_db;
        p.slot_time_ms = slot_time_ms;
        p.p_persistence = p_persistence;
        p.ptt_type_index = ptt_type_index;
        p.vox_tone_freq = vox_tone_freq;
        p.vox_lead_ms = vox_lead_ms;
        p.vox_tail_ms = vox_tail_ms;
        
        presets.push_back(p);
        save_presets();
        return true;
    }


    bool apply_preset(int index) {
        if (index < 0 || index >= (int)presets.size()) return false;
        
        const Preset& p = presets[index];
        modem_type_index = p.modem_type_index;
        mfsk_mode_index = p.mfsk_mode_index;
        robust_mode_index = p.robust_mode_index;
        postamble = p.postamble;
        modulation_index = p.modulation_index;
        code_rate_index = p.code_rate_index;
        frame_size = p.frame_size;
        clamp_micro();
        csma_enabled = p.csma_enabled;
        carrier_threshold_db = p.carrier_threshold_db;
        slot_time_ms = p.slot_time_ms;
        p_persistence = p.p_persistence;
        ptt_type_index = p.ptt_type_index;
        vox_tone_freq = p.vox_tone_freq;
        vox_lead_ms = p.vox_lead_ms;
        vox_tail_ms = p.vox_tail_ms;
        
        update_modem_info();
        return true;
    }


    bool delete_preset(int index) {
        if (index < 0 || index >= (int)presets.size()) return false;
        
        presets.erase(presets.begin() + index);
        if (selected_preset >= (int)presets.size()) {
            selected_preset = presets.size() - 1;
        }
        save_presets();
        return true;
    }


    std::mutex log_mutex;
    std::deque<std::string> log_entries;
    std::atomic<bool> log_unread_error{false};

    std::function<void(TNCUIState&)> on_settings_changed;
    std::function<void()> on_stop_requested;
    std::function<bool()> on_reconnect_audio;
    std::function<float()> on_get_audio_level;  
    
    void add_log(const std::string& msg) {
        std::lock_guard<std::mutex> lock(log_mutex);
        auto now = std::chrono::system_clock::now();
        auto time = std::chrono::system_clock::to_time_t(now);
        struct tm tmv;
        localtime_r(&time, &tmv);
        std::stringstream ss;
        ss << std::put_time(&tmv, "%H:%M:%S") << "  " << msg;
        log_entries.push_back(ss.str());
        if (msg.rfind("(!)", 0) == 0) log_unread_error = true;
        if (log_entries.size() > MAX_LOG_ENTRIES) {
            log_entries.pop_front();
        }
    }
    
    std::vector<std::string> get_log() {
        std::lock_guard<std::mutex> lock(log_mutex);
        return std::vector<std::string>(log_entries.begin(), log_entries.end());
    }

    // "M73:<call>:<text>"
    static constexpr size_t MAX_MESSAGE_CHARS = 150;
    static constexpr size_t MAX_MESSAGES = 100;
    struct TextMessage {
        std::string time;
        std::string from;
        std::string text;
        bool outgoing;
    };
    std::mutex messages_mutex;
    std::deque<TextMessage> messages;
    std::atomic<int> unread_messages{0};

    void add_message(const std::string& from, const std::string& text, bool outgoing) {
        auto now = std::chrono::system_clock::now();
        auto time = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&time), "%H:%M:%S");
        std::lock_guard<std::mutex> lock(messages_mutex);
        messages.push_back({ss.str(), from, text, outgoing});
        while (messages.size() > MAX_MESSAGES)
            messages.pop_front();
        if (!outgoing)
            unread_messages++;
    }

    std::vector<TextMessage> get_messages() {
        std::lock_guard<std::mutex> lock(messages_mutex);
        return std::vector<TextMessage>(messages.begin(), messages.end());
    }

    // rig control via rigctld. commands use the extended response protocol
    // ('+' prefix) so every reply is terminated by an rprt line.
    static bool rig_ok(const std::string& resp) {
        return resp.find("RPRT 0") != std::string::npos;
    }

    static std::string rig_value(const std::string& resp, const char* key) {
        std::string k = std::string(key) + ":";
        size_t p = resp.find(k);
        if (p != std::string::npos) {
            p += k.size();
            while (p < resp.size() && resp[p] == ' ') p++;
            size_t e = resp.find('\n', p);
            return resp.substr(p, e == std::string::npos ? std::string::npos : e - p);
        }
        std::string bare;
        size_t pos = 0;
        while (pos < resp.size()) {
            size_t e = resp.find('\n', pos);
            std::string line = resp.substr(pos, e == std::string::npos ? std::string::npos : e - pos);
            if (line.rfind("RPRT", 0) == 0) break;
            if (!line.empty() && line.find(':') == std::string::npos) bare = line;
            if (e == std::string::npos) break;
            pos = e + 1;
        }
        return bare;
    }

    std::string get_rig_mode() {
        std::lock_guard<std::mutex> lock(rig_mode_mutex);
        return rig_mode;
    }

    void set_rig_mode_cache(const std::string& m) {
        std::lock_guard<std::mutex> lock(rig_mode_mutex);
        rig_mode = m;
    }

    void poll_rig() {
        if (!on_rigctl_command) return;
        int64_t now = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        bool refresh = rig_refresh_requested.exchange(false);
        if (!refresh && now - rig_last_poll_ms < 1000) return;
        rig_last_poll_ms = now;

        bool any = false;
        std::string r = on_rigctl_command("+f");
        if (rig_ok(r)) {
            std::string v = rig_value(r, "Frequency");
            if (!v.empty()) {
                rig_freq_hz = atoll(v.c_str());
                any = true;
            }
        }

        r = on_rigctl_command("+m");
        if (rig_ok(r)) {
            std::string v = rig_value(r, "Mode");
            if (!v.empty()) {
                set_rig_mode_cache(v);
                any = true;
            }
        }

        for (int i = 0; i < RIG_METER_COUNT; i++) {
            r = on_rigctl_command(std::string("+l ") + RIG_METERS[i].level);
            std::string v = rig_ok(r) ? rig_value(r, "Level Value") : "";
            rig_meter_values[i] = v.empty() ? NAN : strtof(v.c_str(), nullptr);
        }

        r = on_rigctl_command("+l RFPOWER");
        if (rig_ok(r)) {
            std::string v = rig_value(r, "Level Value");
            rig_power_level = v.empty() ? -1.0f : strtof(v.c_str(), nullptr);
        } else {
            rig_power_level = -1.0f;
        }

        r = on_rigctl_command("+u TUNER");
        if (rig_ok(r)) {
            std::string v = rig_value(r, "Func Status");
            rig_tuner_on = v.empty() ? -1 : atoi(v.c_str());
            rig_tuner_supported = 1;
        } else {
            rig_tuner_on = -1;
            if (any) rig_tuner_supported = 0;
        }

        rig_data_valid = any;
        if (any) rig_last_update_ms = now;

        // latch the worst SWR seen while PTT is keyed; warn once the burst
        // ends if it crossed the threshold, clear after a clean burst
        bool ptt = ptt_on.load();
        float swr = rig_meter_values[RIG_METER_SWR].load();
        if (ptt && !std::isnan(swr) && swr > swr_burst_max)
            swr_burst_max = swr;
        if (!ptt && swr_prev_ptt && swr_burst_max > 0.0f) {
            if (swr_burst_max >= SWR_WARN_THRESHOLD) {
                char msg[64];
                snprintf(msg, sizeof(msg), "(!) HIGH SWR %.1f during TX",
                         swr_burst_max);
                add_log(msg);
                swr_warn_value = swr_burst_max;
            } else {
                swr_warn_value = 0.0f;
            }
            swr_burst_max = 0.0f;
        }
        swr_prev_ptt = ptt;
    }
};
