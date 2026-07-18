#pragma once

#include <locale.h>
#include <ncurses.h>
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
#include "phy/mfsk_modem.hh"
#include "phy/robust_modem.hh"
#include "perf_log.hh"
#ifdef WITH_CM108
#include "cm108_ptt.hh"
#endif

constexpr size_t MAX_LOG_ENTRIES = 500;

const std::vector<std::string> MODEM_TYPE_OPTIONS = {"OFDM", "MFSK", "ROBUST"};
const std::vector<std::string> ROBUST_MODE_OPTIONS = {"RDM-1200", "RDM-800", "RDM-600", "RDM-300", "RDMN-300", "RDMN-150"};
const std::vector<std::string> ROBUST_MTU_OPTIONS = {"510 B", "170 B (short)"};

// robust_mode ints: 0-4 full-frame, 5-9 short-frame, 10/11 RDM-800/-800S.
// The UI shows a base-mode selector (index into ROBUST_MODE_OPTIONS above,
// display order) plus a frame-size toggle; these map between the two.
inline int robust_base_index(int mode) {
    if (mode >= 10) return 1;                       // RDM-800 family
    int fam = mode % 5;                             // 0-4 family order
    return fam == 0 ? 0 : fam + 1;                  // shifted past RDM-800
}
inline int robust_mode_of(int base, bool short_frame) {
    if (base == 1) return short_frame ? 11 : 10;
    int fam = base == 0 ? 0 : base - 1;
    return fam + (short_frame ? 5 : 0);
}

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

const std::vector<std::string> PTT_TYPE_OPTIONS = {
    "NONE", "RIGCTL", "VOX", "COM"
#ifdef WITH_CM108
    , "CM108"
#endif
};

const std::vector<std::string> PTT_LINE_OPTIONS = {
    "DTR", "RTS", "BOTH"
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

constexpr int UTILS_ACTION_COUNT = 11 + ALT_MODE_COUNT;



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
    int frame_size = 1;        // 0=short, 1=normal, 2=long
    int center_freq = 1500;
    bool postamble = false;

    bool csma_enabled = true;
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
    bool tx_blanking_enabled = false;
    
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
        int frame_size;        // 0=short, 1=normal, 2=long
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

        if (rate == 5) {
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
    
    void update_level(float db, bool dcd = false) {
        carrier_level_db = db;
        std::lock_guard<std::mutex> lock(level_mutex);
        level_history[level_history_pos] = db;
        level_dcd[level_history_pos] = dcd;
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
    
    // Save settings
    bool save_settings() {
        if (config_file.empty()) return false;
        
        FILE* f = fopen(config_file.c_str(), "w");
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
        fprintf(f, "csma_enabled=%d\n", csma_enabled ? 1 : 0);
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
        fprintf(f, "# Audio/PTT\n");
        fprintf(f, "audio_input=%s\n", audio_input_device.c_str());
        fprintf(f, "audio_output=%s\n", audio_output_device.c_str());
        fprintf(f, "ptt_type=%d\n", ptt_type_index);
        fprintf(f, "vox_tone_freq=%d\n", vox_tone_freq);
        fprintf(f, "vox_lead_ms=%d\n", vox_lead_ms);
        fprintf(f, "vox_tail_ms=%d\n", vox_tail_ms);
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
        fprintf(f, "bind_address=%s\n", bind_address.c_str());
        fprintf(f, "control_bind_address=%s\n", control_bind_address.c_str());
        fprintf(f, "# Utils\n");
        fprintf(f, "random_data_size=%d\n", random_data_size);
        fprintf(f, "utils_testing=%d\n", utils_testing_open ? 1 : 0);
        
        fclose(f);
        return true;
    }
    
    // Load settings 
    bool load_settings() {
        if (config_file.empty()) return false;
        
        FILE* f = fopen(config_file.c_str(), "r");
        if (!f) return false;
        
        char line[256];
        while (fgets(line, sizeof(line), f)) {
            if (line[0] == '#') continue;
            
            char key[64], value[192];
            if (sscanf(line, "%63[^=]=%191[^\n]", key, value) == 2) {
                if (strcmp(key, "callsign") == 0) callsign = value;
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
                    if (v >= 0 && v <= 2) frame_size = v;
                }
                else if (strcmp(key, "center_freq") == 0) {
                    int v = atoi(value);
                    if (v >= 300 && v <= 3000) center_freq = v;
                }
                else if (strcmp(key, "postamble") == 0) postamble = atoi(value) != 0;
                else if (strcmp(key, "csma_enabled") == 0) csma_enabled = atoi(value) != 0;
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
        update_modem_info();
        return true;
    }
    

    bool save_presets() {
        if (presets_file.empty()) return false;
        
        FILE* f = fopen(presets_file.c_str(), "w");
        if (!f) return false;
        
        fprintf(f, "# MODEM73 Presets \n");
        for (const auto& p : presets) {
            // sf field keeps legacy semantics: 1=short, 0=normal, 2=long
            fprintf(f, "preset=%s,%d,%d,%d,%d,%d,%.1f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
                    p.name.c_str(),
                    p.modulation_index,
                    p.code_rate_index,
                    p.frame_size == 0 ? 1 : p.frame_size == 2 ? 2 : 0,
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
        
        fclose(f);
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
                p.frame_size = sf == 1 ? 0 : sf == 2 ? 2 : 1;
                p.center_freq = (freq >= 300 && freq <= 3000) ? freq : 1500;
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
        center_freq = p.center_freq;
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
    
    std::function<void(TNCUIState&)> on_settings_changed;
    std::function<void()> on_stop_requested;
    std::function<bool()> on_reconnect_audio;  
    
    void add_log(const std::string& msg) {
        std::lock_guard<std::mutex> lock(log_mutex);
        auto now = std::chrono::system_clock::now();
        auto time = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&time), "%H:%M:%S") << "  " << msg;
        log_entries.push_back(ss.str());
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

class TNCUI {
public:
    TNCUI(TNCUIState& state) : state_(state) {}
    
    ~TNCUI() {
        if (initialized_) {
            endwin();
        }

        if (saved_stderr_ >= 0) {
            dup2(saved_stderr_, STDERR_FILENO);
            close(saved_stderr_);
        }
    }
    
    void run() {
        // set locale LC_ALL for Unicode character support,  
        setlocale(LC_ALL, "");
        

        saved_stderr_ = dup(STDERR_FILENO);
        int devnull = open("/dev/null", O_WRONLY);
        if (devnull >= 0) {
            dup2(devnull, STDERR_FILENO);
            close(devnull);
        }
        
        initscr();
        initialized_ = true;
        cbreak();
        noecho();
        keypad(stdscr, TRUE);
        nodelay(stdscr, TRUE);
        curs_set(0);
        set_escdelay(25);
        
        mousemask(ALL_MOUSE_EVENTS | REPORT_MOUSE_POSITION, NULL);
        mouseinterval(0);
        
        if (has_colors()) {
            start_color();
            use_default_colors();
            init_pair(1, COLOR_GREEN, -1);    // RX/good 
            init_pair(2, COLOR_RED, -1);      // TX/error
            init_pair(3, COLOR_YELLOW, -1);   // Warning 
            init_pair(4, COLOR_CYAN, -1);     // Important 
            init_pair(5, COLOR_WHITE, -1);    // Normal 
            init_pair(6, COLOR_MAGENTA, -1);  // Special
        }
        
        running_ = true;
        
        while (running_ && g_running) {
            int ch = getch();
            if (ch != ERR) {
                handle_input(ch);
            }
            tick_auto_send();
            draw();
            refresh();
            std::this_thread::sleep_for(std::chrono::milliseconds(33));
        }
        
        endwin();
        initialized_ = false;
        

        if (saved_stderr_ >= 0) {
            dup2(saved_stderr_, STDERR_FILENO);
            close(saved_stderr_);
            saved_stderr_ = -1;
        }
    }
    
private:
    enum Field {
        FIELD_CALLSIGN = 0,
        FIELD_MODEM_TYPE,
        FIELD_MODULATION,
        FIELD_CODERATE,
        FIELD_FRAMESIZE,
        FIELD_POSTAMBLE,
        FIELD_MFSK_MODE,
        FIELD_ROBUST_MODE,
        FIELD_ROBUST_MTU,
        FIELD_FREQ,
        FIELD_CSMA,
        FIELD_THRESHOLD,
        FIELD_CSMA_BAND,
        FIELD_CSMA_PRESET,
        FIELD_CSMA_ADV,
        FIELD_CSMA_QUIET,
        FIELD_CSMA_CW,
        FIELD_LEAD_TONE,
        FIELD_RESP_DITHER,
        FIELD_CSMA_BURST,
        FIELD_CSMA_INFO,
        FIELD_FRAGMENTATION,
        FIELD_TX_BLANKING,
        FIELD_AUDIO_INPUT,
        FIELD_AUDIO_OUTPUT,
        FIELD_PTT_TYPE,
        FIELD_VOX_FREQ,
        FIELD_VOX_LEAD,
        FIELD_VOX_TAIL,
        FIELD_COM_PORT,
        FIELD_COM_LINE,
        FIELD_COM_INVERT,
#ifdef WITH_CM108
        FIELD_CM108_GPIO,
        FIELD_CM108_DEVICE,
#endif
        FIELD_NET_PORT,
        FIELD_PRESET,
        FIELD_COUNT
    };

    enum RigField {
        RIG_FIELD_FREQ = 0,
        RIG_FIELD_STEP,
        RIG_FIELD_MODE,
        RIG_FIELD_POWER,
        RIG_FIELD_DRIVE,
        RIG_FIELD_TUNER,
        RIG_FIELD_TUNE,
        RIG_FIELD_COUNT
    };

    int tab_count() const {
        return state_.ptt_type_index == 1 ? 6 : 5;
    }

    bool rig_should_skip(int field) const {
        if (state_.rig_tuner_supported.load() == 0) {
            if (field == RIG_FIELD_TUNER || field == RIG_FIELD_TUNE) return true;
        }
        return false;
    }

    void handle_input(int ch) {
        if (ch == KEY_MOUSE) {
            MEVENT event;
            if (getmouse(&event) == OK) {
                handle_mouse(event);
            }
            return;
        }
        
        if (ch == KEY_F(1)) {
            show_help_ = !show_help_;
            return;
        }
        
        if (show_help_) {
            show_help_ = false;
            return;
        }
        if (show_csma_help_) {
            show_csma_help_ = false;
            return;
        }
        
        switch (ch) {
            case 'q':
            case 'Q':
                if (state_.on_stop_requested) {
                    state_.on_stop_requested();
                }
                running_ = false;
                break;
                
            case '\t':
                current_tab_ = (current_tab_ + 1) % tab_count();
                break;

            case KEY_BTAB:  // shift tab prev
                current_tab_ = (current_tab_ + tab_count() - 1) % tab_count();
                break;
                
            case KEY_UP:
            case 'k':
                if (current_tab_ == 1) {
                    do {
                        current_field_ = (current_field_ + FIELD_COUNT - 1) % FIELD_COUNT;
                    } while (should_skip_field(current_field_));
                } else if (current_tab_ == 2) {
                    if (log_scroll_ > 0) log_scroll_--;
                } else if (current_tab_ == 3) {
                    if (utils_selection_ == 0 && utils_scroll_ > 0) {
                        utils_scroll_--;
                    } else {
                        utils_selection_ = (utils_selection_ + utils_visible_slots() - 1)
                                         % utils_visible_slots();
                        utils_ensure_visible();
                    }
                } else if (current_tab_ == 5) {
                    do {
                        rig_field_ = (rig_field_ + RIG_FIELD_COUNT - 1) % RIG_FIELD_COUNT;
                    } while (rig_should_skip(rig_field_));
                } else if (current_tab_ == 0) {
                    recent_sel_ = recent_sel_ <= 0 ? 0 : recent_sel_ - 1;
                }
                break;
                
            case KEY_DOWN:
            case 'j':
                if (current_tab_ == 1) {
                    do {
                        current_field_ = (current_field_ + 1) % FIELD_COUNT;
                    } while (should_skip_field(current_field_));
                } else if (current_tab_ == 2) {
                    log_scroll_++;
                } else if (current_tab_ == 3) {
                    if (utils_selection_ == utils_visible_slots() - 1 &&
                        utils_scroll_ < utils_max_scroll_) {
                        utils_scroll_++;
                    } else {
                        utils_selection_ = (utils_selection_ + 1) % utils_visible_slots();
                        utils_ensure_visible();
                    }
                } else if (current_tab_ == 5) {
                    do {
                        rig_field_ = (rig_field_ + 1) % RIG_FIELD_COUNT;
                    } while (rig_should_skip(rig_field_));
                } else if (current_tab_ == 0) {
                    int n = (int)state_.get_recent_packets().size();
                    if (recent_sel_ < 0)
                        recent_sel_ = 0;
                    else if (recent_sel_ < n - 1)
                        recent_sel_++;
                }
                break;
                
            case KEY_LEFT:
            case 'h':
                if (current_tab_ == 1) {
                    if (current_field_ == FIELD_PRESET) {
                        if (!state_.presets.empty()) {
                            state_.selected_preset--;
                            if (state_.selected_preset < 0) {
                                state_.selected_preset = state_.presets.size() - 1;
                            }
                        }
                    } else if (current_field_ >= FIELD_MODEM_TYPE && current_field_ != FIELD_PRESET) {
                        adjust_field(-1);
                    }
                } else if (current_tab_ == 3 && (utils_selection_ == 0 || utils_selection_ == 1)) {
                    int step = 1;
                    if (state_.random_data_size >= 1000) step = 100;
                    else if (state_.random_data_size >= 100) step = 10;
                    state_.random_data_size = std::max(1, state_.random_data_size - step);
                } else if (current_tab_ == 5) {
                    adjust_rig_field(-1);
                }
                break;
                
            case KEY_RIGHT:
            case 'l':
                if (current_tab_ == 1) {
                    if (current_field_ == FIELD_PRESET) {
                        if (!state_.presets.empty()) {
                            state_.selected_preset++;
                            if (state_.selected_preset >= (int)state_.presets.size()) {
                                state_.selected_preset = 0;
                            }
                        }
                    } else if (current_field_ >= FIELD_MODEM_TYPE && current_field_ != FIELD_PRESET) {
                        adjust_field(1);
                    }
                } else if (current_tab_ == 3 && (utils_selection_ == 0 || utils_selection_ == 1)) {
                    int step = 1;
                    if (state_.random_data_size >= 1000) step = 100;
                    else if (state_.random_data_size >= 100) step = 10;
                    int max_size = state_.fragmentation_enabled ? 65535 : state_.mtu_bytes;
                    state_.random_data_size = std::min(max_size, state_.random_data_size + step);
                } else if (current_tab_ == 5) {
                    adjust_rig_field(1);
                }
                break;
                
            case KEY_PPAGE:
                if (current_tab_ == 2) log_scroll_ = std::max(0, log_scroll_ - 10);
                else if (current_tab_ == 3) utils_scroll_ = std::max(0, utils_scroll_ - 5);
                break;

            case KEY_NPAGE:
                if (current_tab_ == 2) log_scroll_ += 10;
                else if (current_tab_ == 3) utils_scroll_ += 5;
                break;
                
            case KEY_HOME:
                if (current_tab_ == 2) log_scroll_ = 0;
                break;
                
            case KEY_END:
                if (current_tab_ == 2) log_scroll_ = 999999;
                break;
                
            case '\n':
            case KEY_ENTER:
                if (current_tab_ == 1) {
                    if (current_field_ == FIELD_CSMA_INFO) {
                        show_csma_help_ = true;
                    } else if (current_field_ == FIELD_CSMA_ADV) {
                        state_.csma_advanced_open = !state_.csma_advanced_open;
                    } else if (current_field_ == FIELD_CALLSIGN) {


                        edit_text_field(FIELD_CALLSIGN);


                    } else if (current_field_ == FIELD_FREQ) {


                        edit_text_field(FIELD_FREQ);


                    } else if (current_field_ == FIELD_NET_PORT) {


                        edit_text_field(FIELD_NET_PORT);


                    } else if (current_field_ == FIELD_COM_PORT) {


                        edit_text_field(FIELD_COM_PORT);

#ifdef WITH_CM108
                    } else if (current_field_ == FIELD_CM108_GPIO) {
                        edit_text_field(FIELD_CM108_GPIO);
#endif

                    } else if (current_field_ == FIELD_AUDIO_INPUT) {


                        show_device_select_dialog(true);  


                    } else if (current_field_ == FIELD_AUDIO_OUTPUT) {


                        show_device_select_dialog(false);

#ifdef WITH_CM108
                    } else if (current_field_ == FIELD_CM108_DEVICE) {

                        show_cm108_device_dialog();
#endif

                    } else if (current_field_ == FIELD_PRESET) {


                        load_selected_preset();


                    }

                } else if (current_tab_ == 3) {


                    handle_utils_action();


                } else if (current_tab_ == 5) {

                    rig_enter_action();

                }
                break;

            // Preset field
            case 's': //
                if (current_tab_ == 1 && current_field_ == FIELD_PRESET) {

                    save_preset_dialog();

                }
                break;
            
            case KEY_DC:  
            case 'x':
                if (current_tab_ == 1 && current_field_ == FIELD_PRESET) {

                    delete_selected_preset();

                }
                break;
                
            // utils
            case '1':

                if (current_tab_ == 3) {

                    utils_selection_ = 0;
                    handle_utils_action();

                }
                break;

            case '2':

                if (current_tab_ == 3) {

                    utils_selection_ = 1;
                    handle_utils_action();

                }
                break;

            case '3':

                if (current_tab_ == 3) {

                    utils_selection_ = 2;
                    handle_utils_action();

                }
                break;

            case '4':

                if (current_tab_ == 3) {

                    utils_selection_ = 3;
                    handle_utils_action();

                }
                break;

            case '5':

                if (current_tab_ == 3) {

                    utils_selection_ = 4;
                    handle_utils_action();

                }
                break;

            case '6':

                if (current_tab_ == 3) {

                    utils_selection_ = 5;
                    handle_utils_action();

                }
                break;

            case '7':

                if (current_tab_ == 3) {

                    utils_selection_ = 6;
                    handle_utils_action();

                }
                break;

            case '8':

                if (current_tab_ == 3) {

                    state_.utils_testing_open = true;
                    utils_selection_ = UTILS_TOP_ACTIONS + 1;
                    handle_utils_action();

                }
                break;

        }
    }
    
    void handle_mouse(MEVENT& event) {
        int rows, cols;
        getmaxyx(stdscr, rows, cols);
        (void)rows;  
        
        if (event.bstate & BUTTON1_CLICKED || event.bstate & BUTTON1_PRESSED) {
            // Tab clicks
            if (event.y == 2) {
                int ntabs = tab_count();
                int tab_width = (cols - 4) / ntabs;
                if (event.x >= 2 && tab_width > 0) {
                    int idx = (event.x - 2) / tab_width;
                    if (idx >= ntabs) idx = ntabs - 1;
                    current_tab_ = idx;
                }
            }
            
            if (current_tab_ == 1 && event.x < cols/2 - 2) {
                int logical = event.y - 4 + config_scroll_;
                int field = -1;
                for (int f = 0; f < FIELD_COUNT; f++) {
                    if (should_skip_field(f)) continue;
                    if (config_field_row(f) == logical) { field = f; break; }
                }
                if (field == FIELD_CSMA_INFO)
                    show_csma_help_ = true;
                
                if (field >= 0 && field < FIELD_COUNT) {
                    current_field_ = field;
                    
                    // Handle clicks on interactive elements
                    if (field == FIELD_PRESET) {
                        // Click on preset - determine action by position
                        if (event.x >= 18 && event.x < 22 && !state_.presets.empty()) {
                            // Left arrow
                            state_.selected_preset--;
                            if (state_.selected_preset < 0) 
                                state_.selected_preset = state_.presets.size() - 1;
                        } else if (event.x >= 22 && event.x < 38 && !state_.presets.empty()) {
                            // Name area - load on click
                            load_selected_preset();
                        } else if (event.x >= 38 && !state_.presets.empty()) {
                            // Right arrow area
                            state_.selected_preset++;
                            if (state_.selected_preset >= (int)state_.presets.size())
                                state_.selected_preset = 0;
                        }
                    } else if (event.x >= 18) {
                        // Value area clicks for other fields
                        if (field == FIELD_CALLSIGN || field == FIELD_FREQ) {
                            edit_text_field(field);
                        } else if (field >= FIELD_MODULATION) {
                            if (event.x < 22) adjust_field(-1);
                            else adjust_field(1);
                        }
                    }
                }
            }
        }
        
        // Scroll wheel in log
        if (current_tab_ == 2) {
            if (event.bstate & BUTTON4_PRESSED) {
                if (log_scroll_ > 0) log_scroll_--;
            } else if (event.bstate & BUTTON5_PRESSED) {
                log_scroll_++;
            }
        }

        if (current_tab_ == 3) {
            if (event.bstate & BUTTON4_PRESSED) {
                if (utils_scroll_ > 0) utils_scroll_--;
            } else if (event.bstate & BUTTON5_PRESSED) {
                utils_scroll_++;
            }
        }
    }
    
    // line input at (y, x). returns true on enter, false if cancelled with esc.
    // buf must hold at least max_len + 1 bytes.
    bool prompt_input(int y, int x, char* buf, int max_len) {
        curs_set(1);
        nodelay(stdscr, FALSE);

        int len = (int)strlen(buf);
        bool accepted = false;

        while (true) {
            mvhline(y, x, ' ', max_len);
            mvaddnstr(y, x, buf, len);
            move(y, x + len);
            refresh();

            int ch = getch();
            if (ch == 27) {
                break;
            } else if (ch == '\n' || ch == KEY_ENTER) {
                accepted = true;
                break;
            } else if (ch == KEY_BACKSPACE || ch == 127 || ch == 8) {
                if (len > 0) buf[--len] = '\0';
            } else if (ch >= 32 && ch < 127 && len < max_len) {
                buf[len++] = (char)ch;
                buf[len] = '\0';
            }
        }

        nodelay(stdscr, TRUE);
        curs_set(0);
        return accepted;
    }

    void edit_text_field(int field) {
        // MODEM:4, Callsign:5, Mod:6, Rate:7, Frame:8, Freq:9
        // CSMA:11, Enabled:12, Thresh:13, Persist:14
        // AUDIO:16, Input:17, Output:18, PTT:19
        // VOX:20-21 (if PTT=VOX), COM:20-22 (if PTT=COM)
        
        int row = -1;
        int col = 16;
        int max_len = 10;
        
        if (field == FIELD_CALLSIGN) {
            row = 5;
            max_len = 10;
        } else if (field == FIELD_FREQ) {
            row = 9;
            max_len = 6;
        } else if (field == FIELD_COM_PORT) {
            row = 20;  
            max_len = 20;
#ifdef WITH_CM108
        } else if (field == FIELD_CM108_GPIO) {
            row = 20;  
            max_len = 1;
#endif
        } else if (field == FIELD_NET_PORT) {
            if (state_.ptt_type_index == 2) {  //2 extra rows
                row = 24;
            } else if (state_.ptt_type_index == 3) {  
                row = 25;
            } else {
                row = 22;  
            }
            max_len = 5;
        } else {
            return; // not text editable
        }
        
        // Clear the value area
        move(row, col);
        for (int i = 0; i < 20; i++) addch(' ');

        char buf[64] = {0};
        if (!prompt_input(row, col, buf, max_len)) return;

        if (strlen(buf) > 0) {
            if (field == FIELD_CALLSIGN) {
                for (char* p = buf; *p; p++) *p = toupper(*p);
                state_.callsign = buf;
                apply_settings();
            } else if (field == FIELD_FREQ) {
                try {
                    int freq = std::stoi(buf);
                    if (freq >= 300 && freq <= 3000) {
                        state_.center_freq = freq;
                        apply_settings();
                    }
                } catch (...) {}
            } else if (field == FIELD_COM_PORT) {
                state_.com_port = buf;
                state_.add_log("(!) COM port changed, restart required");
                apply_settings();
#ifdef WITH_CM108
            } else if (field == FIELD_CM108_GPIO) {
                try {
                    int gpio = std::stoi(buf);
                    if (gpio >= 1 && gpio <= 4) {
                        state_.cm108_gpio = gpio;
                        apply_settings();
                    }
                } catch (...) {}
#endif
            } else if (field == FIELD_NET_PORT) {
                try {
                    int port = std::stoi(buf);
                    if (port >= 1024 && port <= 65535) {
                        state_.port = port;
                        state_.add_log("(!) Port changed, restart required");
                        apply_settings();
                    }
                } catch (...) {}
            }
        }
    }

    void compose_message() {
        int rows, cols;
        getmaxyx(stdscr, rows, cols);
        (void)cols;

        move(rows - 3, 0);
        clrtoeol();
        attron(A_BOLD);
        mvaddstr(rows - 3, 2, "Test msg> ");
        attroff(A_BOLD);

        char buf[TNCUIState::MAX_MESSAGE_CHARS + 1] = {0};
        if (!prompt_input(rows - 3, 12, buf, TNCUIState::MAX_MESSAGE_CHARS)) return;

        std::string text(buf);
        if (text.empty()) return;

        std::string payload = "M73:" + state_.callsign + ":" + text;
        if (!state_.fragmentation_enabled && (int)payload.size() > state_.mtu_bytes) {
            state_.add_log("MSG too long for mode (" + std::to_string(payload.size()) +
                           " > " + std::to_string(state_.mtu_bytes) + "B), enable fragmentation");
            return;
        }
        if (state_.on_send_data) {
            state_.on_send_data(std::vector<uint8_t>(payload.begin(), payload.end()));
            state_.add_message(state_.callsign, text, true);
        }
    }

    bool should_skip_field(int field) {
        // Hide OFDM-only fields when in MFSK mode
        if (state_.modem_type_index != 0) {
            if (field == FIELD_MODULATION || field == FIELD_CODERATE ||
                field == FIELD_FRAMESIZE || field == FIELD_POSTAMBLE)
                return true;
        }
        if (state_.modem_type_index != 1 && field == FIELD_MFSK_MODE) return true;
        if (state_.modem_type_index != 2 &&
            (field == FIELD_ROBUST_MODE || field == FIELD_ROBUST_MTU)) return true;
        if (state_.ptt_type_index != 2) {  // not VOX
            if (field == FIELD_VOX_FREQ || field == FIELD_VOX_LEAD || field == FIELD_VOX_TAIL) {
                return true;
            }
        }
        if (state_.ptt_type_index != 3) {  // not COM
            if (field == FIELD_COM_PORT || field == FIELD_COM_LINE || field == FIELD_COM_INVERT) {
                return true;
            }
        }
#ifdef WITH_CM108
        if (state_.ptt_type_index != 4) {  // not CM108
            if (field == FIELD_CM108_GPIO || field == FIELD_CM108_DEVICE) {
                return true;
            }
        }
#endif
        if (!state_.csma_advanced_open) {
            if (field == FIELD_CSMA_QUIET || field == FIELD_CSMA_CW ||
                field == FIELD_LEAD_TONE || field == FIELD_RESP_DITHER ||
                field == FIELD_CSMA_BURST)
                return true;
        }
        return false;
    }
    

    // must mirror draw_config's order/spacing exactly -- a drifted map
    // scrolls the selection off-screen and breaks mouse click mapping
    int config_field_row(int field) const {
        int row = 0;
        row++;
        if (field == FIELD_CALLSIGN) return row;
        row++;
        if (field == FIELD_MODEM_TYPE) return row;
        row++;
        if (state_.modem_type_index == 0) {
            if (field == FIELD_MODULATION) return row;
            row++;
            if (field == FIELD_CODERATE) return row;
            row++;
            if (field == FIELD_FRAMESIZE) return row;
            row++;
            if (field == FIELD_POSTAMBLE) return row;
            row++;
        } else if (state_.modem_type_index == 1) {
            if (field == FIELD_MFSK_MODE) return row;
            row++;
        } else {
            if (field == FIELD_ROBUST_MODE) return row;
            row++;
            if (field == FIELD_ROBUST_MTU) return row;
            row++;
        }
        if (field == FIELD_FREQ) return row;
        row += 2;
        row++;
        if (field == FIELD_CSMA) return row;
        row++;
        if (field == FIELD_THRESHOLD) return row;
        row++;
        row++;
        if (field == FIELD_CSMA_BAND) return row;
        row++;
        if (field == FIELD_CSMA_PRESET) return row;
        row++;
        if (field == FIELD_CSMA_ADV) return row;
        row++;
        if (state_.csma_advanced_open) {
            if (field == FIELD_CSMA_QUIET) return row;
            row++;
            if (field == FIELD_CSMA_CW) return row;
            row++;
            if (field == FIELD_LEAD_TONE) return row;
            row++;
            if (field == FIELD_RESP_DITHER) return row;
            row++;
            if (field == FIELD_CSMA_BURST) return row;
            row++;
        }
        if (field == FIELD_CSMA_INFO) return row;
        row += 2;
        row += 2;
        if (field == FIELD_FRAGMENTATION) return row;
        row += 2;
        row++;
        if (field == FIELD_TX_BLANKING) return row;
        row += 2;
        row++;
        if (field == FIELD_AUDIO_INPUT) return row;
        row++;
        if (field == FIELD_AUDIO_OUTPUT) return row;
        row++;
        if (field == FIELD_PTT_TYPE) return row;
        row++;
        if (state_.ptt_type_index == 2) {
            if (field == FIELD_VOX_FREQ) return row;
            row++;
            if (field == FIELD_VOX_LEAD) return row;
            row++;
            if (field == FIELD_VOX_TAIL) return row;
            row++;
        }
        if (state_.ptt_type_index == 3) {
            if (field == FIELD_COM_PORT) return row;
            row++;
            if (field == FIELD_COM_LINE) return row;
            row++;
            if (field == FIELD_COM_INVERT) return row;
            row++;
        }
#ifdef WITH_CM108
        if (state_.ptt_type_index == 4) {
            if (field == FIELD_CM108_GPIO) return row;
            row++;
            if (field == FIELD_CM108_DEVICE) return row;
            row++;
        }
#endif
        row++;
        row++;
        if (field == FIELD_NET_PORT) return row;
        row += 2;
        row++;
        if (field == FIELD_PRESET) return row;
        return row;
    }

    int csma_preset_match() const {
        for (int i = 0; i < CSMA_PRESET_COUNT; ++i) {
            const CsmaPreset& p = CSMA_PRESETS[state_.csma_band & 1][i];
            if (state_.csma_quiet_ms == p.quiet_ms &&
                state_.csma_cw == p.cw &&
                state_.slot_time_ms == p.slot_ms &&
                state_.csma_burst == p.burst &&
                state_.csma_responder_dither == p.dither &&
                state_.tx_lead_tone == p.lead_tone)
                return i;
        }
        return -1;
    }

    void csma_apply_preset(int idx) {
        const CsmaPreset& p = CSMA_PRESETS[state_.csma_band & 1][idx];
        state_.csma_quiet_ms = p.quiet_ms;
        state_.csma_cw = p.cw;
        state_.slot_time_ms = p.slot_ms;
        state_.csma_burst = p.burst;
        state_.csma_responder_dither = p.dither;
        state_.tx_lead_tone = p.lead_tone;
        state_.add_log(std::string("CSMA preset: ") +
                       CSMA_BAND_NAMES[state_.csma_band & 1] + " " + p.name);
    }

    void adjust_field(int delta) {
        switch (current_field_) {
            case FIELD_MODEM_TYPE:
                state_.modem_type_index = (state_.modem_type_index + delta + 3) % 3;
                state_.update_modem_info();
                break;
            case FIELD_ROBUST_MODE: {
                int n = (int)ROBUST_MODE_OPTIONS.size();
                int base = (robust_base_index(state_.robust_mode_index) + delta + n) % n;
                state_.robust_mode_index = robust_mode_of(base,
                    RobustParams::is_short((RobustMode)state_.robust_mode_index));
                state_.update_modem_info();
                break;
            }
            case FIELD_ROBUST_MTU:
                state_.robust_mode_index = robust_mode_of(
                    robust_base_index(state_.robust_mode_index),
                    !RobustParams::is_short((RobustMode)state_.robust_mode_index));
                state_.update_modem_info();
                break;
            case FIELD_MFSK_MODE:
                state_.mfsk_mode_index = (state_.mfsk_mode_index + delta + 4) % 4;
                state_.update_modem_info();
                break;
            case FIELD_MODULATION:
                state_.modulation_index = (state_.modulation_index + delta + 8) % 8;
                break;
            case FIELD_CODERATE:
                do {
                    state_.code_rate_index = (state_.code_rate_index + delta +
                        (int)CODE_RATE_OPTIONS.size()) % (int)CODE_RATE_OPTIONS.size();
                } while (state_.code_rate_index >= 5);
                break;
            case FIELD_FRAMESIZE:
                state_.frame_size = (state_.frame_size + delta + 3) % 3;
                break;
            case FIELD_POSTAMBLE:
                state_.postamble = !state_.postamble;
                break;
            case FIELD_CSMA:
                state_.csma_enabled = !state_.csma_enabled;
                break;
            case FIELD_THRESHOLD:
                state_.carrier_threshold_db += delta * 2;
                state_.carrier_threshold_db = std::max(-80.0f, std::min(0.0f, state_.carrier_threshold_db));
                break;
            case FIELD_CSMA_BAND: {
                int matched = csma_preset_match();
                state_.csma_band = (state_.csma_band + delta + 2) % 2;
                if (matched >= 0)
                    csma_apply_preset(matched);
                break;
            }
            case FIELD_CSMA_PRESET: {
                int idx = csma_preset_match();
                idx = idx < 0 ? (delta > 0 ? 0 : CSMA_PRESET_COUNT - 1)
                              : (idx + delta + CSMA_PRESET_COUNT) % CSMA_PRESET_COUNT;
                csma_apply_preset(idx);
                break;
            }
            case FIELD_CSMA_ADV:
                state_.csma_advanced_open = !state_.csma_advanced_open;
                return;
            case FIELD_CSMA_QUIET:
                state_.csma_quiet_ms += delta * 250;
                state_.csma_quiet_ms = std::max(0, std::min(10000, state_.csma_quiet_ms));
                break;
            case FIELD_CSMA_CW:
                state_.csma_cw += delta;
                state_.csma_cw = std::max(2, std::min(32, state_.csma_cw));
                break;
            case FIELD_LEAD_TONE:
                state_.tx_lead_tone = !state_.tx_lead_tone;
                break;
            case FIELD_RESP_DITHER:
                state_.csma_responder_dither += delta * 100;
                state_.csma_responder_dither = std::max(0, std::min(3000, state_.csma_responder_dither));
                break;
            case FIELD_CSMA_BURST:
                state_.csma_burst += delta;
                state_.csma_burst = std::max(1, std::min(4, state_.csma_burst));
                break;
            case FIELD_FRAGMENTATION:
                state_.fragmentation_enabled = !state_.fragmentation_enabled;
                state_.update_modem_info();  // Update random_data_size limits
                state_.add_log("(!) Fragmentation changed, restart required");
                break;
            case FIELD_TX_BLANKING:
                state_.tx_blanking_enabled = !state_.tx_blanking_enabled;
                apply_settings();
                state_.add_log(std::string("TX blanking ") + (state_.tx_blanking_enabled ? "enabled" : "disabled"));
                break;
            case FIELD_AUDIO_INPUT:
                break;
            case FIELD_AUDIO_OUTPUT:
                break;
            case FIELD_PTT_TYPE:
#ifdef WITH_CM108
                state_.ptt_type_index = (state_.ptt_type_index + delta + 5) % 5;
#else
                state_.ptt_type_index = (state_.ptt_type_index + delta + 4) % 4;
#endif
                break;
            case FIELD_VOX_FREQ:
                state_.vox_tone_freq += delta * 100;
                state_.vox_tone_freq = std::max(300, std::min(2500, state_.vox_tone_freq));
                break;
            case FIELD_VOX_LEAD:
                state_.vox_lead_ms += delta * 50;
                state_.vox_lead_ms = std::max(50, std::min(2000, state_.vox_lead_ms));
                break;
            case FIELD_VOX_TAIL:
                state_.vox_tail_ms += delta * 50;
                state_.vox_tail_ms = std::max(50, std::min(2000, state_.vox_tail_ms));
                break;
            case FIELD_COM_PORT:
                break;
            case FIELD_COM_LINE:
                state_.com_ptt_line = (state_.com_ptt_line + delta + 3) % 3;
                break;
            case FIELD_COM_INVERT:
                if (delta > 0) {
                    if (!state_.com_invert_dtr && !state_.com_invert_rts) {
                        state_.com_invert_dtr = true;
                    } else if (state_.com_invert_dtr && !state_.com_invert_rts) {
                        state_.com_invert_dtr = false;
                        state_.com_invert_rts = true;
                    } else if (!state_.com_invert_dtr && state_.com_invert_rts) {
                        state_.com_invert_dtr = true;
                    } else {
                        state_.com_invert_dtr = false;
                        state_.com_invert_rts = false;
                    }
                } else {
                    if (!state_.com_invert_dtr && !state_.com_invert_rts) {
                        state_.com_invert_dtr = true;
                        state_.com_invert_rts = true;
                    } else if (state_.com_invert_dtr && state_.com_invert_rts) {
                        state_.com_invert_dtr = false;
                    } else if (!state_.com_invert_dtr && state_.com_invert_rts) {
                        state_.com_invert_rts = false;
                        state_.com_invert_dtr = true;
                    } else {
                        state_.com_invert_dtr = false;
                    }
                }
                break;
            case FIELD_NET_PORT:
                state_.port += delta * 1;
                state_.port = std::max(1024, std::min(65535, state_.port));
                break;
            default:
                return;
        }
        apply_settings();
    }
    
    void apply_settings() {

        state_.update_modem_info();
        

        if (state_.on_settings_changed) {
            state_.on_settings_changed(state_);
        }
        
        
        state_.save_settings();
    }
    
    void show_device_select_dialog(bool is_input) {
        int rows, cols;
        getmaxyx(stdscr, rows, cols);
        
        const std::vector<std::string>& devices = is_input ? 
            state_.available_input_devices : state_.available_output_devices;
        const std::vector<std::string>& descriptions = is_input ?
            state_.input_device_descriptions : state_.output_device_descriptions;
        int& current_index = is_input ? state_.audio_input_index : state_.audio_output_index;
        std::string& current_device = is_input ? state_.audio_input_device : state_.audio_output_device;
        
        if (devices.empty()) {
            state_.add_log("No audio devices found");
            return;
        }
        
        // Dialog dimensions
        int dialog_w = std::min(cols - 4, 58);
        int max_visible = std::min((int)devices.size(), 12);
        int dialog_h = max_visible + 3;
        int dialog_x = (cols - dialog_w) / 2;
        int dialog_y = (rows - dialog_h) / 2;
        
        int selection = current_index;
        int scroll_offset = 0;
        
        if (selection >= max_visible) {
            scroll_offset = selection - max_visible + 1;
        }
        
        nodelay(stdscr, FALSE);
        
        while (true) {
            // Clear dialog area
            for (int y = dialog_y; y < dialog_y + dialog_h; y++) {
                move(y, dialog_x);
                for (int x = 0; x < dialog_w; x++) addch(' ');
            }
            
            // Draw box
            attron(COLOR_PAIR(4) | A_BOLD);
            draw_box(dialog_y, dialog_x, dialog_h, dialog_w);
            attroff(COLOR_PAIR(4) | A_BOLD);
            
            // Title
            const char* title = is_input ? " Input Device " : " Output Device ";
            attron(COLOR_PAIR(4) | A_BOLD);
            mvaddstr(dialog_y, dialog_x + (dialog_w - strlen(title)) / 2, title);
            attroff(COLOR_PAIR(4) | A_BOLD);
            
            // Draw device list
            int visible_count = std::min((int)devices.size() - scroll_offset, max_visible);
            for (int i = 0; i < visible_count; i++) {
                int dev_idx = scroll_offset + i;
                int y = dialog_y + 1 + i;
                
                mvhline(y, dialog_x + 1, ' ', dialog_w - 2);
                
                if (dev_idx == selection) {
                    attron(COLOR_PAIR(4) | A_BOLD);
                    mvaddstr(y, dialog_x + 1, "> ");
                } else {
                    mvaddstr(y, dialog_x + 1, "  ");
                }
                
                std::string desc = (dev_idx < (int)descriptions.size()) ? 
                    descriptions[dev_idx] : devices[dev_idx];
                int max_len = dialog_w - 4;
                if ((int)desc.length() > max_len) {
                    desc = desc.substr(0, max_len - 2) + "..";
                }
                addstr(desc.c_str());
                
                if (dev_idx == selection) {
                    attroff(COLOR_PAIR(4) | A_BOLD);
                }
            }
            
            // Scroll indicators
            if (scroll_offset > 0) {
                attron(A_DIM);
                mvaddstr(dialog_y, dialog_x + dialog_w - 3, "^");
                attroff(A_DIM);
            }
            if (scroll_offset + max_visible < (int)devices.size()) {
                attron(A_DIM);
                mvaddstr(dialog_y + dialog_h - 1, dialog_x + dialog_w - 3, "v");
                attroff(A_DIM);
            }
            
            // Help
            attron(A_DIM);
            mvaddstr(dialog_y + dialog_h - 1, dialog_x + 2, " Enter=OK  Esc=Cancel ");
            mvaddstr(dialog_y + dialog_h - 1, dialog_x + dialog_w - 15, "(needs restart)");
            attroff(A_DIM);
            
            refresh();
            
            int ch = getch();
            
            if (ch == 27 || ch == 'q') {
                break;
            } else if (ch == '\n' || ch == KEY_ENTER) {
                if (selection >= 0 && selection < (int)devices.size()) {
                    current_index = selection;
                    current_device = devices[selection];
                    state_.add_log(std::string(is_input ? "In: " : "Out: ") + 
                                   descriptions[selection] + " (restart to apply)");
                    apply_settings();
                }
                break;
            } else if (ch == KEY_UP || ch == 'k') {
                if (selection > 0) {
                    selection--;
                    if (selection < scroll_offset) scroll_offset = selection;
                }
            } else if (ch == KEY_DOWN || ch == 'j') {
                if (selection < (int)devices.size() - 1) {
                    selection++;
                    if (selection >= scroll_offset + max_visible) {
                        scroll_offset = selection - max_visible + 1;
                    }
                }
            } else if (ch == KEY_PPAGE) {
                selection = std::max(0, selection - max_visible);
                scroll_offset = std::max(0, scroll_offset - max_visible);
            } else if (ch == KEY_NPAGE) {
                selection = std::min((int)devices.size() - 1, selection + max_visible);
                if (selection >= scroll_offset + max_visible) {
                    scroll_offset = selection - max_visible + 1;
                }
            }
        }
        
        nodelay(stdscr, TRUE);
    }

#ifdef WITH_CM108
    void show_cm108_device_dialog() {
        int rows, cols;
        getmaxyx(stdscr, rows, cols);

        auto devices = CM108PTT::enumerate();



        std::vector<std::string> values;
        std::vector<std::string> descriptions;
        
        values.push_back("");
        descriptions.push_back("Auto (first compatible device)");
        for (const auto& d : devices) {
            std::string desc = d.chip;
            if (!d.product.empty()) desc += " - " + d.product;
            if (!d.serial.empty()) {
                values.push_back(d.serial);
                desc += " (sn " + d.serial + ")";
            } else {
                values.push_back(d.path);
                desc += " (" + d.path + ")";
            }
            descriptions.push_back(desc);
        }

        int selection = 0;
        for (int i = 1; i < (int)values.size(); i++) {
            if (values[i] == state_.cm108_device) { selection = i; break; }
        }





        if (selection == 0 && !state_.cm108_device.empty()) {
            values.push_back(state_.cm108_device);
            descriptions.push_back(state_.cm108_device + " (not connected)");
            selection = (int)values.size() - 1; 
        }


        int dialog_w = std::min(cols - 4, 58);
        int max_visible = std::min((int)values.size(), 12);
        int dialog_h = max_visible + 3;
        int dialog_x = (cols - dialog_w) / 2;
        int dialog_y = (rows - dialog_h) / 2;

        int scroll_offset = 0;
        if (selection >= max_visible) {
            scroll_offset = selection - max_visible + 1;
        }


        nodelay(stdscr, FALSE);

        while (true) {
            for (int y = dialog_y; y < dialog_y + dialog_h; y++) {
                move(y, dialog_x);
                for (int x = 0; x < dialog_w; x++) addch(' ');
            }

            attron(COLOR_PAIR(4) | A_BOLD);
            draw_box(dialog_y, dialog_x, dialog_h, dialog_w);
            attroff(COLOR_PAIR(4) | A_BOLD);

            const char* title = " CM108 Device ";
            attron(COLOR_PAIR(4) | A_BOLD);
            mvaddstr(dialog_y, dialog_x + (dialog_w - strlen(title)) / 2, title);
            attroff(COLOR_PAIR(4) | A_BOLD);

            int visible_count = std::min((int)values.size() - scroll_offset, max_visible);
            for (int i = 0; i < visible_count; i++) {
                int dev_idx = scroll_offset + i;
                int y = dialog_y + 1 + i;

                mvhline(y, dialog_x + 1, ' ', dialog_w - 2);

                if (dev_idx == selection) {
                    attron(COLOR_PAIR(4) | A_BOLD);
                    mvaddstr(y, dialog_x + 1, "> ");
                } else {
                    mvaddstr(y, dialog_x + 1, "  ");
                }

                std::string desc = descriptions[dev_idx];
                int max_len = dialog_w - 4;
                if ((int)desc.length() > max_len) {
                    desc = desc.substr(0, max_len - 2) + "..";
                }
                addstr(desc.c_str());

                if (dev_idx == selection) {
                    attroff(COLOR_PAIR(4) | A_BOLD);
                }
            }

            if (scroll_offset > 0) {
                attron(A_DIM);
                mvaddstr(dialog_y, dialog_x + dialog_w - 3, "^");
                attroff(A_DIM);
            }
            if (scroll_offset + max_visible < (int)values.size()) {
                attron(A_DIM);
                mvaddstr(dialog_y + dialog_h - 1, dialog_x + dialog_w - 3, "v");
                attroff(A_DIM);
            }

            attron(A_DIM);
            mvaddstr(dialog_y + dialog_h - 1, dialog_x + 2, " Enter=OK  Esc=Cancel ");
            mvaddstr(dialog_y + dialog_h - 1, dialog_x + dialog_w - 15, "(needs restart)");
            attroff(A_DIM);

            refresh();

            int ch = getch();

            if (ch == 27 || ch == 'q') {
                break;
            } else if (ch == '\n' || ch == KEY_ENTER) {
                if (selection >= 0 && selection < (int)values.size()) {
                    state_.cm108_device = values[selection];
                    state_.add_log("CM108: " + descriptions[selection] + " (restart to apply)");
                    apply_settings();
                }
                break;
            } else if (ch == KEY_UP || ch == 'k') {
                if (selection > 0) {
                    selection--;
                    if (selection < scroll_offset) scroll_offset = selection;
                }
            } else if (ch == KEY_DOWN || ch == 'j') {
                if (selection < (int)values.size() - 1) {
                    selection++;
                    if (selection >= scroll_offset + max_visible) {
                        scroll_offset = selection - max_visible + 1;
                    }
                }
            } else if (ch == KEY_PPAGE) {
                selection = std::max(0, selection - max_visible);
                scroll_offset = std::max(0, scroll_offset - max_visible);
            } else if (ch == KEY_NPAGE) {
                selection = std::min((int)values.size() - 1, selection + max_visible);
                if (selection >= scroll_offset + max_visible) {
                    scroll_offset = selection - max_visible + 1;
                }
            }
        }

        nodelay(stdscr, TRUE);
    }
#endif

    void save_preset_dialog() {
        int rows, cols;
        getmaxyx(stdscr, rows, cols);
        
        // Check if we have room for more presets
        if (state_.presets.size() >= TNCUIState::MAX_PRESETS) {
            state_.add_log("Cannot save: maximum presets reached");
            return;
        }
        
        int dialog_w = 40;
        int dialog_h = 5;
        int dialog_x = (cols - dialog_w) / 2;
        int dialog_y = (rows - dialog_h) / 2;
        
        attron(A_BOLD);
        draw_box(dialog_y, dialog_x, dialog_h, dialog_w);
        attroff(A_BOLD);
        
        mvaddstr(dialog_y, dialog_x + 2, " Save Preset ");
        mvaddstr(dialog_y + 2, dialog_x + 2, "Name: ");

        char buf[32] = {0};
        if (!prompt_input(dialog_y + 2, dialog_x + 8, buf, 24)) return;

        if (strlen(buf) > 0) {
            // replace any commas with underscores, commas are the delimiter
            for (char* p = buf; *p; p++) {
                if (*p == ',') *p = '_';
            }
            
            if (state_.create_preset(buf)) {
                state_.selected_preset = state_.presets.size() - 1;
                state_.add_log("Preset saved: " + std::string(buf));
            } else {
                state_.add_log("Failed to save preset");
            }
        }
    }
    
    void load_selected_preset() {
        if (state_.selected_preset < 0 || state_.selected_preset >= (int)state_.presets.size()) {
            state_.add_log("No preset selected");
            return;
        }
        
        if (state_.apply_preset(state_.selected_preset)) {
            state_.loaded_preset_index = state_.selected_preset; 
            apply_settings();
            state_.add_log("Loaded preset: " + state_.presets[state_.selected_preset].name);
        }
    }
    
    void delete_selected_preset() {
        if (state_.selected_preset < 0 || state_.selected_preset >= (int)state_.presets.size()) {
            state_.add_log("No preset selected");
            return;
        }
        
        std::string name = state_.presets[state_.selected_preset].name;
        int deleted_index = state_.selected_preset;
        if (state_.delete_preset(state_.selected_preset)) {
            state_.add_log("Deleted preset: " + name);
            if (state_.loaded_preset_index == deleted_index) {
                state_.loaded_preset_index = -1;  
            } else if (state_.loaded_preset_index > deleted_index) {
                state_.loaded_preset_index--;  
            }
        }
    }
    
    void draw_box(int y, int x, int h, int w) {
        // corners
        mvaddch(y, x, ACS_ULCORNER);
        mvaddch(y, x + w - 1, ACS_URCORNER);
        mvaddch(y + h - 1, x, ACS_LLCORNER);
        mvaddch(y + h - 1, x + w - 1, ACS_LRCORNER);
        
        // horizontal lines
        mvhline(y, x + 1, ACS_HLINE, w - 2);
        mvhline(y + h - 1, x + 1, ACS_HLINE, w - 2);
        
        // vertical lines
        mvvline(y + 1, x, ACS_VLINE, h - 2);
        mvvline(y + 1, x + w - 1, ACS_VLINE, h - 2);
    }
    
    void draw_hline(int y, int x, int w, bool connect_left = false, bool connect_right = false) {
        mvaddch(y, x, connect_left ? ACS_LTEE : ACS_HLINE);
        mvhline(y, x + 1, ACS_HLINE, w - 2);
        mvaddch(y, x + w - 1, connect_right ? ACS_RTEE : ACS_HLINE);
    }
    
    void draw() {
        frame_counter_++;
        update_calibration();

        if (current_tab_ >= tab_count()) current_tab_ = 0;
        {
            int64_t now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
            if (state_.ptt_on.load() || state_.transmitting.load())
                last_ptt_seen_ms_ = now_ms;
            state_.rig_poll_enabled = (current_tab_ == 5) || (current_tab_ == 0) ||
                (last_ptt_seen_ms_ > 0 && now_ms - last_ptt_seen_ms_ < 3000);
            if (now_ms - occ_hist_ms_ >= 5000) {
                occ_hist_ms_ = now_ms;
                occ_hist_[occ_hist_pos_] = state_.channel_occupancy.load();
                occ_hist_pos_ = (occ_hist_pos_ + 1) % OCC_HIST_SIZE;
                if (occ_hist_n_ < OCC_HIST_SIZE)
                    occ_hist_n_++;
            }
        }
        if (current_tab_ == 5 && rig_should_skip(rig_field_)) rig_field_ = RIG_FIELD_FREQ;

        int rows, cols;
        getmaxyx(stdscr, rows, cols);
        erase();
        

        attron(A_DIM);
        draw_box(0, 0, rows, cols);
        attroff(A_DIM);
        
        // title
        mvaddstr(0, 2, " ");
        attron(A_DIM);
        addstr("/ / / ");
        attroff(A_DIM);
        attron(A_BOLD);
        addstr("MODEM73");
        attroff(A_BOLD);
        addstr(" ");
        
        // PTT status 
        attron(A_DIM);
        addch(ACS_VLINE);
        attroff(A_DIM);
        if (state_.ptt_on || state_.transmitting) {
            attron(COLOR_PAIR(2) | A_BOLD);
            addstr(" TX ");
            attroff(COLOR_PAIR(2) | A_BOLD);
        } else {
            attron(COLOR_PAIR(1) | A_BOLD);
            addstr(" RX ");
            attroff(COLOR_PAIR(1) | A_BOLD);
        }
        attron(A_DIM);
        addch(ACS_VLINE);
        attroff(A_DIM);

        // high-SWR warning chip, visible from every tab
        if (state_.swr_warn_value.load() > 0.0f) {
            attron(COLOR_PAIR(2) | A_BOLD);
            printw(" !SWR %.1f ", state_.swr_warn_value.load());
            attroff(COLOR_PAIR(2) | A_BOLD);
            attron(A_DIM);
            addch(ACS_VLINE);
            attroff(A_DIM);
        }

        // Mode
        addstr(" ");
        attron(A_BOLD);
        addstr(state_.callsign.c_str());
        attroff(A_BOLD);
        if (state_.modem_type_index == 1) {
            printw("  %s %dHz",
                   MFSK_MODE_OPTIONS[state_.mfsk_mode_index].c_str(),
                   state_.center_freq);
        } else if (state_.modem_type_index == 2) {
            printw("  %s %s %dHz",
                   ROBUST_MODE_OPTIONS[robust_base_index(state_.robust_mode_index)].c_str(),
                   RobustParams::is_short((RobustMode)state_.robust_mode_index)
                       ? "170B" : "510B",
                   state_.center_freq);
        } else {
            printw("  %s %s %s %dHz",
                   MODULATION_OPTIONS[state_.modulation_index].c_str(),
                   CODE_RATE_OPTIONS[state_.code_rate_index].c_str(),
                   state_.frame_size == 0 ? "S" : state_.frame_size == 2 ? "L" : "N",
                   state_.center_freq);
        }
        
        // Stats 
        {
            auto now = std::chrono::system_clock::now();
            auto t = std::chrono::system_clock::to_time_t(now);
            struct tm lt;
            localtime_r(&t, &lt);
            attron(A_BOLD);
            mvprintw(0, cols - 31, "%02d:%02d:%02d", lt.tm_hour, lt.tm_min, lt.tm_sec);
            attroff(A_BOLD);
            attron(A_DIM);
            addstr(" |");
            attroff(A_DIM);
        }

        int rx = cols - 20;
        attron(COLOR_PAIR(1) | A_BOLD);
        mvprintw(0, rx, "%d", state_.rx_frame_count.load());
        attroff(COLOR_PAIR(1) | A_BOLD);
        attron(A_DIM);
        addstr("v ");   // temp
        attroff(A_DIM);
        attron(COLOR_PAIR(2) | A_BOLD);
        printw("%d", state_.tx_frame_count.load());
        attroff(COLOR_PAIR(2) | A_BOLD);
        attron(A_DIM);
        addstr("^ ");  // temp 
        attroff(A_DIM);
        printw(" %d", state_.client_count.load());
        attron(A_DIM);
        addstr("c ");
        attroff(A_DIM);
        

        // Tab bar
        attron(A_DIM);
        draw_hline(1, 0, cols, true, true);
        attroff(A_DIM);
        

        // Tabs
        char utils_tab[24];
        int unread = state_.unread_messages.load();
        if (unread > 0)
            snprintf(utils_tab, sizeof(utils_tab), "UTILS(%d)", unread);
        else

            snprintf(utils_tab, sizeof(utils_tab), "UTILS");
        const char* tabs[] = {"STATUS", "CONFIG", "LOG", utils_tab, "SCOPE", "RIG"};
        int ntabs = tab_count();
        int tab_width = (cols - 4) / ntabs;

        for (int i = 0; i < ntabs; i++) {
            int tx = 2 + i * tab_width;

            if (i == current_tab_) {
                attron(A_BOLD);
                mvaddch(2, tx, '>');
                printw(" %s", tabs[i]);
                attroff(A_BOLD);
            } else {
                if (i == 3 && unread > 0) attron(COLOR_PAIR(4) | A_BOLD);
                else attron(A_DIM);
                mvprintw(2, tx, "  %s", tabs[i]);
                if (i == 3 && unread > 0) attroff(COLOR_PAIR(4) | A_BOLD);
                else attroff(A_DIM);
            }
        }
        
        // Content separator
        attron(A_DIM);
        draw_hline(3, 0, cols, true, true);
        attroff(A_DIM);
        
        // Content area
        int content_y = 4;
        int content_h = rows - 6;
        
        if (current_tab_ == 0) {
            draw_status(content_y, content_h, cols);
        } else if (current_tab_ == 1) {
            draw_config(content_y, content_h, cols);
        } else if (current_tab_ == 2) {
            draw_log(content_y, content_h, cols);
        } else if (current_tab_ == 3) {
            draw_utils(content_y, content_h, cols);
        } else if (current_tab_ == 4) {
            draw_scope(content_y, content_h, cols);
        } else {
            draw_rig(content_y, content_h, cols);
        }
        
        // Footer
        attron(A_DIM);
        draw_hline(rows - 2, 0, cols, true, true);
        
        if (current_tab_ == 1) {
            mvaddstr(rows - 1, 2, " ^/v nav  </> adjust  Enter edit  s save  x del  F1 help  Q quit ");
        } else if (current_tab_ == 2) {
            mvaddstr(rows - 1, 2, " ^/v scroll  PgUp/Dn page  F1 help  Q quit ");
        } else if (current_tab_ == 3) {
            mvaddstr(rows - 1, 2, " 1-7 select  Enter run  F1 help  Q quit ");
        } else if (current_tab_ == 5) {
            mvaddstr(rows - 1, 2, " ^/v nav  </> adjust  Enter set/tune  F1 help  Q quit ");
        } else if (current_tab_ == 4) {
            mvaddstr(rows - 1, 2, " Tab switch  F1 help  Q quit ");
        } else {
            mvaddstr(rows - 1, 2, " Tab switch  F1 help  Q quit ");
        }
        attroff(A_DIM);
        
        if (show_help_) {
            draw_help(rows, cols);
        }
        if (show_csma_help_) {
            draw_csma_help(rows, cols);
        }
    }
    
    void draw_status(int y, int h, int cols) {
        int c1 = 3;
        int c2 = 18;
        int c3 = cols / 2 + 2;
        int c4 = cols / 2 + 17;
        

        attron(A_DIM);
        mvaddstr(y, c1, "SIGNAL");
        attroff(A_DIM);
        y++;
        

        mvaddstr(y, c1, "Carrier");
        float lvl = state_.carrier_level_db.load();
        bool busy = lvl > state_.carrier_threshold_db;
        move(y, c2);
        if (busy) {
            attron(COLOR_PAIR(4) | A_BOLD);  
            printw("%6.1f dB", lvl);
            attroff(COLOR_PAIR(4) | A_BOLD);
        } else {
            attron(COLOR_PAIR(1) | A_BOLD);  
            printw("%6.1f dB", lvl);
            attroff(COLOR_PAIR(1) | A_BOLD);
        }
        y++;
        
        //  Meter
        mvaddstr(y, c1, "Level");
        move(y, c2);
        draw_level_meter(lvl, state_.carrier_threshold_db, 20);
        y++;
        
        mvaddstr(y, c1, "Threshold");
        mvprintw(y, c2, "%6.0f dB", state_.carrier_threshold_db);
        y++;
        
        mvaddstr(y, c1, "Last SNR");
        float snr = state_.last_rx_snr.load();
        if (snr > 10.0f) {
            attron(COLOR_PAIR(1) | A_BOLD);  
        } else if (snr > 5.0f) {
            attron(COLOR_PAIR(3) | A_BOLD);  
        }
        mvprintw(y, c2, "%6.1f dB", snr);
        attroff(COLOR_PAIR(1) | A_BOLD);
        attroff(COLOR_PAIR(3) | A_BOLD);
        y++;
        
        // SNR history
        mvaddstr(y, c1, "SNR Hist");
        move(y, c2);
        draw_snr_chart(20);
        y += 2;

        mvaddstr(y, c1, "BER");
        {
            float ber_pct = state_.last_rx_ber.load() * 100.0f;
            if (ber_pct < 0) {
                attron(A_DIM);
                mvaddstr(y, c2, "  ---");
                attroff(A_DIM);
            } else if (state_.modem_type_index == 2) {
                if (ber_pct < 18.0f) {
                    mvprintw(y, c2, "%5.2f%%", ber_pct);
                } else {
                    attron(COLOR_PAIR(2) | A_BOLD);
                    mvprintw(y, c2, "%5.2f%%", ber_pct);
                    attroff(COLOR_PAIR(2) | A_BOLD);
                }
            } else if (ber_pct < 3.0f) {
                attron(COLOR_PAIR(1) | A_BOLD);
                mvprintw(y, c2, "%5.2f%%", ber_pct);
                attroff(COLOR_PAIR(1) | A_BOLD);
            } else if (ber_pct < 13.0f) {
                attron(COLOR_PAIR(3) | A_BOLD);
                mvprintw(y, c2, "%5.2f%%", ber_pct);
                attroff(COLOR_PAIR(3) | A_BOLD);
            } else {
                attron(COLOR_PAIR(2) | A_BOLD);
                mvprintw(y, c2, "%5.2f%%", ber_pct);
                attroff(COLOR_PAIR(2) | A_BOLD);
            }
        }
        y++;

        mvaddstr(y, c1, "Heard");
        {
            auto pkts = state_.get_recent_packets();
            const TNCUIState::PacketInfo* last = nullptr;
            for (auto it = pkts.rbegin(); it != pkts.rend(); ++it) {
                if (!it->is_tx) {
                    last = &*it;
                    break;
                }
            }
            move(y, c2);
            if (!last) {
                attron(A_DIM);
                addstr("  ---");
                attroff(A_DIM);
            } else {
                auto secs = std::chrono::duration_cast<std::chrono::seconds>(
                    std::chrono::steady_clock::now() - last->timestamp).count();
                attron(A_BOLD);
                printw("%s", last->mode.empty() ? "?" : last->mode.c_str());
                attroff(A_BOLD);
                printw(" %s %.0fdB ",
                       last->callsign.empty() ? "-" : last->callsign.c_str(),
                       last->snr);
                attron(A_DIM);
                if (secs < 60)
                    printw("%llds", (long long)secs);
                else if (secs < 3600)
                    printw("%lldm", (long long)(secs / 60));
                else
                    printw("%lldh", (long long)(secs / 3600));
                attroff(A_DIM);
            }
        }
        y++;

        attron(A_DIM);
        mvaddstr(y, c1, "CSMA");
        attroff(A_DIM);
        y++;
        
        mvaddstr(y, c1, "Status");
        move(y, c2);
        if (state_.csma_enabled) {
            attron(COLOR_PAIR(1) | A_BOLD);
            addstr("ON");
            attroff(COLOR_PAIR(1) | A_BOLD);
        } else {
            attron(COLOR_PAIR(3) | A_BOLD);
            addstr("OFF");
            attroff(COLOR_PAIR(3) | A_BOLD);
        }
        {
            bool dcd = state_.dcd_active.load();
            if (busy || dcd) {
                attron(COLOR_PAIR(3) | A_BOLD);
                addstr("  BUSY");
                attroff(COLOR_PAIR(3) | A_BOLD);
                if (dcd) {
                    attron(A_DIM);
                    addstr(" (sync)");
                    attroff(A_DIM);
                }
            }
        }
        y++;

        mvaddstr(y, c1, "Occupancy");
        {
            float occ = state_.channel_occupancy.load() * 100.0f;
            int opair = occ >= 70.0f ? 2 : occ >= 30.0f ? 3 : 1;
            move(y, c2);
            attron(COLOR_PAIR(opair) | A_BOLD);
            printw("%5.0f%%", occ);
            attroff(COLOR_PAIR(opair) | A_BOLD);
            attron(A_DIM);
            addstr(" (30s)");
            attroff(A_DIM);
        }
        y++;

        mvaddstr(y, c1, "Occ Hist");
        move(y, c2);
        draw_occ_chart(20);
        y++;

        mvaddstr(y, c1, "Quiet");
        if (state_.csma_quiet_ms > 0) {
            mvprintw(y, c2, "%d ms", state_.csma_quiet_ms);
        } else {
            int q = (int)(state_.airtime_seconds * 1000.0f) / 4;
            if (q < 300) q = 300;
            if (q > 3500) q = 3500;
            mvprintw(y, c2, "auto ~%d ms", q);
        }
        y++;

        mvaddstr(y, c1, "Window");
        mvprintw(y, c2, "%d slots", state_.csma_cw);
        y++;

        mvaddstr(y, c1, "Slot");
        mvprintw(y, c2, "%d ms", state_.slot_time_ms);
        y++;

        mvaddstr(y, c1, "TX");
        {
            int phase = state_.csma_phase.load();
            int q = state_.tx_queue_size.load();
            move(y, c2);
            if (state_.transmitting.load()) {
                attron(COLOR_PAIR(2) | A_BOLD);
                addstr("sending");
                attroff(COLOR_PAIR(2) | A_BOLD);
            } else if (phase == 1) {
                attron(COLOR_PAIR(3));
                addstr("deferring (RX)");
                attroff(COLOR_PAIR(3));
            } else if (phase == 2) {
                attron(COLOR_PAIR(3));
                printw("quiet %d/%d ms", state_.csma_wait_ms.load(),
                       state_.csma_wait_need.load());
                attroff(COLOR_PAIR(3));
            } else if (phase == 3) {
                attron(COLOR_PAIR(4));
                printw("contending %d ms", state_.csma_wait_ms.load());
                attroff(COLOR_PAIR(4));
            } else {
                attron(A_DIM);
                addstr("idle");
                attroff(A_DIM);
            }
            if (q > 0) {
                attron(A_DIM);
                printw("  %d queued ~%ds", q,
                       (int)(q * state_.airtime_seconds + 0.5f));
                attroff(A_DIM);
            }
        }
        y += 2;

        mvaddstr(y, c1, "Rig");
        {
            move(y, c2);
            if (!state_.rigctl_connected.load()) {
                attron(A_DIM);
                addstr("  ---");
                attroff(A_DIM);
            } else {
                attron(A_BOLD);
                printw("%s", format_freq(state_.rig_freq_hz.load()).c_str());
                attroff(A_BOLD);
                std::string rmode = state_.get_rig_mode();
                if (!rmode.empty())
                    printw(" %s", rmode.c_str());
                float pwr = state_.rig_power_level.load();
                if (pwr >= 0)
                    printw(" %d%%", (int)lround(pwr * 100));
                float swr = state_.rig_meter_values[RIG_METER_SWR].load();
                if (!std::isnan(swr)) {
                    int spair = swr < 1.5f ? 1 : swr < SWR_WARN_THRESHOLD ? 3 : 2;
                    attron(COLOR_PAIR(spair));
                    printw(" SWR %.1f", swr);
                    attroff(COLOR_PAIR(spair));
                }
            }
        }


        y = 4;
        attron(A_DIM);
        mvaddstr(y, c3, "ACTIVITY");
        attroff(A_DIM);
        

        int graph_width = cols - c3 - 4;
        int graph_height = 6;
        draw_signal_graph(y + 1, c3, graph_width, graph_height);
        
        y += graph_height + 2;
        

        attron(COLOR_PAIR(4));
        mvaddstr(y, c3, ">>> STATS");
        attroff(COLOR_PAIR(4));
        y++;
        
        mvaddstr(y, c3, "RX");
        attron(COLOR_PAIR(1) | A_BOLD);
        mvprintw(y, c4, "%d", state_.rx_frame_count.load());
        attroff(COLOR_PAIR(1) | A_BOLD);
            
        //         addstr("  ");
        addstr("  ");
        attroff(A_BOLD);
        addstr("TX");
        attron(COLOR_PAIR(2) | A_BOLD);
        printw(" %d", state_.tx_frame_count.load());
        attroff(COLOR_PAIR(2) | A_BOLD);
        
        addstr("  ");
        int syncs = state_.sync_count.load();
        int total_errors = state_.preamble_errors.load() + 
                          state_.symbol_errors.load() + 
                          state_.crc_errors.load() +
                          state_.rx_error_count.load();
        if (state_.modem_type_index == 1) {
        } else if (syncs > 0) {
            float err_pct = 100.0f * total_errors / syncs;
            addstr("Err");
            if (total_errors == 0) {
                attron(COLOR_PAIR(1));
                printw(" 0/%d", syncs);
                attroff(COLOR_PAIR(1));
            } else if (err_pct < 20.0f) {
                attron(COLOR_PAIR(3));
                printw(" %d/%d", total_errors, syncs);
                attroff(COLOR_PAIR(3));
            } else {
                attron(COLOR_PAIR(2));
                printw(" %d/%d", total_errors, syncs);
                attroff(COLOR_PAIR(2));
            }
            attron(A_DIM);
            printw(" (%.0f%%)", err_pct);
            attroff(A_DIM);
        } else {
            attron(A_DIM);
            addstr("Err 0/0");
            attroff(A_DIM);
        }
        y++;
        

        mvaddstr(y, c3, "Clients");
        int clients = state_.client_count.load();
        if (clients > 0) {
            attron(COLOR_PAIR(4) | A_BOLD);
            mvprintw(y, c4, "%d", clients);
            attroff(COLOR_PAIR(4) | A_BOLD);
        } else {
            attron(A_DIM);
            mvprintw(y, c4, "%d", clients);
            attroff(A_DIM);
        }
        
        addstr("  Queue");
        printw(" %d", state_.tx_queue_size.load());
        

        y += 2;
        draw_recent_packets(y, c3, cols - c3 - 2, h - (y - 4) - 2);
    }
    
    void draw_recent_packets(int y, int x, int /* width */, int max_lines) {
        auto packets = state_.get_recent_packets();
        
        if (packets.empty()) {
            attron(A_DIM);
            mvaddstr(y, x, "Waiting for packets...");
            attroff(A_DIM);
            return;
        }
        
        attron(A_DIM);
        mvaddstr(y, x, "RECENT");
        attroff(A_DIM);
        y++;

        int sel = recent_sel_;
        if (sel >= (int)packets.size())
            sel = (int)packets.size() - 1;
        int limit = max_lines;
        if (sel >= 0 && limit > 2)
            limit -= 2;

        int lines = 0;
        for (auto it = packets.rbegin(); it != packets.rend() && lines < limit; ++it, ++lines) {
            const auto& pkt = *it;

            move(y + lines, x);
            if (lines == sel) {
                attron(A_BOLD);
                addstr("> ");
                attroff(A_BOLD);
            } else {
                addstr("  ");
            }

            if (pkt.is_tx) {
                attron(COLOR_PAIR(2) | A_BOLD);
                addstr("TX ");
                attroff(COLOR_PAIR(2) | A_BOLD);
            } else {
                attron(COLOR_PAIR(1) | A_BOLD);
                addstr("RX ");
                attroff(COLOR_PAIR(1) | A_BOLD);
            }
            
            attron(A_BOLD);
            printw("%4d", pkt.size);
            attroff(A_BOLD);
            attron(A_DIM);
            addstr("B ");
            attroff(A_DIM);
            


            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - pkt.timestamp).count();
            if (elapsed < 60) {
                printw("%2lds", elapsed);
            } else {
                printw("%2ldm", elapsed / 60);
            }
            


            // SNR
            if (!pkt.is_tx && pkt.snr > 0) {
                attron(COLOR_PAIR(4) | A_BOLD);
                printw(" %.0fdB", pkt.snr);
                attroff(COLOR_PAIR(4) | A_BOLD);
            }
            // BER
            if (!pkt.is_tx && pkt.ber >= 0) {
                float ber_pct = pkt.ber;
                bool robust = pkt.mode.rfind("RDM", 0) == 0;
                if (robust) {
                    if (ber_pct >= 18.0f)
                        attron(COLOR_PAIR(2));
                } else if (ber_pct < 1.0f) {
                    attron(COLOR_PAIR(1));
                } else if (ber_pct < 5.0f) {
                    attron(COLOR_PAIR(3));
                } else {
                    attron(COLOR_PAIR(2));
                }
                printw(" %.1f%%", ber_pct);
                attroff(COLOR_PAIR(1));
                attroff(COLOR_PAIR(2));
                attroff(COLOR_PAIR(3));
            }
        }

        if (sel >= 0 && sel < (int)packets.size()) {
            const auto& sp = *(packets.rbegin() + sel);
            move(y + lines + 1, x);
            attron(A_BOLD);
            printw("%s", sp.mode.empty() ? "?" : sp.mode.c_str());
            attroff(A_BOLD);
            printw("  %s", sp.callsign.empty() ? "-" : sp.callsign.c_str());
            if (!sp.is_tx && sp.snr > 0) {
                attron(COLOR_PAIR(4) | A_BOLD);
                printw("  %.1fdB", sp.snr);
                attroff(COLOR_PAIR(4) | A_BOLD);
            }
            if (!sp.is_tx && sp.ber >= 0) {
                bool robust = sp.mode.rfind("RDM", 0) == 0;
                bool bad = robust ? sp.ber >= 18.0f : sp.ber >= 5.0f;
                if (bad)
                    attron(COLOR_PAIR(2));
                printw("  BER %.1f%%", sp.ber);
                if (bad)
                    attroff(COLOR_PAIR(2));
            }
            attron(A_DIM);
            printw("  %dB", sp.size);
            attroff(A_DIM);
        }
    }
    
    void draw_level_meter(float level_db, float threshold_db, int width) {
        float min_db = -80.0f;
        float max_db = 0.0f;
        
        int level_pos = (int)((level_db - min_db) / (max_db - min_db) * width);
        int thresh_pos = (int)((threshold_db - min_db) / (max_db - min_db) * width);
        
        level_pos = std::max(0, std::min(width, level_pos));
        thresh_pos = std::max(0, std::min(width - 1, thresh_pos));
        
        attron(A_DIM);
        addch('[');
        attroff(A_DIM);
        
        for (int i = 0; i < width; i++) {
            if (i < level_pos) {
                if (i >= thresh_pos) {
                    attron(COLOR_PAIR(4) | A_BOLD);  
                    addch('=');
                    attroff(COLOR_PAIR(4) | A_BOLD);
                } else if (i >= width * 2 / 3) {
                    attron(COLOR_PAIR(3) | A_BOLD);  
                    addch('=');
                    attroff(COLOR_PAIR(3) | A_BOLD);
                } else {
                    attron(COLOR_PAIR(1) | A_BOLD); 
                    addch('=');
                    attroff(COLOR_PAIR(1) | A_BOLD);
                }
            } else if (i == thresh_pos) {
                attron(A_DIM);
                addch('|');  
                attroff(A_DIM);
            } else {
                attron(A_DIM);
                addch('-');
                attroff(A_DIM);
            }
        }
        
        attron(A_DIM);
        addch(']');
        attroff(A_DIM);
    }
    
    void draw_occ_chart(int width) {
        if (occ_hist_n_ == 0) {
            attron(A_DIM);
            addstr("[no data]");
            attroff(A_DIM);
            return;
        }
        int count = std::min(occ_hist_n_, width);
        attron(A_DIM);
        for (int i = count; i < width; i++)
            addch('.');
        attroff(A_DIM);
        for (int i = 0; i < count; i++) {
            int idx = (occ_hist_pos_ - count + i + 2 * OCC_HIST_SIZE) % OCC_HIST_SIZE;
            float v = occ_hist_[idx];
            char ch = v > 0.875f ? '#' : v > 0.7f ? '=' : v > 0.5f ? '+'
                    : v > 0.3f ? ':' : v > 0.15f ? '-' : v > 0.03f ? '.' : '_';
            int pair = v >= 0.7f ? 2 : v >= 0.3f ? 3 : 1;
            attron(COLOR_PAIR(pair));
            addch(ch);
            attroff(COLOR_PAIR(pair));
        }
    }

    void draw_snr_chart(int width) {
        auto history = state_.get_snr_history();
        
        if (history.empty()) {
            attron(A_DIM);
            addstr("[no data]");
            attroff(A_DIM);
            return;
        }
        
        // 
        float min_snr = 0.0f;
        float max_snr = 30.0f;
        

        int display_count = std::min((int)history.size(), width);
        int start_idx = (int)history.size() - display_count;
        
        for (int i = 0; i < display_count; i++) {
            float snr = history[start_idx + i];
            float normalized = (snr - min_snr) / (max_snr - min_snr);
            normalized = std::max(0.0f, std::min(1.0f, normalized));
            

            char ch;
            if (normalized > 0.875f) ch = '#';
            else if (normalized > 0.75f) ch = '=';
            else if (normalized > 0.625f) ch = '+';
            else if (normalized > 0.5f) ch = ':';
            else if (normalized > 0.375f) ch = '-';
            else if (normalized > 0.25f) ch = '.';
            else if (normalized > 0.125f) ch = '_';
            else ch = ' ';
            

            if (snr > 15.0f) {
                attron(COLOR_PAIR(1) | A_BOLD);  
            } else if (snr > 8.0f) {
                attron(COLOR_PAIR(3) | A_BOLD);  
            } else if (snr > 3.0f) {
                attron(COLOR_PAIR(4));           
            } else {
                attron(COLOR_PAIR(2));           
            }
            addch(ch);
            attroff(COLOR_PAIR(1) | A_BOLD);
            attroff(COLOR_PAIR(2));
            attroff(COLOR_PAIR(3) | A_BOLD);
            attroff(COLOR_PAIR(4));
        }
        

        attron(A_DIM);
        for (int i = display_count; i < width; i++) {
            addch('.');
        }
        attroff(A_DIM);
    }
    
    void draw_signal_graph(int y, int x, int width, int height) {
        auto history = state_.get_level_history();
        auto dcd_hist = state_.get_level_dcd_history();


        float min_db = -80.0f;
        float max_db = 0.0f;
        float thresh = state_.carrier_threshold_db;


        const char* blocks[] = {" ", ".", ":", "|", "#"};

        for (int row = 0; row < height; row++) {
            move(y + row, x);

            float row_min = max_db - (max_db - min_db) * (row + 1) / height;
            float row_max = max_db - (max_db - min_db) * row / height;

            for (int col = 0; col < width; col++) {
                int hist_idx = col * TNCUIState::LEVEL_HISTORY_SIZE / width;
                if (hist_idx >= (int)history.size()) hist_idx = history.size() - 1;

                float level = history[hist_idx];
                // magenta marks samples taken while the confirmed sync DCD
                // held: the segment of the waterfall carrying a real frame
                int pair = dcd_hist[hist_idx] ? 6 : level > thresh ? 4 : 1;

                if (level >= row_max) {
                    attron(COLOR_PAIR(pair) | A_BOLD);
                    addch(ACS_BLOCK);
                    attroff(COLOR_PAIR(pair) | A_BOLD);
                } else if (level > row_min) {
                    float frac = (level - row_min) / (row_max - row_min);
                    int idx = (int)(frac * 4);
                    if (idx > 4) idx = 4;
                    if (idx < 0) idx = 0;

                    attron(COLOR_PAIR(pair));
                    addstr(blocks[idx]);
                    attroff(COLOR_PAIR(pair));
                } else {
                    addch(' ');
                }
            }
        }
        
        int thresh_row = (int)((max_db - thresh) / (max_db - min_db) * height);
        if (thresh_row >= 0 && thresh_row < height) {
            attron(A_DIM | COLOR_PAIR(3));
            for (int col = 0; col < width; col += 2) {
                mvaddch(y + thresh_row, x + col, '-');
            }
            attroff(A_DIM | COLOR_PAIR(3));
        }
    }
    
    void draw_config(int y, int h, int cols) {
        int c1 = 3;      
        int c2 = 16;     
        int divider = cols/2 - 2;  
        int c3 = cols/2 + 1; 
        int start_y = y;
        int visible_rows = h - 2;  
        

        


        if (current_tab_ == 1) {
            int field_row = config_field_row(current_field_);
            if (field_row < config_scroll_ + 2) {
                config_scroll_ = std::max(0, field_row - 2);
            } else if (field_row > config_scroll_ + visible_rows - 3) {
                config_scroll_ = field_row - visible_rows + 3;
            }
        }

        
        int scroll = config_scroll_;
        int row = 0;  
        
        auto visible_y = [&](int logical_row) -> int {
            int screen_row = logical_row - scroll;
            if (screen_row < 0 || screen_row >= visible_rows) return -1;
            return start_y + screen_row;
        };
        
//
        attron(A_DIM);
        for (int r = start_y; r < start_y + visible_rows; r++) {
            mvaddch(r, divider, ACS_VLINE);
        }

        attroff(A_DIM);
        
        int dy = visible_y(row);
        if (dy >= 0) {
            attron(A_DIM);
            mvaddstr(dy, c1, "MODEM");
            attroff(A_DIM);
        }
        row++;
        
        dy = visible_y(row);
        if (dy >= 0) draw_field(dy, c1, c2, "Callsign", FIELD_CALLSIGN, state_.callsign, true);
        row++;

        dy = visible_y(row);
        if (dy >= 0) draw_selector_field(dy, c1, c2, "Modem", FIELD_MODEM_TYPE,
                           MODEM_TYPE_OPTIONS[state_.modem_type_index]);
        row++;

        if (state_.modem_type_index == 0) {
            // OFDM fields
            dy = visible_y(row);
            if (dy >= 0) draw_selector_field(dy, c1, c2, "Modulation", FIELD_MODULATION,
                               MODULATION_OPTIONS[state_.modulation_index]);
            row++;

            dy = visible_y(row);
            if (dy >= 0) draw_selector_field(dy, c1, c2, "Code Rate", FIELD_CODERATE,
                               CODE_RATE_OPTIONS[state_.code_rate_index]);
            row++;

            dy = visible_y(row);
            if (dy >= 0) draw_selector_field(dy, c1, c2, "Frame Size", FIELD_FRAMESIZE,
                               state_.frame_size == 0 ? "SHORT" : state_.frame_size == 2 ? "LONG" : "NORMAL");
            row++;

            dy = visible_y(row);
            if (dy >= 0) draw_toggle_field(dy, c1, c2, "Postamble", FIELD_POSTAMBLE, state_.postamble);
            row++;
        } else if (state_.modem_type_index == 1) {
            // MFSK field
            dy = visible_y(row);
            if (dy >= 0) draw_selector_field(dy, c1, c2, "MFSK Mode", FIELD_MFSK_MODE,
                               MFSK_MODE_OPTIONS[state_.mfsk_mode_index]);
            row++;
        } else {
            dy = visible_y(row);
            if (dy >= 0) draw_selector_field(dy, c1, c2, "RDM Mode", FIELD_ROBUST_MODE,
                               ROBUST_MODE_OPTIONS[robust_base_index(state_.robust_mode_index)]);
            row++;
            dy = visible_y(row);
            if (dy >= 0) draw_selector_field(dy, c1, c2, "Frame", FIELD_ROBUST_MTU,
                               ROBUST_MTU_OPTIONS[RobustParams::is_short(
                                   (RobustMode)state_.robust_mode_index) ? 1 : 0]);
            row++;
        }

        dy = visible_y(row);
        if (dy >= 0) {
            char freq_buf[32];
            snprintf(freq_buf, sizeof(freq_buf), "%d Hz", state_.center_freq);
            draw_field(dy, c1, c2, "Freq", FIELD_FREQ, freq_buf, true);
        }
        row += 2;
        
        dy = visible_y(row);
        if (dy >= 0) {
            attron(A_DIM);
            mvaddstr(dy, c1, "CSMA");
            mvaddnstr(dy, c1 + 6, "waits for a clear channel before transmitting",
                      std::max(0, divider - (c1 + 6) - 1));
            attroff(A_DIM);
        }
        row++;
        
        dy = visible_y(row);
        if (dy >= 0) draw_toggle_field(dy, c1, c2, "Enabled", FIELD_CSMA, state_.csma_enabled);
        row++;
        
        dy = visible_y(row);
        if (dy >= 0) {
            char thresh_buf[32];
            snprintf(thresh_buf, sizeof(thresh_buf), "%.0f dB", state_.carrier_threshold_db);
            draw_selector_field(dy, c1, c2, "Threshold", FIELD_THRESHOLD, thresh_buf);
            float lvl = state_.carrier_level_db.load();
            if (lvl > state_.carrier_threshold_db) {
                attron(COLOR_PAIR(4) | A_BOLD);
            } else {
                attron(A_DIM);
            }
            mvprintw(dy, c2 + 9, "%.0f", lvl);
            attroff(COLOR_PAIR(4) | A_BOLD);
            attroff(A_DIM);
        }
        row++;
        
        // Level meter bar
        dy = visible_y(row);
        if (dy >= 0) {
            mvaddstr(dy, c1, "Level");
            move(dy, c2);
            float lvl = state_.carrier_level_db.load();
            draw_level_meter(lvl, state_.carrier_threshold_db, 14);
        }
        row++;
        
        dy = visible_y(row);
        if (dy >= 0) draw_selector_field(dy, c1, c2, "Band", FIELD_CSMA_BAND,
                                         CSMA_BAND_NAMES[state_.csma_band & 1]);
        row++;

        dy = visible_y(row);
        if (dy >= 0) {
            int pidx = csma_preset_match();
            draw_selector_field(dy, c1, c2, "Preset", FIELD_CSMA_PRESET,
                                pidx >= 0 ? CSMA_PRESETS[state_.csma_band & 1][pidx].name
                                          : "CUSTOM");
        }
        row++;

        dy = visible_y(row);
        if (dy >= 0) {
            bool sel_adv = (current_field_ == FIELD_CSMA_ADV);
            if (sel_adv) attron(A_BOLD); else attron(A_DIM);
            mvprintw(dy, c1, "%s[%c] Advanced", sel_adv ? "> " : "  ",
                     state_.csma_advanced_open ? '-' : '+');
            if (sel_adv) attroff(A_BOLD); else attroff(A_DIM);
        }
        row++;

        if (state_.csma_advanced_open) {
            dy = visible_y(row);
            if (dy >= 0) {
                char quiet_buf[32];
                if (state_.csma_quiet_ms > 0)
                    snprintf(quiet_buf, sizeof(quiet_buf), "%d ms", state_.csma_quiet_ms);
                else
                    snprintf(quiet_buf, sizeof(quiet_buf), "AUTO");
                draw_selector_field(dy, c1 + 2, c2, "Quiet", FIELD_CSMA_QUIET, quiet_buf);
            }
            row++;

            dy = visible_y(row);
            if (dy >= 0) {
                char cw_buf[32];
                snprintf(cw_buf, sizeof(cw_buf), "%d x %dms", state_.csma_cw, state_.slot_time_ms);
                draw_selector_field(dy, c1 + 2, c2, "Window", FIELD_CSMA_CW, cw_buf);
            }
            row++;

            dy = visible_y(row);
            if (dy >= 0) draw_toggle_field(dy, c1 + 2, c2, "Lead Tone", FIELD_LEAD_TONE, state_.tx_lead_tone);
            row++;

            dy = visible_y(row);
            if (dy >= 0) {
                char dith_buf[32];
                if (state_.csma_responder_dither > 0)
                    snprintf(dith_buf, sizeof(dith_buf), "%d ms", state_.csma_responder_dither);
                else
                    snprintf(dith_buf, sizeof(dith_buf), "OFF");
                draw_selector_field(dy, c1 + 2, c2, "Resp Dither", FIELD_RESP_DITHER, dith_buf);
            }
            row++;

            dy = visible_y(row);
            if (dy >= 0) {
                char burst_buf[32];
                if (state_.csma_burst > 1)
                    snprintf(burst_buf, sizeof(burst_buf), "%d pkts", state_.csma_burst);
                else
                    snprintf(burst_buf, sizeof(burst_buf), "OFF");
                draw_selector_field(dy, c1 + 2, c2, "Burst", FIELD_CSMA_BURST, burst_buf);
            }
            row++;
        }

        dy = visible_y(row);
        if (dy >= 0) {
            bool sel_info = (current_field_ == FIELD_CSMA_INFO);
            if (sel_info) attron(A_BOLD); else attron(A_DIM);
            mvprintw(dy, c1, "%s[?] Info", sel_info ? "> " : "  ");
            if (sel_info) attroff(A_BOLD); else attroff(A_DIM);
        }
        row += 2;
        
        // Fragmentation section
        dy = visible_y(row);
        if (dy >= 0) {
            attron(A_DIM);
            mvaddstr(dy, c1, "FRAGMENTATION");
            mvaddstr(dy, c1 + 14, "(restart)");
            attroff(A_DIM);
        }
        row++;
        
        row++;
        
        dy = visible_y(row);
        if (dy >= 0) draw_toggle_field(dy, c1, c2, "Enabled", FIELD_FRAGMENTATION, state_.fragmentation_enabled);
        row += 2;
        
        // TX Blanking section
        dy = visible_y(row);
        if (dy >= 0) {
            attron(A_DIM);
            mvaddstr(dy, c1, "TX BLANKING");
            attroff(A_DIM);
        }
        row++;
        
        dy = visible_y(row);
        if (dy >= 0) draw_toggle_field(dy, c1, c2, "Enabled", FIELD_TX_BLANKING, state_.tx_blanking_enabled);
        row += 2;
        
        // Audio / ptt

        dy = visible_y(row);
        if (dy >= 0) {
            attron(A_DIM);
            mvaddstr(dy, c1, "AUDIO/PTT");
            mvaddstr(dy, c1 + 10, "(restart)");
            attroff(A_DIM);
        }
        row++;
        
        // Audio input device
        dy = visible_y(row);
        if (dy >= 0) {
            std::string dev_display = state_.audio_input_device;
            if (dev_display.length() > 12) {
                dev_display = dev_display.substr(0, 11) + "~";
            }
            draw_field(dy, c1, c2, "Input", FIELD_AUDIO_INPUT, dev_display, true);
        }
        row++;
        
        // Audio output device
        dy = visible_y(row);
        if (dy >= 0) {
            std::string dev_display = state_.audio_output_device;
            if (dev_display.length() > 12) {
                dev_display = dev_display.substr(0, 11) + "~";
            }
            draw_field(dy, c1, c2, "Output", FIELD_AUDIO_OUTPUT, dev_display, true);
        }
        row++;
        
        dy = visible_y(row);
        if (dy >= 0) draw_selector_field(dy, c1, c2, "PTT", FIELD_PTT_TYPE,
                           PTT_TYPE_OPTIONS[state_.ptt_type_index]);
        row++;
        
        if (state_.ptt_type_index == 2) {  // VOX
            dy = visible_y(row);
            if (dy >= 0) {
                char vox_freq_buf[32];
                snprintf(vox_freq_buf, sizeof(vox_freq_buf), "%d Hz", state_.vox_tone_freq);
                draw_selector_field(dy, c1, c2, "VOX Tone", FIELD_VOX_FREQ, vox_freq_buf);
            }
            row++;
            
            dy = visible_y(row);
            if (dy >= 0) {
                char vox_lead_buf[32];
                snprintf(vox_lead_buf, sizeof(vox_lead_buf), "%d ms", state_.vox_lead_ms);
                draw_selector_field(dy, c1, c2, "VOX Lead", FIELD_VOX_LEAD, vox_lead_buf);
            }
            row++;

            dy = visible_y(row);
            if (dy >= 0) {
                char vox_tail_buf[32];
                snprintf(vox_tail_buf, sizeof(vox_tail_buf), "%d ms", state_.vox_tail_ms);
                draw_selector_field(dy, c1, c2, "VOX Tail", FIELD_VOX_TAIL, vox_tail_buf);
            }
            row++;
        }
        
        if (state_.ptt_type_index == 3) {  // COM
            dy = visible_y(row);
            if (dy >= 0) {
                std::string port_display = state_.com_port;
                if (port_display.length() > 14) {
                    port_display = port_display.substr(0, 13) + "~";
                }
                draw_field(dy, c1, c2, "COM Port", FIELD_COM_PORT, port_display, true);
            }
            row++;
            
            dy = visible_y(row);
            if (dy >= 0) draw_selector_field(dy, c1, c2, "PTT Line", FIELD_COM_LINE,
                               PTT_LINE_OPTIONS[state_.com_ptt_line]);
            row++;
            
            dy = visible_y(row);
            if (dy >= 0) {
                std::string invert_str;
                if (!state_.com_invert_dtr && !state_.com_invert_rts) {
                    invert_str = "NORMAL";
                } else if (state_.com_invert_dtr && !state_.com_invert_rts) {
                    invert_str = "INV DTR";
                } else if (!state_.com_invert_dtr && state_.com_invert_rts) {
                    invert_str = "INV RTS";
                } else {
                    invert_str = "INV BOTH";
                }
                draw_selector_field(dy, c1, c2, "Invert", FIELD_COM_INVERT, invert_str);
            }
            row++;
        }
#ifdef WITH_CM108
        if (state_.ptt_type_index == 4) {  // CM108
            dy = visible_y(row);
            if (dy >= 0) {
                char cm108_gpio_buf[32];
                snprintf(cm108_gpio_buf, sizeof(cm108_gpio_buf), "%d", state_.cm108_gpio);
                draw_field(dy, c1, c2, "GPIO Pin", FIELD_CM108_GPIO, cm108_gpio_buf, true);
            }
            row++;

            dy = visible_y(row);
            if (dy >= 0) {
                std::string dev_display = state_.cm108_device.empty() ? "Auto" : state_.cm108_device;
                if (dev_display.length() > 12) {
                    dev_display = dev_display.substr(0, 11) + "~";
                }
                draw_field(dy, c1, c2, "Device", FIELD_CM108_DEVICE, dev_display, true);
            }
            row++;
        }
#endif
        row++;
        
        // Network section
        dy = visible_y(row);
        if (dy >= 0) {
            attron(A_DIM);
            mvaddstr(dy, c1, "NETWORK");
            mvaddstr(dy, c1 + 8, "(restart)");
            attroff(A_DIM);
        }
        row++;
        
        dy = visible_y(row);
        if (dy >= 0) {
            char port_buf[32];
            snprintf(port_buf, sizeof(port_buf), "%d", state_.port);
            draw_field(dy, c1, c2, "Port", FIELD_NET_PORT, port_buf, true);
        }
        row += 2;
        
        //  Preset section
        dy = visible_y(row);
        if (dy >= 0) {
            attron(A_DIM);
            mvaddstr(dy, c1, "PRESET");
            attroff(A_DIM);
        }
        row++;
        
        dy = visible_y(row);
        if (dy >= 0) {
            bool sel = (current_field_ == FIELD_PRESET);
            if (sel) {
                attron(A_BOLD);
                mvaddch(dy, c1 - 2, '>');
                mvaddstr(dy, c1, "Load");
                attroff(A_BOLD);
            } else {
                attron(A_DIM);
                mvaddstr(dy, c1, "Load");
                attroff(A_DIM);
            }
            
            move(dy, c2);
            if (state_.presets.empty()) {
                attron(A_DIM);
                addstr("(none)");
                attroff(A_DIM);
            } else {
                if (sel) attron(COLOR_PAIR(4) | A_BOLD);
                addstr("< ");
                if (state_.selected_preset >= 0 && state_.selected_preset < (int)state_.presets.size()) {
                    printw("%-10s", state_.presets[state_.selected_preset].name.c_str());
                }
                addstr(" >");
                if (sel) attroff(COLOR_PAIR(4) | A_BOLD);
            }
            
            int hint_y = visible_y(row + 1);
            if (sel && hint_y >= 0) {
                attron(A_DIM);
                mvaddstr(hint_y, c1, "Enter=load s=save x=del");
                attroff(A_DIM);
            }
        }
        
        // Info / stas
        y = start_y;
        
        attron(COLOR_PAIR(4) | A_BOLD);
        mvaddstr(y, c3, "MODEM INFO");
        attroff(COLOR_PAIR(4) | A_BOLD);
        y++;
        
        mvprintw(y, c3, "Payload %d B", state_.mtu_bytes);
        attron(COLOR_PAIR(4) | A_BOLD);
        if (state_.bitrate_bps >= 1000) {
            printw("  %.1f kb/s", state_.bitrate_bps / 1000.0f);
        } else {
            printw("  %d b/s", state_.bitrate_bps);
        }
        attroff(COLOR_PAIR(4) | A_BOLD);
        y++;
        
        mvprintw(y, c3, "Frame %.2fs", state_.airtime_seconds);
        float tx_time = state_.total_tx_time.load();
        printw("  TX ");
        if (tx_time < 60) printw("%.0fs", tx_time);
        else printw("%.1fm", tx_time / 60.0f);
        y++;




        {
            bool hf_ok = (state_.modem_type_index == 1) ||
                         (state_.modulation_index <= 2); // BPSK, QPSK, 8PSK
            mvaddstr(y, c3, "Band  ");
            if (hf_ok) {
                attron(COLOR_PAIR(3) | A_BOLD);
                addstr("HF/VHF");
                attroff(COLOR_PAIR(3) | A_BOLD);
            } else {
                attron(A_DIM);
                addstr("HF/VHF");
                attroff(A_DIM);
            }
            addstr("  ");
            if (!hf_ok) {
                attron(COLOR_PAIR(3) | A_BOLD);
                addstr("VHF/UHF");
                attroff(COLOR_PAIR(3) | A_BOLD);
            } else {
                attron(A_DIM);
                addstr("VHF/UHF");
                attroff(A_DIM);
            }
        }
        y += 2;
        
        // Right side, for audio / ptt status

        attron(COLOR_PAIR(4) | A_BOLD);
        mvaddstr(y, c3, "AUDIO/PTT");
        attroff(COLOR_PAIR(4) | A_BOLD);
        
        // Audio connection status
        if (state_.audio_connected.load()) {
            attron(COLOR_PAIR(1) | A_BOLD);
            addstr(" OK");
            attroff(COLOR_PAIR(1) | A_BOLD);
        } else {
            attron(COLOR_PAIR(2) | A_BOLD);
            addstr(" DISCONNECTED");
            attroff(COLOR_PAIR(2) | A_BOLD);
        }
        y++;
        

        // Show input device
        mvaddstr(y, c3, "In: ");
        {
            std::string dev_short = state_.audio_input_device;
            if (dev_short.length() > 14) dev_short = dev_short.substr(0, 13) + "~";
            if (state_.audio_connected.load()) {
                attron(A_DIM);
                addstr(dev_short.c_str());
                attroff(A_DIM);
            } else {
                attron(COLOR_PAIR(2));
                addstr(dev_short.c_str());
                attroff(COLOR_PAIR(2));
            }
        }
        y++;
        
        // Show output device
        mvaddstr(y, c3, "Out:");
        {
            std::string dev_short = state_.audio_output_device;
            if (dev_short.length() > 14) dev_short = dev_short.substr(0, 13) + "~";
            if (state_.audio_connected.load()) {
                attron(A_DIM);
                addstr(dev_short.c_str());
                attroff(A_DIM);
            } else {
                attron(COLOR_PAIR(2));
                addstr(dev_short.c_str());
                attroff(COLOR_PAIR(2));
            }
        }

        y++;
        
        mvaddstr(y, c3, "PTT: ");
        addstr(PTT_TYPE_OPTIONS[state_.ptt_type_index].c_str());
        if (state_.ptt_type_index == 1) {  // RIGCTL
            if (state_.rigctl_connected.load()) {
                attron(COLOR_PAIR(1) | A_BOLD);
                addstr(" OK");
                attroff(COLOR_PAIR(1) | A_BOLD);
            } else {
                attron(COLOR_PAIR(2) | A_BOLD);
                addstr(" --");
                attroff(COLOR_PAIR(2) | A_BOLD);
            }
        }
        if (state_.ptt_on.load()) {
            attron(COLOR_PAIR(2) | A_BOLD);
            addstr(" TX");
            attroff(COLOR_PAIR(2) | A_BOLD);
        }
        y += 2;
        


        attron(COLOR_PAIR(4) | A_BOLD);
        mvaddstr(y, c3, "NETWORK");
        attroff(COLOR_PAIR(4) | A_BOLD);
        y++;
        
        mvprintw(y, c3, "Port: %d", state_.port);
        printw("  ");
        attron(COLOR_PAIR(4));
        printw("%dc", state_.client_count.load());
        attroff(COLOR_PAIR(4));
        y += 2;
        



        if (state_.selected_preset >= 0 && state_.selected_preset < (int)state_.presets.size()) {
            const auto& p = state_.presets[state_.selected_preset];
            attron(COLOR_PAIR(4) | A_BOLD);
            mvaddstr(y, c3, "PRESET");
            attroff(COLOR_PAIR(4) | A_BOLD);
            attron(A_DIM);
            printw(" %s", p.name.c_str());
            attroff(A_DIM);
            y++;
            
            mvprintw(y, c3, "%s %s %s",
                     MODULATION_OPTIONS[p.modulation_index].c_str(),
                     CODE_RATE_OPTIONS[p.code_rate_index].c_str(),
                     p.frame_size == 0 ? "S" : p.frame_size == 2 ? "L" : "N");
            y++;
            
            mvaddstr(y, c3, "PTT ");
            addstr(PTT_TYPE_OPTIONS[p.ptt_type_index].c_str());
            if (p.ptt_type_index == 2) {
                printw(" %dHz", p.vox_tone_freq);
            }
            y++;
            
            mvaddstr(y, c3, "CSMA ");
            if (p.csma_enabled) {
                attron(COLOR_PAIR(1) | A_BOLD);
                addstr("ON");
                attroff(COLOR_PAIR(1) | A_BOLD);
            } else {
                addstr("OFF");
            }
            y++;
            
            if (current_field_ == FIELD_PRESET) {
                if (state_.selected_preset == state_.loaded_preset_index) {
                    attron(COLOR_PAIR(1) | A_BOLD);
                    mvaddstr(y, c3, "/// loaded");
                    attroff(COLOR_PAIR(1) | A_BOLD);
                } else {
                    bool blink_on = (frame_counter_ / 15) % 2 == 0;
                    if (blink_on) {
                        attron(COLOR_PAIR(4) | A_BOLD);
                        mvaddstr(y, c3, "/// ENTER TO LOAD");
                        attroff(COLOR_PAIR(4) | A_BOLD);
                    }
                }
            }
        }
    }
    
    void draw_field(int y, int c1, int c2, const char* label, int field,
                    const std::string& value, bool editable) {
        draw_field_ex(y, c1, c2, label, field == current_field_, value, editable);
    }

    void draw_field_ex(int y, int c1, int c2, const char* label, bool sel,
                       const std::string& value, bool editable) {

        if (sel) {
            attron(A_BOLD);
            mvaddch(y, c1 - 2, '>');
            mvaddstr(y, c1, label);
            attroff(A_BOLD);
            move(y, c2);
            attron(COLOR_PAIR(4) | A_BOLD);
            addstr(value.c_str());
            attroff(COLOR_PAIR(4) | A_BOLD);
            if (editable) {
                attron(A_DIM);
                addstr("  [enter]");
                attroff(A_DIM);
            }
        } else {
            attron(A_DIM);
            mvaddstr(y, c1, label);
            attroff(A_DIM);
            mvaddstr(y, c2, value.c_str());
        }
    }
    
    void draw_selector_field(int y, int c1, int c2, const char* label, int field,
                             const std::string& value) {
        draw_selector_field_ex(y, c1, c2, label, field == current_field_, value);
    }

    void draw_selector_field_ex(int y, int c1, int c2, const char* label, bool sel,
                                const std::string& value) {

        if (sel) {
            attron(A_BOLD);
            mvaddch(y, c1 - 2, '>');
            mvaddstr(y, c1, label);
            attroff(A_BOLD);
            move(y, c2);
            attron(A_DIM);
            addstr("<");
            attroff(A_DIM);
            attron(COLOR_PAIR(4) | A_BOLD);
            printw(" %s ", value.c_str());
            attroff(COLOR_PAIR(4) | A_BOLD);
            attron(A_DIM);
            addstr(">");
            attroff(A_DIM);
        } else {
            attron(A_DIM);
            mvaddstr(y, c1, label);
            attroff(A_DIM);
            mvprintw(y, c2, "  %s", value.c_str());
        }
    }
    
    void draw_toggle_field(int y, int c1, int c2, const char* label, int field, bool value) {
        bool sel = (field == current_field_);
        
        if (sel) {
            attron(A_BOLD);
            mvaddch(y, c1 - 2, '>');
            mvaddstr(y, c1, label);
            attroff(A_BOLD);
            move(y, c2);
            attron(A_DIM);
            addstr("<");
            attroff(A_DIM);
            if (value) {
                attron(COLOR_PAIR(1) | A_BOLD);
                addstr(" ON ");
                attroff(COLOR_PAIR(1) | A_BOLD);
            } else {
                attron(COLOR_PAIR(3) | A_BOLD);
                addstr(" OFF ");
                attroff(COLOR_PAIR(3) | A_BOLD);
            }
            attron(A_DIM);
            addstr(">");
            attroff(A_DIM);
        } else {
            attron(A_DIM);
            mvaddstr(y, c1, label);
            attroff(A_DIM);
            move(y, c2);
            if (value) {
                attron(COLOR_PAIR(1));
                addstr("  ON");
                attroff(COLOR_PAIR(1));
            } else {
                attron(COLOR_PAIR(3));
                addstr("  OFF");
                attroff(COLOR_PAIR(3));
            }
        }
    }
    
    void draw_log(int y, int h, int cols) {
        // Decode stats header
        int c1 = 3;
        attron(A_DIM);
        mvaddstr(y, c1, "DECODE STATS");
        attroff(A_DIM);
        y++;
        
        int syncs = state_.sync_count.load();
        int pre_err = state_.preamble_errors.load();
        int sym_err = state_.symbol_errors.load();
        int crc_err = state_.crc_errors.load();
        int unframe_err = state_.rx_error_count.load();
        int decoded = state_.rx_frame_count.load();
        
        mvaddstr(y, c1, "Syncs");
        attron(COLOR_PAIR(4));
        printw(" %d", syncs);
        attroff(COLOR_PAIR(4));
        
        addstr("  Decoded");
        attron(COLOR_PAIR(1));
        printw(" %d", decoded);
        attroff(COLOR_PAIR(1));
        
        addstr("  CRC Fail");
        if (crc_err > 0) attron(COLOR_PAIR(2));
        printw(" %d", crc_err);
        if (crc_err > 0) attroff(COLOR_PAIR(2));
        
        addstr("  Seed Err");
        if (sym_err > 0) attron(COLOR_PAIR(2));
        printw(" %d", sym_err);
        if (sym_err > 0) attroff(COLOR_PAIR(2));

        int erased = state_.erased_symbols.load();
        addstr("  Erased");
        if (erased > 0) attron(COLOR_PAIR(3));
        printw(" %d", erased);
        if (erased > 0) attroff(COLOR_PAIR(3));

        addstr("  Pre Err");
        if (pre_err > 0) attron(COLOR_PAIR(2));
        printw(" %d", pre_err);
        if (pre_err > 0) attroff(COLOR_PAIR(2));
        
        if (unframe_err > 0) {
            addstr("  Unframe");
            attron(COLOR_PAIR(2));
            printw(" %d", unframe_err);
            attroff(COLOR_PAIR(2));
        }
        
        y += 2;
        h -= 3;
        
        auto log = state_.get_log();
        int visible = h - 1;
        int max_scroll = std::max(0, (int)log.size() - visible);
        log_scroll_ = std::min(log_scroll_, max_scroll);
        
        int text_width = cols - 5;
        
        for (int i = 0; i < visible && (log_scroll_ + i) < (int)log.size(); i++) {
            const std::string& line = log[log_scroll_ + i];
            
            int pair = 0;
            bool bold = false;
            if (line.find("TX:") != std::string::npos) { pair = 2; bold = true; }
            else if (line.find("RX:") != std::string::npos) { pair = 1; bold = true; }
            else if (line.find("CSMA") != std::string::npos) pair = 3;
            else if (line.find("error") != std::string::npos || 
                     line.find("Error") != std::string::npos ||
                     line.find("failed") != std::string::npos) pair = 2;
            else if (line.find("Client") != std::string::npos) pair = 4;
            
            if (pair) attron(COLOR_PAIR(pair));
            if (bold) attron(A_BOLD);
            
            if ((int)line.length() > text_width) {
                mvprintw(y + i, 2, "%.*s...", text_width - 3, line.c_str());
            } else {
                mvprintw(y + i, 2, "%s", line.c_str());
            }
            
            if (bold) attroff(A_BOLD);
            if (pair) attroff(COLOR_PAIR(pair));
        }
        
        // scrollbar based on dims
        if ((int)log.size() > visible && visible > 2) {
            int sb_height = visible;
            int thumb_size = std::max(1, sb_height * visible / (int)log.size());
            int thumb_pos = max_scroll > 0 ? log_scroll_ * (sb_height - thumb_size) / max_scroll : 0;
            
            for (int i = 0; i < sb_height; i++) {
                if (i >= thumb_pos && i < thumb_pos + thumb_size) {
                    mvaddch(y + i, cols - 2, ACS_BLOCK);
                } else {
                    attron(A_DIM);
                    mvaddch(y + i, cols - 2, ACS_VLINE);
                    attroff(A_DIM);
                }
            }
        }
    }
    
    void draw_constellation(int y, int x, int height, int width) {
        // Height and width are separate to account for terminal character aspect ratio
        if (height < 5) height = 5;
        if (width < 9) width = 9;
        
        // Draw border using ACS characters
        attron(A_DIM);
        mvaddch(y, x, ACS_ULCORNER);
        mvaddch(y, x + width + 1, ACS_URCORNER);
        mvaddch(y + height + 1, x, ACS_LLCORNER);
        mvaddch(y + height + 1, x + width + 1, ACS_LRCORNER);
        for (int i = 1; i <= width; ++i) {
            mvaddch(y, x + i, ACS_HLINE);
            mvaddch(y + height + 1, x + i, ACS_HLINE);
        }
        for (int i = 1; i <= height; ++i) {
            mvaddch(y + i, x, ACS_VLINE);
            mvaddch(y + i, x + width + 1, ACS_VLINE);
        }
        
        // Axis labels
        mvaddstr(y - 1, x + width/2 - 1, "+Im");  // Top center (positive imaginary)
        mvaddstr(y + height + 2, x + width/2 - 1, "-Im");  // Bottom center (negative imaginary)
        mvaddstr(y + height/2, x - 4, "-Re");  // Left (negative real)
        mvaddstr(y + height/2, x + width + 3, "+Re");  // Right (positive real)
        attroff(A_DIM);
        
        // Draw key to the right of constellation
        int key_x = x + width + 8;
        int key_y = y + 1;
        
        attron(A_DIM);
        mvaddstr(key_y, key_x, "DENSITY");
        attroff(A_DIM);
        key_y += 1;
        
        // Show density scale with colors
        attron(COLOR_PAIR(1) | A_BOLD);
        mvaddstr(key_y++, key_x, "@ # * High");
        attroff(COLOR_PAIR(1) | A_BOLD);
        
        attron(COLOR_PAIR(3));
        mvaddstr(key_y++, key_x, "+ = - Med");
        attroff(COLOR_PAIR(3));
        
        attron(A_DIM);
        mvaddstr(key_y++, key_x, ": .   Low");
        attroff(A_DIM);
        
        key_y++;
        attron(A_DIM);
        mvaddstr(key_y++, key_x, "AXES");
        attroff(A_DIM);
        mvaddstr(key_y++, key_x, "Re: In-phase");
        mvaddstr(key_y++, key_x, "Im: Quadrature");
        
        // Check for stale data (10 second timeout)
        auto now = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        bool stale = (now - state_.constellation_update_time.load()) > 10000;
        
        if (!state_.constellation_valid.load() || stale) {
            // No data - show placeholder
            attron(A_DIM);
            int mid_y = height / 2;
            int mid_x = (width - 9) / 2;  // "No signal" is 9 chars
            mvaddstr(y + 1 + mid_y, x + 1 + mid_x, "No signal");
            attroff(A_DIM);
            return;
        }
        
        // Density characters (space to full block)
        const char* density_chars = " .:-=+*#@";
        const int num_chars = 9;
        
        std::lock_guard<std::mutex> lock(state_.constellation_mutex);
        
        // Find peak density for normalization
        int peak = 1;
        for (size_t i = 0; i < state_.constellation_density.size(); ++i) {
            if (state_.constellation_density[i] > peak) {
                peak = state_.constellation_density[i];
            }
        }
        
        // Scale factors to map grid to display (separate for x and y)
        int grid_size = TNCUIState::CONSTELLATION_GRID;
        float scale_y = (float)grid_size / height;
        float scale_x = (float)grid_size / width;
        
        // Draw constellation points
        for (int dy = 0; dy < height; ++dy) {
            for (int dx = 0; dx < width; ++dx) {
                // Map display coords to grid coords
                int gx = (int)(dx * scale_x);
                int gy = (int)(dy * scale_y);
                
                // Ensure at least 1 grid cell per display cell (prevents striping)
                int gx_end = std::max(gx + 1, std::min((int)((dx + 1) * scale_x), grid_size));
                int gy_end = std::max(gy + 1, std::min((int)((dy + 1) * scale_y), grid_size));

                int density = 0;
                for (int sy = gy; sy < gy_end; ++sy) {
                    for (int sx = gx; sx < gx_end; ++sx) {
                        int d = state_.constellation_density[sy * grid_size + sx];
                        if (d > density) density = d;
                    }
                }
                
                // Map density to character
                int char_idx = (density * (num_chars - 1)) / peak;
                if (density > 0 && char_idx == 0) char_idx = 1;
                char_idx = std::min(char_idx, num_chars - 1);
                
                // Apply color based on density
                if (char_idx >= 6) {
                    attron(COLOR_PAIR(1) | A_BOLD);  // Green = high density (good)
                } else if (char_idx >= 3) {
                    attron(COLOR_PAIR(3));           // Yellow = medium
                } else if (char_idx >= 1) {
                    attron(A_DIM);                   // Dim = low density
                }
                
                mvaddch(y + 1 + dy, x + 1 + dx, density_chars[char_idx]);
                
                attroff(COLOR_PAIR(1) | A_BOLD);
                attroff(COLOR_PAIR(3));
                attroff(A_DIM);
            }
        }
        
        // Draw center crosshair (only if cell is empty)
        int mid_y = height / 2;
        int mid_x = width / 2;
        int mid_gx = (int)(mid_x * scale_x);
        int mid_gy = (int)(mid_y * scale_y);
        bool center_empty = (mid_gx < grid_size && mid_gy < grid_size &&
                            state_.constellation_density[mid_gy * grid_size + mid_gx] == 0);
        if (center_empty) {
            attron(A_DIM);
            mvaddch(y + 1 + mid_y, x + 1 + mid_x, '+');
            attroff(A_DIM);
        }
        
        // Show modulation name in top-right of box
        const char* mod_name = "";
        switch (state_.constellation_mod_bits) {
            case 1: mod_name = "BPSK"; break;
            case 2: mod_name = "QPSK"; break;
            case 3: mod_name = "8PSK"; break;
            case 4: mod_name = "QAM16"; break;
            case 6: mod_name = "QAM64"; break;
            case 8: mod_name = "QAM256"; break;
            case 10: mod_name = "QAM1024"; break;
            case 12: mod_name = "QAM4096"; break;
        }
        if (mod_name[0]) {
            int name_len = strlen(mod_name);
            attron(A_DIM);
            mvaddstr(y, x + width + 1 - name_len, mod_name);
            attroff(A_DIM);
        }
    }
    
    void draw_scope(int y, int h, int cols) {
        int c1 = 3;
        
        attron(COLOR_PAIR(4) | A_BOLD);
        mvaddstr(y, c1, "[ CONSTELLATION ]");
        attroff(COLOR_PAIR(4) | A_BOLD);
        y += 2;
        
        // Reserve space for signal info (4 lines at bottom) and margins
        int available_h = h - 10;  // Extra for axis labels
        int available_w = cols - 28;  // Space for key on right + axis labels
        
        // Terminal chars are ~2:1 aspect ratio (taller than wide)
        // For visually square display: width should be ~2x height
        int const_height = available_h;
        int const_width = const_height * 2;  // 2:1 aspect ratio compensation
        
        // Clamp to available width
        if (const_width > available_w) {
            const_width = available_w;
            const_height = const_width / 2;
        }
        
        // Minimum sizes
        if (const_height < 9) const_height = 9;
        if (const_width < 17) const_width = 17;
        
        if (const_height >= 9) {
            // Offset to leave room for left axis label
            int x_offset = 6;
            draw_constellation(y + 1, x_offset, const_height, const_width);  // +1 for top axis label
            y += const_height + 5;  // Extra for axis labels
        } else {
            y += 2;
        }
        
        // Show signal info below constellation
        attron(A_DIM);
        mvaddstr(y, c1, "SIGNAL INFO");
        attroff(A_DIM);
        y++;
        
        // Last SNR
        mvaddstr(y, c1, "Last SNR:");
        float snr = state_.last_rx_snr.load();
        if (snr > 10.0f) {
            attron(COLOR_PAIR(1) | A_BOLD);
        } else if (snr > 5.0f) {
            attron(COLOR_PAIR(3) | A_BOLD);
        }
        mvprintw(y, c1 + 12, "%6.1f dB", snr);
        attroff(COLOR_PAIR(1) | A_BOLD);
        attroff(COLOR_PAIR(3) | A_BOLD);
        
        // Modulation
        const char* mod_name = "";
        switch (state_.constellation_mod_bits) {
            case 1: mod_name = "BPSK"; break;
            case 2: mod_name = "QPSK"; break;
            case 3: mod_name = "8PSK"; break;
            case 4: mod_name = "QAM16"; break;
            case 6: mod_name = "QAM64"; break;
            case 8: mod_name = "QAM256"; break;
            case 10: mod_name = "QAM1024"; break;
            case 12: mod_name = "QAM4096"; break;
        }
        mvaddstr(y, c1 + 28, "Modulation:");
        mvaddstr(y, c1 + 42, mod_name[0] ? mod_name : "---");
        y++;
        
        // Carrier level
        mvaddstr(y, c1, "Carrier:");
        float lvl = state_.carrier_level_db.load();
        bool busy = lvl > state_.carrier_threshold_db;
        if (busy) {
            attron(COLOR_PAIR(4) | A_BOLD);
        }
        mvprintw(y, c1 + 12, "%6.1f dB", lvl);
        attroff(COLOR_PAIR(4) | A_BOLD);
        
        // RX/TX counts
        mvaddstr(y, c1 + 28, "RX:");
        attron(COLOR_PAIR(1));
        printw(" %d", state_.rx_frame_count.load());
        attroff(COLOR_PAIR(1));
        addstr("  TX:");
        attron(COLOR_PAIR(2));
        printw(" %d", state_.tx_frame_count.load());
        attroff(COLOR_PAIR(2));
    }
    
    void draw_utils(int y, int h, int cols) {
        int c1 = 3;
        int c2 = cols / 2 + 2;
        int start_y = y;

        auto msgs = state_.get_messages();
        int show = std::min((int)msgs.size(), 4);

        int total_rows = 10 + utils_visible_slots() + std::max(1, show);
        if (state_.perf_logger) {
            int right_rows = 4 + std::max(1, (int)state_.perf_logger->snapshot().size());
            total_rows = std::max(total_rows, right_rows);
        }
        int max_scroll = std::max(0, total_rows - h);
        utils_max_scroll_ = max_scroll;
        if (utils_scroll_ > max_scroll) utils_scroll_ = max_scroll;
        if (utils_scroll_ < 0) utils_scroll_ = 0;
        int row = 0;
        auto vy = [&](int logical_row) -> int {
            int screen_row = logical_row - utils_scroll_;
            if (screen_row < 0 || screen_row >= h) return -1;
            return start_y + screen_row;
        };

        if (state_.perf_logger) {
            int ry = start_y;
            auto rrow = [&](int r) -> int {
                int screen_row = r - utils_scroll_;
                if (screen_row < 0 || screen_row >= h) return -1;
                return start_y + screen_row;
            };
            int r = 0, dy2;
            if ((dy2 = rrow(r++)) >= 0) {
                attron(COLOR_PAIR(4) | A_BOLD);
                mvaddstr(dy2, c2, "[ RX PERFORMANCE ]");
                attroff(COLOR_PAIR(4) | A_BOLD);
            }
            if ((dy2 = rrow(r++)) >= 0) {
                attron(A_DIM);
                mvprintw(dy2, c2, "%-13s %4s %4s %6s %6s %6s %6s",
                         "MODE", "N", "LOST", "SNRav", "SNRmn", "SNRmx", "BERav");
                attroff(A_DIM);
            }
            auto rows = state_.perf_logger->snapshot();
            for (const auto& a : rows) {
                if ((dy2 = rrow(r++)) < 0)
                    break;
                char mode[14];
                snprintf(mode, sizeof(mode), "%s", a.mode.c_str());
                if (a.ber_avg() >= 0)
                    mvprintw(dy2, c2, "%-13s %4d %4d %5.1f  %5.1f  %5.1f  %5.2f%%",
                             mode, a.frames, a.lost, a.snr_avg(), a.snr_min,
                             a.snr_max, a.ber_avg());
                else
                    mvprintw(dy2, c2, "%-13s %4d %4d %5.1f  %5.1f  %5.1f      -",
                             mode, a.frames, a.lost, a.snr_avg(), a.snr_min,
                             a.snr_max);
            }
            if (rows.empty() && (dy2 = rrow(r++)) >= 0) {
                attron(A_DIM);
                mvaddstr(dy2, c2, "(no frames decoded yet)");
                attroff(A_DIM);
            }
            r++;
            if ((dy2 = rrow(r++)) >= 0) {
                attron(A_DIM);
                if (state_.perf_logger->csv_enabled())
                    mvprintw(dy2, c2, "CSV: ON  %d rows  %s",
                             state_.perf_logger->total(),
                             state_.perf_logger->csv_path().c_str());
                else
                    mvprintw(dy2, c2, "CSV: OFF (%d frames tracked)",
                             state_.perf_logger->total());
                attroff(A_DIM);
            }
            (void)ry;
        }

        int dy = vy(row);
        if (dy >= 0) {
            attron(COLOR_PAIR(4) | A_BOLD);
            mvaddstr(dy, c1, "[ ACTIONS ]");
            attroff(COLOR_PAIR(4) | A_BOLD);
        }
        row++;

        const char* actions[] = {
            "Send Test Pattern",
            "Send Random Data",
            "Send Ping",
            "Clear Stats",
            "Auto Threshold",
            "Reconnect Audio",
            "Compose Message",
            "Auto Send",
            "Perf Log CSV",
            "Reset Perf Stats",
            "Auto Alternate"
        };

        int nslots = utils_visible_slots();
        for (int slot = 0; slot < nslots; slot++) {
            dy = vy(row);
            row++;
            if (dy < 0) continue;
            bool sel = (utils_selection_ == slot);
            if (slot == UTILS_TOP_ACTIONS) {
                if (sel) attron(A_BOLD); else attron(A_DIM);
                mvprintw(dy, c1, "%s[%c] TESTING", sel ? "> " : "  ",
                         state_.utils_testing_open ? '-' : '+');
                if (sel) attroff(A_BOLD); else attroff(A_DIM);
                if (!state_.utils_testing_open) {
                    attron(A_DIM);
                    printw("  send tools and mode roster");
                    attroff(A_DIM);
                }
                continue;
            }
            int i = utils_slot_action(slot);
            const char* label = i >= 11 ? ALT_MODES[i - 11].label : actions[i];
            const char* indent = i >= 11 ? "   " : "";
            if (sel) {
                attron(A_BOLD);
                mvprintw(dy, c1, "> %d. %s%s", i + 1, indent, label);
                attroff(A_BOLD);
            } else {
                attron(A_DIM);
                mvprintw(dy, c1, "  %d. %s%s", i + 1, indent, label);
                attroff(A_DIM);
            }
            if (i >= 11) {
                bool on = (state_.alt_mode_mask >> (i - 11)) & 1;
                if (on) {
                    attron(COLOR_PAIR(1) | A_BOLD);
                    printw("  [x]");
                    attroff(COLOR_PAIR(1) | A_BOLD);
                } else {
                    printw("  [ ]");
                }
                if (auto_alt_enabled_ && alt_index_ == i - 11) {
                    attron(COLOR_PAIR(4) | A_BOLD);
                    printw("  <TX");
                    attroff(COLOR_PAIR(4) | A_BOLD);
                }
            }
            if (i == 10) {
                if (auto_alt_enabled_) {
                    attron(COLOR_PAIR(1) | A_BOLD);
                    printw("  [x]");
                    attroff(COLOR_PAIR(1) | A_BOLD);
                    attron(A_DIM);
                    printw(" cycling %d modes @ 25%% duty", __builtin_popcount(state_.alt_mode_mask));
                    attroff(A_DIM);
                } else {
                    printw("  [ ]");
                    attron(A_DIM);
                    printw(" cycle the checked modes below");
                    attroff(A_DIM);
                }
            }
            if (i == 4 && calibrating_threshold_ && sel) {
                int elapsed = (frame_counter_ - calibration_start_frame_) / 30;
                attron(COLOR_PAIR(4) | A_BOLD);
                printw("  [%ds...]", 3 - elapsed);
                attroff(COLOR_PAIR(4) | A_BOLD);
            }
            if (i == 8 && state_.perf_logger) {
                if (state_.perf_logger->csv_enabled()) {
                    attron(COLOR_PAIR(1) | A_BOLD);
                    printw("  [x]");
                    attroff(COLOR_PAIR(1) | A_BOLD);
                } else {
                    printw("  [ ]");
                }
            }
            if (i == 7) {
                if (auto_send_enabled_) {
                    attron(COLOR_PAIR(1) | A_BOLD);
                    printw("  [x]");
                    attroff(COLOR_PAIR(1) | A_BOLD);
                    attron(A_DIM);
                    if (state_.transmitting.load()) {
                        printw(" lorem %dB, TX...", state_.mtu_bytes);
                    } else {
                        float period = auto_send_period();
                        float wait = period - std::chrono::duration<float>(
                            std::chrono::steady_clock::now() - auto_send_last_).count();
                        if (wait < 0) wait = 0;
                        printw(" lorem %dB, next in %.0fs", state_.mtu_bytes, wait);
                    }
                    attroff(A_DIM);
                } else {
                    if (sel) attron(A_BOLD); else attron(A_DIM);
                    printw("  [ ]");
                    if (sel) attroff(A_BOLD); else attroff(A_DIM);
                    attron(A_DIM);
                    printw(" lorem %dB @ 25%% duty", state_.mtu_bytes);
                    attroff(A_DIM);
                }
            }
        }

        row++;

        dy = vy(row);
        if (dy >= 0) {
            attron(COLOR_PAIR(4) | A_BOLD);
            mvaddstr(dy, c1, "[ TEST INFO ]");
            attroff(COLOR_PAIR(4) | A_BOLD);
        }
        row++;

        dy = vy(row);
        if (dy >= 0) {
            attron(A_DIM);
            mvaddstr(dy, c1, "MTU");
            attroff(A_DIM);
            mvprintw(dy, c1 + 14, "%d bytes", state_.mtu_bytes);
            if (state_.fragmentation_enabled) {
                attron(COLOR_PAIR(4));
                printw(" [FRAG]");
                attroff(COLOR_PAIR(4));
            }
        }
        row++;

        dy = vy(row);
        if (dy >= 0) {
            bool size_selected = (utils_selection_ == 0 || utils_selection_ == 1);
            if (size_selected) {
                attron(A_BOLD | COLOR_PAIR(4));
            } else {
                attron(A_DIM);
            }
            mvaddstr(dy, c1, "Test Size");
            if (size_selected) {
                attroff(A_BOLD | COLOR_PAIR(4));
                mvprintw(dy, c1 + 14, "< %d bytes >", state_.random_data_size);
            } else {
                attroff(A_DIM);
                mvprintw(dy, c1 + 14, "%d bytes", state_.random_data_size);
            }
            if (state_.fragmentation_enabled && state_.random_data_size > state_.mtu_bytes) {
                int data_per_frag = state_.mtu_bytes - 5;  // 5-byte fragment header
                int num_frags = (state_.random_data_size + data_per_frag - 1) / data_per_frag;
                attron(COLOR_PAIR(3));
                printw(" (%d frags)", num_frags);
                attroff(COLOR_PAIR(3));
            }
        }
        row++;

        dy = vy(row);
        if (dy >= 0) {
            attron(A_DIM);
            mvaddstr(dy, c1, "Pattern");
            attroff(A_DIM);
            mvaddstr(dy, c1 + 14, "0x55 (alternating)");
        }
        row++;

        dy = vy(row);
        if (dy >= 0) {
            attron(A_DIM);
            mvaddstr(dy, c1, "Frames Sent");
            attroff(A_DIM);
            mvprintw(dy, c1 + 14, "%d", state_.tx_frame_count.load());
        }
        row++;

        row++;
        state_.unread_messages = 0;
        dy = vy(row);
        if (dy >= 0) {
            attron(COLOR_PAIR(4) | A_BOLD);
            mvaddstr(dy, c1, "[ MESSAGES ]");
            attroff(COLOR_PAIR(4) | A_BOLD);
        }
        row++;
        dy = vy(row);
        if (dy >= 0) {
            attron(A_DIM);
            mvaddstr(dy, c1, "Use this for testing only");
            attroff(A_DIM);
        }
        row++;
        {
            int width = cols / 2 - c1 - 2;
            if (width < 20) width = 20;
            if (msgs.empty()) {
                dy = vy(row);
                row++;
                if (dy >= 0) {
                    attron(A_DIM);
                    mvaddstr(dy, c1, "(no messages)");
                    attroff(A_DIM);
                }
            }
            for (int i = (int)msgs.size() - show; i < (int)msgs.size(); i++) {
                dy = vy(row);
                row++;
                if (dy < 0) continue;
                const auto& m = msgs[i];
                std::string line = m.time + " " + (m.outgoing ? "-> " : "<- ") + m.from + ": " + m.text;
                int pair = m.outgoing ? 2 : 1;
                attron(COLOR_PAIR(pair));
                mvaddnstr(dy, c1, line.c_str(), width);
                attroff(COLOR_PAIR(pair));
            }
        }

        int ry = 4;
        attron(COLOR_PAIR(4) | A_BOLD);
        mvaddstr(ry, c2, "[ RECENT ACTIVITY ]");
        attroff(COLOR_PAIR(4) | A_BOLD);
        ry++;
        
        auto packets = state_.get_recent_packets();
        int display_count = std::min((int)packets.size(), h - 3);
        
        for (int i = packets.size() - display_count; i < (int)packets.size(); i++) {
            const auto& pkt = packets[i];
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now() - pkt.timestamp).count();
            
            if (pkt.is_tx) {
                attron(COLOR_PAIR(2) | A_BOLD);
                mvaddstr(ry, c2, "TX");
                attroff(COLOR_PAIR(2) | A_BOLD);
            } else {
                attron(COLOR_PAIR(1) | A_BOLD);
                mvaddstr(ry, c2, "RX");
                attroff(COLOR_PAIR(1) | A_BOLD);
            }
            
            mvprintw(ry, c2 + 3, "%4dB", pkt.size);
            
            // Time ago
            attron(A_DIM);
            if (elapsed < 60) {
                mvprintw(ry, c2 + 10, "%lds ago", elapsed);
            } else {
                mvprintw(ry, c2 + 10, "%ldm ago", elapsed / 60);
            }
            attroff(A_DIM);
            
            // SNR for RX
            if (!pkt.is_tx && pkt.snr > 0) {
                attron(COLOR_PAIR(4) | A_BOLD);
                mvprintw(ry, c2 + 20, "%.0fdB", pkt.snr);
                attroff(COLOR_PAIR(4) | A_BOLD);
            }
            
            ry++;
        }
        
        if (packets.empty()) {
            attron(A_DIM);
            mvaddstr(ry, c2, "No recent packets");
            attroff(A_DIM);
        }
    }
    
    void handle_utils_action() {
        int action = utils_slot_action(utils_selection_);
        if (action < 0) {
            state_.utils_testing_open = !state_.utils_testing_open;
            return;
        }
        switch (action) {
            case 0: {
                if (state_.on_send_data) {
                    std::vector<uint8_t> data(state_.random_data_size, 0x55);
                    state_.on_send_data(data);
                    state_.add_log("Sent test pattern (" + std::to_string(state_.random_data_size) + " bytes)");
                }
                break;
            }
            case 1: {
                if (state_.on_send_data) {
                    std::vector<uint8_t> data(state_.random_data_size);
                    std::random_device rd;
                    std::mt19937 gen(rd());
                    std::uniform_int_distribution<> dis(0, 255);
                    for (auto& b : data) b = dis(gen);
                    state_.on_send_data(data);
                    state_.add_log("Sent random data (" + std::to_string(state_.random_data_size) + " bytes)");
                }
                break;
            }
            case 2: {
                if (state_.on_send_data) {
                    std::string ping = "PING:" + state_.callsign;
                    std::vector<uint8_t> data(ping.begin(), ping.end());
                    state_.on_send_data(data);
                    state_.add_log("Sent ping");
                }
                break;
            }
            case 3: {
                // Clear stats
                state_.rx_frame_count = 0;
                state_.tx_frame_count = 0;
                state_.rx_error_count = 0;
                state_.sync_count = 0;
                state_.preamble_errors = 0;
                state_.symbol_errors = 0;
                state_.erased_symbols = 0;
                state_.crc_errors = 0;
                state_.stats_reset_requested = true;
                state_.total_tx_time = 0;
                state_.add_log("Stats cleared");
                break;
            }
            case 4: {
                // Auto Threshold 
                if (!calibrating_threshold_) {
                    calibrating_threshold_ = true;
                    calibration_start_frame_ = frame_counter_;
                    calibration_max_level_ = -100.0f;
                    state_.add_log("Calibrating threshold...");
                }
                break;
            }
            case 5: {
                state_.add_log("Reconnecting audio...");
                if (state_.on_reconnect_audio) {
                    if (state_.on_reconnect_audio()) {
                        state_.audio_connected = true;
                        state_.add_log("Audio reconnected OK");
                    } else {
                        state_.audio_connected = false;
                        state_.add_log("Audio reconnect FAILED");
                    }
                }
                break;
            }
            case 6: {
                compose_message();
                break;
            }
            case 7: {
                auto_send_enabled_ = !auto_send_enabled_;
                if (auto_send_enabled_) {
                    auto_alt_enabled_ = false;
                    auto_send_last_ = std::chrono::steady_clock::now() -
                        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                            std::chrono::duration<float>(auto_send_period()));
                    state_.add_log("Auto send ON (lorem, 25% duty)");
                } else {
                    state_.add_log("Auto send OFF");
                }
                break;
            }
            case 8: {
                if (state_.perf_logger) {
                    bool on = !state_.perf_logger->csv_enabled();
                    state_.perf_logger->set_csv_enabled(on);
                    state_.add_log(on ? "Perf CSV log ON: " + state_.perf_logger->csv_path()
                                      : "Perf CSV log OFF");
                }
                break;
            }
            case 9: {
                if (state_.perf_logger) {
                    state_.perf_logger->reset();
                    state_.add_log("Perf stats reset");
                }
                break;
            }
            case 10: {
                if (!auto_alt_enabled_ && state_.alt_mode_mask == 0) {
                    state_.add_log("Auto alternate: check some modes below first");
                    break;
                }
                auto_alt_enabled_ = !auto_alt_enabled_;
                if (auto_alt_enabled_) {
                    auto_send_enabled_ = false;
                    auto_send_last_ = std::chrono::steady_clock::now() -
                        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                            std::chrono::duration<float>(auto_send_period()));
                    state_.add_log("Auto alternate ON (" +
                        std::to_string(__builtin_popcount(state_.alt_mode_mask)) +
                        " modes, 25% duty)");
                } else {
                    state_.add_log("Auto alternate OFF");
                }
                break;
            }
            default: {
                if (action >= 11 &&
                    action < 11 + ALT_MODE_COUNT) {
                    int bit = action - 11;
                    state_.alt_mode_mask ^= 1 << bit;
                    state_.save_settings();
                }
                break;
            }
        }
    }

    void apply_alt_mode(int idx) {
        const AltMode& m = ALT_MODES[idx];
        state_.modem_type_index = m.modem_type;
        if (m.modem_type == 0) {
            state_.modulation_index = m.modulation;
            state_.code_rate_index = m.code_rate;
            state_.frame_size = m.frame_size;
        } else if (m.modem_type == 1) {
            state_.mfsk_mode_index = m.mfsk_mode;
        } else {
            state_.robust_mode_index = m.robust_mode;
        }
        state_.update_modem_info();
        if (state_.on_settings_changed)
            state_.on_settings_changed(state_);
    }

    // one frame on air, three frame-times idle: 25% duty cycle keeps the
    // finals safe on rigs rated for 50% digital operation.
    float auto_send_period() const {
        float p = state_.airtime_seconds * 4.0f;
        return p > 4.0f ? p : 4.0f;
    }

    std::vector<uint8_t> lorem_payload(int n) {
        static const char LOREM[] =
            "Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do "
            "eiusmod tempor incididunt ut labore et dolore magna aliqua. Ut "
            "enim ad minim veniam, quis nostrud exercitation ullamco laboris "
            "nisi ut aliquip ex ea commodo consequat. Duis aute irure dolor "
            "in reprehenderit in voluptate velit esse cillum dolore eu "
            "fugiat nulla pariatur. ";
        const int L = (int)sizeof(LOREM) - 1;
        if (n < 1) n = 1;
        int off = std::uniform_int_distribution<int>(0, L - 1)(auto_send_rng_);
        std::vector<uint8_t> out(n);
        for (int i = 0; i < n; i++)
            out[i] = (uint8_t)LOREM[(off + i) % L];
        return out;
    }

    static constexpr int UTILS_TOP_ACTIONS = 7;
    int utils_visible_slots() const {
        return state_.utils_testing_open ? UTILS_ACTION_COUNT + 1
                                         : UTILS_TOP_ACTIONS + 1;
    }
    static int utils_slot_action(int slot) {
        if (slot < UTILS_TOP_ACTIONS) return slot;
        if (slot == UTILS_TOP_ACTIONS) return -1;
        return slot - 1;
    }

    void utils_ensure_visible() {
        int rows, cols;
        getmaxyx(stdscr, rows, cols);
        (void)cols;
        int vis = rows - 6;
        if (vis < 1) vis = 1;
        int sel_row = 1 + utils_selection_;
        if (sel_row < utils_scroll_)
            utils_scroll_ = sel_row;
        else if (sel_row >= utils_scroll_ + vis)
            utils_scroll_ = sel_row - vis + 1;
    }

    void tick_auto_send() {
        if ((!auto_send_enabled_ && !auto_alt_enabled_) || !state_.on_send_data)
            return;
        if (state_.transmitting.load() || state_.ptt_on.load() ||
            state_.tx_queue_size.load() > 0) return;
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration<float>(now - auto_send_last_).count() < auto_send_period())
            return;
        auto_send_last_ = now;
        unsigned seq;
        if (auto_alt_enabled_) {
            if (state_.alt_mode_mask == 0)
                return;
            do {
                alt_index_ = (alt_index_ + 1) % ALT_MODE_COUNT;
            } while (!((state_.alt_mode_mask >> alt_index_) & 1));
            apply_alt_mode(alt_index_);
            seq = alt_seqs_[alt_index_]++ % 100000;
        } else {
            seq = auto_send_seq_++ % 100000;
        }
        auto payload = lorem_payload(state_.mtu_bytes);
        char hdr[16];
        int hn = snprintf(hdr, sizeof(hdr), "SEQ:%05u:", seq);
        for (int i = 0; i < hn && i < (int)payload.size(); i++)
            payload[i] = (uint8_t)hdr[i];
        state_.on_send_data(payload);
    }


    std::string rig_cmd(const std::string& cmd) {
        if (!state_.on_rigctl_command) return "";
        return state_.on_rigctl_command(cmd);
    }

    static std::string format_freq(long long hz) {
        if (hz <= 0) return "---";
        char buf[32];
        if (hz >= 1000000) {
            snprintf(buf, sizeof(buf), "%lld.%03lld.%03lld", hz / 1000000, (hz / 1000) % 1000, hz % 1000);
        } else {
            snprintf(buf, sizeof(buf), "%lld.%03lld", hz / 1000, hz % 1000);
        }
        return buf;
    }

    void set_rig_freq(long long hz) {
        char cmd[48];
        snprintf(cmd, sizeof(cmd), "+F %lld", hz);
        if (TNCUIState::rig_ok(rig_cmd(cmd))) {
            state_.rig_freq_hz = hz;
            state_.add_log("Rig: freq set to " + format_freq(hz));
        } else {
            state_.add_log("Rig: set freq failed");
        }
        state_.rig_refresh_requested = true;
    }

    void adjust_rig_field(int delta) {
        switch (rig_field_) {
            case RIG_FIELD_FREQ: {
                long long f = state_.rig_freq_hz.load();
                if (f <= 0) {
                    state_.add_log("Rig: no frequency data yet");
                    break;
                }
                long long nf = f + delta * RIG_STEP_HZ[rig_step_index_];
                if (nf < 0) nf = 0;
                set_rig_freq(nf);
                break;
            }
            case RIG_FIELD_STEP:
                rig_step_index_ = (rig_step_index_ + delta + RIG_STEP_COUNT) % RIG_STEP_COUNT;
                break;
            case RIG_FIELD_MODE: {
                std::string cur = state_.get_rig_mode();
                int n = (int)RIG_MODE_OPTIONS.size();
                int idx = -1;
                for (int i = 0; i < n; i++) {
                    if (RIG_MODE_OPTIONS[i] == cur) { idx = i; break; }
                }
                idx = idx < 0 ? (delta > 0 ? 0 : n - 1) : (idx + delta + n) % n;
                const std::string& m = RIG_MODE_OPTIONS[idx];
                if (TNCUIState::rig_ok(rig_cmd("+M " + m + " 0"))) {
                    state_.set_rig_mode_cache(m);
                    state_.add_log("Rig: mode set to " + m);
                } else {
                    state_.add_log("Rig: set mode " + m + " failed");
                }
                state_.rig_refresh_requested = true;
                break;
            }
            case RIG_FIELD_POWER: {
                float p = state_.rig_power_level.load();
                if (p < 0) {
                    state_.add_log("Rig: no power data yet");
                    break;
                }
                p = std::max(0.0f, std::min(1.0f, p + delta * 0.05f));
                char cmd[48];
                snprintf(cmd, sizeof(cmd), "+L RFPOWER %.2f", p);
                if (TNCUIState::rig_ok(rig_cmd(cmd))) {
                    state_.rig_power_level = p;
                } else {
                    state_.add_log("Rig: set power failed");
                }
                state_.rig_refresh_requested = true;
                break;
            }
            case RIG_FIELD_DRIVE: {
                float d = state_.tx_drive.load();
                d = std::max(0.05f, std::min(1.0f, d + delta * 0.05f));
                state_.tx_drive = d;
                if (state_.on_settings_changed)
                    state_.on_settings_changed(state_);
                state_.save_settings();
                break;
            }
            case RIG_FIELD_TUNER: {
                int nv = state_.rig_tuner_on.load() == 1 ? 0 : 1;
                char cmd[32];
                snprintf(cmd, sizeof(cmd), "+U TUNER %d", nv);
                if (TNCUIState::rig_ok(rig_cmd(cmd))) {
                    state_.rig_tuner_on = nv;
                    state_.add_log(std::string("Rig: tuner ") + (nv ? "ON" : "OFF"));
                } else {
                    state_.add_log("Rig: tuner toggle failed");
                }
                state_.rig_refresh_requested = true;
                break;
            }
        }
    }

    void rig_enter_action() {
        if (rig_field_ == RIG_FIELD_FREQ) {
            edit_rig_freq();
        } else if (rig_field_ == RIG_FIELD_DRIVE) {
            if (!state_.on_alc_tune) {
                state_.add_log("ALC tune: unavailable");
            } else if (!state_.alc_tune_running.exchange(true)) {
                state_.add_log("ALC tune: starting...");
                std::thread([s = &state_] {
                    float r = s->on_alc_tune();
                    if (!g_running.load())
                        return;
                    if (r > 0) {
                        s->tx_drive = r;
                        s->save_settings();
                        char b[48];
                        snprintf(b, sizeof(b), "ALC tune: drive set to %d%%",
                                 (int)lround(r * 100));
                        s->add_log(b);
                    } else {
                        s->add_log("ALC tune: failed");
                    }
                    s->alc_tune_running = false;
                }).detach();
            }
        } else if (rig_field_ == RIG_FIELD_TUNER) {
            adjust_rig_field(1);
        } else if (rig_field_ == RIG_FIELD_TUNE) {
            state_.add_log("Rig: starting tune cycle...");
            if (TNCUIState::rig_ok(rig_cmd("+G TUNE"))) {
                state_.add_log("Rig: tune cycle started");
            } else {
                state_.add_log("Rig: tune failed (not supported?)");
            }
            state_.rig_refresh_requested = true;
        }
    }

    void edit_rig_freq() {
        int rows, cols;
        getmaxyx(stdscr, rows, cols);

        int dialog_w = 44;
        int dialog_h = 5;
        int dialog_x = (cols - dialog_w) / 2;
        int dialog_y = (rows - dialog_h) / 2;

        for (int y = dialog_y; y < dialog_y + dialog_h; y++) {
            move(y, dialog_x);
            for (int x = 0; x < dialog_w; x++) addch(' ');
        }

        attron(A_BOLD);
        draw_box(dialog_y, dialog_x, dialog_h, dialog_w);
        attroff(A_BOLD);
        mvaddstr(dialog_y, dialog_x + 2, " Set Frequency ");
        attron(A_DIM);
        mvaddstr(dialog_y + 3, dialog_x + 2, "e.g. 7074 (kHz) or 14.074 (MHz)");
        attroff(A_DIM);
        mvaddstr(dialog_y + 2, dialog_x + 2, "Freq: ");

        char buf[24] = {0};
        if (!prompt_input(dialog_y + 2, dialog_x + 8, buf, 15)) return;

        if (strlen(buf) == 0) return;

        long long hz;
        if (strchr(buf, '.')) {
            hz = (long long)llround(atof(buf) * 1e6);
        } else {
            long long v = atoll(buf);
            hz = v < 100000 ? v * 1000 : v;
        }
        if (hz < 10000 || hz > 470000000LL) {
            state_.add_log("Rig: invalid frequency");
            return;
        }
        set_rig_freq(hz);
    }

    void draw_rig_meter_bar(float value, float minv, float maxv, int width, int color_pair) {
        attron(A_DIM);
        addch('[');
        attroff(A_DIM);

        if (std::isnan(value)) {
            attron(A_DIM);
            for (int i = 0; i < width; i++) addch('-');
            attroff(A_DIM);
        } else {
            float frac = (value - minv) / (maxv - minv);
            frac = std::max(0.0f, std::min(1.0f, frac));
            int filled = (int)(frac * width + 0.5f);
            attron(COLOR_PAIR(color_pair) | A_BOLD);
            for (int i = 0; i < filled; i++) addch('=');
            attroff(COLOR_PAIR(color_pair) | A_BOLD);
            for (int i = filled; i < width; i++) addch(' ');
        }

        attron(A_DIM);
        addch(']');
        attroff(A_DIM);
    }

    void draw_rig(int y, int h, int cols) {
        (void)h;
        int c1 = 3;
        int c2 = 16;
        int c3 = cols / 2 + 2;
        int top = y;

        attron(COLOR_PAIR(4) | A_BOLD);
        mvaddstr(y, c1, "[ RIG CONTROL ]");
        attroff(COLOR_PAIR(4) | A_BOLD);
        attron(A_DIM);
        printw("  rigctld %s:%d", state_.rigctl_host.c_str(), state_.rigctl_port);
        attroff(A_DIM);
        if (state_.rigctl_connected.load()) {
            attron(COLOR_PAIR(1) | A_BOLD);
            addstr("  OK");
            attroff(COLOR_PAIR(1) | A_BOLD);
        } else {
            attron(COLOR_PAIR(2) | A_BOLD);
            addstr("  --");
            attroff(COLOR_PAIR(2) | A_BOLD);
        }
        y += 2;

        attron(A_DIM);
        mvaddstr(y, c1, "FREQUENCY");
        attroff(A_DIM);
        y++;

        draw_field_ex(y, c1, c2, "Freq", rig_field_ == RIG_FIELD_FREQ,
                      format_freq(state_.rig_freq_hz.load()), true);
        y++;

        draw_selector_field_ex(y, c1, c2, "Step", rig_field_ == RIG_FIELD_STEP,
                               RIG_STEP_LABELS[rig_step_index_]);
        y++;

        std::string mode = state_.get_rig_mode();
        draw_selector_field_ex(y, c1, c2, "Mode", rig_field_ == RIG_FIELD_MODE,
                               mode.empty() ? "---" : mode);
        y++;

        float pwr = state_.rig_power_level.load();
        char pwr_buf[16];
        if (pwr >= 0) {
            snprintf(pwr_buf, sizeof(pwr_buf), "%d%%", (int)lround(pwr * 100));
        } else {
            snprintf(pwr_buf, sizeof(pwr_buf), "---");
        }
        draw_selector_field_ex(y, c1, c2, "RF Power", rig_field_ == RIG_FIELD_POWER, pwr_buf);
        y++;

        char drv_buf[32];
        if (state_.alc_tune_running.load())
            snprintf(drv_buf, sizeof(drv_buf), "TUNING...");
        else
            snprintf(drv_buf, sizeof(drv_buf), "%d%%  (Enter=ALC tune)",
                     (int)lround(state_.tx_drive.load() * 100));
        draw_selector_field_ex(y, c1, c2, "TX Drive", rig_field_ == RIG_FIELD_DRIVE, drv_buf);
        y += 2;

        attron(A_DIM);
        mvaddstr(y, c1, "ANTENNA TUNER");
        attroff(A_DIM);
        if (state_.rig_tuner_supported.load() == 0) {
            attron(A_DIM);
            mvaddstr(y + 1, c1, "(not supported by rig)");
            attroff(A_DIM);
            y += 3;
        } else {
            y++;

            int tuner = state_.rig_tuner_on.load();
            draw_selector_field_ex(y, c1, c2, "Tuner", rig_field_ == RIG_FIELD_TUNER,
                                   tuner < 0 ? "---" : (tuner ? "ON" : "OFF"));
            y++;

            bool sel = (rig_field_ == RIG_FIELD_TUNE);
            if (sel) {
                attron(A_BOLD);
                mvaddch(y, c1 - 2, '>');
                mvaddstr(y, c1, "Tune");
                attroff(A_BOLD);
                attron(COLOR_PAIR(4) | A_BOLD);
                mvaddstr(y, c2, "[ START TUNE ]");
                attroff(COLOR_PAIR(4) | A_BOLD);
                attron(A_DIM);
                addstr("  [enter]");
                attroff(A_DIM);
            } else {
                attron(A_DIM);
                mvaddstr(y, c1, "Tune");
                mvaddstr(y, c2, "[ START TUNE ]");
                attroff(A_DIM);
            }
            y += 2;
        }

        attron(A_DIM);
        mvaddstr(y, c1, "Freq: Enter=type, </> step");
        attroff(A_DIM);

        int ry = top;
        attron(COLOR_PAIR(4) | A_BOLD);
        mvaddstr(ry, c3, "[ METERS ]");
        attroff(COLOR_PAIR(4) | A_BOLD);

        int64_t last = state_.rig_last_update_ms.load();
        if (last > 0) {
            int64_t now = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
            attron(A_DIM);
            printw("  %.0fs ago", (now - last) / 1000.0f);
            attroff(A_DIM);
        }
        ry += 2;

        int bar_w = std::min(24, cols - c3 - 22);
        if (bar_w < 10) bar_w = 10;

        for (int i = 0; i < RIG_METER_COUNT; i++) {
            float v = state_.rig_meter_values[i].load();
            bool is_temp = strcmp(RIG_METERS[i].level, "TEMP_METER") == 0;
            if (is_temp && std::isnan(v))
                continue;

            attron(A_DIM);
            mvaddstr(ry, c3, RIG_METERS[i].label);
            attroff(A_DIM);
            move(ry, c3 + 9);

            int pair = 1;
            if (strcmp(RIG_METERS[i].level, "SWR") == 0 && !std::isnan(v)) {
                pair = v < 1.5f ? 1 : v < SWR_WARN_THRESHOLD ? 3 : 2;
            } else if (is_temp) {
                pair = v < 60.0f ? 1 : v < 80.0f ? 3 : 2;
            }
            draw_rig_meter_bar(v, RIG_METERS[i].min, RIG_METERS[i].max, bar_w, pair);

            if (std::isnan(v)) {
                attron(A_DIM);
                addstr(" ---");
                attroff(A_DIM);
            } else if (strcmp(RIG_METERS[i].level, "STRENGTH") == 0) {
                // strength is db relative to s9, ~6 db per s-unit
                if (v <= 0) {
                    float s = std::max(0.0f, 9.0f + v / 6.0f);
                    printw(" S%.0f", s);
                } else {
                    printw(" S9+%.0f", v);
                }
            } else if (strcmp(RIG_METERS[i].level, "SWR") == 0) {
                if (pair != 1) attron(COLOR_PAIR(pair) | A_BOLD);
                printw(" %.1f", v);
                if (pair != 1) attroff(COLOR_PAIR(pair) | A_BOLD);
            } else if (is_temp) {
                if (pair != 1) attron(COLOR_PAIR(pair) | A_BOLD);
                printw(" %.0fC", v);
                if (pair != 1) attroff(COLOR_PAIR(pair) | A_BOLD);
            } else if (strstr(RIG_METERS[i].level, "WATTS") != nullptr) {
                printw(" %.0fW", v);
            } else {
                printw(" %.2f", v);
            }
            ry++;
        }
        ry++;

        mvaddstr(ry, c3, "PTT ");
        if (state_.ptt_on.load()) {
            attron(COLOR_PAIR(2) | A_BOLD);
            addstr("TX");
            attroff(COLOR_PAIR(2) | A_BOLD);
        } else {
            attron(COLOR_PAIR(1) | A_BOLD);
            addstr("RX");
            attroff(COLOR_PAIR(1) | A_BOLD);
        }
        ry++;

        attron(A_DIM);
        mvaddstr(ry, c3, "SWR/Power/ALC valid during TX");
        attroff(A_DIM);

        float swr_warn = state_.swr_warn_value.load();
        if (swr_warn > 0.0f) {
            ry += 2;
            attron(COLOR_PAIR(2) | A_BOLD);
            mvprintw(ry, c3, "(!) HIGH SWR %.1f on last TX - check antenna",
                     swr_warn);
            attroff(COLOR_PAIR(2) | A_BOLD);
            if (state_.rig_tuner_supported.load() == 1 &&
                state_.rig_tuner_on.load() == 0) {
                ry++;
                attron(COLOR_PAIR(3));
                mvaddstr(ry, c3, "    (tuner is off)");
                attroff(COLOR_PAIR(3));
            }
        }

        if (!state_.rig_data_valid.load()) {
            ry += 2;
            attron(COLOR_PAIR(3));
            mvaddstr(ry, c3, "No data from rigctld yet...");
            attroff(COLOR_PAIR(3));
        }
    }

    void update_calibration() {
        if (!calibrating_threshold_) return;
        
        // sample current level
        float level = state_.carrier_level_db.load();
        if (level > calibration_max_level_) {
            calibration_max_level_ = level;
        }
        
        int elapsed_frames = frame_counter_ - calibration_start_frame_;
        if (elapsed_frames >= 90) {
            calibrating_threshold_ = false;
            
            // threshold is max + 6dB margin
            float new_threshold = calibration_max_level_ + 6.0f;
            new_threshold = std::max(-80.0f, std::min(0.0f, new_threshold));
            
            state_.carrier_threshold_db = new_threshold;
            apply_settings();
            
            char msg[64];
            snprintf(msg, sizeof(msg), "Threshold set to %.0f dB (noise: %.0f dB)", 
                     new_threshold, calibration_max_level_);
            state_.add_log(msg);
        }
    }
    
    void draw_csma_help(int rows, int cols) {
        int w = 58, h = 19;
        int x0 = (cols - w) / 2, y0 = (rows - h) / 2;
        attron(COLOR_PAIR(4));
        for (int y = y0; y < y0 + h && y < rows; y++)
            mvhline(y, x0, ' ', w);
        mvhline(y0, x0, ACS_HLINE, w);
        mvhline(y0 + h - 1, x0, ACS_HLINE, w);
        mvvline(y0, x0, ACS_VLINE, h);
        mvvline(y0, x0 + w - 1, ACS_VLINE, h);
        mvaddch(y0, x0, ACS_ULCORNER);
        mvaddch(y0, x0 + w - 1, ACS_URCORNER);
        mvaddch(y0 + h - 1, x0, ACS_LLCORNER);
        mvaddch(y0 + h - 1, x0 + w - 1, ACS_LRCORNER);
        attron(A_BOLD);
        mvaddstr(y0, x0 + 3, " CSMA ");
        attroff(A_BOLD);
        attroff(COLOR_PAIR(4));
        int y = y0 + 2, lx = x0 + 2, rx = x0 + 14;
        mvaddstr(y++, lx, "Listens before transmitting so stations");
        mvaddstr(y++, lx, "do not talk over each other.");
        y++;
        auto item = [&](const char* k, const char* v) {
            attron(A_BOLD); mvaddstr(y, lx, k); attroff(A_BOLD);
            mvaddstr(y, rx, v); y++;
        };
        item("Enabled",   "turn channel checking on or off");
        item("Threshold", "level above this counts as busy");
        item("Level",     "what the channel measures right now");
        item("Band",      "HF or VHF/UHF timing for presets");
        item("Preset",    "quick setups by how busy the band is");
        item("Quiet",     "idle time required before contending");
        item("Window",    "random wait range drawn after quiet");
        item("Lead Tone", "keyup tone so others hear us sooner");
        item("Dither",    "callsign delay that separates replies");
        item("Burst",     "packets sent per channel win");
        y++;
        attron(A_DIM);
        mvaddstr(y, lx, "any key to close");
        attroff(A_DIM);
    }

    void draw_help(int rows, int cols) {
        int help_w = 48;
        int help_h = 19;
        int start_x = (cols - help_w) / 2;
        int start_y = (rows - help_h) / 2;
        
        attron(COLOR_PAIR(4));
        for (int y = start_y; y < start_y + help_h && y < rows; y++) {
            mvhline(y, start_x, ' ', help_w);
        }
        
        mvhline(start_y, start_x, ACS_HLINE, help_w);
        mvhline(start_y + help_h - 1, start_x, ACS_HLINE, help_w);
        mvvline(start_y, start_x, ACS_VLINE, help_h);
        mvvline(start_y, start_x + help_w - 1, ACS_VLINE, help_h);
        mvaddch(start_y, start_x, ACS_ULCORNER);
        mvaddch(start_y, start_x + help_w - 1, ACS_URCORNER);
        mvaddch(start_y + help_h - 1, start_x, ACS_LLCORNER);
        mvaddch(start_y + help_h - 1, start_x + help_w - 1, ACS_LRCORNER);
        
        attron(A_BOLD);
        mvaddstr(start_y, start_x + 3, " MODEM73 HELP ");
        attroff(A_BOLD);
        attroff(COLOR_PAIR(4));
        
        int y = start_y + 2;
        int lx = start_x + 2;
        int rx = start_x + 22;
        
        attron(A_BOLD);
        mvaddstr(y, lx, "Navigation");
        attroff(A_BOLD);
        y++;
        mvaddstr(y, lx, "Tab / Shift-Tab");
        mvaddstr(y, rx, "Switch tabs");
        y++;
        mvaddstr(y, lx, "Up / Down");
        mvaddstr(y, rx, "Navigate fields");
        y++;
        mvaddstr(y, lx, "Left / Right");
        mvaddstr(y, rx, "Adjust values");
        y++;
        mvaddstr(y, lx, "Enter");
        mvaddstr(y, rx, "Edit / activate");
        y += 2;
        
        attron(A_BOLD);
        mvaddstr(y, lx, "Config");
        attroff(A_BOLD);
        y++;
        mvaddstr(y, lx, "s");
        mvaddstr(y, rx, "Save preset");
        y++;
        mvaddstr(y, lx, "x");
        mvaddstr(y, rx, "Delete preset");
        y += 2;
        
        attron(A_BOLD);
        mvaddstr(y, lx, "General");
        attroff(A_BOLD);
        y++;
        mvaddstr(y, lx, "Esc");
        mvaddstr(y, rx, "Cancel text input");
        y++;
        mvaddstr(y, lx, "F1");
        mvaddstr(y, rx, "Toggle this help");
        y++;
        mvaddstr(y, lx, "Q");
        mvaddstr(y, rx, "Quit");
        
        attron(A_DIM);
        mvaddstr(start_y + help_h - 2, start_x + (help_w - 24) / 2, "Press any key to close");
        attroff(A_DIM);
    }
    
    TNCUIState& state_;
    bool initialized_ = false;
    std::atomic<bool> running_{false};
    int current_tab_ = 0;
    int current_field_ = 0;
    int config_scroll_ = 0;
    int log_scroll_ = 0;
    int utils_selection_ = 0;
    int utils_scroll_ = 0;
    int utils_max_scroll_ = 0;
    bool auto_send_enabled_ = false;
    std::chrono::steady_clock::time_point auto_send_last_{};
    std::mt19937 auto_send_rng_{std::random_device{}()};
    unsigned auto_send_seq_ = 0;
    bool auto_alt_enabled_ = false;
    int alt_index_ = -1;
    unsigned alt_seqs_[ALT_MODE_COUNT] = {0};
    int rig_field_ = 0;
    int64_t last_ptt_seen_ms_ = 0;
    int recent_sel_ = -1;
    static constexpr int OCC_HIST_SIZE = 60;
    float occ_hist_[OCC_HIST_SIZE] = {};
    int occ_hist_n_ = 0;
    int occ_hist_pos_ = 0;
    int64_t occ_hist_ms_ = 0;
    int rig_step_index_ = 2;
    int saved_stderr_ = -1;
    int frame_counter_ = 0;  
    bool show_help_ = false;  
    bool show_csma_help_ = false;
    
    bool calibrating_threshold_ = false;
    int calibration_start_frame_ = 0;
    float calibration_max_level_ = -100.0f;
};
