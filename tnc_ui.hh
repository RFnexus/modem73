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
#include <cstdio>
#include <cstdlib>
#include <algorithm>
#include <random>
#include <unistd.h>
#include <fcntl.h>

#include "kiss_tnc.hh"

constexpr size_t MAX_LOG_ENTRIES = 500;

const std::vector<std::string> MODULATION_OPTIONS = {
    "BPSK", "QPSK", "8PSK", "QAM16", "QAM64", "QAM256", "QAM1024", "QAM4096"
};

const std::vector<std::string> CODE_RATE_OPTIONS = {
    "1/2", "2/3", "3/4", "5/6"
};

const std::vector<std::string> PTT_TYPE_OPTIONS = {
    "NONE", "RIGCTL", "VOX", "COM"
};

const std::vector<std::string> PTT_LINE_OPTIONS = {
    "DTR", "RTS", "BOTH"
};

struct TNCUIState {
    std::string callsign = "N0CALL";
    int modulation_index = 1;  // default QSPK N 1/2
    int code_rate_index = 0;   
    bool short_frame = false;  
    int center_freq = 1500;
    
    bool csma_enabled = true;
    float carrier_threshold_db = -30.0f;
    int slot_time_ms = 500;
    int p_persistence = 128;
    
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
    
    // PTT 
    int ptt_type_index = 1;  // 0=NONE, 1=RIGCTL, 2=VOX
    
    // Rigctl settings (PTT type 1)
    std::string rigctl_host = "localhost";
    int rigctl_port = 4532;
    std::atomic<bool> rigctl_connected{false};
    std::atomic<bool> audio_connected{true};  // Track audio device health
    
    // VOX settings (PTT type 2)
    int vox_tone_freq = 1200;   // Hz
    int vox_lead_ms = 150;      // ms
    int vox_tail_ms = 100;      // ms
    
    // COM/Serial PTT settings (PTT type 3)
    std::string com_port = "/dev/ttyUSB0";
    int com_ptt_line = 1;       // 0=DTR, 1=RTS, 2=BOTH
    bool com_invert_dtr = false;
    bool com_invert_rts = false;
    

    int mtu_bytes = 0;
    int bitrate_bps = 0;
    float airtime_seconds = 0.0f;
    int random_data_size = 0;
    bool fragmentation_enabled = false;
    
    // stats
    std::atomic<float> total_tx_time{0.0f};  
    

    std::string config_file;
    std::string presets_file;
    
    // Presets 
    struct Preset {
        std::string name;
        // Modem
        int modulation_index;
        int code_rate_index;
        bool short_frame;
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
    
    // Signal visualization
    static constexpr int LEVEL_HISTORY_SIZE = 60;
    std::mutex level_mutex;
    float level_history[LEVEL_HISTORY_SIZE]; 
    int level_history_pos = 0;
    std::atomic<bool> decoding_active{false};
    std::atomic<int> sync_count{0};  
    
    // SNR history
    static constexpr int SNR_HISTORY_SIZE = 32;
    std::mutex snr_mutex;
    float snr_history[SNR_HISTORY_SIZE];
    int snr_history_pos = 0;
    int snr_history_count = 0;  


    struct PacketInfo {
        bool is_tx;
        int size;
        float snr;
        std::chrono::steady_clock::time_point timestamp;
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
        update_modem_info();
    }
    
    // TEMP modem tables
    void update_modem_info() {
        // Modulations: BPSK=0, QPSK=1, 8PSK=2, QAM16=3, QAM64=4, QAM256=5, QAM1024=6, QAM4096=7
        // Code rates: 1/2=0, 2/3=1, 3/4=2, 5/6=3
        static const int payload_short[8][4] = {
            {128, 171, 192, 213},     // BPSK
            {128, 171, 192, 213},     // QPSK
            {512, 684, 768, 852},     // 8PSK
            {256, 342, 384, 426},     // QAM16
            {1024, 1368, 1536, 1704}, // QAM64
            {1024, 1368, 1536, 1704}, // QAM256
            {2048, 2736, 3072, 3408}, // QAM1024
            {2048, 2736, 3072, 3408}, // QAM4096
        };
        
        static const int payload_normal[8][4] = {
            {256, 342, 384, 426},     // BPSK
            {512, 684, 768, 852},     // QPSK
            {1024, 1368, 1536, 1704}, // 8PSK
            {1024, 1368, 1536, 1704}, // QAM16
            {2048, 2736, 3072, 3408}, // QAM64
            {2048, 2736, 3072, 3408}, // QAM256
            {4096, 5472, 6144, 6816}, // QAM1024
            {4096, 5472, 6144, 6816}, // QAM4096
        };
        
        // Bitrate tables in bps 
        static const int bitrate_short[8][4] = {
            {700, 900, 1000, 1100},      // BPSK
            {1100, 1400, 1600, 1800},    // QPSK
            {2100, 2900, 3200, 3600},    // 8PSK
            {2100, 2900, 3200, 3600},    // QAM16
            {4300, 5700, 6400, 7100},    // QAM64
            {5400, 7300, 8200, 9100},    // QAM256
            {7500, 10000, 11200, 12500}, // QAM1024
            {8600, 11400, 12800, 14200}, // QAM4096
        };
        
        static const int bitrate_normal[8][4] = {
            {800, 1100, 1200, 1300},     // BPSK
            {1600, 2100, 2400, 2600},    // QPSK
            {2400, 3200, 3600, 4000},    // 8PSK
            {3200, 4200, 4700, 5200},    // QAM16
            {4800, 6400, 7200, 8000},    // QAM64
            {6300, 8400, 9500, 10500},   // QAM256
            {8300, 11000, 12400, 13800}, // QAM1024
            {9600, 12800, 14400, 16000}, // QAM4096
        };
        
        static const int duration_short[8] = {1500, 1000, 1900, 1000, 1900, 1500, 2200, 1900};
        static const int duration_normal[8] = {2600, 2600, 3400, 2600, 3400, 2600, 4000, 3400};
        
        int mod = modulation_index;
        int rate = code_rate_index;
        
        if (mod < 0 || mod > 7) mod = 1;  
        if (rate < 0 || rate > 3) rate = 0;  
        
        if (short_frame) {
            mtu_bytes = payload_short[mod][rate] - 2; 
            bitrate_bps = bitrate_short[mod][rate];
            airtime_seconds = duration_short[mod] / 1000.0f;
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
    
    void update_level(float db) {
        carrier_level_db = db;
        std::lock_guard<std::mutex> lock(level_mutex);
        level_history[level_history_pos] = db;
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
    
    void add_packet(bool is_tx, int size, float snr = 0.0f) {
        {
            std::lock_guard<std::mutex> lock(packets_mutex);
            recent_packets.push_back({is_tx, size, snr, std::chrono::steady_clock::now()});
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
    
    // Save settings
    bool save_settings() {
        if (config_file.empty()) return false;
        
        FILE* f = fopen(config_file.c_str(), "w");
        if (!f) return false;
        
        fprintf(f, "# MODEM73 Settings\n");
        fprintf(f, "callsign=%s\n", callsign.c_str());
        fprintf(f, "modulation=%d\n", modulation_index);
        fprintf(f, "code_rate=%d\n", code_rate_index);
        fprintf(f, "short_frame=%d\n", short_frame ? 1 : 0);
        fprintf(f, "center_freq=%d\n", center_freq);
        fprintf(f, "csma_enabled=%d\n", csma_enabled ? 1 : 0);
        fprintf(f, "carrier_threshold_db=%.1f\n", carrier_threshold_db);
        fprintf(f, "slot_time_ms=%d\n", slot_time_ms);
        fprintf(f, "p_persistence=%d\n", p_persistence);
        fprintf(f, "fragmentation_enabled=%d\n", fragmentation_enabled ? 1 : 0);
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
        fprintf(f, "# Network\n");
        fprintf(f, "port=%d\n", port);
        fprintf(f, "# Utils\n");
        fprintf(f, "random_data_size=%d\n", random_data_size);
        
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
                else if (strcmp(key, "modulation") == 0) modulation_index = atoi(value);
                else if (strcmp(key, "code_rate") == 0) code_rate_index = atoi(value);
                else if (strcmp(key, "short_frame") == 0) short_frame = atoi(value) != 0;
                else if (strcmp(key, "center_freq") == 0) center_freq = atoi(value);
                else if (strcmp(key, "csma_enabled") == 0) csma_enabled = atoi(value) != 0;
                else if (strcmp(key, "carrier_threshold_db") == 0) carrier_threshold_db = atof(value);
                else if (strcmp(key, "slot_time_ms") == 0) slot_time_ms = atoi(value);
                else if (strcmp(key, "p_persistence") == 0) p_persistence = atoi(value);
                else if (strcmp(key, "fragmentation_enabled") == 0) fragmentation_enabled = atoi(value) != 0;
                else if (strcmp(key, "audio_input") == 0) audio_input_device = value;
                else if (strcmp(key, "audio_output") == 0) audio_output_device = value;
                else if (strcmp(key, "audio_device") == 0) {
                    audio_input_device = value;
                    audio_output_device = value;
                }
                else if (strcmp(key, "ptt_type") == 0) ptt_type_index = atoi(value);
                else if (strcmp(key, "vox_tone_freq") == 0) vox_tone_freq = atoi(value);
                else if (strcmp(key, "vox_lead_ms") == 0) vox_lead_ms = atoi(value);
                else if (strcmp(key, "vox_tail_ms") == 0) vox_tail_ms = atoi(value);
                else if (strcmp(key, "com_port") == 0) com_port = value;
                else if (strcmp(key, "com_ptt_line") == 0) com_ptt_line = atoi(value);
                else if (strcmp(key, "com_invert_dtr") == 0) com_invert_dtr = atoi(value) != 0;
                else if (strcmp(key, "com_invert_rts") == 0) com_invert_rts = atoi(value) != 0;
                else if (strcmp(key, "port") == 0) port = atoi(value);
                else if (strcmp(key, "random_data_size") == 0) random_data_size = atoi(value);
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
            // name,mod,rate,sf,freq,csma,thresh,slot,persist,ptt,vox_freq,vox_lead,vox_tail
            fprintf(f, "preset=%s,%d,%d,%d,%d,%d,%.1f,%d,%d,%d,%d,%d,%d\n",
                    p.name.c_str(),
                    p.modulation_index,
                    p.code_rate_index,
                    p.short_frame ? 1 : 0,
                    p.center_freq,
                    p.csma_enabled ? 1 : 0,
                    p.carrier_threshold_db,
                    p.slot_time_ms,
                    p.p_persistence,
                    p.ptt_type_index,
                    p.vox_tone_freq,
                    p.vox_lead_ms,
                    p.vox_tail_ms);
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
            float thresh;
            
            int n = sscanf(line + 7, "%63[^,],%d,%d,%d,%d,%d,%f,%d,%d,%d,%d,%d,%d",
                       name, &mod, &rate, &sf, &freq, &csma, &thresh, &slot, &persist,
                       &ptt_type, &vox_freq, &vox_lead, &vox_tail);
            
            if (n >= 9) {  
                Preset p;
                p.name = name;
                p.modulation_index = mod;
                p.code_rate_index = rate;
                p.short_frame = sf != 0;
                p.center_freq = freq;
                p.csma_enabled = csma != 0;
                p.carrier_threshold_db = thresh;
                p.slot_time_ms = slot;
                p.p_persistence = persist;

                p.ptt_type_index = (n >= 10) ? ptt_type : 1;
                p.vox_tone_freq = (n >= 11) ? vox_freq : 1200;
                p.vox_lead_ms = (n >= 12) ? vox_lead : 150;
                p.vox_tail_ms = (n >= 13) ? vox_tail : 100;
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
        p.modulation_index = modulation_index;
        p.code_rate_index = code_rate_index;
        p.short_frame = short_frame;
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
        modulation_index = p.modulation_index;
        code_rate_index = p.code_rate_index;
        short_frame = p.short_frame;
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
        
        while (running_) {
            int ch = getch();
            if (ch != ERR) {
                handle_input(ch);
            }
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
        FIELD_MODULATION,
        FIELD_CODERATE,
        FIELD_FRAMESIZE,
        FIELD_FREQ,
        FIELD_CSMA,
        FIELD_THRESHOLD,
        FIELD_PERSISTENCE,
        FIELD_FRAGMENTATION,
        FIELD_AUDIO_INPUT,
        FIELD_AUDIO_OUTPUT,
        FIELD_PTT_TYPE,
        FIELD_VOX_FREQ,
        FIELD_VOX_LEAD,
        FIELD_VOX_TAIL,
        FIELD_COM_PORT,
        FIELD_COM_LINE,
        FIELD_COM_INVERT,
        FIELD_NET_PORT,
        FIELD_PRESET,      
        FIELD_COUNT
    };
    
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
        
        switch (ch) {
            case 'q':
            case 'Q':
                if (state_.on_stop_requested) {
                    state_.on_stop_requested();
                }
                running_ = false;
                break;
                
            case '\t':
                current_tab_ = (current_tab_ + 1) % 4;
                break;
                
            case KEY_BTAB:  // shift tab prev
                current_tab_ = (current_tab_ + 3) % 4;
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
                    utils_selection_ = (utils_selection_ + 5) % 6;
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
                    utils_selection_ = (utils_selection_ + 1) % 6;
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
                    } else if (current_field_ >= FIELD_MODULATION && current_field_ != FIELD_PRESET) {
                        adjust_field(-1);
                    }
                } else if (current_tab_ == 3 && (utils_selection_ == 0 || utils_selection_ == 1)) {
                    int step = 1;
                    if (state_.random_data_size >= 1000) step = 100;
                    else if (state_.random_data_size >= 100) step = 10;
                    state_.random_data_size = std::max(1, state_.random_data_size - step);
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
                    } else if (current_field_ >= FIELD_MODULATION && current_field_ != FIELD_PRESET) {
                        adjust_field(1);
                    }
                } else if (current_tab_ == 3 && (utils_selection_ == 0 || utils_selection_ == 1)) {
                    int step = 1;
                    if (state_.random_data_size >= 1000) step = 100;
                    else if (state_.random_data_size >= 100) step = 10;
                    int max_size = state_.fragmentation_enabled ? 65535 : state_.mtu_bytes;
                    state_.random_data_size = std::min(max_size, state_.random_data_size + step);
                }
                break;
                
            case KEY_PPAGE:
                if (current_tab_ == 2) log_scroll_ = std::max(0, log_scroll_ - 10);
                break;
                
            case KEY_NPAGE:
                if (current_tab_ == 2) log_scroll_ += 10;
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
                    if (current_field_ == FIELD_CALLSIGN) {


                        edit_text_field(FIELD_CALLSIGN);


                    } else if (current_field_ == FIELD_FREQ) {


                        edit_text_field(FIELD_FREQ);


                    } else if (current_field_ == FIELD_NET_PORT) {


                        edit_text_field(FIELD_NET_PORT);


                    } else if (current_field_ == FIELD_COM_PORT) {


                        edit_text_field(FIELD_COM_PORT);


                    } else if (current_field_ == FIELD_AUDIO_INPUT) {


                        show_device_select_dialog(true);  


                    } else if (current_field_ == FIELD_AUDIO_OUTPUT) {


                        show_device_select_dialog(false); 


                    } else if (current_field_ == FIELD_PRESET) {


                        load_selected_preset();


                    }

                } else if (current_tab_ == 3) {
                    

                    handle_utils_action();
                

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

        }
    }
    
    void handle_mouse(MEVENT& event) {
        int rows, cols;
        getmaxyx(stdscr, rows, cols);
        (void)rows;  
        
        if (event.bstate & BUTTON1_CLICKED || event.bstate & BUTTON1_PRESSED) {
            // Tab clicks 
            if (event.y == 2) {
                int tab_width = (cols - 4) / 4;
                if (event.x >= 2 && event.x < 2 + tab_width) {
                    current_tab_ = 0;
                } else if (event.x >= 2 + tab_width && event.x < 2 + tab_width * 2) {
                    current_tab_ = 1;
                } else if (event.x >= 2 + tab_width * 2 && event.x < 2 + tab_width * 3) {
                    current_tab_ = 2;
                } else if (event.x >= 2 + tab_width * 3) {
                    current_tab_ = 3;
                }
            }
            
            if (current_tab_ == 1 && event.x < cols/2 - 2) {  
                int field = -1;
                
                // MODEM section 
                if (event.y >= 5 && event.y <= 9) {
                    field = FIELD_CALLSIGN + (event.y - 5);
                }
                // CSMA section 
                else if (event.y >= 12 && event.y <= 15) {
                    field = FIELD_CSMA + (event.y - 12);
                }
                // PRESET row 
                else if (event.y == 18) {
                    field = FIELD_PRESET;
                }
                
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
        
        curs_set(1);
        echo();
        nodelay(stdscr, FALSE);
        
        // Clear the value area
        move(row, col);
        for (int i = 0; i < 20; i++) addch(' ');
        
        char buf[64] = {0};
        mvgetnstr(row, col, buf, max_len);
        
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
        
        nodelay(stdscr, TRUE);
        noecho();
        curs_set(0);
    }
    
    bool should_skip_field(int field) {
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
        return false;
    }
    
    void adjust_field(int delta) {
        switch (current_field_) {
            case FIELD_MODULATION:
                state_.modulation_index = (state_.modulation_index + delta + 8) % 8;
                break;
            case FIELD_CODERATE:
                state_.code_rate_index = (state_.code_rate_index + delta + 4) % 4;
                break;
            case FIELD_FRAMESIZE:
                state_.short_frame = !state_.short_frame;
                break;
            case FIELD_CSMA:
                state_.csma_enabled = !state_.csma_enabled;
                break;
            case FIELD_THRESHOLD:
                state_.carrier_threshold_db += delta * 2;
                state_.carrier_threshold_db = std::max(-80.0f, std::min(0.0f, state_.carrier_threshold_db));
                break;
            case FIELD_PERSISTENCE:
                state_.p_persistence += delta * 8;
                state_.p_persistence = std::max(0, std::min(255, state_.p_persistence));
                break;
            case FIELD_FRAGMENTATION:
                state_.fragmentation_enabled = !state_.fragmentation_enabled;
                state_.update_modem_info();  // Update random_data_size limits
                state_.add_log("(!) Fragmentation changed, restart required");
                break;
            case FIELD_AUDIO_INPUT:
                break;
            case FIELD_AUDIO_OUTPUT:
                break;
            case FIELD_PTT_TYPE:
                state_.ptt_type_index = (state_.ptt_type_index + delta + 4) % 4;
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
        
        curs_set(1);
        echo();
        nodelay(stdscr, FALSE);
        
        char buf[32] = {0};
        mvgetnstr(dialog_y + 2, dialog_x + 8, buf, 24);
        
        nodelay(stdscr, TRUE);
        noecho();
        curs_set(0);
        
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
        if (state_.ptt_on) {
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
        
        // Mode 
        addstr(" ");
        attron(A_BOLD);
        addstr(state_.callsign.c_str());
        attroff(A_BOLD);
        printw("  %s %s %s %dHz",
               MODULATION_OPTIONS[state_.modulation_index].c_str(),
               CODE_RATE_OPTIONS[state_.code_rate_index].c_str(),
               state_.short_frame ? "S" : "N",
               state_.center_freq);
        
        // Stats 
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
        const char* tabs[] = {"STATUS", "CONFIG", "LOG", "UTILS"};
        int tab_width = (cols - 4) / 4;
        
        for (int i = 0; i < 4; i++) {
            int tx = 2 + i * tab_width;
            
            if (i == current_tab_) {
                attron(A_BOLD);
                mvaddch(2, tx, '>');
                printw(" %s", tabs[i]);
                attroff(A_BOLD);
            } else {
                attron(A_DIM);
                mvprintw(2, tx, "  %s", tabs[i]);
                attroff(A_DIM);
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
        } else {
            draw_utils(content_y, content_h, cols);
        }
        
        // Footer
        attron(A_DIM);
        draw_hline(rows - 2, 0, cols, true, true);
        
        if (current_tab_ == 1) {
            mvaddstr(rows - 1, 2, " ^/v nav  </> adjust  Enter edit  s save  x del  F1 help  Q quit ");
        } else if (current_tab_ == 2) {
            mvaddstr(rows - 1, 2, " ^/v scroll  PgUp/Dn page  F1 help  Q quit ");
        } else if (current_tab_ == 3) {
            mvaddstr(rows - 1, 2, " 1-6 select  Enter run  F1 help  Q quit ");
        } else {
            mvaddstr(rows - 1, 2, " Tab switch  F1 help  Q quit ");
        }
        attroff(A_DIM);
        
        if (show_help_) {
            draw_help(rows, cols);
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
        if (busy) {
            attron(COLOR_PAIR(3) | A_BOLD);
            addstr("  BUSY");
            attroff(COLOR_PAIR(3) | A_BOLD);
        }
        y++;
        
        mvaddstr(y, c1, "Persist");
        mvprintw(y, c2, "%d/%d", state_.p_persistence, 255);
        y++;
        
        mvaddstr(y, c1, "Slot");
        mvprintw(y, c2, "%d ms", state_.slot_time_ms);
        

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
        addstr("Err");
        int errs = state_.rx_error_count.load();
        if (errs > 0) {
            attron(COLOR_PAIR(2));
            printw(" %d", errs);
            attroff(COLOR_PAIR(2));
        } else {
            printw(" %d", errs);
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
        
        int lines = 0;
        for (auto it = packets.rbegin(); it != packets.rend() && lines < max_lines; ++it, ++lines) {
            const auto& pkt = *it;
            
            move(y + lines, x);
            
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
                
                if (level >= row_max) {
                    if (level > thresh) {
                        attron(COLOR_PAIR(4) | A_BOLD);  
                    } else {
                        attron(COLOR_PAIR(1) | A_BOLD);  
                    }
                    addch(ACS_BLOCK);
                    attroff(COLOR_PAIR(1) | A_BOLD);
                    attroff(COLOR_PAIR(4) | A_BOLD);
                } else if (level > row_min) {
                    float frac = (level - row_min) / (row_max - row_min);
                    int idx = (int)(frac * 4);
                    if (idx > 4) idx = 4;
                    if (idx < 0) idx = 0;
                    
                    if (level > thresh) {
                        attron(COLOR_PAIR(4));  
                    } else {
                        attron(COLOR_PAIR(1));  
                    }
                    addstr(blocks[idx]);
                    attroff(COLOR_PAIR(1));
                    attroff(COLOR_PAIR(4));
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
        
        auto get_field_row = [this](int field) -> int {
            int row = 0;
            row++; // header
            if (field == FIELD_CALLSIGN) return row;
            row++;
            if (field == FIELD_MODULATION) return row;
            row++;
            if (field == FIELD_CODERATE) return row;
            row++;
            if (field == FIELD_FRAMESIZE) return row;
            row++;
            if (field == FIELD_FREQ) return row;
            row += 2;
            // CSMA section
            row++; // header
            if (field == FIELD_CSMA) return row;
            row++;
            if (field == FIELD_THRESHOLD) return row;
            row++;
            row++; // 
            if (field == FIELD_PERSISTENCE) return row;
            row += 2;
            // AUDIO/PTT section
            row++; // header
            if (field == FIELD_AUDIO_INPUT) return row;
            row++;
            if (field == FIELD_AUDIO_OUTPUT) return row;
            row++;
            if (field == FIELD_PTT_TYPE) return row;
            row++;
            // VOX fields 
            if (state_.ptt_type_index == 2) {
                if (field == FIELD_VOX_FREQ) return row;
                row++;
                if (field == FIELD_VOX_LEAD) return row;
                row++;
            }
            // COM fields, only when com selected as ptt
            if (state_.ptt_type_index == 3) {
                if (field == FIELD_COM_PORT) return row;
                row++;
                if (field == FIELD_COM_LINE) return row;
                row++;
                if (field == FIELD_COM_INVERT) return row;
                row++;
            }
            row++;
            // NETWORK section
            row++; // header
            if (field == FIELD_NET_PORT) return row;
            row += 2;
            // PRESET section
            row++; // header
            if (field == FIELD_PRESET) return row;
            return row;
        };
        


        if (current_tab_ == 1) {
            int field_row = get_field_row(current_field_);
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
        if (dy >= 0) draw_selector_field(dy, c1, c2, "Modulation", FIELD_MODULATION,
                           MODULATION_OPTIONS[state_.modulation_index]);
        row++;
        
        dy = visible_y(row);
        if (dy >= 0) draw_selector_field(dy, c1, c2, "Code Rate", FIELD_CODERATE,
                           CODE_RATE_OPTIONS[state_.code_rate_index]);
        row++;
        
        dy = visible_y(row);
        if (dy >= 0) draw_selector_field(dy, c1, c2, "Frame Size", FIELD_FRAMESIZE,
                           state_.short_frame ? "SHORT" : "NORMAL");
        row++;
        
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
        if (dy >= 0) {
            char persist_buf[32];
            snprintf(persist_buf, sizeof(persist_buf), "%d", state_.p_persistence);
            draw_selector_field(dy, c1, c2, "Persist", FIELD_PERSISTENCE, persist_buf);
            char slot_buf[32];
            snprintf(slot_buf, sizeof(slot_buf), "%dms", state_.slot_time_ms);
            mvaddstr(dy, c2 + 6, slot_buf);
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
        
        dy = visible_y(row);
        if (dy >= 0) draw_toggle_field(dy, c1, c2, "Enabled", FIELD_FRAGMENTATION, state_.fragmentation_enabled);
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
                char vox_tail_buf[32];
                snprintf(vox_tail_buf, sizeof(vox_tail_buf), "%dms", state_.vox_tail_ms);
                mvaddstr(dy, c2 + 8, vox_tail_buf);
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
                     p.short_frame ? "S" : "N");
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
        bool sel = (field == current_field_);
        
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
    
    void draw_utils(int y, int h, int cols) {
        int c1 = 3;
        int c2 = cols / 2 + 2;
        
        attron(COLOR_PAIR(4) | A_BOLD);
        mvaddstr(y, c1, "[ ACTIONS ]");
        attroff(COLOR_PAIR(4) | A_BOLD);
        y++;
        
        const char* actions[] = {
            "Send Test Pattern",
            "Send Random Data",
            "Send Ping",
            "Clear Stats",
            "Auto Threshold",
            "Reconnect Audio"
        };
        
        for (int i = 0; i < 6; i++) {
            bool sel = (utils_selection_ == i);
            if (sel) {
                attron(A_BOLD);
                mvprintw(y, c1, "> %d. %s", i + 1, actions[i]);
                attroff(A_BOLD);
                
                if (i == 4 && calibrating_threshold_) {
                    int elapsed = (frame_counter_ - calibration_start_frame_) / 30;
                    attron(COLOR_PAIR(4) | A_BOLD);
                    printw("  [%ds...]", 3 - elapsed);
                    attroff(COLOR_PAIR(4) | A_BOLD);
                }
            } else {
                attron(A_DIM);
                mvprintw(y, c1, "  %d. %s", i + 1, actions[i]);
                attroff(A_DIM);
            }
            y++;
        }
        
        y++;
        
        attron(COLOR_PAIR(4) | A_BOLD);
        mvaddstr(y, c1, "[ TEST INFO ]");
        attroff(COLOR_PAIR(4) | A_BOLD);
        y++;
        
        attron(A_DIM);
        mvaddstr(y, c1, "MTU");
        attroff(A_DIM);
        mvprintw(y, c1 + 14, "%d bytes", state_.mtu_bytes);
        if (state_.fragmentation_enabled) {
            attron(COLOR_PAIR(4));
            printw(" [FRAG]");
            attroff(COLOR_PAIR(4));
        }
        y++;
        
        bool size_selected = (utils_selection_ == 0 || utils_selection_ == 1);
        if (size_selected) {
            attron(A_BOLD | COLOR_PAIR(4));
        } else {
            attron(A_DIM);
        }
        mvaddstr(y, c1, "Test Size");
        if (size_selected) {
            attroff(A_BOLD | COLOR_PAIR(4));
            mvprintw(y, c1 + 14, "< %d bytes >", state_.random_data_size);
        } else {
            attroff(A_DIM);
            mvprintw(y, c1 + 14, "%d bytes", state_.random_data_size);
        }
        
        if (state_.fragmentation_enabled && state_.random_data_size > state_.mtu_bytes) {
            int data_per_frag = state_.mtu_bytes - 5;  // 5-byte fragment header
            int num_frags = (state_.random_data_size + data_per_frag - 1) / data_per_frag;
            attron(COLOR_PAIR(3));
            printw(" (%d frags)", num_frags);
            attroff(COLOR_PAIR(3));
        }
        y++;
        
        attron(A_DIM);
        mvaddstr(y, c1, "Pattern");
        attroff(A_DIM);
        mvaddstr(y, c1 + 14, "0x55 (alternating)");
        y++;
        
        attron(A_DIM);
        mvaddstr(y, c1, "Frames Sent");
        attroff(A_DIM);
        mvprintw(y, c1 + 14, "%d", state_.tx_frame_count.load());
        y++;
        
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
        switch (utils_selection_) {
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
                state_.total_tx_time = 0;
                state_.add_log("S");
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
    
    void draw_help(int rows, int cols) {
        int help_w = 40;
        int help_h = 7;
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
        
        mvaddstr(start_y + 2, start_x + (help_w - 11) / 2, "---");
        

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
    int saved_stderr_ = -1;
    int frame_counter_ = 0;  
    bool show_help_ = false;  
    
    bool calibrating_threshold_ = false;
    int calibration_start_frame_ = 0;
    float calibration_max_level_ = -100.0f;
};
