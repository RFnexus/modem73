#pragma once

#include <iostream>
#include <thread>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstring>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <vector>
#include <list>
#include <set>
#include <mutex>
#include <memory>
#include <random>
#include <algorithm>
#include <cctype>

// Network
#include <sys/socket.h>
#include <sys/stat.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>

// Local includes
#include "kiss_tnc.hh"
#include "csma.hh"
#include "tone_dcd.hh"
#include "miniaudio_audio.hh"
#include "rigctl_ptt.hh"
#include "hamlib_ptt.hh"
#include "serial_ptt.hh"
#ifdef WITH_CM108
#include "cm108_ptt.hh"
#endif
#include "modem.hh"
#include "phy/mfsk_modem.hh"
#include "phy/robust_modem.hh"
#include "perf_log.hh"
#include "control_port.hh"

#ifdef WITH_UI
#include "tnc_ui_state.hh"
#endif

inline std::atomic<bool> g_running{true};
inline std::string g_fatal_error;
inline TNCConfig g_config;
inline bool g_verbose = false;
inline bool g_debug = false;
#ifdef WITH_UI
inline bool g_use_ui = true;
#else
inline bool g_use_ui = false;
#endif

#ifdef WITH_UI
inline TNCUIState* g_ui_state = nullptr;
#endif



inline void ui_log(const std::string& msg) {
#ifdef WITH_UI
    if (g_ui_state) {
        g_ui_state->add_log(msg);
    }
#endif
    if (!g_use_ui) {
        std::cout << msg << std::endl;
    } else if (g_verbose) {
        std::cerr << msg << std::endl;
    }
}

inline bool valid_bind_address(const std::string& addr) {
    struct in_addr a;
    return inet_pton(AF_INET, addr.c_str(), &a) == 1;
}

inline bool check_port_available(const std::string& bind_address, int port) {
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        return false;
    }

    int opt = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    if (inet_pton(AF_INET, bind_address.c_str(), &addr.sin_addr) != 1) {
        close(sock);
        return false;
    }
    addr.sin_port = htons(port);

    int result = bind(sock, (struct sockaddr*)&addr, sizeof(addr));
    close(sock);

    return result == 0;
}




class ClientConnection {
public:
    int fd;
    KISSParser parser;
    std::vector<uint8_t> write_buffer;
    std::mutex write_mutex;
    bool connected = true;
    
    ClientConnection(int fd, std::function<void(uint8_t, uint8_t, const std::vector<uint8_t>&)> callback)
        : fd(fd), parser(callback) {}
    
    static constexpr size_t MAX_WRITE_BUFFER = 1024 * 1024;

    void send(const std::vector<uint8_t>& data) {
        std::lock_guard<std::mutex> lock(write_mutex);
        if (write_buffer.size() + data.size() > MAX_WRITE_BUFFER)
            return;
        write_buffer.insert(write_buffer.end(), data.begin(), data.end());
    }
    
    bool flush() {
        std::lock_guard<std::mutex> lock(write_mutex);
        if (write_buffer.empty()) return true;
        
        ssize_t sent = ::send(fd, write_buffer.data(), write_buffer.size(), MSG_NOSIGNAL);
        if (sent < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) return true;
            return false;
        }
        write_buffer.erase(write_buffer.begin(), write_buffer.begin() + sent);
        return true;
    }
};




// TNC
static int parse_seq(const std::vector<uint8_t>& p) {
    if (p.size() < 10 || memcmp(p.data(), "SEQ:", 4) != 0 || p[9] != ':')
        return -1;
    int v = 0;
    for (int i = 4; i < 9; ++i) {
        if (p[i] < '0' || p[i] > '9')
            return -1;
        v = v * 10 + (p[i] - '0');
    }
    return v;
}

// human-readable name for a decoded OFDM operating-mode byte
static std::string ofdm_mode_name(int m) {
    static const char* mods[] = {"BPSK", "QPSK", "8PSK", "QAM16",
                                 "QAM64", "QAM256", "QAM1024", "QAM4096"};
    static const char* rates[] = {"1/2", "2/3", "3/4", "5/6", "1/4",
                                  "1/2x2", "1/4x2", "?"};

    std::string s = mods[(m >> 4) & 7];


    s += " ";
    s += rates[(m >> 1) & 7];
    s += (m & 128) ? " L" : (m & 1) ? " N" : " S";
    return s;
}

class KISSTNC {
public:
    PerfLogger perf_log_;
    std::function<void(float snr, float ber_pct, float level_db)> rx_stats_callback;
    std::function<bool(bool on)> external_ptt;
    std::function<void(const float* samples, int n)> rx_tap;

    KISSTNC(const TNCConfig& config) : config_(config) {
        // Allocate OFDM encoder/decoder
        std::cerr << "  Creating OFDM encoder/decoder" << std::endl;
        encoder_ = std::make_unique<Encoder48k>();
        decoder_ = std::make_unique<Decoder48k>();
        decoder_->configure_frontend(config.center_freq, config.rx_filter_enabled);

        // Allocate MFSK encoder/decoder
        std::cerr << "  Creating MFSK encoder/decoder" << std::endl;
        mfsk_encoder_ = std::make_unique<MFSKEncoder>();
        // one rx instance per tone family so every mfsk mode decodes 


        for (int i = 0; i < 3; ++i)
            mfsk_decoders_[i] = std::make_unique<MFSKDecoder>(
                MFSK_RX_MODES[i], config.center_freq);

        if (config.perf_log)
            perf_log_.set_csv_enabled(true);

        std::cerr << "  Creating RDM encoder/decoder" << std::endl;
        robust_encoder_ = std::make_unique<RobustEncoder>();
        robust_decoder_ = std::make_unique<RobustDecoder>(config.center_freq);
        robust_decoder_n_ = std::make_unique<RobustDecoder>(config.center_freq, true);
        robust_decoder_->debug_log = g_debug;
        robust_decoder_n_->debug_log = g_debug;
        tone_dcd_ = std::make_unique<ToneDCD>(config.center_freq, config.sample_rate);

        std::cerr << "  All encoders/decoders created" << std::endl;


        // Set up constellation callback for UI display
#ifdef WITH_UI
        decoder_->constellation_callback = [this](const DSP::Complex<float>* symbols, int count, int mod_bits) {
            if (g_ui_state) {
                // DSP::Complex<float> is layout-compatible with std::complex<float>
                g_ui_state->update_constellation(
                    reinterpret_cast<const std::complex<float>*>(symbols),
                    count,
                    mod_bits,
                    decoder_->seed_off
                );
            }
        };
#endif

        // Init modem configuration
        modem_config_.sample_rate = config.sample_rate;
        modem_config_.center_freq = config.center_freq;
        modem_config_.call_sign = ModemConfig::encode_callsign(config.callsign.c_str());
        modem_config_.oper_mode = ModemConfig::encode_mode(
            config.modulation.c_str(),
            config.code_rate.c_str(),
            config.frame_size
        );

        if (modem_config_.call_sign < 0) {
            throw std::runtime_error("Invalid callsign '" + config.callsign +
                                     "' (A-Z 0-9 / only, 1-9 characters)");
        }
        if (modem_config_.oper_mode < 0) {
            throw std::runtime_error("Unsupported OFDM combination: " + config.modulation +
                                     " " + config.code_rate + " " +
                                     ModemConfig::frame_size_name(config.frame_size) +
                                     " (micro needs QPSK 1/2; the x2 rates and long frames"
                                     " do not reach the higher QAM orders)");
        }

        if (config.modem_type == 1) {
            payload_size_ = mfsk_encoder_->get_payload_size((MFSKMode)config.mfsk_mode);
        } else if (config.modem_type == 2) {
            payload_size_ = robust_encoder_->get_payload_size((RobustMode)config.robust_mode);
        } else {
            payload_size_ = encoder_->get_payload_size(modem_config_.oper_mode);
        }
        std::cerr << "Payload size: " << payload_size_ << " bytes" << std::endl;
    }
    
    void run() {
        audio_ = std::make_unique<MiniAudio>(config_.audio_input_device,
                                             config_.audio_output_device,
                                             config_.sample_rate);
        audio_->set_log_sink([](const std::string& msg) { ui_log(msg); });
        if (!audio_->open_playback()) {
            throw std::runtime_error("Failed to open audio output");
        }
        if (!audio_->open_capture()) {
            throw std::runtime_error("Failed to open audio capture");
        }
        audio_->set_tx_gain(config_.tx_drive);
        
        std::cerr << "Audio input:  " << config_.audio_input_device << std::endl;
        std::cerr << "Audio output: " << config_.audio_output_device << std::endl;
        
        // Initialize PTT based on ptt_type
        if (config_.ptt_type == PTTType::RIGCTL) {
            rigctl_ = std::make_unique<RigctlPTT>(config_.rigctl_host, config_.rigctl_port);
            if (!rigctl_->connect()) {
                std::cerr << "Could not connect to rigctl" << std::endl;
            }
        } else if (config_.ptt_type == PTTType::HAMLIB) {
#ifdef WITH_HAMLIB
            hamlib_ptt_ = std::make_unique<HamlibPTT>();
            std::string err;
            if (!hamlib_ptt_->open(config_.hamlib_model, config_.hamlib_device, config_.hamlib_baud, err)) {
                ui_log("(!) Hamlib: " + err);
                ui_log("(!) PTT will not key the radio - check rig model and device");
            } else {
                ui_log("Hamlib: rig model " + std::to_string(config_.hamlib_model) + " opened on " + config_.hamlib_device);
            }
#else
            dummy_ptt_ = std::make_unique<DummyPTT>();
            dummy_ptt_->connect();
#endif
        } else if (external_ptt && (config_.ptt_type == PTTType::COM
#ifdef WITH_CM108
                                    || config_.ptt_type == PTTType::CM108
#endif
                                    )) {
            std::cerr << "PTT: external backend" << std::endl;
        } else if (config_.ptt_type == PTTType::COM) {
            serial_ptt_ = std::make_unique<SerialPTT>();
            if (!serial_ptt_->open(config_.com_port, 
                                   static_cast<PTTLine>(config_.com_ptt_line),
                                   config_.com_invert_dtr, 
                                   config_.com_invert_rts)) {
                std::cerr << "Could not open COM port: " << serial_ptt_->last_error() << std::endl;
                ui_log(std::string("(!) COM PTT: ") + serial_ptt_->last_error());
                ui_log("(!) PTT will not key the radio - check COM port in settings");
            }
#ifdef WITH_CM108
        } else if (config_.ptt_type == PTTType::CM108) {
            cm108_ptt_ = std::make_unique<CM108PTT>();
            if (!cm108_ptt_->open(config_.cm108_gpio, config_.cm108_device)) {
                std::cerr << "Could not open CM108 PTT device" << std::endl;
                ui_log("(!) CM108 PTT: device open failed");
                ui_log("(!) PTT will not key the radio - check CM108 in settings");
            }
#endif
        } else {
            dummy_ptt_ = std::make_unique<DummyPTT>();
            dummy_ptt_->connect();
        }
        
        server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
        if (server_fd_ < 0) {
            throw std::runtime_error("Failed to create socket");
        }
        
        int opt = 1;
        setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
        
        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        if (inet_pton(AF_INET, config_.bind_address.c_str(), &addr.sin_addr) != 1) {
            close(server_fd_);
            throw std::runtime_error("Invalid bind address: " + config_.bind_address);
        }
        addr.sin_port = htons(config_.port);

        if (bind(server_fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            close(server_fd_);
            throw std::runtime_error("Failed to bind to port " + std::to_string(config_.port));
        }
        
        if (listen(server_fd_, 5) < 0) {
            close(server_fd_);
            throw std::runtime_error("Failed to listen");
        }
        
        fcntl(server_fd_, F_SETFL, O_NONBLOCK);
        
        std::cerr << "KISS TNC listening on " << config_.bind_address << ":" << config_.port << std::endl;
        std::cerr << "Callsign: " << config_.callsign << std::endl;
        if (config_.modem_type == 1) {
            std::cerr << "Mode: MFSK " << MFSK_MODE_NAMES[config_.mfsk_mode] << std::endl;
        } else if (config_.modem_type == 2) {
            std::cerr << "Mode: ROBUST " << ROBUST_MODE_NAMES[config_.robust_mode] << std::endl;
        } else {
            std::cerr << "Mode: OFDM " << config_.modulation << " " << config_.code_rate
                      << " " << ModemConfig::frame_size_name(config_.frame_size) << std::endl;
        }
        std::cerr << "Payload: " << payload_size_ << " bytes (including 2-byte length prefix)" << std::endl;

        if (config_.csma_enabled) {
            std::cerr << "CSMA: enabled ("
                      << "mode=" << (!config_.csma_sync_only ? "threshold"
                          : config_.csma_ranked ? "ranked" : "sync")
                      << ", threshold=" << config_.carrier_threshold_db
                      << " dB, slot=" << config_.slot_time_ms
                      << " ms, cw=" << config_.csma_cw
                      << ", quiet=" << (config_.csma_quiet_ms > 0
                             ? std::to_string(config_.csma_quiet_ms) + " ms" : "auto")
                      << ", burst=" << config_.csma_burst
                      << ", dither=" << config_.csma_responder_dither
                      << " ms)" << std::endl;
        } else {
            std::cerr << "CSMA: disabled" << std::endl;
        }
        
        std::cerr << "MFSK RX decoders: " << (config_.mfsk_rx_enabled ? "enabled" : "disabled (!) WARNING: MFSK frames will NOT be received") << std::endl;
        std::cerr << "OFDM RX decoder: " << (config_.ofdm_rx_enabled ? "enabled" : "disabled (!) WARNING: OFDM frames will NOT be received") << std::endl;
        std::cerr << "ROBUST RX decoders: " << (config_.robust_rx_enabled ? "enabled" : "disabled (!) WARNING: ROBUST frames will NOT be received") << std::endl;
        if (!config_.mfsk_rx_enabled && config_.modem_type != 1)
            ui_log("(!) MFSK RX decoding is disabled, MFSK frames will NOT be received");
        if (!config_.ofdm_rx_enabled && config_.modem_type != 0)
            ui_log("(!) OFDM RX decoding is disabled, OFDM frames will NOT be received");
        if (!config_.robust_rx_enabled && config_.modem_type != 2)
            ui_log("(!) ROBUST RX decoding is disabled, ROBUST frames will NOT be received");
        std::cerr << "Fragmentation: " << (config_.fragmentation_enabled ? "enabled" : "disabled") << std::endl;
        std::cerr << "TX Blanking: " << (config_.tx_blanking_enabled ? "enabled" : "disabled") << std::endl;
        
        // Show PTT status
        switch (config_.ptt_type) {
            case PTTType::NONE:
                std::cerr << "PTT: disabled" << std::endl;
                break;
            case PTTType::RIGCTL:
                std::cerr << "PTT: rigctl " << config_.rigctl_host << ":" << config_.rigctl_port << std::endl;
                break;
            case PTTType::VOX:
                std::cerr << "PTT: VOX " << config_.vox_tone_freq << "Hz" << std::endl;
                break;
            case PTTType::HAMLIB:
                std::cerr << "PTT: hamlib model " << config_.hamlib_model
                          << " on " << config_.hamlib_device << std::endl;
                break;
            case PTTType::COM:
                std::cerr << "PTT: COM " << config_.com_port 
                          << " (" << PTT_LINE_OPTIONS[config_.com_ptt_line] << ")" << std::endl;
                break;
            case PTTType::CM108:
#ifdef WITH_CM108
                std::cerr << "PTT: CM108 (GPIO" << config_.cm108_gpio << ")" << std::endl;
#else
                std::cerr << "PTT: CM108 not available in this build" << std::endl;
#endif
                break;
        }
        
        // Start threads
        std::thread rx_thread(&KISSTNC::rx_loop, this);
        std::thread tx_thread(&KISSTNC::tx_loop, this);
        std::thread watchdog_thread(&KISSTNC::ptt_watchdog_loop, this);

        int64_t last_audio_check_ms = 0;
        int64_t next_audio_retry_ms = 0;
        int64_t audio_retry_backoff_ms = 5000;

        // Main
        while (g_running) {
            struct sockaddr_in client_addr;
            socklen_t client_len = sizeof(client_addr);
            int client_fd = accept(server_fd_, (struct sockaddr*)&client_addr, &client_len);
            
            if (client_fd >= 0) {
                {
                    std::lock_guard<std::mutex> lock(clients_mutex_);
                    if (clients_.size() >= MAX_CLIENTS) {
                        ui_log("KISS: client limit reached, rejecting connection");
                        close(client_fd);
                        client_fd = -1;
                    }
                }
            }

            if (client_fd >= 0) {
                // Set TCP_NODELAY
                int flag = 1;
                setsockopt(client_fd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));
                fcntl(client_fd, F_SETFL, O_NONBLOCK);

                char ip_str[INET_ADDRSTRLEN];
                inet_ntop(AF_INET, &client_addr.sin_addr, ip_str, sizeof(ip_str));
                ui_log(std::string("Client connected: ") + ip_str + ":" + std::to_string(ntohs(client_addr.sin_port)));

                auto callback = [this](uint8_t port, uint8_t cmd, const std::vector<uint8_t>& data) {
                    handle_kiss_frame(port, cmd, data);
                };

                std::lock_guard<std::mutex> lock(clients_mutex_);
                clients_.emplace_back(std::make_unique<ClientConnection>(client_fd, callback));
                
#ifdef WITH_UI
                if (g_ui_state) {
                    g_ui_state->client_count = clients_.size();
                }
#endif
            }
            
            // Poll clients for data
            {
                std::lock_guard<std::mutex> lock(clients_mutex_);
                for (auto it = clients_.begin(); it != clients_.end();) {
                    auto& client = *it;
                    
                    // Read data
                    uint8_t buf[4096];
                    ssize_t n = recv(client->fd, buf, sizeof(buf), MSG_DONTWAIT);
                    
                    if (n > 0) {
                        client->parser.process(buf, n);
                    } else if (n == 0 || (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK)) {
                        // Disconnected
                        ui_log("Client disconnected");
                        close(client->fd);
                        it = clients_.erase(it);
#ifdef WITH_UI
                        if (g_ui_state) {
                            g_ui_state->client_count = clients_.size();
                        }
#endif
                        continue;
                    }
                    
                    // Flush write buffer
                    if (!client->flush()) {
                        ui_log("Client write error, disconnecting");
                        close(client->fd);
                        it = clients_.erase(it);
#ifdef WITH_UI
                        if (g_ui_state) {
                            g_ui_state->client_count = clients_.size();
                        }
#endif
                        continue;
                    }
                    
                    ++it;
                }
            }
            
            int64_t audio_now_ms = steady_now_ms();
            if (audio_now_ms - last_audio_check_ms >= 1000) {
                last_audio_check_ms = audio_now_ms;
                if (audio_ && !audio_->is_healthy() && audio_now_ms >= next_audio_retry_ms) {
                    ui_log("(!) Audio unhealthy - attempting reconnect");
                    if (audio_->reconnect()) {
                        audio_->set_tx_gain(config_.tx_drive);
                        ui_log("Audio reconnected");
                        audio_retry_backoff_ms = 5000;
                        next_audio_retry_ms = 0;
                    } else {
                        next_audio_retry_ms = audio_now_ms + audio_retry_backoff_ms;
                        ui_log("(!) Audio reconnect failed, retrying in " +
                               std::to_string(audio_retry_backoff_ms / 1000) + "s");
                        audio_retry_backoff_ms = std::min<int64_t>(audio_retry_backoff_ms * 2, 60000);
                    }
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        // Cleanup
        tx_running_ = false;
        rx_running_ = false;

        tx_thread.join();
        rx_thread.join();
        watchdog_thread.join();

        set_ptt(false);

        for (auto& client : clients_) {
            close(client->fd);
        }
        close(server_fd_);
    }
    
private:
    void handle_kiss_frame(uint8_t /*port*/, uint8_t cmd, const std::vector<uint8_t>& data) {
        if (cmd == KISS::CMD_DATA) {
            if (g_verbose) {
                std::cerr << kiss_frame_visualize(data.data(), data.size()) << std::endl;
            }
            
            size_t max_payload = payload_size_ - 2;
            
            if (config_.fragmentation_enabled && fragmenter_.needs_fragmentation(data.size(), max_payload)) {
                auto fragments = fragmenter_.fragment(data, max_payload);
                ui_log("TX: Fragmenting " + std::to_string(data.size()) + " bytes into " + 
                       std::to_string(fragments.size()) + " fragments");
                for (auto& frag : fragments) {
                    if (g_verbose) {
                        std::cerr << packet_visualize(frag.data(), frag.size(), true, true) << std::endl;
                    }
                    tx_queue_.push(TxPacket(std::move(frag)));
                }
#ifdef WITH_UI
                if (g_ui_state) {
                    g_ui_state->tx_queue_size = tx_queue_.size();
                }
#endif
            } else {
                std::vector<uint8_t> frame_data = data;
                if (frame_data.size() > max_payload) {
                    std::cerr << "Warning: Frame too large (" << frame_data.size()
                              << " > " << max_payload << "), truncating" << std::endl;
                    frame_data.resize(max_payload);
                }
                if (g_verbose) {
                    std::cerr << packet_visualize(frame_data.data(), frame_data.size(), true, config_.fragmentation_enabled) << std::endl;
                }
                tx_queue_.push(TxPacket(frame_data));
#ifdef WITH_UI
                if (g_ui_state) {
                    g_ui_state->tx_queue_size = tx_queue_.size();
                }
#endif
            }
        } else {
            std::lock_guard<std::mutex> lock(config_mutex_);
            switch (cmd) {
                // unused handled by modem73 config
            case KISS::CMD_TXDELAY:
                if (!data.empty()) {
                    //
                }
                break;
            case KISS::CMD_P:
                if (!data.empty()) {
                    // 
                }
                break;
            case KISS::CMD_SLOTTIME:
                if (!data.empty()) {
                    //
                }
                break;
            case KISS::CMD_TXTAIL:
                if (!data.empty()) {
                    //
                }
                break;
            case KISS::CMD_FULLDUPLEX:
                if (!data.empty()) {
                    ui_log("KISS full duplex request ignored");
                }
                break;
            case KISS::CMD_SETHW:
                break;
            case KISS::CMD_RETURN:
                break;
            default:
                if (g_verbose) {
                    std::cerr << "Unknown KISS command: 0x" << std::hex << (int)cmd << std::dec << std::endl;
                }
            }
        }
    }
    
    void tx_loop() {
        tx_running_ = true;
        
        // Random number generator for CSMA
        std::random_device rd;
        std::mt19937 gen(rd());
        if (station_id_ == 0)
            station_id_ = (uint16_t)((gen() % 0xFFFE) + 1);
        int csma_stage = 0;
        int64_t csma_stage_ms = 0;
        int boot_attempt = 0;
        int64_t last_burst_end = steady_now_ms() - PARTICIPATION_MS - 1;
        auto beacon_interval_ms = [&]() {
            std::lock_guard<std::mutex> lock(config_mutex_);
            return (int64_t)config_.beacon_interval_s * 1000;
        };
        auto beacon_due = [&]() {
            return beacon_interval_ms() * (70 + (int64_t)(gen() % 61)) / 100;
        };

        int64_t last_id_air_ms = steady_now_ms() - HEARD_EXPIRY_MS - 1;
        int64_t beacon_anchor_ms = steady_now_ms() - beacon_interval_ms() / 2;
        int64_t beacon_due_ms = beacon_due();
        int64_t tx_start_ms = steady_now_ms();

        

        while (tx_running_ && g_running) {
            TxPacket pkt;
            if (tx_queue_.pop(pkt)) {
#ifdef WITH_UI
                if (g_ui_state) {
                    g_ui_state->tx_queue_size = tx_queue_.size();
                }
#endif
                if (pkt.beacon) {
                    bool still_want;
                    {
                        std::lock_guard<std::mutex> lock(config_mutex_);
                        still_want = config_.csma_enabled && config_.csma_sync_only &&
                                     ranked_active();
                    }
                    bool drop = pkt.manual
                        ? !tx_queue_.empty()
                        : (!still_want || !tx_queue_.empty() || !is_tx_allowed());
                    if (drop) {
                        beacon_anchor_ms = steady_now_ms();
                        beacon_due_ms = beacon_due();
                        continue;
                    }
                }
                // CSMA
                bool csma_enabled, csma_sync_only, csma_fast_floor, csma_ranked;
                int csma_band;
                int carrier_sense_ms, slot_time_ms, csma_quiet_ms, csma_cw, csma_dither, csma_burst, modem_type;
                float carrier_threshold_db;
                std::string csma_callsign;
                {
                    std::lock_guard<std::mutex> lock(config_mutex_);
                    csma_enabled = config_.csma_enabled;
                    csma_sync_only = config_.csma_sync_only;
                    csma_fast_floor = config_.csma_fast_floor;
                    csma_band = config_.csma_band;
                    csma_ranked = ranked_active();
                    carrier_sense_ms = config_.carrier_sense_ms;
                    carrier_threshold_db = config_.carrier_threshold_db;
                    slot_time_ms = config_.slot_time_ms;
                    csma_quiet_ms = config_.csma_quiet_ms;
                    csma_cw = config_.csma_cw;
                    csma_dither = config_.csma_responder_dither;
                    csma_burst = std::max(1, std::min(4, config_.csma_burst));
                    csma_callsign = config_.callsign;
                    modem_type = config_.modem_type;
                    if (config_.modem_type == 0) {
                        bool short_ofdm = pkt.oper_mode >= 0
                            ? (pkt.oper_mode & 1) == 0
                            : (config_.frame_size == 0 || config_.frame_size == 3);
                        if (short_ofdm) {
                            slot_time_ms = std::min(slot_time_ms, 300);
                            csma_burst = 4;
                        }
                    }
                }
                if (csma_enabled) {
                    // Wait for TX lockout to clear
                    if (!is_tx_allowed()) {
                        std::cerr << "TX: Waiting for lockout to clear..." << std::endl;
#ifdef WITH_UI
                        if (g_ui_state) g_ui_state->csma_phase = 1;
#endif
                        wait_for_tx_allowed();
                    }

                    CsmaConfig gcfg;
                    gcfg.threshold_db = carrier_threshold_db;
                    gcfg.sync_only = csma_sync_only;
                    gcfg.quiet_ms = csma_quiet_ms > 0 ? csma_quiet_ms : auto_quiet_ms();
                    gcfg.cw = csma_cw;
                    gcfg.slot_ms = slot_time_ms;
                    gcfg.dcd_detect_ms = csma_fast_floor ? 550
                                       : modem_type == 2 ? 780 : 1310;
                    gcfg.contenders = csma_sync_only
                                        ? n_contenders(csma_band == 0) : -1;
                    int raw_pop = gcfg.contenders;
                    while (csma_stage > 0 &&
                           steady_now_ms() - csma_stage_ms >= CSMA_STAGE_DECAY_MS) {
                        csma_stage--;
                        csma_stage_ms += CSMA_STAGE_DECAY_MS;
                    }
                    if (occupancy_pct_.load() > 55 || csma_stage >= 1)
                        gcfg.contenders = -1;
                    if (steady_now_ms() - last_burst_end < 3000 &&
                        gcfg.contenders >= 0 && gcfg.contenders <= 1)
                        gcfg.contenders = 2;
                    if (csma_sync_only && csma_quiet_ms <= 0 &&
                        gcfg.contenders >= 0 && gcfg.contenders <= 1)
                        gcfg.quiet_ms = std::min(gcfg.quiet_ms, 1000);
                    if (pkt.beacon) {
                        gcfg.quiet_ms = RANKED_QUIET_MS;
                        gcfg.contenders = 0;
                        gcfg.slot_ms = 500;
                        gcfg.dcd_detect_ms = 550;
                        gcfg.extra_delay_ms =
                            std::min(7, std::max(4, known_others() + 1)) *
                            CsmaGate::RANKED_SLOT_MS;
                    }
                    int boot_rank = -1;
                    if (csma_ranked && csma_sync_only && !pkt.beacon) {
                        gcfg.quiet_ms = RANKED_QUIET_MS;
                        bool forgotten =
                            steady_now_ms() - last_id_air_ms > HEARD_EXPIRY_MS;
                        gcfg.rank = forgotten ? -1 : ranked_slot(&gcfg.rank_n);
                        if (gcfg.rank >= gcfg.rank_n)
                            yield_attempt_++;
                        if (gcfg.rank < 0) {
                            if (forgotten) {
                                boot_rank = (int)(id_mix(station_id_.load(),
                                                         boot_attempt) % 4);
                                boot_attempt++;
                            }
                            int known = known_others();
                            if (boot_rank >= 0 && known > 0) {
                                gcfg.rank = std::max(4, known + 1) + boot_rank;
                                gcfg.rank_n = gcfg.rank + 1;
                                if (g_debug)
                                    ui_log("CSMA: silent too long, entering "
                                           "after known turns");
                            } else {
                                gcfg.rank_n = 0;
                            }
                        } else {
                            boot_attempt = 0;
                        }
                    }
                    if (g_debug && csma_sync_only) {
                        char dbg[128];
                        char rankbuf[24];
                        if (gcfg.rank < 0)
                            snprintf(rankbuf, sizeof rankbuf, "none");
                        else if (gcfg.rank >= gcfg.rank_n)
                            snprintf(rankbuf, sizeof rankbuf, "yield/%d", gcfg.rank_n);
                        else
                            snprintf(rankbuf, sizeof rankbuf, "%d/%d",
                                     gcfg.rank, gcfg.rank_n);
                        snprintf(dbg, sizeof dbg,
                                 "CSMA: pop %d stage %d occupancy %d%% "
                                 "quiet %d ms rank %s winner %04X",
                                 raw_pop, csma_stage,
                                 occupancy_pct_.load(), gcfg.quiet_ms,
                                 rankbuf, last_winner_id_.load());
                        ui_log(dbg);
                    }
                    gcfg.busy_limit_ms = std::max(30000, 8 * channel_air_ms());
                    int64_t idle_since = steady_now_ms() - last_channel_busy_ms_.load();
                    gcfg.idle_credit_ms = (int)std::max<int64_t>(0,
                        std::min<int64_t>(idle_since, 1000000));
                    if (csma_dither > 0) {
                        uint32_t hash = 2166136261u;
                        for (char c : csma_callsign) {
                            hash ^= (uint8_t)c;
                            hash *= 16777619u;
                        }
                        gcfg.responder_dither_ms = (int)(hash % (uint32_t)csma_dither);
                    }
                    int64_t rx_ms = last_rx_done_ms_.load();
                    gcfg.responder = !pkt.beacon && !(csma_ranked && csma_sync_only) &&
                                     rx_ms > 0 &&
                                     pkt.enqueue_ms >= rx_ms &&
                                     pkt.enqueue_ms - rx_ms <= 5000 &&
                                     steady_now_ms() - rx_ms <= 8000;
                    CsmaGate gate(gcfg, (uint32_t)gen());
#ifdef WITH_UI
                    if (g_ui_state) g_ui_state->csma_window_ms = gate.window_ms();
#endif

                    if (gcfg.responder) {
                        std::cerr << "CSMA: responder priority, quiet "
                                  << gate.quiet_needed_ms() << " ms" << std::endl;
                    } else if (gcfg.idle_credit_ms >= 250) {
                        std::cerr << "CSMA: idle credit " << gcfg.idle_credit_ms
                                  << " ms, window " << gate.window_ms() << " ms"
                                  << std::endl;
                    }

                    bool was_busy = false, was_deaf = false, quiet_logged = false;
                    int busy_episodes = 0;
                    int cur_rank = -1, cur_rank_n = 0;
                    bool beacon_yield = false;
                    while (g_running) {
                        if (pkt.beacon && !tx_queue_.empty()) {
                            beacon_yield = true;
                            break;
                        }
                        bool alive = audio_->capture_alive();
                        float level_db = audio_->instant_level_db(carrier_sense_ms);
                        bool allowed = is_tx_allowed();
                        if (csma_ranked && csma_sync_only) {
                            if (boot_rank >= 0) {
                                int known = known_others();
                                if (known > 0) {
                                    cur_rank = std::max(4, known + 1) + boot_rank;
                                    cur_rank_n = cur_rank + 1;
                                } else {
                                    cur_rank = -1;
                                    cur_rank_n = 0;
                                }
                            } else {
                                cur_rank = ranked_slot(&cur_rank_n);
                            }
                            gate.set_rank(cur_rank, cur_rank_n);
                        }
                        auto v = gate.step(level_db, alive, allowed);
                        if (v == CsmaGate::Verdict::TRANSMIT) {
                            switch (gate.reason()) {
                            case CsmaGate::Reason::NO_AUDIO:
                                std::cerr << "CSMA: no capture audio for "
                                          << gate.deaf_ms() << " ms, transmitting blind"
                                          << std::endl;
                                break;
                            case CsmaGate::Reason::BUSY_OVERRIDE:
                                std::cerr << "CSMA: channel busy for "
                                          << gate.busy_ms() << " ms, transmitting anyway"
                                          << std::endl;
                                break;
                            default:
                                std::cerr << "CSMA: Channel clear (" << level_db
                                          << " dB), transmitting" << std::endl;
                            }
                            break;
                        }
                        if (!alive && !was_deaf) {
                            std::cerr << "CSMA: no capture audio, holding TX" << std::endl;
                        }
                        was_deaf = !alive;
                        bool busy = alive && (!allowed ||
                            (!csma_sync_only && level_db > carrier_threshold_db));
                        if (busy && !was_busy) {
                            busy_episodes++;
                            if (!allowed) {
                                std::cerr << "CSMA: receiving, deferring" << std::endl;
                            } else {
                                std::cerr << "CSMA: Channel busy (" << level_db << " dB > "
                                          << carrier_threshold_db << " dB), deferring"
                                          << std::endl;
                            }
                            quiet_logged = false;
                        }
                        was_busy = busy;
                        if (!quiet_logged && gate.quiet_met()) {
                            std::cerr << "CSMA: Quiet " << gate.quiet_needed_ms()
                                      << " ms met, contention " << gate.contention_left_ms()
                                      << "/" << gate.contention_drawn_ms() << " ms"
                                      << std::endl;
                            quiet_logged = true;
                        }
#ifdef WITH_UI
                        if (g_ui_state) {
                            g_ui_state->csma_rank = cur_rank;
                            g_ui_state->csma_rank_n = cur_rank_n;
                            if (gate.quiet_met()) {
                                g_ui_state->csma_phase = 3;
                                g_ui_state->csma_wait_ms = gate.contention_left_ms();
                                g_ui_state->csma_wait_need = gate.contention_drawn_ms();
                            } else {
                                g_ui_state->csma_phase = 2;
                                g_ui_state->csma_wait_ms = gate.idle_ms();
                                g_ui_state->csma_wait_need = gate.quiet_needed_ms();
                            }
                        }
#endif
                        std::this_thread::sleep_for(std::chrono::milliseconds(gcfg.poll_ms));
                    }
                    if (beacon_yield) {
#ifdef WITH_UI
                        if (g_ui_state) g_ui_state->csma_phase = 0;
#endif
                        beacon_anchor_ms = steady_now_ms();
                        beacon_due_ms = beacon_due();
                        continue;
                    }
                    if (csma_sync_only) {
                        if (busy_episodes >= 2)
                            csma_stage = 2;
                        else if (csma_stage > 0)
                            csma_stage--;
                        csma_stage_ms = steady_now_ms();
                    }
                    if (!g_running)
                        break;
                }

#ifdef WITH_UI
                if (g_ui_state) g_ui_state->csma_phase = 0;
#endif
                TxPacket cur = std::move(pkt);
                bool first = true;
                int remaining = cur.beacon ? 0 : csma_burst - 1;
                while (true) {
                    TxPacket next;
                    bool have_next = remaining > 0 && tx_queue_.pop(next);
#ifdef WITH_UI
                    if (g_ui_state) {
                        g_ui_state->tx_queue_size = tx_queue_.size();
                    }
#endif
                    bool sent = transmit(cur.data, cur.oper_mode, first, !have_next,
                                         cur.beacon);
                    if (!have_next) {
                        if (!cur.beacon)
                            last_burst_end = steady_now_ms();
                        if (sent) {
                            last_id_air_ms = steady_now_ms();
                            beacon_anchor_ms = last_id_air_ms;
                            beacon_due_ms = beacon_due();
                            if (csma_ranked)
                                last_winner_id_.store(station_id_.load());
                        }
                        break;
                    }
                    std::cerr << "CSMA: burst continuation ("
                              << remaining << " left)" << std::endl;
                    cur = std::move(next);
                    if (sent)
                        first = false;
                    --remaining;
                }
            } else {
                int64_t bnow = steady_now_ms();
                if (bnow - beacon_anchor_ms >= beacon_due_ms) {
                    bool want;
                    {
                        std::lock_guard<std::mutex> lock(config_mutex_);
                        want = config_.csma_enabled && config_.csma_sync_only &&
                               ranked_active();
                    }
                    bool participating =
                        bnow - last_burst_end < PARTICIPATION_MS ||
                        bnow - tx_start_ms < PARTICIPATION_MS ||
                        !tx_queue_.empty();
                    if (!participating)
                        want = false;
                    if (want) {
                        TxPacket b;
                        b.beacon = true;
                        tx_queue_.push(std::move(b));
                    } else {
                        beacon_anchor_ms = bnow;
                        beacon_due_ms = beacon_due();
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
    }

    int frame_air_ms() {
        int ps = payload_size_.load();
        uint64_t key = ((uint64_t)config_.modem_type << 48) ^
                       ((uint64_t)(uint32_t)ps << 16) ^
                       (uint32_t)(config_.modem_type == 1 ? config_.mfsk_mode :
                                  config_.modem_type == 2 ? config_.robust_mode :
                                  modem_config_.oper_mode);
        if (key == frame_air_key_) return frame_air_ms_cache_;
        std::vector<uint8_t> dummy(ps > 2 ? ps - 2 : 1, 0x55);
        auto framed = frame_with_length(dummy);
        std::vector<float> samples;
        if (config_.modem_type == 1) {
            samples = mfsk_encoder_->encode(framed.data(), framed.size(),
                modem_config_.center_freq, (MFSKMode)config_.mfsk_mode);
        } else if (config_.modem_type == 2) {
            samples = robust_encoder_->encode(framed.data(), framed.size(),
                modem_config_.center_freq, (RobustMode)config_.robust_mode);
        } else {
            samples = encoder_->encode(framed.data(), framed.size(),
                modem_config_.center_freq, modem_config_.call_sign,
                modem_config_.oper_mode, config_.postamble);
        }
        if (!samples.empty()) {
            frame_air_ms_cache_ = (int)(1000.0f * samples.size() / config_.sample_rate);
            frame_air_key_ = key;
        } else if (frame_air_ms_cache_ <= 0) {
            frame_air_ms_cache_ = 3000;
        }
        return frame_air_ms_cache_;
    }

    int channel_air_ms() {
        int heard = 0;
        if (steady_now_ms() - heard_air_at_ms_.load() <= 120000)
            heard = heard_air_ms_.load();
        return std::max(frame_air_ms(), heard);
    }

    int auto_quiet_ms() {
        int q = channel_air_ms() / 4;
        if (q < 300) q = 300;
        if (q > 3500) q = 3500;
        return q;
    }

    bool ranked_active() const {
        return config_.csma_ranked && config_.tx_lead_tone &&
               (config_.ptt_type == PTTType::VOX ||
                config_.tx_delay_ms >= 250);
    }

    int tx_lead_ms() const {
        if (config_.csma_enabled && config_.tx_lead_tone &&
            config_.tx_delay_ms >= 250)
            return std::max(config_.tx_delay_ms,
                            ToneDCD::MIN_LEAD_MS + TONE_LEAD_GAP_MS);
        return config_.tx_delay_ms;
    }

    bool transmit(const std::vector<uint8_t>& data, int oper_mode_override = -1,
                  bool first = true, bool last = true, bool beacon = false) {
        while (alc_tune_active_.load() && g_running)
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        int tx_mode = (oper_mode_override >= 0) ? oper_mode_override : modem_config_.oper_mode;

        if (beacon) {
            ui_log("TX: presence tone");
        } else if (oper_mode_override >= 0) {
            ui_log("TX: " + std::to_string(data.size()) + " bytes (mode override)");
        } else {
            ui_log("TX: " + std::to_string(data.size()) + " bytes");
        }
        if (g_verbose) {
            std::cerr << packet_visualize(data.data(), data.size(), true, config_.fragmentation_enabled) << std::endl;
        }

        tx_on_air_ = true;
        if (config_.tx_blanking_enabled) {
            tx_blanking_active_ = true;
        }

#ifdef WITH_UI
        if (g_ui_state && !beacon) {
            g_ui_state->transmitting = true;
            g_ui_state->tx_frame_count++;
            std::string mname = config_.modem_type == 2
                ? ROBUST_MODE_NAMES[(oper_mode_override >= 0 &&
                                     oper_mode_override < ROBUST_MODE_COUNT)
                                        ? oper_mode_override : config_.robust_mode]
                : config_.modem_type == 1
                    ? MFSK_MODE_NAMES[config_.mfsk_mode]
                    : ofdm_mode_name(tx_mode);
            g_ui_state->add_packet(true, data.size(), 0, -1.0f, mname);
        }
#endif

        // Add length prefix framing
        bool data_oversize = false;
        {
            size_t cap = payload_size_;
            if (config_.modem_type == 0 && oper_mode_override >= 0)
                cap = encoder_->get_payload_size(oper_mode_override);
            else if (config_.modem_type == 2 && oper_mode_override >= 0 &&
                     oper_mode_override < ROBUST_MODE_COUNT)
                cap = robust_encoder_->get_payload_size((RobustMode)oper_mode_override);
            if (cap < 2 || data.size() > cap - 2) {
                ui_log("(!) TX: " + std::to_string(data.size()) + " byte frame exceeds " +
                       std::to_string(cap >= 2 ? cap - 2 : 0) + " byte capacity of current mode, dropped");
                data_oversize = true;
            }
        }
        auto framed_data = frame_with_length(data);

        // Encode to audio
        std::vector<float> samples;
        if (beacon) {
            // tone only, no data frame
        } else if (config_.modem_type == 1) {
            samples = mfsk_encoder_->encode(
                framed_data.data(), framed_data.size(),
                modem_config_.center_freq,
                (MFSKMode)config_.mfsk_mode
            );
        } else if (config_.modem_type == 2) {
            RobustMode tx_rmode = (oper_mode_override >= 0 &&
                                   oper_mode_override < ROBUST_MODE_COUNT)
                ? (RobustMode)oper_mode_override
                : (RobustMode)config_.robust_mode;
            samples = robust_encoder_->encode(
                framed_data.data(), framed_data.size(),
                modem_config_.center_freq,
                tx_rmode
            );
        } else {
            samples = encoder_->encode(
                framed_data.data(), framed_data.size(),
                modem_config_.center_freq,
                modem_config_.call_sign,
                tx_mode,
                config_.postamble
            );
        }
        
        if (data_oversize) samples.clear();
        if (samples.empty() && !beacon) {
            if (!data_oversize) ui_log("TX: Encoding failed");
            if (!first && last && config_.ptt_type != PTTType::VOX) {
                audio_->write_silence(config_.ptt_tail_ms * config_.sample_rate / 1000);
                audio_->drain_playback();
                if (config_.ptt_type == PTTType::RIGCTL || config_.ptt_type == PTTType::HAMLIB || config_.ptt_type == PTTType::COM
#ifdef WITH_CM108
                    || config_.ptt_type == PTTType::CM108
#endif
                ) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(config_.ptt_tail_ms));
                    set_ptt(false);
                }
            }
            if (last) {
                tx_on_air_ = false;
                tx_blanking_active_ = false;
#ifdef WITH_UI
                if (g_ui_state) g_ui_state->transmitting = false;
#endif
            }
            return false;
        }
        
        float duration = samples.size() / (float)config_.sample_rate;
        float total_tx_duration = duration;

        int64_t overhead_ms = tx_lead_ms() + config_.ptt_tail_ms +
                              config_.vox_lead_ms + config_.vox_tail_ms + BURST_GAP_MS;
        arm_ptt_watchdog((int64_t)(duration * 1000.0f) + overhead_ms);

        // Handle PTT based on type
        if (config_.ptt_type == PTTType::VOX) {
            // VOX mode: generate tone to trigger radio's VOX
            int lead_samples = config_.vox_lead_ms * config_.sample_rate / 1000;
            int tail_ms = beacon ? 0 : config_.vox_tail_ms;
            int tail_samples = tail_ms * config_.sample_rate / 1000;

            bool sig_lead = first &&
                            (beacon || (config_.csma_enabled &&
                                        config_.tx_lead_tone));
            int sig_lead_ms = ToneDCD::MIN_LEAD_MS +
                              std::max(0, config_.vox_lead_ms - 150);
            int gap_frames = 0;
            std::vector<float> lead_tone;
            if (sig_lead) {
                if (!beacon) {
                    gap_frames = TONE_LEAD_GAP_MS * config_.sample_rate / 1000;
                }
                lead_tone = ToneDCD::signature_lead(
                    modem_config_.center_freq,
                    sig_lead_ms * config_.sample_rate / 1000,
                    ToneDCD::LEAD_AMPLITUDE, config_.sample_rate,
                    station_id_);
            } else {
                lead_tone = generate_tone(config_.vox_tone_freq, lead_samples, 0.8f);
            }

            // Generate tail tone
            auto tail_tone = generate_tone(config_.vox_tone_freq, tail_samples, 0.8f);

            total_tx_duration += (sig_lead ? sig_lead_ms
                                           : config_.vox_lead_ms) / 1000.0f +
                                 (gap_frames + tail_samples) /
                                     (float)config_.sample_rate;

            if (sig_lead)
                ui_log("TX: VOX mode, signature lead " +
                       std::to_string(sig_lead_ms) + "ms, " +
                       std::to_string(tail_ms) + "ms tail");
            else
                ui_log("TX: VOX mode, " + std::to_string(config_.vox_tone_freq) + "Hz tone, " +
                       std::to_string(config_.vox_lead_ms) + "ms lead, " +
                       std::to_string(tail_ms) + "ms tail");

#ifdef WITH_UI
            if (g_ui_state) g_ui_state->ptt_on = true;
#endif

            // Transmit: lead tone -> OFDM data -> tail tone
            const int chunk_size = 1024;

            // Lead tone
            for (size_t i = 0; i < lead_tone.size(); i += chunk_size) {
                int n = std::min(chunk_size, (int)(lead_tone.size() - i));
                audio_->write(lead_tone.data() + i, n);
            }
            if (gap_frames > 0)
                audio_->write_silence(gap_frames);

            // OFDM data
            for (size_t i = 0; i < samples.size(); i += chunk_size) {
                int n = std::min(chunk_size, (int)(samples.size() - i));
                if (audio_->write(samples.data() + i, n) < n) break;
            }

            // Tail tone
            for (size_t i = 0; i < tail_tone.size(); i += chunk_size) {
                int n = std::min(chunk_size, (int)(tail_tone.size() - i));
                audio_->write(tail_tone.data() + i, n);
            }
            
            audio_->drain_playback();
            
#ifdef WITH_UI
            if (g_ui_state) g_ui_state->ptt_on = false;
#endif
        } else {
            // RIGCTL, COM, or NONE mode
            total_tx_duration += (first ? tx_lead_ms() : BURST_GAP_MS) / 1000.0f;
            if (last)
                total_tx_duration += config_.ptt_tail_ms / 1000.0f;
            
            ui_log("TX: " + std::to_string(samples.size()) + " samples, " + 
                   std::to_string(duration) + " seconds");
            
            if (first) {
                // PTT on (for RIGCTL or COM mode)
                if (config_.ptt_type == PTTType::RIGCTL || config_.ptt_type == PTTType::HAMLIB || config_.ptt_type == PTTType::COM
#ifdef WITH_CM108
                    || config_.ptt_type == PTTType::CM108
#endif
                ) {
                    set_ptt(true);
                    std::this_thread::sleep_for(std::chrono::milliseconds(config_.ptt_delay_ms));
                }

                // Leading silence (TXDelay)
                int lead_frames = tx_lead_ms() * config_.sample_rate / 1000;
                if (beacon)
                    lead_frames = std::max(tx_lead_ms(),
                                           ToneDCD::MIN_LEAD_MS + TONE_LEAD_GAP_MS) *
                                  config_.sample_rate / 1000;
                if (beacon || (config_.csma_enabled && config_.tx_lead_tone &&
                               config_.tx_delay_ms >= 250)) {
                    int gap_frames = TONE_LEAD_GAP_MS * config_.sample_rate / 1000;
                    auto lead = ToneDCD::signature_lead(modem_config_.center_freq,
                                                        lead_frames - gap_frames,
                                                        ToneDCD::LEAD_AMPLITUDE,
                                                        config_.sample_rate,
                                                        station_id_);
                    for (size_t i = 0; i < lead.size(); i += 1024) {
                        int n = std::min(1024, (int)(lead.size() - i));
                        audio_->write(lead.data() + i, n);
                    }
                    audio_->write_silence(gap_frames);
                } else {
                    audio_->write_silence(lead_frames);
                }
            } else {
                audio_->write_silence(BURST_GAP_MS * config_.sample_rate / 1000);
            }

            // Transmit audio
            const int chunk_size = 1024;
            for (size_t i = 0; i < samples.size(); i += chunk_size) {
                int n = std::min(chunk_size, (int)(samples.size() - i));
                if (audio_->write(samples.data() + i, n) < n) break;
            }

            if (last) {
                // Trailing silence
                audio_->write_silence(config_.ptt_tail_ms * config_.sample_rate / 1000);
                audio_->drain_playback();

                // PTT off
                if (config_.ptt_type == PTTType::RIGCTL || config_.ptt_type == PTTType::HAMLIB || config_.ptt_type == PTTType::COM
#ifdef WITH_CM108
                    || config_.ptt_type == PTTType::CM108
#endif
                ) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(config_.ptt_tail_ms));
                    set_ptt(false);
                }
            }
        }
        
        if (last) {
            tx_on_air_ = false;
            tx_blanking_active_ = false;
        }
        last_channel_busy_ms_.store(steady_now_ms());

#ifdef WITH_UI
        if (g_ui_state) {
            if (last) {
                g_ui_state->transmitting = false;
            }
            g_ui_state->total_tx_time = g_ui_state->total_tx_time.load() + total_tx_duration;
        }
#endif
        return true;
    }

    // Generate a sine wave tone for VOX triggering
    std::vector<float> generate_tone(int freq_hz, int num_samples, float amplitude = 0.8f) {
        std::vector<float> tone(num_samples);
        float phase_inc = 2.0f * M_PI * freq_hz / config_.sample_rate;
        
        for (int i = 0; i < num_samples; i++) {
            // Apply envelope to avoid clicks
            float envelope = 1.0f;
            int ramp_samples = config_.sample_rate / 100;  
            if (i < ramp_samples) {
                envelope = (float)i / ramp_samples;
            } else if (i > num_samples - ramp_samples) {
                envelope = (float)(num_samples - i) / ramp_samples;
            }
            
            tone[i] = amplitude * envelope * std::sin(phase_inc * i);
        }
        
        return tone;
    }
    
    void rx_loop() {
        rx_running_ = true;
        
        std::vector<float> buffer(1024);
        int level_update_counter = 0;
        const int LEVEL_UPDATE_INTERVAL = 5;
        
        auto deliver_to_clients = [this](const std::vector<uint8_t>& payload, float snr, float ber_pct, bool was_reassembled,
                                         const std::string& mode = "", std::string callsign = "") {
            last_rx_done_ms_.store(steady_now_ms());
            ui_log("RX: " + std::to_string(payload.size()) + " bytes" +
                   (mode.empty() ? "" : " " + mode) + ", SNR=" +
                   std::to_string((int)snr) + "dB" + (was_reassembled ? " (reassembled)" : ""));
            if (g_verbose) {
                std::cerr << packet_visualize(payload.data(), payload.size(), false, false) << std::endl;
            }

#ifdef WITH_UI
            if (g_ui_state) {
                if (callsign.empty() && payload.size() > 4 && !memcmp(payload.data(), "M73:", 4)) {
                    auto sep = std::find(payload.begin() + 4, payload.end(), (uint8_t)':');
                    if (sep != payload.end() && sep - payload.begin() <= 16)
                        callsign.assign(payload.begin() + 4, sep);
                }
                g_ui_state->add_packet(false, payload.size(), snr, ber_pct, mode, callsign);
            }
#endif

            if (payload.size() > 4 && !memcmp(payload.data(), "M73:", 4)) {
                auto sep = std::find(payload.begin() + 4, payload.end(), (uint8_t)':');
                if (sep != payload.end() && sep - payload.begin() <= 16) {
                    std::string from(payload.begin() + 4, sep);
                    std::string text(sep + 1, payload.end());
                    if (text.size() <= 200) {
                        for (auto& c : from)
                            if (!isprint((unsigned char)c)) c = '?';
                        for (auto& c : text)
                            if ((unsigned char)c < 32) c = ' ';
                        std::cerr << "MSG from " << from << ": " << text << std::endl;
#ifdef WITH_UI
                        if (g_ui_state) {
                            g_ui_state->add_message(from, text, false);
                            g_ui_state->add_log("MSG from " + from);
                        }
#endif
                    }
                }
            }

            if (rx_stats_callback) {
                float level_db = audio_ ? audio_->instant_level_db(200) : 0.0f;
                rx_stats_callback(snr, ber_pct, level_db);
            }

            auto kiss_frame = KISSParser::wrap(payload);

            std::lock_guard<std::mutex> lock(clients_mutex_);
            for (auto& client : clients_) {
                client->send(kiss_frame);
            }
        };
        
        // OFDM frame callback
        auto frame_callback = [this, &deliver_to_clients](const uint8_t* data, size_t len) {
            set_tx_lockout(RX_LOCKOUT_SECONDS);

            float snr = decoder_->get_last_snr();
            float last_ber = decoder_->get_last_ber();
            float ber_pct = (last_ber >= 0) ? last_ber * 100.0f : -1.0f;
            float ber_ema = decoder_->get_ber_ema();

#ifdef WITH_UI
            if (g_ui_state) {
                g_ui_state->rx_frame_count++;
                g_ui_state->receiving = false;
                g_ui_state->last_rx_snr = snr;
                if (ber_ema >= 0)
                    g_ui_state->last_rx_ber = ber_ema;
            }
#endif

            auto payload = unframe_length(data, len);
            perf_log_.record(ofdm_mode_name(decoder_->oper_mode), snr, ber_pct,
                             (int)len, parse_seq(payload));

            if (payload.empty()) {
                ui_log("RX: Empty payload after unframing");
#ifdef WITH_UI
                if (g_ui_state) g_ui_state->rx_error_count++;
#endif
                return;
            }

            if (reassembler_.is_fragment(payload)) {
                if (g_verbose) {
                    std::cerr << packet_visualize(payload.data(), payload.size(), false, true) << std::endl;
                }

                auto reassembled = reassembler_.process(payload);
                if (!reassembled.empty()) {
                    ui_log("RX: Reassembled " + std::to_string(reassembled.size()) + " bytes from fragments");
                    deliver_to_clients(reassembled, snr, ber_pct, true,
                                       ofdm_mode_name(decoder_->oper_mode), decoder_->last_call_);
                }
            } else {
                deliver_to_clients(payload, snr, ber_pct, false,
                                   ofdm_mode_name(decoder_->oper_mode), decoder_->last_call_);
            }
        };

        auto robust_frame_callback = [this, &deliver_to_clients](const uint8_t* data, size_t len) {
            set_tx_lockout(RX_LOCKOUT_SECONDS);
            float snr = robust_decoder_->get_last_snr();
            float ber_pct = 100.0f * robust_decoder_->get_last_ber();
#ifdef WITH_UI
            if (g_ui_state) {
                g_ui_state->rx_frame_count++;
                g_ui_state->receiving = false;
                g_ui_state->last_rx_snr = snr;
                g_ui_state->last_rx_ber = ber_pct >= 0 ? ber_pct / 100.0f : -1.0f;
            }
#endif
            auto payload = unframe_length(data, len);
            perf_log_.record(ROBUST_MODE_NAMES[(int)robust_decoder_->get_last_mode()],
                             snr, ber_pct, (int)len, parse_seq(payload));
            if (payload.empty()) {
                ui_log("RDM RX: Empty payload after unframing");
#ifdef WITH_UI
                if (g_ui_state) g_ui_state->rx_error_count++;
#endif
                return;
            }
            if (reassembler_.is_fragment(payload)) {
                auto reassembled = reassembler_.process(payload);
                if (!reassembled.empty()) {
                    ui_log("RDM RX: Reassembled " + std::to_string(reassembled.size()) + " bytes");
                    deliver_to_clients(reassembled, snr, ber_pct, true,
                                       ROBUST_MODE_NAMES[(int)robust_decoder_->get_last_mode()]);
                }
            } else {
                deliver_to_clients(payload, snr, ber_pct, false,
                                   ROBUST_MODE_NAMES[(int)robust_decoder_->get_last_mode()]);
            }
        };

        auto robust_n_frame_callback = [this, &deliver_to_clients](const uint8_t* data, size_t len) {
            set_tx_lockout(RX_LOCKOUT_SECONDS);
            float snr = robust_decoder_n_->get_last_snr();
            float ber_pct = 100.0f * robust_decoder_n_->get_last_ber();
#ifdef WITH_UI
            if (g_ui_state) {
                g_ui_state->rx_frame_count++;
                g_ui_state->receiving = false;
                g_ui_state->last_rx_snr = snr;
                g_ui_state->last_rx_ber = ber_pct >= 0 ? ber_pct / 100.0f : -1.0f;
            }
#endif
            auto payload = unframe_length(data, len);
            perf_log_.record(ROBUST_MODE_NAMES[(int)robust_decoder_n_->get_last_mode()],
                             snr, ber_pct, (int)len, parse_seq(payload));
            if (payload.empty()) {
                ui_log("RDMn RX: Empty payload after unframing");
#ifdef WITH_UI
                if (g_ui_state) g_ui_state->rx_error_count++;
#endif
                return;
            }
            if (reassembler_.is_fragment(payload)) {
                auto reassembled = reassembler_.process(payload);
                if (!reassembled.empty()) {
                    ui_log("RDMn RX: Reassembled " + std::to_string(reassembled.size()) + " bytes");
                    deliver_to_clients(reassembled, snr, ber_pct, true,
                                       ROBUST_MODE_NAMES[(int)robust_decoder_n_->get_last_mode()]);
                }
            } else {
                deliver_to_clients(payload, snr, ber_pct, false,
                                   ROBUST_MODE_NAMES[(int)robust_decoder_n_->get_last_mode()]);
            }
        };

        auto make_mfsk_callback = [this, &deliver_to_clients](MFSKDecoder* dec) {
          return [this, &deliver_to_clients, dec](const uint8_t* data, size_t len) {
            set_tx_lockout(RX_LOCKOUT_SECONDS);

            float snr = dec->get_last_snr();
            float last_ber = dec->get_last_ber();
            float ber_pct = (last_ber >= 0) ? last_ber * 100.0f : -1.0f;

#ifdef WITH_UI
            if (g_ui_state)
                g_ui_state->receiving = false;
#endif

            auto payload = unframe_length(data, len);
            if (payload.empty()) {
                ++mfsk_soft_errors_;
                if (g_verbose)
                    std::cerr << "MFSK RX: empty payload (soft error "
                              << mfsk_soft_errors_ << ")" << std::endl;
                return;
            }
#ifdef WITH_UI
            if (g_ui_state) {
                g_ui_state->rx_frame_count++;
                g_ui_state->last_rx_snr = snr;
            }
#endif
            perf_log_.record(MFSK_MODE_NAMES[(int)dec->get_last_decoded_mode()],
                             snr, ber_pct, (int)len, parse_seq(payload));

            if (reassembler_.is_fragment(payload)) {
                auto reassembled = reassembler_.process(payload);
                if (!reassembled.empty()) {
                    ui_log("MFSK RX: Reassembled " + std::to_string(reassembled.size()) + " bytes");
                    deliver_to_clients(reassembled, snr, ber_pct, true,
                                       MFSK_MODE_NAMES[(int)dec->get_last_decoded_mode()]);
                }
            } else {
                deliver_to_clients(payload, snr, ber_pct, false,
                                   MFSK_MODE_NAMES[(int)dec->get_last_decoded_mode()]);
            }
          };
        };
        MFSKDecoder::FrameCallback mfsk_callbacks[3];
        for (int i = 0; i < 3; ++i)
            mfsk_callbacks[i] = make_mfsk_callback(mfsk_decoders_[i].get());

        bool was_blanking = false;
        bool was_on_air = false;

        while (rx_running_ && g_running) {
            int n = audio_->read(buffer.data(), buffer.size());
            if (n > 0) {
                if (rx_tap) rx_tap(buffer.data(), n);
                bool blanking = tx_blanking_active_.load();

                {
                    int64_t now_ms = steady_now_ms();
                    bool loud = audio_->instant_level_db(config_.carrier_sense_ms) >
                                config_.carrier_threshold_db;
                    bool occupied = (loud && !config_.csma_sync_only) || !is_tx_allowed();
                    if (occupied || blanking)
                        last_channel_busy_ms_.store(now_ms);
                    if (occupied) {
                        if (spell_start_ms_ < 0)
                            spell_start_ms_ = now_ms;
                        spell_last_ms_ = now_ms;
                    } else if (spell_start_ms_ >= 0) {
                        int64_t spell = spell_last_ms_ - spell_start_ms_;
                        if (spell >= 700 &&
                            (spell > heard_air_ms_.load() ||
                             now_ms - heard_air_at_ms_.load() > 120000)) {
                            heard_air_ms_.store((int)std::min<int64_t>(spell, 60000));
                            heard_air_at_ms_.store(now_ms);
                        }
                        spell_start_ms_ = -1;
                    }
                    if (occ_last_ms_ > 0 && now_ms > occ_last_ms_) {
                        float dt = (now_ms - occ_last_ms_) / 1000.0f;
                        if (dt < 5.0f) {
                            float a = std::min(1.0f, dt / 30.0f);
                            float x = ((loud && !config_.csma_sync_only) ||
                                       blanking || dcd_active_) ? 1.0f : 0.0f;
                            occupancy_ema_ += (x - occupancy_ema_) * a;
                            float xo = ((loud && !config_.csma_sync_only) ||
                                        dcd_active_) ? 1.0f : 0.0f;
                            occupancy_other_ema_ += (xo - occupancy_other_ema_) * a;
                            occupancy_pct_.store(
                                (int)(occupancy_other_ema_ * 100.0f));
                        }
                    }
                    occ_last_ms_ = now_ms;
#ifdef WITH_UI
                    if (g_ui_state) {
                        g_ui_state->channel_occupancy = occupancy_ema_;
                        g_ui_state->dcd_active = dcd_active_;
                    }
#endif
                }

                if (blanking) {
                    was_blanking = true;
                    dcd_active_ = false;
                } else {
                    if (decoder_reconfig_pending_.exchange(false)) {
                        int cf;
                        bool rxf;
                        {
                            std::lock_guard<std::mutex> lock(config_mutex_);
                            cf = config_.center_freq;
                            rxf = config_.rx_filter_enabled;
                        }
                        decoder_->configure_frontend(cf, rxf);
                        for (int i = 0; i < 3; ++i)
                            mfsk_decoders_[i]->configure(MFSK_RX_MODES[i], cf);
                        robust_decoder_->configure(cf);
                        robust_decoder_n_->configure(cf);
                        tone_dcd_->configure(cf);
                    }
                    if (was_blanking) {
                        decoder_->reset();
                        for (auto& d : mfsk_decoders_) d->reset();
                        robust_decoder_->reset();
                        robust_decoder_n_->reset();
                        tone_dcd_->reset();
                        tone_hold_until_ms_ = 0;
                        tone_run_start_ms_ = -1;
                        was_blanking = false;
                    }
                    bool mfsk_rx = config_.mfsk_rx_enabled || config_.modem_type == 1;
                    bool ofdm_rx = config_.ofdm_rx_enabled || config_.modem_type == 0;
                    bool robust_rx = config_.robust_rx_enabled || config_.modem_type == 2;
                    if (ofdm_rx)
                        decoder_->process(buffer.data(), n, frame_callback);
                    if (mfsk_rx)
                        for (int i = 0; i < 3; ++i)
                            mfsk_decoders_[i]->process(buffer.data(), n, mfsk_callbacks[i]);
                    if (robust_rx) {
                        robust_decoder_->process(buffer.data(), n, robust_frame_callback);
                        robust_decoder_n_->process(buffer.data(), n, robust_n_frame_callback);
                    }

                    bool on_air = tx_on_air_.load();
                    if (!on_air) {
                        if (was_on_air)
                            tone_dcd_->reset();
                        tone_dcd_->process(buffer.data(), n);
                    }
                    was_on_air = on_air;
                    int64_t tnow = steady_now_ms();
                    uint16_t heard_id;
                    if (tone_dcd_->consume_station_id(&heard_id)) {
                        size_t pop;
                        if (heard_id == station_id_.load()) {
                            std::random_device rd;
                            station_id_.store((uint16_t)((rd() % 0xFFFE) + 1));
                            ui_log("TONE: station ID collision, re-rolled");
                        }
                        last_winner_id_.store(heard_id);
                        {
                            std::lock_guard<std::mutex> hl(heard_mutex_);
                            heard_ids_[heard_id] = tnow;
                            last_id_ms_ = tnow;
                            pop = heard_ids_.size();
                        }
                        if (g_debug) {
                            char dbg[96];
                            snprintf(dbg, sizeof dbg,
                                     "TONE: station %04X heard, population %zu",
                                     heard_id, pop);
                            ui_log(dbg);
                        }
                    }
                    if (tone_dcd_->consume_id_failure()) {
                        pending_unattrib_ms_ = -1;
                        if (g_debug)
                            ui_log("TONE: signature ID unreadable, "
                                   "population unchanged");
                    }
                    if (tone_dcd_->consume_signature()) {
                        if (tnow >= tone_hold_until_ms_)
                            ui_log("CSMA: TX signature heard, deferring");
                        tone_hold_until_ms_ = tnow + 1500;
#ifdef WITH_UI
                        if (g_ui_state) g_ui_state->wf_sig_ms = tnow;
#endif
                    } else if (tone_dcd_->tone_run_active()) {
                        if (tone_run_start_ms_ < 0)
                            tone_run_start_ms_ = tnow;
                        if (tnow - tone_run_start_ms_ <= 2500)
                            tone_hold_until_ms_ =
                                std::max(tone_hold_until_ms_, tnow + 400);
                    } else {
                        tone_run_start_ms_ = -1;
                    }

                    // sync DCD: OFDM meta-validated in_frame and pilot-confirmed
                    // RDM collects only; MFSK syncs are too loose to gate TX on
                    dcd_active_ = (ofdm_rx && decoder_->in_frame()) ||
                                  (robust_rx &&
                                   (robust_decoder_->carrier_active() ||
                                    robust_decoder_n_->carrier_active())) ||
                                  (config_.csma_sync_only &&
                                   tnow < tone_hold_until_ms_);
                    if (dcd_active_) {
                        if (tnow - last_dcd_ms_ > 1500 &&
                            tnow - last_id_ms_ > 2500 &&
                            pending_unattrib_ms_ < 0)
                            pending_unattrib_ms_ = tnow;
                        last_dcd_ms_ = tnow;
                        set_tx_lockout(RX_LOCKOUT_SECONDS);
                    }
                    if (pending_unattrib_ms_ >= 0) {
                        if (tnow - last_id_ms_ < 2500) {
                            pending_unattrib_ms_ = -1;
                        } else if (tnow - pending_unattrib_ms_ > 1200) {
                            pending_unattrib_ms_ = -1;
                            if (tnow - unattrib_seen_ms_ <= 30000) {
                                last_unattrib_ms_.store(tnow);
                                if (g_debug)
                                    ui_log("TONE: carrier without station ID, "
                                           "population unknown for 90s");
                            } else if (g_debug) {
                                ui_log("TONE: carrier without station ID, "
                                       "ignored once");
                            }
                            unattrib_seen_ms_ = tnow;
                        }
                    }
                }

#ifdef WITH_UI
                if (g_ui_state && g_ui_state->scope_active.load(std::memory_order_relaxed) &&
                    !blanking && !g_ui_state->ptt_on.load(std::memory_order_relaxed) &&
                    !g_ui_state->transmitting.load(std::memory_order_relaxed))
                    g_ui_state->push_scope_audio(buffer.data(), n);
                if (++level_update_counter >= LEVEL_UPDATE_INTERVAL) {
                    level_update_counter = 0;
                    {
                        auto note = [this](int cur, int& last, const char* what) {
                            if (cur > last && last >= 0)
                                ui_log(std::string("RDM: ") + what + " recovered a frame");
                            last = cur;
                        };
                        note(robust_decoder_->stats_backward_rescues, last_bw_, "backward rescue");
                        note(robust_decoder_n_->stats_backward_rescues, last_bw_n_, "backward rescue");
                        note(robust_decoder_->stats_ladder_rescues, last_ld_, "retry ladder");
                        note(robust_decoder_n_->stats_ladder_rescues, last_ld_n_, "retry ladder");
                        note(robust_decoder_->stats_rescues - robust_decoder_->stats_backward_rescues, last_rescues_, "tail rescue");
                        note(robust_decoder_n_->stats_rescues - robust_decoder_n_->stats_backward_rescues, last_rescues_n_, "tail rescue");
                        note(robust_decoder_->stats_retry_success - robust_decoder_->stats_ladder_rescues, last_retries_, "retry decode");
                        note(robust_decoder_n_->stats_retry_success - robust_decoder_n_->stats_ladder_rescues, last_retries_n_, "retry decode");
                    }
                }
                if (g_ui_state && level_update_counter == 0) {

                    // Copy decoder stats
                    if (g_ui_state->stats_reset_requested.exchange(false)) {
                        decoder_->stats_sync_count = 0;
                        decoder_->stats_preamble_errors = 0;
                        decoder_->stats_symbol_errors = 0;
                        decoder_->stats_erased_symbols = 0;
                        decoder_->stats_crc_errors = 0;
                        decoder_->reset_ber();
                        for (auto& d : mfsk_decoders_) d->reset_stats();
                        robust_decoder_->reset_stats();
                        robust_decoder_n_->reset_stats();
                        g_ui_state->last_rx_ber = -1.0f;
                    }
                    if (config_.modem_type == 2) {
                        auto& rd = RobustParams::is_narrow((RobustMode)config_.robust_mode)
                                 ? robust_decoder_n_ : robust_decoder_;
                        g_ui_state->sync_count = rd->stats_sync_count;
                        g_ui_state->preamble_errors = rd->stats_preamble_errors;
                        g_ui_state->symbol_errors = 0;
                        g_ui_state->erased_symbols = rd->stats_rescues;
                        g_ui_state->crc_errors = rd->stats_crc_errors;
                    } else if (config_.modem_type == 1) {
                        g_ui_state->sync_count = cur_mfsk()->stats_sync_count;
                        g_ui_state->preamble_errors = cur_mfsk()->stats_preamble_errors;
                        g_ui_state->symbol_errors = 0;
                        g_ui_state->erased_symbols = 0;
                        g_ui_state->preamble_errors = 0;
                        g_ui_state->crc_errors = 0;
                    } else {
                        g_ui_state->sync_count = decoder_->stats_sync_count;
                        g_ui_state->preamble_errors = decoder_->stats_preamble_errors;
                        g_ui_state->symbol_errors = decoder_->stats_symbol_errors;
                        g_ui_state->erased_symbols = decoder_->stats_erased_symbols;
                        g_ui_state->crc_errors = decoder_->stats_crc_errors;
                    }
                }
#endif
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
        }
    }

    bool set_ptt(bool on) {
        std::lock_guard<std::mutex> lock(ptt_mutex_);
        bool ok = true;
        bool want_external = external_ptt && (config_.ptt_type == PTTType::COM
#ifdef WITH_CM108
                                              || config_.ptt_type == PTTType::CM108
#endif
                                              ) && !serial_ptt_;
        if (want_external) {
            ok = external_ptt(on);
#ifdef WITH_HAMLIB
        } else if (hamlib_ptt_) {
            ok = hamlib_ptt_->set_ptt(on);
#endif
        } else if (external_ptt && !rigctl_ && !serial_ptt_ && !dummy_ptt_) {
            ok = external_ptt(on);
        } else if (rigctl_) {
            ok = rigctl_->set_ptt(on);
        } else if (serial_ptt_) {
            ok = on ? serial_ptt_->ptt_on() : serial_ptt_->ptt_off();
#ifdef WITH_CM108
        } else if (cm108_ptt_) {
            ok = cm108_ptt_->set_ptt(on);
#endif
        } else if (dummy_ptt_) {
            ok = dummy_ptt_->set_ptt(on);
        }
        if (on) {
            ptt_state_.store(true);
            if (!ok && !ptt_fail_logged_) {
                ptt_fail_logged_ = true;
                ui_log("(!) PTT key failed - radio is NOT transmitting, check PTT settings");
            }
            ptt_failed_.store(!ok);
        } else if (ok) {
            ptt_state_.store(false);
            ptt_deadline_ms_.store(0);
            ptt_unkey_retries_ = 0;
        } else if (++ptt_unkey_retries_ < 5) {
            ptt_state_.store(true);
            ptt_deadline_ms_.store(steady_now_ms() + 1000);
        } else {
            ptt_state_.store(false);
            ptt_deadline_ms_.store(0);
            ptt_unkey_retries_ = 0;
            ui_log("(!) PTT unkey failed repeatedly - check the radio is not stuck in TX and that your PTT settings are correct");
        }

#ifdef WITH_UI
        if (g_ui_state) {
            g_ui_state->ptt_on = ptt_state_.load();
            g_ui_state->ptt_failed = ptt_failed_.load();
        }
#endif
        return ok;
    }

    void arm_ptt_watchdog(int64_t expected_ms) {
        ptt_deadline_ms_.store(steady_now_ms() + expected_ms + PTT_WATCHDOG_SLACK_MS);
    }

    void ptt_watchdog_loop() {
        while (g_running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            int64_t deadline = ptt_deadline_ms_.load();
            if (deadline != 0 && ptt_state_.load() && steady_now_ms() > deadline) {
                ptt_deadline_ms_.store(0);
                std::cerr << "PTT watchdog: max keyed time exceeded, forcing unkey"
                          << std::endl;
                ui_log("PTT watchdog: forcing unkey");
                set_ptt(false);
                tx_on_air_ = false;
            }
        }
    }
    
    void set_tx_lockout(float seconds) {
        std::lock_guard<std::mutex> lock(lockout_mutex_);
        auto lockout_until = std::chrono::steady_clock::now() + 
            std::chrono::milliseconds(static_cast<int>(seconds * 1000));

        if (lockout_until > tx_lockout_until_) {
            tx_lockout_until_ = lockout_until;
            if (g_verbose) {
                std::cerr << "TX lockout set for " << seconds << "s" << std::endl;
            }
        }

    }
    
    bool is_tx_allowed() {
        std::lock_guard<std::mutex> lock(lockout_mutex_);
        return std::chrono::steady_clock::now() >= tx_lockout_until_;
    }
    
    void wait_for_tx_allowed(int timeout_ms = 30000) {
        auto start = std::chrono::steady_clock::now();
        while (!is_tx_allowed() && g_running) {
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - start).count();
            if (elapsed > timeout_ms) {
                std::cerr << "TX lockout timeout, transmitting anyway" << std::endl;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }
    
    static constexpr int BURST_GAP_MS = 200;
    static constexpr int TONE_LEAD_GAP_MS = 150;

    TNCConfig config_;
    ModemConfig modem_config_;
    std::atomic<int> payload_size_{0};
    std::atomic<bool> decoder_reconfig_pending_{false};
    int frame_air_ms_cache_ = 0;
    uint64_t frame_air_key_ = (uint64_t)-1;
    
    std::unique_ptr<Encoder48k> encoder_;
    std::unique_ptr<Decoder48k> decoder_;
    std::unique_ptr<MFSKEncoder> mfsk_encoder_;
    static constexpr MFSKMode MFSK_RX_MODES[3] = {
        MFSKMode::MFSK_8, MFSKMode::MFSK_16, MFSKMode::MFSK_32};
    std::unique_ptr<MFSKDecoder> mfsk_decoders_[3];
    MFSKDecoder* cur_mfsk() const {
        return mfsk_decoders_[config_.mfsk_mode <= 1 ? config_.mfsk_mode : 2].get();
    }
    std::unique_ptr<RobustEncoder> robust_encoder_;
    std::unique_ptr<RobustDecoder> robust_decoder_;
    std::unique_ptr<RobustDecoder> robust_decoder_n_;

    std::unique_ptr<MiniAudio> audio_;
    std::unique_ptr<RigctlPTT> rigctl_;
#ifdef WITH_HAMLIB
    std::unique_ptr<HamlibPTT> hamlib_ptt_;
#endif
    std::unique_ptr<SerialPTT> serial_ptt_;
#ifdef WITH_CM108
    std::unique_ptr<CM108PTT> cm108_ptt_;
#endif
    std::unique_ptr<DummyPTT> dummy_ptt_;
    
    static constexpr size_t MAX_CLIENTS = 16;
    int server_fd_ = -1;
    std::list<std::unique_ptr<ClientConnection>> clients_;
    mutable std::mutex clients_mutex_;
    
    PacketQueue<TxPacket> tx_queue_;
    std::atomic<bool> tx_running_{false};
    std::atomic<int> mfsk_soft_errors_{0};
    std::atomic<bool> rx_running_{false};
    
    Fragmenter fragmenter_;
    int last_rescues_ = 0;
    int last_rescues_n_ = 0;
    int last_retries_ = 0;
    int last_retries_n_ = 0;
    int last_bw_ = 0;
    int last_bw_n_ = 0;
    int last_ld_ = 0;
    int last_ld_n_ = 0;
    Reassembler reassembler_;
    
    mutable std::mutex config_mutex_;

    // TX lockout - prevents TX while receiving
    mutable std::mutex lockout_mutex_;
    std::chrono::steady_clock::time_point tx_lockout_until_;
    static constexpr float RX_LOCKOUT_SECONDS = 0.5f;
    std::atomic<int64_t> last_rx_done_ms_{0};
    std::atomic<int64_t> last_channel_busy_ms_{steady_now_ms()};
    std::atomic<int> heard_air_ms_{0};
    std::atomic<int64_t> heard_air_at_ms_{0};
    int64_t spell_start_ms_ = -1;
    int64_t spell_last_ms_ = 0;
    std::unique_ptr<ToneDCD> tone_dcd_;
    int64_t tone_hold_until_ms_ = 0;
    int64_t tone_run_start_ms_ = -1;
    std::atomic<uint16_t> station_id_{0};
    std::atomic<uint16_t> last_winner_id_{0};
    std::mutex heard_mutex_;
    static constexpr int64_t HEARD_EXPIRY_MS = 300000;
    static constexpr int64_t UNATTRIB_DISTRUST_MS = 90000;
    static constexpr int RANKED_QUIET_MS = 1000;

    // stage a decay after 60 seconds for our contention window
    static constexpr int64_t CSMA_STAGE_DECAY_MS = 60000;
    static constexpr int YIELD_BUCKETS = 4;
    static constexpr int64_t PARTICIPATION_MS = 720000;
    int yield_attempt_ = 0;
    std::map<uint16_t, int64_t> heard_ids_;
    int64_t last_id_ms_ = -1000000;
    int64_t last_dcd_ms_ = -1000000;
    int64_t pending_unattrib_ms_ = -1;
    int64_t unattrib_seen_ms_ = -1000000;
    std::atomic<int64_t> last_unattrib_ms_{-1000000};

    int n_contenders(bool keep_on_stray = false) {
        int64_t now = steady_now_ms();
        if (!keep_on_stray &&
            now - last_unattrib_ms_.load() <= UNATTRIB_DISTRUST_MS)
            return -1;
        std::lock_guard<std::mutex> hl(heard_mutex_);
        for (auto it = heard_ids_.begin(); it != heard_ids_.end();) {
            if (now - it->second > HEARD_EXPIRY_MS)
                it = heard_ids_.erase(it);
            else
                ++it;
        }
        return (int)heard_ids_.size();
    }

    int known_others() {
        int64_t now = steady_now_ms();
        std::lock_guard<std::mutex> hl(heard_mutex_);
        int c = 0;
        for (const auto& kv : heard_ids_)
            if (now - kv.second <= HEARD_EXPIRY_MS)
                c++;
        return c;
    }

    int ranked_slot(int* n_out) {
        int64_t now = steady_now_ms();
        std::lock_guard<std::mutex> hl(heard_mutex_);
        for (auto it = heard_ids_.begin(); it != heard_ids_.end();) {
            if (now - it->second > HEARD_EXPIRY_MS)
                it = heard_ids_.erase(it);
            else
                ++it;
        }
        if (heard_ids_.empty())
            return -1;
        uint16_t self = station_id_.load();
        std::vector<uint16_t> ids;
        ids.push_back(self);
        for (const auto& kv : heard_ids_)
            if (kv.first != self)
                ids.push_back(kv.first);
        std::sort(ids.begin(), ids.end());
        int n = (int)ids.size();
        int i = (int)(std::find(ids.begin(), ids.end(), self) - ids.begin());
        *n_out = n;
        auto w = std::find(ids.begin(), ids.end(), last_winner_id_.load());
        if (w == ids.end())
            return i;
        if (*w == self) {
            *n_out = n;
            return n - 1 + (int)(id_mix(self, yield_attempt_) % YIELD_BUCKETS);
        }
        yield_attempt_ = 0;
        return (i - (int)(w - ids.begin()) - 1 + n) % n;
    }

    static uint32_t id_mix(uint16_t id, int attempt) {
        uint32_t h = (uint32_t)id ^ (uint32_t)(attempt * 0x9E37u);
        h *= 0x9E3779B1u;
        h ^= h >> 16;
        h *= 0x85EBCA6Bu;
        h ^= h >> 13;
        return h;
    }
    float occupancy_ema_ = 0.0f;
    float occupancy_other_ema_ = 0.0f;
    std::atomic<int> occupancy_pct_{0};
    int64_t occ_last_ms_ = 0;
    bool dcd_active_ = false;
    std::atomic<bool> alc_tune_active_{false};

    std::mutex ptt_mutex_;
    std::atomic<bool> ptt_state_{false};
    bool ptt_fail_logged_ = false;
    std::atomic<bool> ptt_failed_{false};
    int ptt_unkey_retries_ = 0;
    std::atomic<int64_t> ptt_deadline_ms_{0};
    static constexpr int64_t PTT_WATCHDOG_SLACK_MS = 5000;

    // TX blanking
    std::atomic<bool> tx_blanking_active_{false};
    std::atomic<bool> tx_on_air_{false};
    
public:
    float alc_auto_tune() {
        if (alc_tune_active_.exchange(true))
            return -1.0f;
        bool busy = tx_blanking_active_.load() || tx_on_air_.load();
#ifdef WITH_UI
        if (g_ui_state && g_ui_state->transmitting.load())
            busy = true;
#endif
        if (busy) {
            ui_log("ALC tune: TX in progress, try again");
            alc_tune_active_ = false;
            return -1.0f;
        }
        float result = -1.0f;
        tx_on_air_ = true;
        tx_blanking_active_ = true;
        set_ptt(true);
        arm_ptt_watchdog(2000);
        float drive = 0.10f;
        float prev = drive;
        float alc_base = NAN;
        for (int step = 0; step < 14 && g_running; ++step) {
            arm_ptt_watchdog(2000);
            audio_->drain_playback();
            audio_->set_tx_gain(drive);
            auto tone = generate_tone(modem_config_.center_freq,
                                      config_.sample_rate * 7 / 10, 0.8f);
            audio_->write(tone.data(), tone.size());
            std::this_thread::sleep_for(std::chrono::milliseconds(450));
            std::string r = rigctl_command("+l ALC");
            float alc = NAN;
            if (r.find("RPRT 0") != std::string::npos) {
                size_t p = r.find("Level Value:");
                if (p != std::string::npos) {
                    alc = strtof(r.c_str() + p + 12, nullptr);
                } else {
                    size_t pos = 0;
                    while (pos < r.size()) {
                        size_t e = r.find('\n', pos);
                        std::string line = r.substr(pos,
                            e == std::string::npos ? std::string::npos : e - pos);
                        if (line.rfind("RPRT", 0) == 0)
                            break;
                        if (!line.empty() && line.find(':') == std::string::npos)
                            alc = strtof(line.c_str(), nullptr);
                        if (e == std::string::npos)
                            break;
                        pos = e + 1;
                    }
                }
            }
            if (std::isnan(alc)) {
                ui_log("ALC tune: no ALC reading from rig: " + r);
                break;
            }
            char buf[64];
            snprintf(buf, sizeof(buf), "ALC tune: drive %d%% ALC %.2f",
                     (int)lround(drive * 100), alc);
            ui_log(buf);
            if (std::isnan(alc_base)) {
                alc_base = alc;
                if (alc_base > 0.3f) {
                    ui_log("ALC tune: ALC already high at 10% drive - reduce rig input gain");
                    break;
                }
            } else if (alc > alc_base + 0.05f) {
                result = prev;
                break;
            } else if (drive >= 0.999f) {
                result = 1.0f;
                ui_log("ALC tune: no ALC movement at full drive - rig input gain may be low");
                break;
            }
            prev = drive;
            drive = std::min(1.0f, drive * 1.25f);
        }
        audio_->drain_playback();
        set_ptt(false);
        tx_on_air_ = false;
        tx_blanking_active_ = false;
        if (result > 0) {
            std::lock_guard<std::mutex> lock(config_mutex_);
            config_.tx_drive = result;
        }
        audio_->set_tx_gain(config_.tx_drive);
        alc_tune_active_ = false;
        return result;
    }

    // Update config at runtime (called from UI)
    std::vector<std::string> update_config(const TNCConfig& new_config) {
        std::vector<std::string> rejected;
        std::lock_guard<std::mutex> lock(config_mutex_);
        {
            config_.csma_enabled = new_config.csma_enabled;
            config_.csma_sync_only = new_config.csma_sync_only;
            config_.csma_fast_floor = new_config.csma_fast_floor;
            config_.csma_ranked = new_config.csma_ranked;
            config_.beacon_interval_s = new_config.beacon_interval_s;
            config_.csma_band = new_config.csma_band;
            config_.postamble = new_config.postamble;
            config_.carrier_threshold_db = new_config.carrier_threshold_db;
            config_.p_persistence = new_config.p_persistence;
            config_.slot_time_ms = new_config.slot_time_ms;
            config_.csma_quiet_ms = new_config.csma_quiet_ms;
            config_.csma_cw = new_config.csma_cw;
            config_.csma_responder_dither = new_config.csma_responder_dither;
            config_.csma_burst = new_config.csma_burst;
            config_.tx_lead_tone = new_config.tx_lead_tone;
            config_.tx_blanking_enabled = new_config.tx_blanking_enabled || new_config.csma_enabled;
            config_.fragmentation_enabled = new_config.fragmentation_enabled;
            config_.tx_delay_ms = new_config.tx_delay_ms;
            config_.mfsk_rx_enabled = new_config.mfsk_rx_enabled;
            config_.ofdm_rx_enabled = new_config.ofdm_rx_enabled;
            config_.robust_rx_enabled = new_config.robust_rx_enabled;
            if (config_.tx_drive != new_config.tx_drive) {
                config_.tx_drive = new_config.tx_drive;
                if (audio_) audio_->set_tx_gain(config_.tx_drive);
            }
        }
        
        // Update callsign if changed
        if (config_.callsign != new_config.callsign) {
            if (ModemConfig::valid_callsign(new_config.callsign.c_str())) {
                config_.callsign = new_config.callsign;
                modem_config_.call_sign = ModemConfig::encode_callsign(config_.callsign.c_str());
                ui_log("Callsign changed to " + config_.callsign);
            } else {
                rejected.push_back("callsign");
                ui_log("(!) Invalid callsign '" + new_config.callsign +
                       "' (A-Z 0-9 / only, 1-9 chars), keeping " + config_.callsign);
            }
        }
        
        // Update center frequency
        if (config_.center_freq != new_config.center_freq) {
            config_.center_freq = new_config.center_freq;
            modem_config_.center_freq = config_.center_freq;
            decoder_reconfig_pending_.store(true);
            ui_log("Center frequency changed to " + std::to_string(config_.center_freq) + " Hz");
        }

        // Update modem type and sub-mode
        if (config_.robust_mode != new_config.robust_mode ||
            (config_.modem_type != new_config.modem_type && new_config.modem_type == 2)) {
            config_.robust_mode = new_config.robust_mode;
            if (new_config.modem_type == 2) {
                RobustMode rmode = (RobustMode)config_.robust_mode;
                payload_size_ = robust_encoder_->get_payload_size(rmode);
                ui_log("Mode changed to " + std::string(ROBUST_MODE_NAMES[(int)rmode]) +
                       " (" + std::to_string(RobustParams::bitrate(rmode)) + " bps)");
            }
        }
        if (config_.modem_type != new_config.modem_type || config_.mfsk_mode != new_config.mfsk_mode) {
            config_.modem_type = new_config.modem_type;
            config_.mfsk_mode = new_config.mfsk_mode;
            if (config_.modem_type == 1) {
                MFSKMode mmode = (MFSKMode)config_.mfsk_mode;
                payload_size_ = mfsk_encoder_->get_payload_size(mmode);
                ui_log("Mode changed to " + std::string(MFSK_MODE_NAMES[(int)mmode]) +
                       " (" + std::to_string(MFSKParams::max_payload(mmode)) + " bytes)");
            } else if (config_.modem_type == 2) {
                payload_size_ = robust_encoder_->get_payload_size((RobustMode)config_.robust_mode);
            } else {
                payload_size_ = encoder_->get_payload_size(modem_config_.oper_mode);
            }
        }

        // Update OFDM modulation settings
        bool mode_changed = (config_.modulation != new_config.modulation ||
                            config_.code_rate != new_config.code_rate ||
                            config_.frame_size != new_config.frame_size);

        if (mode_changed) {
            int new_mode = ModemConfig::encode_mode(
                new_config.modulation.c_str(),
                new_config.code_rate.c_str(),
                new_config.frame_size
            );

            if (new_mode >= 0) {
                config_.modulation = new_config.modulation;
                config_.code_rate = new_config.code_rate;
                config_.frame_size = new_config.frame_size;
                modem_config_.oper_mode = new_mode;
                if (config_.modem_type == 0) {
                    payload_size_ = encoder_->get_payload_size(modem_config_.oper_mode);
                }
                ui_log("OFDM mode changed to " + config_.modulation + " " + config_.code_rate +
                       " " + ModemConfig::frame_size_name(config_.frame_size) +
                       " (" + std::to_string(encoder_->get_payload_size(modem_config_.oper_mode)) + " bytes)");
            } else {
                rejected.push_back("modulation/code_rate/frame_size");
                ui_log("(!) Invalid OFDM mode " + new_config.modulation + " " + new_config.code_rate +
                       " " + ModemConfig::frame_size_name(new_config.frame_size) +
                       ", keeping " + config_.modulation + " " + config_.code_rate);
            }
        }

        return rejected;
    }
    
    TNCConfig get_config() {
        std::lock_guard<std::mutex> lock(config_mutex_);
        return config_;
    }

    int get_payload_size() const { return payload_size_; }

    struct DecoderStats {
        int sync_count, preamble_errors, symbol_errors, erased_symbols, crc_errors;
        float last_snr, last_ber, ber_ema;
    };

    DecoderStats get_decoder_stats() const {
        if (config_.modem_type == 2) {
            auto& rd = RobustParams::is_narrow((RobustMode)config_.robust_mode)
                     ? robust_decoder_n_ : robust_decoder_;
            return {
                rd->stats_sync_count,
                rd->stats_preamble_errors,
                0,
                rd->stats_rescues,
                rd->stats_crc_errors,
                rd->get_last_snr(),
                rd->get_last_ber(),
                rd->get_ber_ema()
            };
        }
        if (config_.modem_type == 1) {
            return {
                cur_mfsk()->stats_sync_count,
                cur_mfsk()->stats_preamble_errors,
                0, // MFSK has no symbol errors stat
                0, // MFSK has no symbol errors stat
                cur_mfsk()->stats_crc_errors,
                cur_mfsk()->get_last_snr(),
                cur_mfsk()->get_last_ber(),
                cur_mfsk()->get_ber_ema()
            };
        }
        return {
            decoder_->stats_sync_count,
            decoder_->stats_preamble_errors,
            decoder_->stats_symbol_errors,
            decoder_->stats_erased_symbols,
            decoder_->stats_crc_errors,
            decoder_->get_last_snr(),
            decoder_->get_last_ber(),
            decoder_->get_ber_ema()
        };
    }

    bool is_transmitting() const {
        return tx_on_air_.load() || tx_blanking_active_.load();
    }

    void unkey() {
        set_ptt(false);
        tx_on_air_ = false;
        tx_blanking_active_ = false;
    }

    size_t tx_queue_depth() const { return tx_queue_.size(); }

    int channel_population() { return known_others(); }

    bool queue_beacon() {
        TxPacket b;
        b.beacon = true;
        b.manual = true;
        tx_queue_.push(std::move(b));
        return true;
    }

    int channel_occupancy() const { return occupancy_pct_.load(); }

    bool is_receiving() const {
        std::lock_guard<std::mutex> lock(lockout_mutex_);
        return std::chrono::steady_clock::now() < tx_lockout_until_;
    }

    int get_client_count() const {
        std::lock_guard<std::mutex> lock(clients_mutex_);
        return clients_.size();
    }

    bool ptt_failed() const {
        return ptt_failed_.load();
    }

    std::string rigctl_command(const std::string& cmd) {
        if (rigctl_) return rigctl_->send_command(cmd);
#ifdef WITH_HAMLIB
        if (hamlib_ptt_) return hamlib_ptt_->command(cmd);
#endif
        return "ERR: rigctl not enabled";
    }

    bool is_rigctl_connected() const {
        if (rigctl_) return rigctl_->is_connected();
#ifdef WITH_HAMLIB
        if (hamlib_ptt_) return hamlib_ptt_->is_connected();
#endif
        return false;
    }

    bool hamlib_get_freq(double& hz) {
#ifdef WITH_HAMLIB
        if (hamlib_ptt_) return hamlib_ptt_->get_freq(hz);
#endif
        (void)hz;
        return false;
    }
    
    float get_audio_level() {
        if (!audio_ || !audio_->capture_alive()) return -100.0f;
        return audio_->instant_level_db(100);
    }

    bool is_audio_healthy() const {
        if (audio_) return audio_->is_healthy();
        return false;
    }
    
    bool reconnect_audio() {
        if (audio_) {
            return audio_->reconnect();
        }
        return false;
    }
    
    void queue_data(const std::vector<uint8_t>& data) {
        queue_data_ex(data, -1);
    }

    // Queue data with an optional per-packet oper_mode override (-1 = default)
    void queue_data_ex(const std::vector<uint8_t>& data, int oper_mode) {
        size_t effective_payload;
        if (oper_mode >= 0) {
            if (config_.modem_type == 2 && oper_mode < ROBUST_MODE_COUNT)
                effective_payload = robust_encoder_->get_payload_size((RobustMode)oper_mode) - 2;
            else
                effective_payload = encoder_->get_payload_size(oper_mode) - 2;
        } else {
            effective_payload = payload_size_ - 2;
        }

        if (config_.fragmentation_enabled && fragmenter_.needs_fragmentation(data.size(), effective_payload)) {
            auto fragments = fragmenter_.fragment(data, effective_payload);
            ui_log("TX: Fragmenting " + std::to_string(data.size()) + " bytes into " +
                   std::to_string(fragments.size()) + " fragments");
            for (auto& frag : fragments) {
                tx_queue_.push(TxPacket(std::move(frag), oper_mode));
            }
        } else {
            tx_queue_.push(TxPacket(data, oper_mode));
        }
#ifdef WITH_UI
        if (g_ui_state) {
            g_ui_state->tx_queue_size = tx_queue_.size();
        }
#endif
    }

    // Compute oper_mode for a given frame_size setting using current modulation/code_rate
    int compute_oper_mode(int frame_size) const {
        return ModemConfig::encode_mode(
            config_.modulation.c_str(),
            config_.code_rate.c_str(),
            frame_size
        );
    }
};
