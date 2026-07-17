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
#include "miniaudio_audio.hh"
#include "rigctl_ptt.hh"
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
#include "tnc_ui.hh"
#endif

std::atomic<bool> g_running{true};
TNCConfig g_config;
bool g_verbose = false;
#ifdef WITH_UI
bool g_use_ui = true;  
#else
bool g_use_ui = false;
#endif

#ifdef WITH_UI
TNCUIState* g_ui_state = nullptr;
#endif

void signal_handler(int /*sig*/) {
    std::cerr << "\nShutting down..." << std::endl;
    g_running = false;
}



inline void ui_log(const std::string& msg) {
#ifdef WITH_UI
    if (g_ui_state) {
        g_ui_state->add_log(msg);
    }
#endif
    if (g_verbose || !g_use_ui) {
        std::cerr << msg << std::endl;
    }
}

bool valid_bind_address(const std::string& addr) {
    struct in_addr a;
    return inet_pton(AF_INET, addr.c_str(), &a) == 1;
}

bool check_port_available(const std::string& bind_address, int port) {
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
            throw std::runtime_error("Invalid callsign");
        }
        if (modem_config_.oper_mode < 0) {
            throw std::runtime_error("Invalid modulation or code rate");
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
        if (!audio_->open_playback()) {
            throw std::runtime_error("Failed to open audio input");
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
        } else if (config_.ptt_type == PTTType::COM) {
            serial_ptt_ = std::make_unique<SerialPTT>();
            if (!serial_ptt_->open(config_.com_port, 
                                   static_cast<PTTLine>(config_.com_ptt_line),
                                   config_.com_invert_dtr, 
                                   config_.com_invert_rts)) {
                std::cerr << "Could not open COM port: " << serial_ptt_->last_error() << std::endl;
            }
#ifdef WITH_CM108
        } else if (config_.ptt_type == PTTType::CM108) {
            cm108_ptt_ = std::make_unique<CM108PTT>();
            cm108_ptt_->open(config_.cm108_gpio, config_.cm108_device);
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
        std::cerr << "Modulation: " << config_.modulation << " " << config_.code_rate
                  << " " << ModemConfig::frame_size_name(config_.frame_size) << std::endl;
        std::cerr << "Payload: " << payload_size_ << " bytes (including 2-byte length prefix)" << std::endl;
        
        if (config_.csma_enabled) {
            std::cerr << "CSMA: enabled (threshold=" << config_.carrier_threshold_db
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
        
        std::cerr << "MFSK RX decoders: " << (config_.mfsk_rx_enabled ? "enabled" : "disabled") << std::endl;
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
            case PTTType::COM:
                std::cerr << "PTT: COM " << config_.com_port 
                          << " (" << PTT_LINE_OPTIONS[config_.com_ptt_line] << ")" << std::endl;
                break;
#ifdef WITH_CM108
            case PTTType::CM108:
                std::cerr << "PTT: CM108 (GPIO" << config_.cm108_gpio << ")" << std::endl;
                break;
#endif
        }
        
        // Start threads
        std::thread rx_thread(&KISSTNC::rx_loop, this);
        std::thread tx_thread(&KISSTNC::tx_loop, this);
        std::thread watchdog_thread(&KISSTNC::ptt_watchdog_loop, this);
        
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
            case KISS::CMD_TXDELAY:
                if (!data.empty()) {
                    config_.tx_delay_ms = data[0] * 10;
                    ui_log("TXDelay set to " + std::to_string(config_.tx_delay_ms) + " ms");
                }
                break;
            case KISS::CMD_P:
                if (!data.empty()) {
                    ui_log("KISS P-persistence " + std::to_string(data[0]) +
                           " ignored (unused)");
                }
                break;
            case KISS::CMD_SLOTTIME:
                if (!data.empty()) {
                    int prev = config_.slot_time_ms;
                    config_.slot_time_ms = data[0] * 10;
                    ui_log("KISS client set slot time to " +
                           std::to_string(config_.slot_time_ms) + " ms (was " +
                           std::to_string(prev) + " ms)");
                }
                break;
            case KISS::CMD_TXTAIL:
                if (!data.empty()) {
                    config_.ptt_tail_ms = data[0] * 10;
                    ui_log("TXTail set to " + std::to_string(config_.ptt_tail_ms) + " ms");
                }
                break;
            case KISS::CMD_FULLDUPLEX:
                if (!data.empty()) {
                    config_.full_duplex = data[0] != 0;
                    ui_log(std::string("Full duplex ") + (config_.full_duplex ? "enabled" : "disabled"));
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
        
        while (tx_running_ && g_running) {
            TxPacket pkt;
            if (tx_queue_.pop(pkt)) {
#ifdef WITH_UI
                if (g_ui_state) {
                    g_ui_state->tx_queue_size = tx_queue_.size();
                }
#endif
                // CSMA
                bool csma_enabled;
                int carrier_sense_ms, slot_time_ms, csma_quiet_ms, csma_cw, csma_dither, csma_burst;
                float carrier_threshold_db;
                std::string csma_callsign;
                {
                    std::lock_guard<std::mutex> lock(config_mutex_);
                    csma_enabled = config_.csma_enabled;
                    carrier_sense_ms = config_.carrier_sense_ms;
                    carrier_threshold_db = config_.carrier_threshold_db;
                    slot_time_ms = config_.slot_time_ms;
                    csma_quiet_ms = config_.csma_quiet_ms;
                    csma_cw = config_.csma_cw;
                    csma_dither = config_.csma_responder_dither;
                    csma_burst = std::max(1, std::min(4, config_.csma_burst));
                    csma_callsign = config_.callsign;
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
                    gcfg.quiet_ms = csma_quiet_ms > 0 ? csma_quiet_ms : auto_quiet_ms();
                    gcfg.cw = csma_cw;
                    gcfg.slot_ms = slot_time_ms;
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
                    gcfg.responder = rx_ms > 0 && pkt.enqueue_ms >= rx_ms &&
                                     pkt.enqueue_ms - rx_ms <= 2000 &&
                                     steady_now_ms() - rx_ms <= 5000;
                    CsmaGate gate(gcfg, (uint32_t)gen());

                    if (gcfg.responder) {
                        std::cerr << "CSMA: responder priority, quiet "
                                  << gate.quiet_needed_ms() << " ms" << std::endl;
                    } else if (gcfg.idle_credit_ms >= 250) {
                        std::cerr << "CSMA: idle credit " << gcfg.idle_credit_ms
                                  << " ms, window " << gate.window_ms() << " ms"
                                  << std::endl;
                    }

                    bool was_busy = false, was_deaf = false, quiet_logged = false;
                    while (g_running) {
                        bool alive = audio_->capture_alive();
                        float level_db = audio_->instant_level_db(carrier_sense_ms);
                        bool allowed = is_tx_allowed();
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
                        bool busy = alive && (!allowed || level_db > carrier_threshold_db);
                        if (busy && !was_busy) {
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
                    if (!g_running)
                        break;
                }

#ifdef WITH_UI
                if (g_ui_state) g_ui_state->csma_phase = 0;
#endif
                TxPacket cur = std::move(pkt);
                bool first = true;
                int remaining = csma_burst - 1;
                while (true) {
                    TxPacket next;
                    bool have_next = remaining > 0 && tx_queue_.pop(next);
#ifdef WITH_UI
                    if (g_ui_state) {
                        g_ui_state->tx_queue_size = tx_queue_.size();
                    }
#endif
                    bool sent = transmit(cur.data, cur.oper_mode, first, !have_next);
                    if (!have_next)
                        break;
                    std::cerr << "CSMA: burst continuation ("
                              << remaining << " left)" << std::endl;
                    cur = std::move(next);
                    if (sent)
                        first = false;
                    --remaining;
                }
            } else {
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

    bool transmit(const std::vector<uint8_t>& data, int oper_mode_override = -1,
                  bool first = true, bool last = true) {
        while (alc_tune_active_.load() && g_running)
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        int tx_mode = (oper_mode_override >= 0) ? oper_mode_override : modem_config_.oper_mode;

        if (oper_mode_override >= 0) {
            ui_log("TX: " + std::to_string(data.size()) + " bytes (mode override)");
        } else {
            ui_log("TX: " + std::to_string(data.size()) + " bytes");
        }
        if (g_verbose) {
            std::cerr << packet_visualize(data.data(), data.size(), true, config_.fragmentation_enabled) << std::endl;
        }

        if (config_.tx_blanking_enabled) {
            tx_blanking_active_ = true;
        }

#ifdef WITH_UI
        if (g_ui_state) {
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
        auto framed_data = frame_with_length(data);

        // Encode to audio
        std::vector<float> samples;
        if (config_.modem_type == 1) {
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
        
        if (samples.empty()) {
            ui_log("TX: Encoding failed");
            if (!first && last && config_.ptt_type != PTTType::VOX) {
                audio_->write_silence(config_.ptt_tail_ms * config_.sample_rate / 1000);
                audio_->drain_playback();
                if (config_.ptt_type == PTTType::RIGCTL || config_.ptt_type == PTTType::COM
#ifdef WITH_CM108
                    || config_.ptt_type == PTTType::CM108
#endif
                ) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(config_.ptt_tail_ms));
                    set_ptt(false);
                }
            }
            if (last) {
                tx_blanking_active_ = false;
#ifdef WITH_UI
                if (g_ui_state) g_ui_state->transmitting = false;
#endif
            }
            return false;
        }
        
        float duration = samples.size() / (float)config_.sample_rate;
        float total_tx_duration = duration;

        int64_t overhead_ms = config_.tx_delay_ms + config_.ptt_tail_ms +
                              config_.vox_lead_ms + config_.vox_tail_ms + BURST_GAP_MS;
        arm_ptt_watchdog((int64_t)(duration * 1000.0f) + overhead_ms);

        // Handle PTT based on type
        if (config_.ptt_type == PTTType::VOX) {
            // VOX mode: generate tone to trigger radio's VOX
            int lead_samples = config_.vox_lead_ms * config_.sample_rate / 1000;
            int tail_samples = config_.vox_tail_ms * config_.sample_rate / 1000;
            
            // Generate lead tone
            auto lead_tone = generate_tone(config_.vox_tone_freq, lead_samples, 0.8f);
            
            // Generate tail tone  
            auto tail_tone = generate_tone(config_.vox_tone_freq, tail_samples, 0.8f);
            
            total_tx_duration += (config_.vox_lead_ms + config_.vox_tail_ms) / 1000.0f;
            
            ui_log("TX: VOX mode, " + std::to_string(config_.vox_tone_freq) + "Hz tone, " +
                   std::to_string(config_.vox_lead_ms) + "ms lead, " +
                   std::to_string(config_.vox_tail_ms) + "ms tail");
            
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
            total_tx_duration += (first ? config_.tx_delay_ms : BURST_GAP_MS) / 1000.0f;
            if (last)
                total_tx_duration += config_.ptt_tail_ms / 1000.0f;
            
            ui_log("TX: " + std::to_string(samples.size()) + " samples, " + 
                   std::to_string(duration) + " seconds");
            
            if (first) {
                // PTT on (for RIGCTL or COM mode)
                if (config_.ptt_type == PTTType::RIGCTL || config_.ptt_type == PTTType::COM
#ifdef WITH_CM108
                    || config_.ptt_type == PTTType::CM108
#endif
                ) {
                    set_ptt(true);
                    std::this_thread::sleep_for(std::chrono::milliseconds(config_.ptt_delay_ms));
                }

                // Leading silence (TXDelay)
                int lead_frames = config_.tx_delay_ms * config_.sample_rate / 1000;
                if (config_.tx_lead_tone && config_.tx_delay_ms >= 250) {
                    int gap_frames = 150 * config_.sample_rate / 1000;
                    auto lead = generate_tone(modem_config_.center_freq,
                                              lead_frames - gap_frames, 0.6f);
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
                if (config_.ptt_type == PTTType::RIGCTL || config_.ptt_type == PTTType::COM
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
            ui_log("RX: " + std::to_string(payload.size()) + " bytes, SNR=" +
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

        while (rx_running_ && g_running) {
            int n = audio_->read(buffer.data(), buffer.size());
            if (n > 0) {
                bool blanking = tx_blanking_active_.load();

                {
                    int64_t now_ms = steady_now_ms();
                    bool loud = audio_->instant_level_db(config_.carrier_sense_ms) >
                                config_.carrier_threshold_db;
                    bool occupied = loud || !is_tx_allowed();
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
                            float x = (loud || blanking || dcd_active_) ? 1.0f : 0.0f;
                            occupancy_ema_ += (x - occupancy_ema_) * a;
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
                    }
                    if (was_blanking) {
                        decoder_->reset();
                        for (auto& d : mfsk_decoders_) d->reset();
                        robust_decoder_->reset();
                        robust_decoder_n_->reset();
                        was_blanking = false;
                    }
                    bool mfsk_rx = config_.mfsk_rx_enabled || config_.modem_type == 1;
                    decoder_->process(buffer.data(), n, frame_callback);
                    if (mfsk_rx)
                        for (int i = 0; i < 3; ++i)
                            mfsk_decoders_[i]->process(buffer.data(), n, mfsk_callbacks[i]);
                    robust_decoder_->process(buffer.data(), n, robust_frame_callback);
                    robust_decoder_n_->process(buffer.data(), n, robust_n_frame_callback);

                    // sync DCD: OFDM meta-validated in_frame and pilot-confirmed
                    // RDM collects only; MFSK syncs are too loose to gate TX on
                    dcd_active_ = decoder_->in_frame() ||
                                  robust_decoder_->carrier_active() ||
                                  robust_decoder_n_->carrier_active();
                    if (dcd_active_)
                        set_tx_lockout(RX_LOCKOUT_SECONDS);
                }

#ifdef WITH_UI
                if (g_ui_state && ++level_update_counter >= LEVEL_UPDATE_INTERVAL) {
                    level_update_counter = 0;

                    // Calculate RMS level in dB
                    float sum_sq = 0.0f;
                    for (int i = 0; i < n; i++) {
                        sum_sq += buffer[i] * buffer[i];
                    }
                    float rms = std::sqrt(sum_sq / n);
                    float db = 20.0f * std::log10(rms + 1e-10f);

                    g_ui_state->update_level(db, dcd_active_);

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
            }
        }
    }
    
    bool set_ptt(bool on) {
        std::lock_guard<std::mutex> lock(ptt_mutex_);
        bool ok = true;
        if (rigctl_) {
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
        } else if (ok) {
            ptt_state_.store(false);
            ptt_deadline_ms_.store(0);
        } else {
            ptt_state_.store(true);
            ptt_deadline_ms_.store(steady_now_ms() + 1000);
        }

#ifdef WITH_UI
        if (g_ui_state) {
            g_ui_state->ptt_on = ptt_state_.load();
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
    float occupancy_ema_ = 0.0f;
    int64_t occ_last_ms_ = 0;
    bool dcd_active_ = false;
    std::atomic<bool> alc_tune_active_{false};

    std::mutex ptt_mutex_;
    std::atomic<bool> ptt_state_{false};
    std::atomic<int64_t> ptt_deadline_ms_{0};
    static constexpr int64_t PTT_WATCHDOG_SLACK_MS = 5000;

    // TX blanking
    std::atomic<bool> tx_blanking_active_{false};
    
public:
    float alc_auto_tune() {
        if (alc_tune_active_.exchange(true))
            return -1.0f;
        bool busy = tx_blanking_active_.load();
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
    void update_config(const TNCConfig& new_config) {
        std::lock_guard<std::mutex> lock(config_mutex_);
        {
            config_.csma_enabled = new_config.csma_enabled;
            config_.postamble = new_config.postamble;
            config_.carrier_threshold_db = new_config.carrier_threshold_db;
            config_.p_persistence = new_config.p_persistence;
            config_.slot_time_ms = new_config.slot_time_ms;
            config_.csma_quiet_ms = new_config.csma_quiet_ms;
            config_.csma_cw = new_config.csma_cw;
            config_.csma_responder_dither = new_config.csma_responder_dither;
            config_.csma_burst = new_config.csma_burst;
            config_.tx_lead_tone = new_config.tx_lead_tone;
            config_.tx_blanking_enabled = new_config.tx_blanking_enabled;
            if (config_.tx_drive != new_config.tx_drive) {
                config_.tx_drive = new_config.tx_drive;
                if (audio_) audio_->set_tx_gain(config_.tx_drive);
            }
        }
        
        // Update callsign if changed
        if (config_.callsign != new_config.callsign) {
            config_.callsign = new_config.callsign;
            modem_config_.call_sign = ModemConfig::encode_callsign(config_.callsign.c_str());
            ui_log("Callsign changed to " + config_.callsign);
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
            config_.modulation = new_config.modulation;
            config_.code_rate = new_config.code_rate;
            config_.frame_size = new_config.frame_size;

            int new_mode = ModemConfig::encode_mode(
                config_.modulation.c_str(),
                config_.code_rate.c_str(),
                config_.frame_size
            );

            if (new_mode >= 0) {
                modem_config_.oper_mode = new_mode;
                if (config_.modem_type == 0) {
                    payload_size_ = encoder_->get_payload_size(modem_config_.oper_mode);
                }
                ui_log("OFDM mode changed to " + config_.modulation + " " + config_.code_rate +
                       " " + ModemConfig::frame_size_name(config_.frame_size) +
                       " (" + std::to_string(encoder_->get_payload_size(modem_config_.oper_mode)) + " bytes)");
            } else {
                ui_log("Invalid OFDM mode " + config_.modulation + " " + config_.code_rate +
                       " " + ModemConfig::frame_size_name(config_.frame_size) + ", keeping previous");
            }
        }
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

    bool is_transmitting() const { return tx_blanking_active_.load(); }

    size_t tx_queue_depth() const { return tx_queue_.size(); }

    bool is_receiving() const {
        std::lock_guard<std::mutex> lock(lockout_mutex_);
        return std::chrono::steady_clock::now() < tx_lockout_until_;
    }

    int get_client_count() const {
        std::lock_guard<std::mutex> lock(clients_mutex_);
        return clients_.size();
    }

    std::string rigctl_command(const std::string& cmd) {
        if (rigctl_) return rigctl_->send_command(cmd);
        return "ERR: rigctl not enabled";
    }

    bool is_rigctl_connected() const {
        if (rigctl_) return rigctl_->is_connected();
        return false;
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

// Load key=value settings from path into config when --config is passed
static bool apply_settings_file(const std::string& path, TNCConfig& config,
                                const std::set<std::string>& cli_set) {
    static const char* MOD_OPTS[] = {
        "BPSK", "QPSK", "8PSK", "QAM16", "QAM64", "QAM256", "QAM1024", "QAM4096"
    };
    static const int N_MOD = sizeof(MOD_OPTS) / sizeof(*MOD_OPTS);
    static const char* RATE_OPTS[] = {"1/2", "2/3", "3/4", "5/6", "1/4", "1/2x2", "1/4x2"};
    static const int N_RATE = sizeof(RATE_OPTS) / sizeof(*RATE_OPTS);

    FILE* f = fopen(path.c_str(), "r");
    if (!f) return false;

    auto take = [&](const char* k) {
        return cli_set.find(k) == cli_set.end();
    };

    char line[256];
    while (fgets(line, sizeof(line), f)) {
        if (line[0] == '#' || line[0] == '\n') continue;
        char key[64], value[192];
        if (sscanf(line, "%63[^=]=%191[^\n]", key, value) != 2) continue;

        if (!strcmp(key, "callsign") && take(key)) config.callsign = value;
        else if (!strcmp(key, "modem_type") && take(key)) {
            int v = atoi(value);
            if (v >= 0 && v <= 2) config.modem_type = v;
        }
        else if (!strcmp(key, "mfsk_mode") && take(key)) {
            int v = atoi(value);
            if (v >= 0 && v <= 3) config.mfsk_mode = v;
        }
        else if (!strcmp(key, "robust_mode") && take(key)) {
            int v = atoi(value);
            if (v >= 0 && v < ROBUST_MODE_COUNT) config.robust_mode = v;
        }
        else if (!strcmp(key, "perf_log") && take(key)) config.perf_log = atoi(value) != 0;
        else if (!strcmp(key, "modulation") && take(key)) {
            int idx = atoi(value);
            if (idx >= 0 && idx < N_MOD) config.modulation = MOD_OPTS[idx];
        }
        else if (!strcmp(key, "code_rate") && take(key)) {
            int idx = atoi(value);
            if (idx >= 0 && idx < N_RATE) config.code_rate = RATE_OPTS[idx];
        }
        else if (!strcmp(key, "short_frame") && take("frame_size")) config.frame_size = atoi(value) != 0 ? 0 : 1;
        else if (!strcmp(key, "frame_size") && take(key)) {
            int v = atoi(value);
            if (v >= 0 && v <= 2) config.frame_size = v;
        }
        else if (!strcmp(key, "center_freq") && take(key)) config.center_freq = atoi(value);
        else if (!strcmp(key, "rx_filter_enabled") && take(key)) config.rx_filter_enabled = atoi(value) != 0;
        else if (!strcmp(key, "postamble") && take(key)) config.postamble = atoi(value) != 0;
        else if (!strcmp(key, "mfsk_rx_enabled") && take(key)) config.mfsk_rx_enabled = atoi(value) != 0;
        else if (!strcmp(key, "csma_enabled") && take(key)) config.csma_enabled = atoi(value) != 0;
        else if (!strcmp(key, "carrier_threshold_db") && take(key)) config.carrier_threshold_db = atof(value);
        else if (!strcmp(key, "slot_time_ms") && take(key)) config.slot_time_ms = atoi(value);
        else if (!strcmp(key, "csma_quiet_ms") && take(key)) config.csma_quiet_ms = atoi(value);
        else if (!strcmp(key, "csma_cw") && take(key)) config.csma_cw = atoi(value);
        else if (!strcmp(key, "csma_responder_dither") && take(key)) config.csma_responder_dither = atoi(value);
        else if (!strcmp(key, "csma_burst") && take(key)) config.csma_burst = atoi(value);
        else if (!strcmp(key, "tx_lead_tone") && take(key)) config.tx_lead_tone = atoi(value) != 0;
        else if (!strcmp(key, "p_persistence") && take(key)) config.p_persistence = atoi(value);
        else if (!strcmp(key, "fragmentation_enabled") && take(key)) config.fragmentation_enabled = atoi(value) != 0;
        else if (!strcmp(key, "tx_blanking_enabled") && take(key)) config.tx_blanking_enabled = atoi(value) != 0;
        else if (!strcmp(key, "audio_input") && take(key)) config.audio_input_device = value;
        else if (!strcmp(key, "audio_output") && take(key)) config.audio_output_device = value;
        else if (!strcmp(key, "audio_device")) {
            if (take("audio_input")) config.audio_input_device = value;
            if (take("audio_output")) config.audio_output_device = value;
        }
        else if (!strcmp(key, "ptt_type") && take(key)) config.ptt_type = static_cast<PTTType>(atoi(value));
        else if (!strcmp(key, "vox_tone_freq") && take(key)) config.vox_tone_freq = atoi(value);
        else if (!strcmp(key, "vox_lead_ms") && take(key)) config.vox_lead_ms = atoi(value);
        else if (!strcmp(key, "vox_tail_ms") && take(key)) config.vox_tail_ms = atoi(value);
        else if (!strcmp(key, "com_port") && take(key)) config.com_port = value;
        else if (!strcmp(key, "com_ptt_line") && take(key)) config.com_ptt_line = atoi(value);
        else if (!strcmp(key, "com_invert_dtr") && take(key)) config.com_invert_dtr = atoi(value) != 0;
        else if (!strcmp(key, "com_invert_rts") && take(key)) config.com_invert_rts = atoi(value) != 0;
#ifdef WITH_CM108
        else if (!strcmp(key, "cm108_gpio") && take(key)) config.cm108_gpio = atoi(value);
        else if (!strcmp(key, "cm108_device") && take(key)) config.cm108_device = value;
#endif
        else if (!strcmp(key, "port") && take(key)) config.port = atoi(value);
        else if (!strcmp(key, "bind_address") && take(key)) config.bind_address = value;
        else if (!strcmp(key, "control_bind_address") && take(key)) config.control_bind_address = value;
    }

    fclose(f);
    return true;
}

void print_help(const char* prog) {
    std::cerr << "MODEM73\n\n"
              << "Usage: " << prog << " [options]\n\n"
              << "Options:\n"
              << "  -p, --port PORT         KISS TCP port (default: 8001)\n"
              << "  --bind ADDR             KISS bind address (default: 0.0.0.0)\n"
              << "  --control-port PORT     Control port (default: 8073, 0 to disable)\n"
              << "  --control-bind ADDR     Control port bind address (default: 127.0.0.1)\n"
              << "  -d, --device DEV        Audio device for both I/O\n"
              << "  --input-device DEV      Audio input  device\n"
              << "  --output-device DEV     Audio output device\n"
              << "  --list-audio            List available audio devices and exit\n"
              << "  -c, --callsign CALL     Callsign (default: N0CALL)\n"
              << "  -m, --modulation MOD    BPSK/QPSK/8PSK/QAM16/QAM64/QAM256 (default: QPSK)\n"
              << "  -r, --rate RATE         Code rate: 1/2, 2/3, 3/4, 5/6, 1/4 (default: 1/2)\n"
              << "  -f, --freq FREQ         Center frequency in Hz (default: 1500)\n"
              << "  --short                 Use short frames\n"
              << "  --normal                Use normal frames (default)\n"
              << "  --long                  Use long frames\n"
              << "  --no-rxfilter           Disable RX bandpass in front of the OFDM decoder\n"
              << "\nPTT options:\n"
              << "  --ptt TYPE              PTT type: none, rigctl, vox, com"
#ifdef WITH_CM108
              << ", cm108"
#endif
              << " (default: rigctl)\n"
              << "  --rigctl HOST:PORT      Rigctl address (default: localhost:4532)\n"
              << "  --com-port PORT         Serial port for COM PTT (default: /dev/ttyUSB0)\n"
              << "  --com-line LINE         COM PTT line: dtr, rts, both, -dtr, -rts, -both\n"
              << "                          (prefix '-' inverts polarity; default: rts)\n"
              << "  --vox-freq HZ           VOX tone frequency (default: 1200)\n"
              << "  --vox-lead MS           VOX lead time in ms (default: 150)\n"
              << "  --vox-tail MS           VOX tail time in ms (default: 100)\n"
#ifdef WITH_CM108
              << "  --cm108-gpio N          CM108 GPIO pin for PTT (default: 3)\n"
              << "  --cm108-device SPEC     CM108 device to use: serial or USB path\n"
              << "                          (default: first compatible device)\n"
              << "  --list-cm108            List CM108-compatible devices and exit\n"
#endif
              << "  --ptt-delay MS          PTT delay before TX (default: 50)\n"
              << "  --ptt-tail MS           PTT tail after TX (default: 50)\n"
              << "\nRX decoder options:\n"
              << "  --no-mfsk-rx            Disable the 3 always-on MFSK RX decoders to save CPU\n"
              << "                          (ignored while an MFSK mode is selected for TX)\n"
              << "\nCSMA options:\n"
              << "  --no-csma               Disable CSMA carrier sense\n"
              << "  --csma-threshold DB     Carrier sense threshold (default: -30)\n"
              << "  --csma-slot MS          Slot time in ms (default: 500)\n"
              << "  --csma-quiet MS         Idle time before contending (default: 0 = auto from frame airtime)\n"
              << "  --csma-cw N             Contention window in slots (default: 8)\n"
              << "  --csma-dither MS        Responder delay spread from callsign hash (default: 0 = off)\n"
              << "  --csma-burst N          Packets sent per channel acquisition, 1-4 (default: 2)\n"
              << "  --lead-tone             Send tone during TXDelay so others detect keyup (default)\n"
              << "  --no-lead-tone          Send silence during TXDelay instead\n"
              << "  --csma-persist N        P-persistence 0-255 (deprecated, unused)\n"
              << "\nFragmentation:\n"
              << "  --frag                  Enable packet fragmentation/reassembly\n"
              << "  --no-frag               Disable fragmentation (default)\n"
              << "\nTX Blanking:\n"
              << "  --tx-blank              Suppress decoder during TX\n"
              << "  --no-tx-blank           Disable TX blanking (default)\n"
              << "\n"
#ifdef WITH_UI
              << "  -h, --headless          Run without TUI\n"
#endif
              << "  -v, --verbose           Verbose output\n"
              << "  --config [FILE]         Load options from FILE\n"
              << "                          (defaults to ~/.config/modem73/settings)\n"
              << "  --help                  Show this help\n"
              << "\nSettings are saved to ~/.config/modem73/settings\n";
}

int main(int argc, char** argv) {
    std::cerr << "MODEM73 build " << __DATE__ << " " << __TIME__ << std::endl;

    TNCConfig config;

    // Track which settings were explicitly set on CLI
    std::set<std::string> cli_set;
    bool cli_control_port = false;
    bool cli_config = false;

    // Parse arguments
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        
        if (arg == "--help") {
            print_help(argv[0]);
            return 0;
        } else if (arg == "--list-audio") {
            std::cout << "Input  0devices:\n";
            auto input_devices = MiniAudio::list_capture_devices();
            for (const auto& dev : input_devices) {
                std::cout << "  " << dev.second << "\n";
            }
            std::cout << "\nOutput devices:\n";
            auto output_devices = MiniAudio::list_playback_devices();
            for (const auto& dev : output_devices) {
                std::cout << "  " << dev.second << "\n";
            }
            return 0;
#ifdef WITH_CM108
        } else if (arg == "--list-cm108") {
            auto devices = CM108PTT::enumerate();
            if (devices.empty()) {
                std::cout << "No CM108-compatible devices found\n";
            } else {
                std::cout << "CM108-compatible devices:\n";
                for (const auto& d : devices) {
                    std::cout << "  " << d.chip;
                    if (!d.product.empty()) std::cout << " [" << d.product << "]";
                    std::cout << "\n    serial: " << (d.serial.empty() ? "(none)" : d.serial)
                              << "\n    path:   " << d.path << "\n";
                }
            }
            return 0;
#endif
        } else if (arg == "-v" || arg == "--verbose") {
            g_verbose = true;
        } else if (arg == "-h" || arg == "--headless") {
#ifdef WITH_UI
            g_use_ui = false;
#endif
        } else if ((arg == "-p" || arg == "--port") && i + 1 < argc) {
            config.port = std::atoi(argv[++i]);
            cli_set.insert("port");
        } else if (arg == "--bind" && i + 1 < argc) {
            config.bind_address = argv[++i];
            cli_set.insert("bind_address");
        } else if (arg == "--control-bind" && i + 1 < argc) {
            config.control_bind_address = argv[++i];
            cli_set.insert("control_bind_address");
        } else if (arg == "--control-port" && i + 1 < argc) {
            config.control_port = std::atoi(argv[++i]);
            cli_control_port = true;
        } else if (arg == "--config") {
            cli_config = true;
            if (i + 1 < argc && argv[i + 1][0] != '-') {
                config.config_file = argv[++i];
            } else {
                const char* home = getenv("HOME");
                if (home) {
                    config.config_file = std::string(home) + "/.config/modem73/settings";
                }
            }
        } else if ((arg == "-d" || arg == "--device") && i + 1 < argc) {
            // Set both input and output to same device
            config.audio_input_device = argv[++i];
            config.audio_output_device = config.audio_input_device;
            cli_set.insert("audio_input");
            cli_set.insert("audio_output");
        } else if (arg == "--input-device" && i + 1 < argc) {
            config.audio_input_device = argv[++i];
            cli_set.insert("audio_input");
        } else if (arg == "--output-device" && i + 1 < argc) {
            config.audio_output_device = argv[++i];
            cli_set.insert("audio_output");
        } else if ((arg == "-c" || arg == "--callsign") && i + 1 < argc) {
            config.callsign = argv[++i];
            cli_set.insert("callsign");
        } else if ((arg == "-m" || arg == "--modulation") && i + 1 < argc) {
            config.modulation = argv[++i];
            cli_set.insert("modulation");
        } else if ((arg == "-r" || arg == "--rate") && i + 1 < argc) {
            config.code_rate = argv[++i];
            cli_set.insert("code_rate");
        } else if ((arg == "-f" || arg == "--freq") && i + 1 < argc) {
            config.center_freq = std::atoi(argv[++i]);
            cli_set.insert("center_freq");
        } else if (arg == "--short") {
            config.frame_size = 0;
            cli_set.insert("frame_size");
        } else if (arg == "--normal") {
            config.frame_size = 1;
            cli_set.insert("frame_size");
        } else if (arg == "--long") {
            config.frame_size = 2;
            cli_set.insert("frame_size");
        } else if (arg == "--no-rxfilter") {
            config.rx_filter_enabled = false;
            cli_set.insert("rx_filter_enabled");
        } else if (arg == "--rigctl" && i + 1 < argc) {
            config.ptt_type = PTTType::RIGCTL;
            cli_set.insert("ptt_type");
            std::string hostport = argv[++i];
            size_t colon = hostport.find(':');
            if (colon != std::string::npos) {
                config.rigctl_host = hostport.substr(0, colon);
                config.rigctl_port = std::atoi(hostport.substr(colon + 1).c_str());
            } else {
                config.rigctl_host = hostport;
            }
        } else if (arg == "--com-port" && i + 1 < argc) {
            config.com_port = argv[++i];
            cli_set.insert("com_port");
        } else if (arg == "--com-line" && i + 1 < argc) {
            std::string line = argv[++i];
            bool invert_specified = false;
            if (line == "dtr") {
                config.com_ptt_line = 0;
            } else if (line == "rts") {
                config.com_ptt_line = 1;
            } else if (line == "both") {
                config.com_ptt_line = 2;
            } else if (line == "-dtr") {
                config.com_ptt_line = 0;
                config.com_invert_dtr = true;
                config.com_invert_rts = false;
                invert_specified = true;
            } else if (line == "-rts") {
                config.com_ptt_line = 1;
                config.com_invert_dtr = false;
                config.com_invert_rts = true;
                invert_specified = true;
            } else if (line == "-both") {
                config.com_ptt_line = 2;
                config.com_invert_dtr = true;
                config.com_invert_rts = true;
                invert_specified = true;
            } else {
                std::cerr << "Unknown COM PTT line: " << line
                          << " (use dtr, rts, both, -dtr, -rts, -both)\n";
                return 1;
            }
            cli_set.insert("com_ptt_line");
            if (invert_specified) {
                cli_set.insert("com_invert_dtr");
                cli_set.insert("com_invert_rts");
            }
        } else if (arg == "--ptt" && i + 1 < argc) {
            cli_set.insert("ptt_type");
            std::string ptt_type = argv[++i];
            if (ptt_type == "none") config.ptt_type = PTTType::NONE;
            else if (ptt_type == "rigctl") config.ptt_type = PTTType::RIGCTL;
            else if (ptt_type == "vox") config.ptt_type = PTTType::VOX;
            else if (ptt_type == "com") config.ptt_type = PTTType::COM;
#ifdef WITH_CM108
            else if (ptt_type == "cm108") config.ptt_type = PTTType::CM108;
#endif
            else {
                std::cerr << "Unknown PTT type: " << ptt_type << " (use none, rigctl, vox, com"
#ifdef WITH_CM108
                          << ", cm108"
#endif
                          << ")\n";
                return 1;
            }
        } else if (arg == "--vox-freq" && i + 1 < argc) {
            config.vox_tone_freq = std::atoi(argv[++i]);
            cli_set.insert("vox_tone_freq");
        } else if (arg == "--vox-lead" && i + 1 < argc) {
            config.vox_lead_ms = std::atoi(argv[++i]);
            cli_set.insert("vox_lead_ms");
        } else if (arg == "--vox-tail" && i + 1 < argc) {
            config.vox_tail_ms = std::atoi(argv[++i]);
            cli_set.insert("vox_tail_ms");
#ifdef WITH_CM108
        } else if (arg == "--cm108-gpio" && i + 1 < argc) {
            config.cm108_gpio = std::atoi(argv[++i]);
            cli_set.insert("cm108_gpio");
        } else if (arg == "--cm108-device" && i + 1 < argc) {
            config.cm108_device = argv[++i];
            cli_set.insert("cm108_device");
#endif
        } else if (arg == "--ptt-delay" && i + 1 < argc) {
            config.ptt_delay_ms = std::atoi(argv[++i]);
        } else if (arg == "--ptt-tail" && i + 1 < argc) {
            config.ptt_tail_ms = std::atoi(argv[++i]);
        } else if (arg == "--no-rigctl") {
            config.ptt_type = PTTType::NONE;
            cli_set.insert("ptt_type");
        } else if (arg == "--no-mfsk-rx") {
            config.mfsk_rx_enabled = false;
            cli_set.insert("mfsk_rx_enabled");
        } else if (arg == "--no-csma") {
            config.csma_enabled = false;
            cli_set.insert("csma_enabled");
        } else if (arg == "--csma-threshold" && i + 1 < argc) {
            config.carrier_threshold_db = std::atof(argv[++i]);
            cli_set.insert("carrier_threshold_db");
        } else if (arg == "--csma-slot" && i + 1 < argc) {
            config.slot_time_ms = std::atoi(argv[++i]);
            cli_set.insert("slot_time_ms");
        } else if (arg == "--csma-quiet" && i + 1 < argc) {
            config.csma_quiet_ms = std::atoi(argv[++i]);
            cli_set.insert("csma_quiet_ms");
        } else if (arg == "--csma-cw" && i + 1 < argc) {
            config.csma_cw = std::atoi(argv[++i]);
            cli_set.insert("csma_cw");
        } else if (arg == "--csma-dither" && i + 1 < argc) {
            config.csma_responder_dither = std::atoi(argv[++i]);
            cli_set.insert("csma_responder_dither");
        } else if (arg == "--csma-burst" && i + 1 < argc) {
            config.csma_burst = std::atoi(argv[++i]);
            cli_set.insert("csma_burst");
        } else if (arg == "--lead-tone") {
            config.tx_lead_tone = true;
            cli_set.insert("tx_lead_tone");
        } else if (arg == "--no-lead-tone") {
            config.tx_lead_tone = false;
            cli_set.insert("tx_lead_tone");
        } else if (arg == "--csma-persist" && i + 1 < argc) {
            config.p_persistence = std::atoi(argv[++i]);
            cli_set.insert("p_persistence");
        } else if (arg == "--frag") {
            config.fragmentation_enabled = true;
            cli_set.insert("fragmentation_enabled");
        } else if (arg == "--no-frag") {
            config.fragmentation_enabled = false;
            cli_set.insert("fragmentation_enabled");
        } else if (arg == "--tx-blank") {
            config.tx_blanking_enabled = true;
            cli_set.insert("tx_blanking_enabled");
        } else if (arg == "--no-tx-blank") {
            config.tx_blanking_enabled = false;
            cli_set.insert("tx_blanking_enabled");
        } else {
            std::cerr << "Unknown option: " << arg << std::endl;
            print_help(argv[0]);
            return 1;
        }
    }


    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    signal(SIGPIPE, SIG_IGN);

    if (!g_use_ui && cli_config && !config.config_file.empty()) {
        if (apply_settings_file(config.config_file, config, cli_set)) {
            std::cerr << "Loaded settings from " << config.config_file << std::endl;
        } else {
            std::cerr << "Could not read config file: " << config.config_file << std::endl;
        }
    }

#ifdef WITH_UI
    TNCUIState ui_state;
    if (g_use_ui) {
        g_ui_state = &ui_state;
        
        // Set up config file path
        const char* home = getenv("HOME");
        if (home) {
            std::string config_dir = std::string(home) + "/.config/modem73";
            mkdir(config_dir.c_str(), 0755);
            ui_state.config_file = cli_config && !config.config_file.empty()
                                       ? config.config_file
                                       : config_dir + "/settings";
            ui_state.presets_file = config_dir + "/presets";
            
            auto input_devices = MiniAudio::list_capture_devices();
            for (const auto& dev : input_devices) {
                ui_state.available_input_devices.push_back(dev.first);
                ui_state.input_device_descriptions.push_back(dev.second);
            }
            if (ui_state.available_input_devices.empty()) {
                ui_state.available_input_devices.push_back("default");
                ui_state.input_device_descriptions.push_back("default - System Default");
            }
            
            auto output_devices = MiniAudio::list_playback_devices();
            for (const auto& dev : output_devices) {
                ui_state.available_output_devices.push_back(dev.first);
                ui_state.output_device_descriptions.push_back(dev.second);
            }
            if (ui_state.available_output_devices.empty()) {
                ui_state.available_output_devices.push_back("default");
                ui_state.output_device_descriptions.push_back("default - System Default");
            }
            
            // Try to load saved settings
            if (ui_state.load_settings()) {
                // Apply loaded settings to config
                if (!cli_set.count("callsign"))
                    config.callsign = ui_state.callsign;
                config.modem_type = ui_state.modem_type_index;
                config.mfsk_mode = ui_state.mfsk_mode_index;
                config.robust_mode = ui_state.robust_mode_index;
                config.tx_drive = ui_state.tx_drive;
                config.center_freq = ui_state.center_freq;
                config.modulation = MODULATION_OPTIONS[ui_state.modulation_index];
                config.code_rate = CODE_RATE_OPTIONS[ui_state.code_rate_index];
                config.frame_size = ui_state.frame_size;
                config.postamble = ui_state.postamble;
                config.csma_enabled = ui_state.csma_enabled;
                config.carrier_threshold_db = ui_state.carrier_threshold_db;
                config.slot_time_ms = ui_state.slot_time_ms;
                config.csma_quiet_ms = ui_state.csma_quiet_ms;
                config.csma_cw = ui_state.csma_cw;
                config.csma_responder_dither = ui_state.csma_responder_dither;
                config.csma_burst = ui_state.csma_burst;
                config.tx_lead_tone = ui_state.tx_lead_tone;
                config.p_persistence = ui_state.p_persistence;
                config.fragmentation_enabled = ui_state.fragmentation_enabled;
                config.tx_blanking_enabled = ui_state.tx_blanking_enabled;
                // Audio devices
                config.audio_input_device = ui_state.audio_input_device;
                config.audio_output_device = ui_state.audio_output_device;
                // PTT settings
                if (!cli_set.count("ptt_type"))
                    config.ptt_type = static_cast<PTTType>(ui_state.ptt_type_index);
                config.vox_tone_freq = ui_state.vox_tone_freq;
                config.vox_lead_ms = ui_state.vox_lead_ms;
                config.vox_tail_ms = ui_state.vox_tail_ms;

                // COM PTT settings
                config.com_port = ui_state.com_port;
                config.com_ptt_line = ui_state.com_ptt_line;
                config.com_invert_dtr = ui_state.com_invert_dtr;
                config.com_invert_rts = ui_state.com_invert_rts;

#ifdef WITH_CM108
                // CM108 PTT settings

                if (!cli_set.count("cm108_gpio"))
                    config.cm108_gpio = ui_state.cm108_gpio;

                if (!cli_set.count("cm108_device"))
                    config.cm108_device = ui_state.cm108_device;
                
#endif

                // Network settings
                if (!cli_set.count("port"))
                    config.port = ui_state.port;
                if (!cli_set.count("bind_address"))
                    config.bind_address = ui_state.bind_address;
                if (!cli_set.count("control_bind_address"))
                    config.control_bind_address = ui_state.control_bind_address;

                // Find audio device indices
                for (size_t i = 0; i < ui_state.available_input_devices.size(); i++) {
                    if (ui_state.available_input_devices[i] == ui_state.audio_input_device) {
                        ui_state.audio_input_index = i;
                        break;
                    }
                }
                for (size_t i = 0; i < ui_state.available_output_devices.size(); i++) {
                    if (ui_state.available_output_devices[i] == ui_state.audio_output_device) {
                        ui_state.audio_output_index = i;
                        break;
                    }
                }
                
                std::cerr << "Loaded settings from " << ui_state.config_file << std::endl;
            } else {

                ui_state.callsign = config.callsign;
                ui_state.center_freq = config.center_freq;
                ui_state.csma_enabled = config.csma_enabled;
                ui_state.carrier_threshold_db = config.carrier_threshold_db;
                ui_state.slot_time_ms = config.slot_time_ms;
                ui_state.csma_quiet_ms = config.csma_quiet_ms;
                ui_state.csma_cw = config.csma_cw;
                ui_state.csma_responder_dither = config.csma_responder_dither;
                ui_state.csma_burst = config.csma_burst;
                ui_state.tx_lead_tone = config.tx_lead_tone;
                ui_state.p_persistence = config.p_persistence;
                ui_state.frame_size = config.frame_size;
                ui_state.postamble = config.postamble;
                ui_state.fragmentation_enabled = config.fragmentation_enabled;
                ui_state.tx_blanking_enabled = config.tx_blanking_enabled;
                // Audio devices
                ui_state.audio_input_device = config.audio_input_device;
                ui_state.audio_output_device = config.audio_output_device;




                // PTT settings
                ui_state.ptt_type_index = static_cast<int>(config.ptt_type);
                ui_state.vox_tone_freq = config.vox_tone_freq;
                ui_state.vox_lead_ms = config.vox_lead_ms;
                ui_state.vox_tail_ms = config.vox_tail_ms;
                // COM PTT settings
                ui_state.com_port = config.com_port;
                ui_state.com_ptt_line = config.com_ptt_line;
                ui_state.com_invert_dtr = config.com_invert_dtr;
                ui_state.com_invert_rts = config.com_invert_rts;
#ifdef WITH_CM108
                // CM108 PTT settings
                ui_state.cm108_gpio = config.cm108_gpio;
                ui_state.cm108_device = config.cm108_device;
#endif
                // Network settings
                ui_state.port = config.port;
                ui_state.bind_address = config.bind_address;
                ui_state.control_bind_address = config.control_bind_address;

                // Find modulation index
                for (size_t i = 0; i < MODULATION_OPTIONS.size(); ++i) {
                    if (MODULATION_OPTIONS[i] == config.modulation) {
                        ui_state.modulation_index = i;
                        break;
                    }
                }
                
                // Find code rate index
                for (size_t i = 0; i < CODE_RATE_OPTIONS.size(); ++i) {
                    if (CODE_RATE_OPTIONS[i] == config.code_rate) {
                        ui_state.code_rate_index = i;
                        break;
                    }
                }
            }
        }
        
        // Set PTT info for display
        ui_state.ptt_type_index = static_cast<int>(config.ptt_type);
        ui_state.rigctl_host = config.rigctl_host;
        ui_state.rigctl_port = config.rigctl_port;
        ui_state.vox_tone_freq = config.vox_tone_freq;
        ui_state.vox_lead_ms = config.vox_lead_ms;
        ui_state.vox_tail_ms = config.vox_tail_ms;
        



        ui_state.load_presets();
        
        // Sync fragmentation setting from command line to UI
        ui_state.fragmentation_enabled = config.fragmentation_enabled;
        ui_state.tx_blanking_enabled = config.tx_blanking_enabled;

        ui_state.update_modem_info();
        
        // Set up stop callback
        ui_state.on_stop_requested = []() {
            g_running = false;
        };
    }
#endif
    
    if (!valid_bind_address(config.bind_address)) {
        std::cerr << "Error: invalid bind address '" << config.bind_address << "'" << std::endl;
        return 1;
    }
    if (!valid_bind_address(config.control_bind_address)) {
        std::cerr << "Error: invalid control bind address '"
                  << config.control_bind_address << "'" << std::endl;
        return 1;
    }

    while (!check_port_available(config.bind_address, config.port)) {
        std::cerr << "Error: Port " << config.port << " is already in use or cannot be bound" << std::endl;
        std::cerr << "Another instance of modem73 may be running, or another application is using this port." << std::endl;
        
        if (!g_use_ui) {
            std::cerr << "Use --port to specify a different port." << std::endl;
            return 1;
        }
        
        std::cerr << "\nEnter a different port number (or 'q' to quit): ";
        std::string input;
        if (!std::getline(std::cin, input) || input.empty() || input == "q" || input == "Q") {
            std::cerr << "Exiting." << std::endl;
            return 1;
        }
        
        try {
            int new_port = std::stoi(input);
            if (new_port < 1 || new_port > 65535) {
                std::cerr << "Invalid port number. Must be between 1 and 65535." << std::endl;
                continue;
            }
            config.port = new_port;
#ifdef WITH_UI
            if (g_use_ui) {
                ui_state.port = new_port;
            }
#endif
            std::cerr << "Trying port " << config.port << "..." << std::endl;
        } catch (const std::exception&) {
            std::cerr << "Invalid input. Please enter a number." << std::endl;
        }
    }
    
    while (config.control_port > 0 && !check_port_available(config.control_bind_address, config.control_port)) {
        std::cerr << "Error: Control port " << config.control_port << " is already in use" << std::endl;

        if (!g_use_ui) {
            std::cerr << "Use --control-port to specify a different port." << std::endl;
            return 1;
        }

        std::cerr << "\nEnter a different control port (or 'q' to quit, 0 to disable): ";
        std::string input;
        if (!std::getline(std::cin, input) || input.empty() || input == "q" || input == "Q") {
            std::cerr << "Exiting." << std::endl;
            return 1;
        }

        try {
            int new_port = std::stoi(input);
            if (new_port < 0 || new_port > 65535) {
                std::cerr << "Invalid port number. Must be 0-65535." << std::endl;
                continue;
            }
            config.control_port = new_port;
            if (new_port == 0)
                std::cerr << "Control port disabled." << std::endl;
            else
                std::cerr << "Trying control port " << config.control_port << "..." << std::endl;
        } catch (const std::exception&) {
            std::cerr << "Invalid input. Please enter a number." << std::endl;
        }
    }

    try {
        KISSTNC tnc(config);

        // Set up control port
        std::unique_ptr<ControlPort> ctrl;
        if (config.control_port > 0) {
            ControlPort::TNCInterface ctrl_iface;

            ctrl_iface.get_status = [&tnc]() -> cJSON* {
                cJSON* j = cJSON_CreateObject();
                auto stats = tnc.get_decoder_stats();

                // Channel state
                const char* state = "idle";
                if (tnc.is_transmitting()) state = "tx";
                else if (tnc.is_receiving()) state = "rx";
                cJSON_AddStringToObject(j, "channel_state", state);

                cJSON_AddBoolToObject(j, "ptt_on", tnc.is_transmitting());
                cJSON_AddNumberToObject(j, "tx_queue", (double)tnc.tx_queue_depth());
                cJSON_AddNumberToObject(j, "rx_frame_count", stats.sync_count - stats.preamble_errors - stats.crc_errors);
                cJSON_AddNumberToObject(j, "tx_frame_count", 0); // TODO: add tx counter to KISSTNC
                cJSON_AddNumberToObject(j, "rx_error_count", stats.preamble_errors + stats.crc_errors);
                cJSON_AddNumberToObject(j, "sync_count", stats.sync_count);
                cJSON_AddNumberToObject(j, "preamble_errors", stats.preamble_errors);
                cJSON_AddNumberToObject(j, "symbol_errors", stats.symbol_errors);
                cJSON_AddNumberToObject(j, "erased_symbols", stats.erased_symbols);
                cJSON_AddNumberToObject(j, "crc_errors", stats.crc_errors);
                cJSON_AddNumberToObject(j, "last_snr", stats.last_snr);
                cJSON_AddNumberToObject(j, "last_ber", stats.last_ber);
                cJSON_AddNumberToObject(j, "ber_ema", stats.ber_ema);
                cJSON_AddNumberToObject(j, "client_count", tnc.get_client_count());
                cJSON_AddBoolToObject(j, "rigctl_connected", tnc.is_rigctl_connected());
                cJSON_AddBoolToObject(j, "audio_connected", tnc.is_audio_healthy());

                return j;
            };

            ctrl_iface.get_config = [&tnc]() -> cJSON* {
                cJSON* j = cJSON_CreateObject();
                TNCConfig cfg = tnc.get_config();

                cJSON_AddStringToObject(j, "callsign", cfg.callsign.c_str());
                cJSON_AddNumberToObject(j, "modem_type", cfg.modem_type);
                cJSON_AddNumberToObject(j, "mfsk_mode", cfg.mfsk_mode);
                cJSON_AddNumberToObject(j, "robust_mode", cfg.robust_mode);
                if (cfg.modem_type == 1) {
                    cJSON_AddStringToObject(j, "modulation",
                        MFSK_MODE_NAMES[cfg.mfsk_mode < 4 ? cfg.mfsk_mode : 0]);
                } else if (cfg.modem_type == 2) {
                    cJSON_AddStringToObject(j, "modulation",
                        ROBUST_MODE_NAMES[cfg.robust_mode >= 0 &&
                            cfg.robust_mode < ROBUST_MODE_COUNT ? cfg.robust_mode : 0]);
                } else {
                    cJSON_AddStringToObject(j, "modulation", cfg.modulation.c_str());
                }
                cJSON_AddStringToObject(j, "code_rate", cfg.code_rate.c_str());
                cJSON_AddBoolToObject(j, "short_frame", cfg.frame_size == 0);
                cJSON_AddNumberToObject(j, "frame_size", cfg.frame_size);
                cJSON_AddBoolToObject(j, "postamble", cfg.postamble);
                cJSON_AddNumberToObject(j, "center_freq", cfg.center_freq);
                cJSON_AddNumberToObject(j, "payload_size", tnc.get_payload_size());
                cJSON_AddBoolToObject(j, "csma_enabled", cfg.csma_enabled);
                cJSON_AddNumberToObject(j, "carrier_threshold_db", cfg.carrier_threshold_db);
                cJSON_AddNumberToObject(j, "p_persistence", cfg.p_persistence);
                cJSON_AddNumberToObject(j, "slot_time_ms", cfg.slot_time_ms);
                cJSON_AddNumberToObject(j, "csma_quiet_ms", cfg.csma_quiet_ms);
                cJSON_AddNumberToObject(j, "csma_cw", cfg.csma_cw);
                cJSON_AddNumberToObject(j, "csma_responder_dither", cfg.csma_responder_dither);
                cJSON_AddNumberToObject(j, "csma_burst", cfg.csma_burst);
                cJSON_AddBoolToObject(j, "tx_lead_tone", cfg.tx_lead_tone);
                cJSON_AddBoolToObject(j, "tx_blanking_enabled", cfg.tx_blanking_enabled);
                cJSON_AddBoolToObject(j, "fragmentation_enabled", cfg.fragmentation_enabled);
                cJSON_AddBoolToObject(j, "mfsk_rx_enabled", cfg.mfsk_rx_enabled);

                return j;
            };

            ctrl_iface.set_config = [&tnc](cJSON* params) -> bool {
                TNCConfig new_config = tnc.get_config();

                cJSON* item;
                if ((item = cJSON_GetObjectItemCaseSensitive(params, "modem_type")) && cJSON_IsNumber(item)
                    && item->valueint >= 0 && item->valueint <= 2)
                    new_config.modem_type = item->valueint;
                if ((item = cJSON_GetObjectItemCaseSensitive(params, "mfsk_mode")) && cJSON_IsNumber(item)
                    && item->valueint >= 0 && item->valueint <= 3)
                    new_config.mfsk_mode = item->valueint;
                if ((item = cJSON_GetObjectItemCaseSensitive(params, "robust_mode")) && cJSON_IsNumber(item)
                    && item->valueint >= 0 && item->valueint < ROBUST_MODE_COUNT)
                    new_config.robust_mode = item->valueint;
                if ((item = cJSON_GetObjectItemCaseSensitive(params, "callsign")) && cJSON_IsString(item))
                    new_config.callsign = item->valuestring;
                if ((item = cJSON_GetObjectItemCaseSensitive(params, "modulation")) && cJSON_IsString(item))
                    new_config.modulation = item->valuestring;
                if ((item = cJSON_GetObjectItemCaseSensitive(params, "code_rate")) && cJSON_IsString(item))
                    new_config.code_rate = item->valuestring;
                if ((item = cJSON_GetObjectItemCaseSensitive(params, "short_frame")) && cJSON_IsBool(item))
                    new_config.frame_size = cJSON_IsTrue(item) ? 0 : 1;
                if ((item = cJSON_GetObjectItemCaseSensitive(params, "frame_size")) && cJSON_IsNumber(item)
                    && item->valueint >= 0 && item->valueint <= 2)
                    new_config.frame_size = item->valueint;
                if ((item = cJSON_GetObjectItemCaseSensitive(params, "center_freq")) && cJSON_IsNumber(item))
                    new_config.center_freq = item->valueint;
                if ((item = cJSON_GetObjectItemCaseSensitive(params, "postamble")) && cJSON_IsBool(item))
                    new_config.postamble = cJSON_IsTrue(item);
                if ((item = cJSON_GetObjectItemCaseSensitive(params, "csma_enabled")) && cJSON_IsBool(item))
                    new_config.csma_enabled = cJSON_IsTrue(item);
                if ((item = cJSON_GetObjectItemCaseSensitive(params, "carrier_threshold_db")) && cJSON_IsNumber(item))
                    new_config.carrier_threshold_db = (float)item->valuedouble;
                if ((item = cJSON_GetObjectItemCaseSensitive(params, "p_persistence")) && cJSON_IsNumber(item))
                    new_config.p_persistence = item->valueint;
                if ((item = cJSON_GetObjectItemCaseSensitive(params, "slot_time_ms")) && cJSON_IsNumber(item))
                    new_config.slot_time_ms = item->valueint;
                if ((item = cJSON_GetObjectItemCaseSensitive(params, "csma_quiet_ms")) && cJSON_IsNumber(item))
                    new_config.csma_quiet_ms = item->valueint;
                if ((item = cJSON_GetObjectItemCaseSensitive(params, "csma_cw")) && cJSON_IsNumber(item))
                    new_config.csma_cw = item->valueint;
                if ((item = cJSON_GetObjectItemCaseSensitive(params, "csma_responder_dither")) && cJSON_IsNumber(item))
                    new_config.csma_responder_dither = item->valueint;
                if ((item = cJSON_GetObjectItemCaseSensitive(params, "csma_burst")) && cJSON_IsNumber(item))
                    new_config.csma_burst = item->valueint;
                if ((item = cJSON_GetObjectItemCaseSensitive(params, "tx_lead_tone")) && cJSON_IsBool(item))
                    new_config.tx_lead_tone = cJSON_IsTrue(item);
                if ((item = cJSON_GetObjectItemCaseSensitive(params, "tx_blanking_enabled")) && cJSON_IsBool(item))
                    new_config.tx_blanking_enabled = cJSON_IsTrue(item);
                if ((item = cJSON_GetObjectItemCaseSensitive(params, "fragmentation_enabled")) && cJSON_IsBool(item))
                    new_config.fragmentation_enabled = cJSON_IsTrue(item);

                tnc.update_config(new_config);

#ifdef WITH_UI
                // Sync config back to TUI state so the UI reflects changes
                if (g_ui_state) {
                    g_ui_state->callsign = new_config.callsign;
                    g_ui_state->modem_type_index = new_config.modem_type;
                    g_ui_state->mfsk_mode_index = new_config.mfsk_mode;
                    g_ui_state->robust_mode_index = new_config.robust_mode;
                    g_ui_state->center_freq = new_config.center_freq;
                    g_ui_state->frame_size = new_config.frame_size;
                    g_ui_state->postamble = new_config.postamble;
                    g_ui_state->csma_enabled = new_config.csma_enabled;
                    g_ui_state->carrier_threshold_db = new_config.carrier_threshold_db;
                    g_ui_state->p_persistence = new_config.p_persistence;
                    g_ui_state->slot_time_ms = new_config.slot_time_ms;
                    g_ui_state->tx_blanking_enabled = new_config.tx_blanking_enabled;
                    g_ui_state->fragmentation_enabled = new_config.fragmentation_enabled;

                    // Map modulation string back to index
                    for (size_t i = 0; i < MODULATION_OPTIONS.size(); i++) {
                        if (MODULATION_OPTIONS[i] == new_config.modulation) {
                            g_ui_state->modulation_index = i;
                            break;
                        }
                    }
                    // Map code rate string back to index
                    for (size_t i = 0; i < CODE_RATE_OPTIONS.size(); i++) {
                        if (CODE_RATE_OPTIONS[i] == new_config.code_rate) {
                            g_ui_state->code_rate_index = i;
                            break;
                        }
                    }

                    g_ui_state->update_modem_info();
                }
#endif
                return true;
            };

            ctrl_iface.rigctl_command = [&tnc](const std::string& cmd) -> std::string {
                return tnc.rigctl_command(cmd);
            };

            ctrl_iface.tx_data = [&tnc](const std::vector<uint8_t>& data, int oper_mode) -> bool {
                tnc.queue_data_ex(data, oper_mode);
                return true;
            };

            ctrl = std::make_unique<ControlPort>(config.control_port, config.control_bind_address, ctrl_iface);
            ctrl->start();

            tnc.rx_stats_callback = [&ctrl](float snr, float ber_pct, float level_db) {
                if (ctrl) ctrl->notify_rx_frame(snr, ber_pct, level_db);
            };
        }

#ifdef WITH_UI
        if (g_use_ui) {
            ui_state.perf_logger = &tnc.perf_log_;
            ui_state.on_settings_changed = [&tnc, &ctrl](TNCUIState& state) {
                TNCConfig new_config = tnc.get_config();
                new_config.modem_type = state.modem_type_index;
                new_config.mfsk_mode = state.mfsk_mode_index;
                new_config.robust_mode = state.robust_mode_index;
                new_config.callsign = state.callsign;
                new_config.center_freq = state.center_freq;
                new_config.modulation = MODULATION_OPTIONS[state.modulation_index];
                new_config.code_rate = CODE_RATE_OPTIONS[state.code_rate_index];
                new_config.frame_size = state.frame_size;
                new_config.postamble = state.postamble;
                new_config.csma_enabled = state.csma_enabled;
                new_config.carrier_threshold_db = state.carrier_threshold_db;
                new_config.p_persistence = state.p_persistence;
                new_config.slot_time_ms = state.slot_time_ms;
                new_config.csma_quiet_ms = state.csma_quiet_ms;
                new_config.csma_cw = state.csma_cw;
                new_config.csma_responder_dither = state.csma_responder_dither;
                new_config.csma_burst = state.csma_burst;
                new_config.tx_lead_tone = state.tx_lead_tone;
                new_config.fragmentation_enabled = state.fragmentation_enabled;
                new_config.tx_blanking_enabled = state.tx_blanking_enabled;
                new_config.tx_drive = state.tx_drive;
                new_config.audio_input_device = state.audio_input_device;
                new_config.audio_output_device = state.audio_output_device;
                // PTT settings
                new_config.ptt_type = static_cast<PTTType>(state.ptt_type_index);
                new_config.vox_tone_freq = state.vox_tone_freq;
                new_config.vox_lead_ms = state.vox_lead_ms;
                new_config.vox_tail_ms = state.vox_tail_ms;
                // COM PTT settings
                new_config.com_port = state.com_port;
                new_config.com_ptt_line = state.com_ptt_line;
                new_config.com_invert_dtr = state.com_invert_dtr;
                new_config.com_invert_rts = state.com_invert_rts;

                tnc.update_config(new_config);
                if (ctrl) ctrl->notify_config_changed();
            };
            
            // Set up send data callback for UTILS tab
            ui_state.on_send_data = [&tnc](const std::vector<uint8_t>& data) {
                tnc.queue_data(data);
            };
            
            // Set up audio reconnect callback
            ui_state.on_reconnect_audio = [&tnc]() -> bool {
                return tnc.reconnect_audio();
            };

            ui_state.on_alc_tune = [&tnc]() -> float {
                return tnc.alc_auto_tune();
            };

            ui_state.on_rigctl_command = [&tnc](const std::string& cmd) -> std::string {
                return tnc.rigctl_command(cmd);
            };

            // Run TNC in background thread
            std::thread tnc_thread([&tnc]() {
                tnc.run();
            });
            
            // Status update thread 
            std::thread status_thread([&tnc, &ui_state]() {
                while (g_running) {
                    ui_state.rigctl_connected = tnc.is_rigctl_connected();
                    ui_state.audio_connected = tnc.is_audio_healthy();
                    if (ui_state.rig_poll_enabled.load()) {
                        ui_state.poll_rig();
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                }
            });
            
            TNCUI ui(ui_state);
            ui.run();
            
            // cleanup
            g_running = false;
            status_thread.join();
            tnc_thread.join();

            for (int i = 0; i < 100 && ui_state.alc_tune_running.load(); ++i)
                std::this_thread::sleep_for(std::chrono::milliseconds(100));

        } else {
            tnc.run();
        }
#else
        tnc.run();
#endif
        if (ctrl) ctrl->stop();
    } catch (const std::exception& e) {
        std::cerr << "error " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}