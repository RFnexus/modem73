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
#include <mutex>
#include <memory>
#include <random>

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
#include "miniaudio_audio.hh"
#include "rigctl_ptt.hh"
#include "serial_ptt.hh"
#ifdef WITH_CM108
#include "cm108_ptt.hh"
#endif
#include "modem.hh"
#include "phy/mfsk_modem.hh"
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
    addr.sin_addr.s_addr = inet_addr(bind_address.c_str());
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
    
    void send(const std::vector<uint8_t>& data) {
        std::lock_guard<std::mutex> lock(write_mutex);
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
class KISSTNC {
public:
    KISSTNC(const TNCConfig& config) : config_(config) {
        // Allocate OFDM encoder/decoder
        std::cerr << "  Creating OFDM encoder/decoder" << std::endl;
        encoder_ = std::make_unique<Encoder48k>();
        decoder_ = std::make_unique<Decoder48k>();

        // Allocate MFSK encoder/decoder
        std::cerr << "  Creating MFSK encoder/decoder" << std::endl;
        mfsk_encoder_ = std::make_unique<MFSKEncoder>();
        mfsk_decoder_ = std::make_unique<MFSKDecoder>(
            (MFSKMode)config.mfsk_mode, config.center_freq);

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
            config.short_frame
        );

        if (modem_config_.call_sign < 0) {
            throw std::runtime_error("Invalid callsign");
        }
        if (modem_config_.oper_mode < 0) {
            throw std::runtime_error("Invalid modulation or code rate");
        }

        if (config.modem_type == 1) {
            payload_size_ = mfsk_encoder_->get_payload_size((MFSKMode)config.mfsk_mode);
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
            cm108_ptt_->open(config_.cm108_gpio);
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
        addr.sin_addr.s_addr = inet_addr(config_.bind_address.c_str());
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
                  << " " << (config_.short_frame ? "short" : "normal") << std::endl;
        std::cerr << "Payload: " << payload_size_ << " bytes (including 2-byte length prefix)" << std::endl;
        
        if (config_.csma_enabled) {
            std::cerr << "CSMA: enabled (threshold=" << config_.carrier_threshold_db 
                      << " dB, slot=" << config_.slot_time_ms 
                      << " ms, p=" << config_.p_persistence << "/255)" << std::endl;
        } else {
            std::cerr << "CSMA: disabled" << std::endl;
        }
        
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
        
        // Main  
        while (g_running) {
            struct sockaddr_in client_addr;
            socklen_t client_len = sizeof(client_addr);
            int client_fd = accept(server_fd_, (struct sockaddr*)&client_addr, &client_len);
            
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
            switch (cmd) {
            case KISS::CMD_TXDELAY:
                if (!data.empty()) {
                    config_.tx_delay_ms = data[0] * 10;
                    ui_log("TXDelay set to " + std::to_string(config_.tx_delay_ms) + " ms");
                }
                break;
            case KISS::CMD_P:
                if (!data.empty()) {
                    config_.p_persistence = data[0];
                    ui_log("P-persistence set to " + std::to_string(config_.p_persistence));
                }
                break;
            case KISS::CMD_SLOTTIME:
                if (!data.empty()) {
                    config_.slot_time_ms = data[0] * 10;
                    ui_log("Slot time set to " + std::to_string(config_.slot_time_ms) + " ms");
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
                // Wait for TX lockout to clear 
                if (!is_tx_allowed()) {
                    std::cerr << "TX: Waiting for lockout to clear..." << std::endl;
                    wait_for_tx_allowed();
                }
                
                // CSMA
                if (config_.csma_enabled) {
                    int backoff_count = 0;
                    
                    while (backoff_count < config_.max_backoff_slots) {
                        // Re-check lockout after backoff
                        if (!is_tx_allowed()) {
                            wait_for_tx_allowed();
                        }
                        
                        // Check carrier
                        float level_db = audio_->measure_level(config_.carrier_sense_ms);
                        bool is_busy = (level_db > config_.carrier_threshold_db);
                        
                        if (is_busy) {
                            // Channel busy - wait
                            std::uniform_int_distribution<> slots_dist(1, 
                                std::min(1 << backoff_count, config_.max_backoff_slots));
                            int slots = slots_dist(gen);
                            int wait_ms = slots * config_.slot_time_ms;
                            
                            std::cerr << "CSMA: Channel busy (" << level_db << " dB > " 
                                      << config_.carrier_threshold_db << " dB), backing off " 
                                      << slots << " slots (" << wait_ms << " ms)" << std::endl;
                            
                            std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));
                            backoff_count++;
                        } else {
                            // Channel clear - apply p-persistence
                            std::uniform_int_distribution<> p_dist(0, 255);
                            if (p_dist(gen) < config_.p_persistence) {
                                std::cerr << "CSMA: Channel clear (" << level_db << " dB), transmitting" << std::endl;
                                break;
                            } else {
                                std::cerr << "CSMA: Channel clear but deferring (p=" 
                                          << config_.p_persistence << "/255)" << std::endl;
                                std::this_thread::sleep_for(
                                    std::chrono::milliseconds(config_.slot_time_ms));
                            }
                        }
                    }
                    
                    if (backoff_count >= config_.max_backoff_slots) {
                        std::cerr << "CSMA: Max backoff reached, transmitting anyway" << std::endl;
                    }
                }
                
                transmit(pkt.data, pkt.oper_mode);
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
    }
    
    void transmit(const std::vector<uint8_t>& data, int oper_mode_override = -1) {
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
            g_ui_state->add_packet(true, data.size(), 0);
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
        } else {
            samples = encoder_->encode(
                framed_data.data(), framed_data.size(),
                modem_config_.center_freq,
                modem_config_.call_sign,
                tx_mode
            );
        }
        
        if (samples.empty()) {
            ui_log("TX: Encoding failed");
            tx_blanking_active_ = false;
#ifdef WITH_UI
            if (g_ui_state) g_ui_state->transmitting = false;
#endif
            return;
        }
        
        float duration = samples.size() / (float)config_.sample_rate;
        float total_tx_duration = duration;
        
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
                audio_->write(samples.data() + i, n);
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
            total_tx_duration += (config_.tx_delay_ms + config_.ptt_tail_ms) / 1000.0f;
            
            ui_log("TX: " + std::to_string(samples.size()) + " samples, " + 
                   std::to_string(duration) + " seconds");
            
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
            audio_->write_silence(config_.tx_delay_ms * config_.sample_rate / 1000);
            
            // Transmit audio
            const int chunk_size = 1024;
            for (size_t i = 0; i < samples.size(); i += chunk_size) {
                int n = std::min(chunk_size, (int)(samples.size() - i));
                audio_->write(samples.data() + i, n);
            }
            
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
        
        tx_blanking_active_ = false;

#ifdef WITH_UI
        if (g_ui_state) {
            g_ui_state->transmitting = false;
            g_ui_state->total_tx_time = g_ui_state->total_tx_time.load() + total_tx_duration;
        }
#endif
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
        
        auto deliver_to_clients = [this](const std::vector<uint8_t>& payload, float snr, float ber_pct, bool was_reassembled) {
            ui_log("RX: " + std::to_string(payload.size()) + " bytes, SNR=" +
                   std::to_string((int)snr) + "dB" + (was_reassembled ? " (reassembled)" : ""));
            if (g_verbose) {
                std::cerr << packet_visualize(payload.data(), payload.size(), false, false) << std::endl;
            }

#ifdef WITH_UI
            if (g_ui_state) {
                g_ui_state->add_packet(false, payload.size(), snr, ber_pct);
            }
#endif
            
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
                    deliver_to_clients(reassembled, snr, ber_pct, true);
                }
            } else {
                deliver_to_clients(payload, snr, ber_pct, false);
            }
        };

        // MFSK frame callback
        auto mfsk_frame_callback = [this, &deliver_to_clients](const uint8_t* data, size_t len) {
            set_tx_lockout(RX_LOCKOUT_SECONDS);

            float snr = mfsk_decoder_->get_last_snr();
            float ber_pct = -1.0f;

#ifdef WITH_UI
            if (g_ui_state) {
                g_ui_state->rx_frame_count++;
                g_ui_state->receiving = false;
                g_ui_state->last_rx_snr = snr;
            }
#endif

            auto payload = unframe_length(data, len);

            if (payload.empty()) {
                ui_log("MFSK RX: Empty payload after unframing");
#ifdef WITH_UI
                if (g_ui_state) g_ui_state->rx_error_count++;
#endif
                return;
            }

            if (reassembler_.is_fragment(payload)) {
                auto reassembled = reassembler_.process(payload);
                if (!reassembled.empty()) {
                    ui_log("MFSK RX: Reassembled " + std::to_string(reassembled.size()) + " bytes");
                    deliver_to_clients(reassembled, snr, ber_pct, true);
                }
            } else {
                deliver_to_clients(payload, snr, ber_pct, false);
            }
        };

        bool was_blanking = false;

        while (rx_running_ && g_running) {
            int n = audio_->read(buffer.data(), buffer.size());
            if (n > 0) {
                bool blanking = tx_blanking_active_.load();

                if (blanking) {
                    was_blanking = true;
                } else {
                    if (was_blanking) {
                        decoder_->reset();
                        mfsk_decoder_->reset();
                        was_blanking = false;
                    }
                    // Feed same audio to both decod,ers
                    decoder_->process(buffer.data(), n, frame_callback);
                    mfsk_decoder_->process(buffer.data(), n, mfsk_frame_callback);
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

                    g_ui_state->update_level(db);

                    // Copy decoder stats
                    if (g_ui_state->stats_reset_requested.exchange(false)) {
                        decoder_->stats_sync_count = 0;
                        decoder_->stats_preamble_errors = 0;
                        decoder_->stats_symbol_errors = 0;
                        decoder_->stats_crc_errors = 0;
                        decoder_->reset_ber();
                        mfsk_decoder_->reset_stats();
                        g_ui_state->last_rx_ber = -1.0f;
                    }
                    if (config_.modem_type == 1) {
                        g_ui_state->sync_count = mfsk_decoder_->stats_sync_count;
                        g_ui_state->preamble_errors = mfsk_decoder_->stats_preamble_errors;
                        g_ui_state->symbol_errors = 0;
                        g_ui_state->crc_errors = mfsk_decoder_->stats_crc_errors;
                    } else {
                        g_ui_state->sync_count = decoder_->stats_sync_count;
                        g_ui_state->preamble_errors = decoder_->stats_preamble_errors;
                        g_ui_state->symbol_errors = decoder_->stats_symbol_errors;
                        g_ui_state->crc_errors = decoder_->stats_crc_errors;
                    }
                }
#endif
            }
        }
    }
    
    void set_ptt(bool on) {
        if (rigctl_) {
            rigctl_->set_ptt(on);
        } else if (serial_ptt_) {
            if (on) {
                serial_ptt_->ptt_on();
            } else {
                serial_ptt_->ptt_off();
            }
#ifdef WITH_CM108
        } else if (cm108_ptt_) {
            cm108_ptt_->set_ptt(on);
#endif
        } else if (dummy_ptt_) {
            dummy_ptt_->set_ptt(on);
        }
        
#ifdef WITH_UI
        if (g_ui_state) {
            g_ui_state->ptt_on = on;
        }
#endif
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
    
    TNCConfig config_;
    ModemConfig modem_config_;
    int payload_size_;
    
    std::unique_ptr<Encoder48k> encoder_;
    std::unique_ptr<Decoder48k> decoder_;
    std::unique_ptr<MFSKEncoder> mfsk_encoder_;
    std::unique_ptr<MFSKDecoder> mfsk_decoder_;

    std::unique_ptr<MiniAudio> audio_;
    std::unique_ptr<RigctlPTT> rigctl_;
    std::unique_ptr<SerialPTT> serial_ptt_;
#ifdef WITH_CM108
    std::unique_ptr<CM108PTT> cm108_ptt_;
#endif
    std::unique_ptr<DummyPTT> dummy_ptt_;
    
    int server_fd_ = -1;
    std::list<std::unique_ptr<ClientConnection>> clients_;
    mutable std::mutex clients_mutex_;
    
    PacketQueue<TxPacket> tx_queue_;
    std::atomic<bool> tx_running_{false};
    std::atomic<bool> rx_running_{false};
    
    Fragmenter fragmenter_;
    Reassembler reassembler_;
    
    // TX lockout - prevents TX while receiving
    mutable std::mutex lockout_mutex_;
    std::chrono::steady_clock::time_point tx_lockout_until_;
    static constexpr float RX_LOCKOUT_SECONDS = 0.5f;
    
    // TX blanking
    std::atomic<bool> tx_blanking_active_{false};
    
public:
    // Update config at runtime (called from UI)
    void update_config(const TNCConfig& new_config) {
        // Update CSMA settings (safe to change at runtime)
        config_.csma_enabled = new_config.csma_enabled;
        config_.carrier_threshold_db = new_config.carrier_threshold_db;
        config_.p_persistence = new_config.p_persistence;
        config_.slot_time_ms = new_config.slot_time_ms;
        
        // TX blanking
        config_.tx_blanking_enabled = new_config.tx_blanking_enabled;
        
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
            // Reconfigure MFSK decoder with new center freq
            mfsk_decoder_->configure((MFSKMode)config_.mfsk_mode, config_.center_freq);
            ui_log("Center frequency changed to " + std::to_string(config_.center_freq) + " Hz");
        }

        // Update modem type and sub-mode
        if (config_.modem_type != new_config.modem_type || config_.mfsk_mode != new_config.mfsk_mode) {
            config_.modem_type = new_config.modem_type;
            config_.mfsk_mode = new_config.mfsk_mode;
            if (config_.modem_type == 1) {
                MFSKMode mmode = (MFSKMode)config_.mfsk_mode;
                mfsk_decoder_->configure(mmode, config_.center_freq);
                payload_size_ = mfsk_encoder_->get_payload_size(mmode);
                ui_log("Mode changed to " + std::string(MFSK_MODE_NAMES[(int)mmode]) +
                       " (" + std::to_string(MFSKParams::max_payload(mmode)) + " bytes)");
            } else {
                payload_size_ = encoder_->get_payload_size(modem_config_.oper_mode);
            }
        }

        // Update OFDM modulation settings
        bool mode_changed = (config_.modulation != new_config.modulation ||
                            config_.code_rate != new_config.code_rate ||
                            config_.short_frame != new_config.short_frame);

        if (mode_changed) {
            config_.modulation = new_config.modulation;
            config_.code_rate = new_config.code_rate;
            config_.short_frame = new_config.short_frame;

            int new_mode = ModemConfig::encode_mode(
                config_.modulation.c_str(),
                config_.code_rate.c_str(),
                config_.short_frame
            );

            if (new_mode >= 0) {
                modem_config_.oper_mode = new_mode;
                if (config_.modem_type == 0) {
                    payload_size_ = encoder_->get_payload_size(modem_config_.oper_mode);
                }
                ui_log("OFDM mode changed to " + config_.modulation + " " + config_.code_rate +
                       " " + (config_.short_frame ? "short" : "normal") +
                       " (" + std::to_string(encoder_->get_payload_size(modem_config_.oper_mode)) + " bytes)");
            }
        }
    }
    
    TNCConfig& get_config() { return config_; }

    int get_payload_size() const { return payload_size_; }

    struct DecoderStats {
        int sync_count, preamble_errors, symbol_errors, crc_errors;
        float last_snr, last_ber, ber_ema;
    };

    DecoderStats get_decoder_stats() const {
        if (config_.modem_type == 1) {
            return {
                mfsk_decoder_->stats_sync_count,
                mfsk_decoder_->stats_preamble_errors,
                0, // MFSK has no symbol errors stat
                mfsk_decoder_->stats_crc_errors,
                mfsk_decoder_->get_last_snr(),
                mfsk_decoder_->get_last_ber(),
                mfsk_decoder_->get_ber_ema()
            };
        }
        return {
            decoder_->stats_sync_count,
            decoder_->stats_preamble_errors,
            decoder_->stats_symbol_errors,
            decoder_->stats_crc_errors,
            decoder_->get_last_snr(),
            decoder_->get_last_ber(),
            decoder_->get_ber_ema()
        };
    }

    bool is_transmitting() const { return tx_blanking_active_.load(); }

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

    // Compute oper_mode for a given short_frame setting using current modulation/code_rate
    int compute_oper_mode(bool short_frame) const {
        return ModemConfig::encode_mode(
            config_.modulation.c_str(),
            config_.code_rate.c_str(),
            short_frame
        );
    }
};

void print_help(const char* prog) {
    std::cerr << "MODEM73\n\n"
              << "Usage: " << prog << " [options]\n\n"
              << "Options:\n"
              << "  -p, --port PORT         KISS TCP port (default: 8001)\n"
              << "  --control-port PORT     Control port (default: 8073, 0 to disable)\n"
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
              << "\nPTT options:\n"
              << "  --ptt TYPE              PTT type: none, rigctl, vox, com"
#ifdef WITH_CM108
              << ", cm108"
#endif
              << " (default: rigctl)\n"
              << "  --rigctl HOST:PORT      Rigctl address (default: localhost:4532)\n"
              << "  --com-port PORT         Serial port for COM PTT (default: /dev/ttyUSB0)\n"
              << "  --com-line LINE         COM PTT line: dtr, rts, both (default: rts)\n"
              << "  --vox-freq HZ           VOX tone frequency (default: 1200)\n"
              << "  --vox-lead MS           VOX lead time in ms (default: 150)\n"
              << "  --vox-tail MS           VOX tail time in ms (default: 100)\n"
#ifdef WITH_CM108
              << "  --cm108-gpio N          CM108 GPIO pin for PTT (default: 3)\n"
#endif
              << "  --ptt-delay MS          PTT delay before TX (default: 50)\n"
              << "  --ptt-tail MS           PTT tail after TX (default: 50)\n"
              << "\nCSMA options:\n"
              << "  --no-csma               Disable CSMA carrier sense\n"
              << "  --csma-threshold DB     Carrier sense threshold (default: -30)\n"
              << "  --csma-slot MS          Slot time in ms (default: 500)\n"
              << "  --csma-persist N        P-persistence 0-255 (default: 128 = 50%)\n"
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
              << "  --help                  Show this help\n"
              << "\nSettings are saved to ~/.config/modem73/settings\n";
}

int main(int argc, char** argv) {
    TNCConfig config;

    // Track which settings were explicitly set on CLI
    bool cli_port = false;
    bool cli_control_port = false;
    bool cli_callsign = false;
    bool cli_ptt = false;

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
        } else if (arg == "-v" || arg == "--verbose") {
            g_verbose = true;
        } else if (arg == "-h" || arg == "--headless") {
#ifdef WITH_UI
            g_use_ui = false;
#endif
        } else if ((arg == "-p" || arg == "--port") && i + 1 < argc) {
            config.port = std::atoi(argv[++i]);
            cli_port = true;
        } else if (arg == "--control-port" && i + 1 < argc) {
            config.control_port = std::atoi(argv[++i]);
            cli_control_port = true;
        } else if ((arg == "-d" || arg == "--device") && i + 1 < argc) {
            // Set both input and output to same device
            config.audio_input_device = argv[++i];
            config.audio_output_device = config.audio_input_device;
        } else if (arg == "--input-device" && i + 1 < argc) {
            config.audio_input_device = argv[++i];
        } else if (arg == "--output-device" && i + 1 < argc) {
            config.audio_output_device = argv[++i];
        } else if ((arg == "-c" || arg == "--callsign") && i + 1 < argc) {
            config.callsign = argv[++i];
            cli_callsign = true;
        } else if ((arg == "-m" || arg == "--modulation") && i + 1 < argc) {
            config.modulation = argv[++i];
        } else if ((arg == "-r" || arg == "--rate") && i + 1 < argc) {
            config.code_rate = argv[++i];
        } else if ((arg == "-f" || arg == "--freq") && i + 1 < argc) {
            config.center_freq = std::atoi(argv[++i]);
        } else if (arg == "--short") {
            config.short_frame = true;
        } else if (arg == "--normal") {
            config.short_frame = false;
        } else if (arg == "--rigctl" && i + 1 < argc) {
            config.ptt_type = PTTType::RIGCTL;
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
        } else if (arg == "--com-line" && i + 1 < argc) {
            std::string line = argv[++i];
            if (line == "dtr") config.com_ptt_line = 0;
            else if (line == "rts") config.com_ptt_line = 1;
            else if (line == "both") config.com_ptt_line = 2;
            else {
                std::cerr << "Unknown COM PTT line: " << line << " (use dtr, rts, or both)\n";
                return 1;
            }
        } else if (arg == "--ptt" && i + 1 < argc) {
            cli_ptt = true;
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
        } else if (arg == "--vox-lead" && i + 1 < argc) {
            config.vox_lead_ms = std::atoi(argv[++i]);
        } else if (arg == "--vox-tail" && i + 1 < argc) {
            config.vox_tail_ms = std::atoi(argv[++i]);
#ifdef WITH_CM108
        } else if (arg == "--cm108-gpio" && i + 1 < argc) {
            config.cm108_gpio = std::atoi(argv[++i]);
#endif
        } else if (arg == "--ptt-delay" && i + 1 < argc) {
            config.ptt_delay_ms = std::atoi(argv[++i]);
        } else if (arg == "--ptt-tail" && i + 1 < argc) {
            config.ptt_tail_ms = std::atoi(argv[++i]);
        } else if (arg == "--no-rigctl") {
            config.ptt_type = PTTType::NONE; 
        } else if (arg == "--no-csma") {
            config.csma_enabled = false;
        } else if (arg == "--csma-threshold" && i + 1 < argc) {
            config.carrier_threshold_db = std::atof(argv[++i]);
        } else if (arg == "--csma-slot" && i + 1 < argc) {
            config.slot_time_ms = std::atoi(argv[++i]);
        } else if (arg == "--csma-persist" && i + 1 < argc) {
            config.p_persistence = std::atoi(argv[++i]);
        } else if (arg == "--frag") {
            config.fragmentation_enabled = true;
        } else if (arg == "--no-frag") {
            config.fragmentation_enabled = false;
        } else if (arg == "--tx-blank") {
            config.tx_blanking_enabled = true;
        } else if (arg == "--no-tx-blank") {
            config.tx_blanking_enabled = false;
        } else {
            std::cerr << "Unknown option: " << arg << std::endl;
            print_help(argv[0]);
            return 1;
        }
    }


    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    signal(SIGPIPE, SIG_IGN);
    
#ifdef WITH_UI
    TNCUIState ui_state;
    if (g_use_ui) {
        g_ui_state = &ui_state;
        
        // Set up config file path
        const char* home = getenv("HOME");
        if (home) {
            std::string config_dir = std::string(home) + "/.config/modem73";
            mkdir(config_dir.c_str(), 0755);
            ui_state.config_file = config_dir + "/settings";
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
                if (!cli_callsign)
                    config.callsign = ui_state.callsign;
                config.modem_type = ui_state.modem_type_index;
                config.mfsk_mode = ui_state.mfsk_mode_index;
                config.center_freq = ui_state.center_freq;
                config.modulation = MODULATION_OPTIONS[ui_state.modulation_index];
                config.code_rate = CODE_RATE_OPTIONS[ui_state.code_rate_index];
                config.short_frame = ui_state.short_frame;
                config.csma_enabled = ui_state.csma_enabled;
                config.carrier_threshold_db = ui_state.carrier_threshold_db;
                config.slot_time_ms = ui_state.slot_time_ms;
                config.p_persistence = ui_state.p_persistence;
                config.fragmentation_enabled = ui_state.fragmentation_enabled;
                config.tx_blanking_enabled = ui_state.tx_blanking_enabled;
                // Audio devices
                config.audio_input_device = ui_state.audio_input_device;
                config.audio_output_device = ui_state.audio_output_device;
                // PTT settings
                if (!cli_ptt)
                    config.ptt_type = static_cast<PTTType>(ui_state.ptt_type_index);
                config.vox_tone_freq = ui_state.vox_tone_freq;
                config.vox_lead_ms = ui_state.vox_lead_ms;
                config.vox_tail_ms = ui_state.vox_tail_ms;

                // COM PTT settings
                config.com_port = ui_state.com_port;
                config.com_ptt_line = ui_state.com_ptt_line;
                config.com_invert_dtr = ui_state.com_invert_dtr;
                config.com_invert_rts = ui_state.com_invert_rts;


                // Network settings 
                if (!cli_port)
                    config.port = ui_state.port;

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
                ui_state.p_persistence = config.p_persistence;
                ui_state.short_frame = config.short_frame;
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
                // Network settings
                ui_state.port = config.port;
                
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
    
    while (config.control_port > 0 && !check_port_available(config.bind_address, config.control_port)) {
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
                cJSON_AddNumberToObject(j, "rx_frame_count", stats.sync_count - stats.preamble_errors - stats.crc_errors);
                cJSON_AddNumberToObject(j, "tx_frame_count", 0); // TODO: add tx counter to KISSTNC
                cJSON_AddNumberToObject(j, "rx_error_count", stats.preamble_errors + stats.crc_errors);
                cJSON_AddNumberToObject(j, "sync_count", stats.sync_count);
                cJSON_AddNumberToObject(j, "preamble_errors", stats.preamble_errors);
                cJSON_AddNumberToObject(j, "symbol_errors", stats.symbol_errors);
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
                auto& cfg = tnc.get_config();

                cJSON_AddStringToObject(j, "callsign", cfg.callsign.c_str());
                cJSON_AddNumberToObject(j, "modem_type", cfg.modem_type);
                cJSON_AddNumberToObject(j, "mfsk_mode", cfg.mfsk_mode);
                if (cfg.modem_type == 1) {
                    cJSON_AddStringToObject(j, "modulation",
                        MFSK_MODE_NAMES[cfg.mfsk_mode < 4 ? cfg.mfsk_mode : 0]);
                } else {
                    cJSON_AddStringToObject(j, "modulation", cfg.modulation.c_str());
                }
                cJSON_AddStringToObject(j, "code_rate", cfg.code_rate.c_str());
                cJSON_AddBoolToObject(j, "short_frame", cfg.short_frame);
                cJSON_AddNumberToObject(j, "center_freq", cfg.center_freq);
                cJSON_AddNumberToObject(j, "payload_size", tnc.get_payload_size());
                cJSON_AddBoolToObject(j, "csma_enabled", cfg.csma_enabled);
                cJSON_AddNumberToObject(j, "carrier_threshold_db", cfg.carrier_threshold_db);
                cJSON_AddNumberToObject(j, "p_persistence", cfg.p_persistence);
                cJSON_AddNumberToObject(j, "slot_time_ms", cfg.slot_time_ms);
                cJSON_AddBoolToObject(j, "tx_blanking_enabled", cfg.tx_blanking_enabled);

                return j;
            };

            ctrl_iface.set_config = [&tnc](cJSON* params) -> bool {
                TNCConfig new_config = tnc.get_config();

                cJSON* item;
                if ((item = cJSON_GetObjectItemCaseSensitive(params, "modem_type")) && cJSON_IsNumber(item))
                    new_config.modem_type = item->valueint;
                if ((item = cJSON_GetObjectItemCaseSensitive(params, "mfsk_mode")) && cJSON_IsNumber(item))
                    new_config.mfsk_mode = item->valueint;
                if ((item = cJSON_GetObjectItemCaseSensitive(params, "callsign")) && cJSON_IsString(item))
                    new_config.callsign = item->valuestring;
                if ((item = cJSON_GetObjectItemCaseSensitive(params, "modulation")) && cJSON_IsString(item))
                    new_config.modulation = item->valuestring;
                if ((item = cJSON_GetObjectItemCaseSensitive(params, "code_rate")) && cJSON_IsString(item))
                    new_config.code_rate = item->valuestring;
                if ((item = cJSON_GetObjectItemCaseSensitive(params, "short_frame")) && cJSON_IsBool(item))
                    new_config.short_frame = cJSON_IsTrue(item);
                if ((item = cJSON_GetObjectItemCaseSensitive(params, "center_freq")) && cJSON_IsNumber(item))
                    new_config.center_freq = item->valueint;
                if ((item = cJSON_GetObjectItemCaseSensitive(params, "csma_enabled")) && cJSON_IsBool(item))
                    new_config.csma_enabled = cJSON_IsTrue(item);
                if ((item = cJSON_GetObjectItemCaseSensitive(params, "carrier_threshold_db")) && cJSON_IsNumber(item))
                    new_config.carrier_threshold_db = (float)item->valuedouble;
                if ((item = cJSON_GetObjectItemCaseSensitive(params, "p_persistence")) && cJSON_IsNumber(item))
                    new_config.p_persistence = item->valueint;
                if ((item = cJSON_GetObjectItemCaseSensitive(params, "slot_time_ms")) && cJSON_IsNumber(item))
                    new_config.slot_time_ms = item->valueint;
                if ((item = cJSON_GetObjectItemCaseSensitive(params, "tx_blanking_enabled")) && cJSON_IsBool(item))
                    new_config.tx_blanking_enabled = cJSON_IsTrue(item);

                tnc.update_config(new_config);

#ifdef WITH_UI
                // Sync config back to TUI state so the UI reflects changes
                if (g_ui_state) {
                    g_ui_state->callsign = new_config.callsign;
                    g_ui_state->modem_type_index = new_config.modem_type;
                    g_ui_state->mfsk_mode_index = new_config.mfsk_mode;
                    g_ui_state->center_freq = new_config.center_freq;
                    g_ui_state->short_frame = new_config.short_frame;
                    g_ui_state->csma_enabled = new_config.csma_enabled;
                    g_ui_state->carrier_threshold_db = new_config.carrier_threshold_db;
                    g_ui_state->p_persistence = new_config.p_persistence;
                    g_ui_state->slot_time_ms = new_config.slot_time_ms;
                    g_ui_state->tx_blanking_enabled = new_config.tx_blanking_enabled;

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

            ctrl = std::make_unique<ControlPort>(config.control_port, config.bind_address, ctrl_iface);
            ctrl->start();
        }

#ifdef WITH_UI
        if (g_use_ui) {
            ui_state.on_settings_changed = [&tnc, &ctrl](TNCUIState& state) {
                TNCConfig new_config = tnc.get_config();
                new_config.modem_type = state.modem_type_index;
                new_config.mfsk_mode = state.mfsk_mode_index;
                new_config.callsign = state.callsign;
                new_config.center_freq = state.center_freq;
                new_config.modulation = MODULATION_OPTIONS[state.modulation_index];
                new_config.code_rate = CODE_RATE_OPTIONS[state.code_rate_index];
                new_config.short_frame = state.short_frame;
                new_config.csma_enabled = state.csma_enabled;
                new_config.carrier_threshold_db = state.carrier_threshold_db;
                new_config.p_persistence = state.p_persistence;
                new_config.slot_time_ms = state.slot_time_ms;
                new_config.fragmentation_enabled = state.fragmentation_enabled;
                new_config.tx_blanking_enabled = state.tx_blanking_enabled;
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
            
            // Run TNC in background thread
            std::thread tnc_thread([&tnc]() {
                tnc.run();
            });
            
            // Status update thread 
            std::thread status_thread([&tnc, &ui_state]() {
                while (g_running) {
                    ui_state.rigctl_connected = tnc.is_rigctl_connected();
                    ui_state.audio_connected = tnc.is_audio_healthy();
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                }
            });
            
            TNCUI ui(ui_state);
            ui.run();
            
            // cleanup
            g_running = false;
            status_thread.join();
            tnc_thread.join();


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