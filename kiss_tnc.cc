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
#include "modem.hh"

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
        // Allocate encoder/decoder on heap 
        std::cerr << "  Creating encoder" << std::endl;
        encoder_ = std::make_unique<Encoder48k>();
        std::cerr << "  Creating decoder" << std::endl;
        decoder_ = std::make_unique<Decoder48k>();
        std::cerr << "  Encoder/decoder created" << std::endl;
        
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
        
        payload_size_ = encoder_->get_payload_size(modem_config_.oper_mode);
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
                    tx_queue_.push(std::move(frag));
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
                tx_queue_.push(frame_data);
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
            std::vector<uint8_t> frame;
            if (tx_queue_.pop(frame)) {
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
                
                transmit(frame);
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
    }
    
    void transmit(const std::vector<uint8_t>& data) {
        ui_log("TX: " + std::to_string(data.size()) + " bytes");
        if (g_verbose) {
            std::cerr << packet_visualize(data.data(), data.size(), true, config_.fragmentation_enabled) << std::endl;
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
        auto samples = encoder_->encode(
            framed_data.data(), framed_data.size(),
            modem_config_.center_freq,
            modem_config_.call_sign,
            modem_config_.oper_mode
        );
        
        if (samples.empty()) {
            ui_log("TX: Encoding failed");
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
            if (config_.ptt_type == PTTType::RIGCTL || config_.ptt_type == PTTType::COM) {
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
            if (config_.ptt_type == PTTType::RIGCTL || config_.ptt_type == PTTType::COM) {
                std::this_thread::sleep_for(std::chrono::milliseconds(config_.ptt_tail_ms));
                set_ptt(false);
            }
        }
        
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
        
        auto deliver_to_clients = [this](const std::vector<uint8_t>& payload, float snr, bool was_reassembled) {
            ui_log("RX: " + std::to_string(payload.size()) + " bytes, SNR=" + 
                   std::to_string((int)snr) + "dB" + (was_reassembled ? " (reassembled)" : ""));
            if (g_verbose) {
                std::cerr << packet_visualize(payload.data(), payload.size(), false, false) << std::endl;
            }
            
#ifdef WITH_UI
            if (g_ui_state) {
                g_ui_state->add_packet(false, payload.size(), snr);
            }
#endif
            
            auto kiss_frame = KISSParser::wrap(payload);
            
            std::lock_guard<std::mutex> lock(clients_mutex_);
            for (auto& client : clients_) {
                client->send(kiss_frame);
            }
        };
        
        auto frame_callback = [this, &deliver_to_clients](const uint8_t* data, size_t len) {
            set_tx_lockout(RX_LOCKOUT_SECONDS);
            
            float snr = decoder_->get_last_snr();
            
#ifdef WITH_UI
            if (g_ui_state) {
                g_ui_state->rx_frame_count++;
                g_ui_state->receiving = false;
                g_ui_state->last_rx_snr = snr;
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
            
            if (config_.fragmentation_enabled && reassembler_.is_fragment(payload)) {
                if (g_verbose) {
                    std::cerr << packet_visualize(payload.data(), payload.size(), false, true) << std::endl;
                }
                
                auto reassembled = reassembler_.process(payload);
                if (!reassembled.empty()) {
                    ui_log("RX: Reassembled " + std::to_string(reassembled.size()) + " bytes from fragments");
                    deliver_to_clients(reassembled, snr, true);
                }
            } else {
                deliver_to_clients(payload, snr, false);
            }
        };
        
        while (rx_running_ && g_running) {
            int n = audio_->read(buffer.data(), buffer.size());
            if (n > 0) {
                decoder_->process(buffer.data(), n, frame_callback);
                
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
    
    std::unique_ptr<MiniAudio> audio_;
    std::unique_ptr<RigctlPTT> rigctl_;
    std::unique_ptr<SerialPTT> serial_ptt_;
    std::unique_ptr<DummyPTT> dummy_ptt_;
    
    int server_fd_ = -1;
    std::list<std::unique_ptr<ClientConnection>> clients_;
    std::mutex clients_mutex_;
    
    PacketQueue<std::vector<uint8_t>> tx_queue_;
    std::atomic<bool> tx_running_{false};
    std::atomic<bool> rx_running_{false};
    
    Fragmenter fragmenter_;
    Reassembler reassembler_;
    
    // TX lockout - prevents TX while receiving
    std::mutex lockout_mutex_;
    std::chrono::steady_clock::time_point tx_lockout_until_;
    static constexpr float RX_LOCKOUT_SECONDS = 0.5f;
    
public:
    // Update config at runtime (called from UI)
    void update_config(const TNCConfig& new_config) {
        // Update CSMA settings (safe to change at runtime)
        config_.csma_enabled = new_config.csma_enabled;
        config_.carrier_threshold_db = new_config.carrier_threshold_db;
        config_.p_persistence = new_config.p_persistence;
        config_.slot_time_ms = new_config.slot_time_ms;
        
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
            ui_log("Center frequency changed to " + std::to_string(config_.center_freq) + " Hz");
        }
        
        // Update modulation settings
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
                payload_size_ = encoder_->get_payload_size(modem_config_.oper_mode);
                ui_log("Mode changed to " + config_.modulation + " " + config_.code_rate + 
                       " " + (config_.short_frame ? "short" : "normal") +
                       " (" + std::to_string(payload_size_) + " bytes)");
            }
        }
    }
    
    TNCConfig& get_config() { return config_; }
    
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
        size_t max_payload = payload_size_ - 2;
        
        if (config_.fragmentation_enabled && fragmenter_.needs_fragmentation(data.size(), max_payload)) {
            auto fragments = fragmenter_.fragment(data, max_payload);
            ui_log("TX: Fragmenting " + std::to_string(data.size()) + " bytes into " + 
                   std::to_string(fragments.size()) + " fragments");
            for (auto& frag : fragments) {
                tx_queue_.push(std::move(frag));
            }
        } else {
            tx_queue_.push(data);
        }
#ifdef WITH_UI
        if (g_ui_state) {
            g_ui_state->tx_queue_size = tx_queue_.size();
        }
#endif
    }
};

void print_help(const char* prog) {
    std::cerr << "MODEM73\n\n"
              << "Usage: " << prog << " [options]\n\n"
              << "Options:\n"
              << "  -p, --port PORT         TCP port (default: 8001)\n"
              << "  -d, --device DEV        Audio device for both I/O\n"
              << "  --input-device DEV      Audio input  device\n"
              << "  --output-device DEV     Audio output device\n"
              << "  --list-audio            List available audio devices and exit\n"
              << "  -c, --callsign CALL     Callsign (default: N0CALL)\n"
              << "  -m, --modulation MOD    BPSK/QPSK/8PSK/QAM16/QAM64/QAM256 (default: QPSK)\n"
              << "  -r, --rate RATE         Code rate: 1/2, 2/3, 3/4, 5/6 (default: 1/2)\n"
              << "  -f, --freq FREQ         Center frequency in Hz (default: 1500)\n"
              << "  --short                 Use short frames\n"
              << "  --normal                Use normal frames (default)\n"
              << "\nPTT options:\n"
              << "  --ptt TYPE              PTT type: none, rigctl, vox (default: rigctl)\n"
              << "  --rigctl HOST:PORT      Rigctl address (default: localhost:4532)\n"
              << "  --vox-freq HZ           VOX tone frequency (default: 1200)\n"
              << "  --vox-lead MS           VOX lead time in ms (default: 150)\n"
              << "  --vox-tail MS           VOX tail time in ms (default: 100)\n"
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
        } else if (arg == "--ptt" && i + 1 < argc) {
            std::string ptt_type = argv[++i];
            if (ptt_type == "none") config.ptt_type = PTTType::NONE;
            else if (ptt_type == "rigctl") config.ptt_type = PTTType::RIGCTL;
            else if (ptt_type == "vox") config.ptt_type = PTTType::VOX;
            else {
                std::cerr << "Unknown PTT type: " << ptt_type << " (use none, rigctl, or vox)\n";
                return 1;
            }
        } else if (arg == "--vox-freq" && i + 1 < argc) {
            config.vox_tone_freq = std::atoi(argv[++i]);
        } else if (arg == "--vox-lead" && i + 1 < argc) {
            config.vox_lead_ms = std::atoi(argv[++i]);
        } else if (arg == "--vox-tail" && i + 1 < argc) {
            config.vox_tail_ms = std::atoi(argv[++i]);
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
                config.callsign = ui_state.callsign;
                config.center_freq = ui_state.center_freq;
                config.modulation = MODULATION_OPTIONS[ui_state.modulation_index];
                config.code_rate = CODE_RATE_OPTIONS[ui_state.code_rate_index];
                config.short_frame = ui_state.short_frame;
                config.csma_enabled = ui_state.csma_enabled;
                config.carrier_threshold_db = ui_state.carrier_threshold_db;
                config.slot_time_ms = ui_state.slot_time_ms;
                config.p_persistence = ui_state.p_persistence;
                config.fragmentation_enabled = ui_state.fragmentation_enabled;
                // Audio devices
                config.audio_input_device = ui_state.audio_input_device;
                config.audio_output_device = ui_state.audio_output_device;
                // PTT settings
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
    
    try {
        KISSTNC tnc(config);
        
#ifdef WITH_UI
        if (g_use_ui) {
            ui_state.on_settings_changed = [&tnc](TNCUIState& state) {
                TNCConfig new_config = tnc.get_config();
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
                // Audio devices 
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
    } catch (const std::exception& e) {
        std::cerr << "error " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}