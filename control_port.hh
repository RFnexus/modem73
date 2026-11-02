#pragma once

#include <string>
#include <thread>
#include <atomic>
#include <functional>
#include <memory>
#include <vector>
#include <list>
#include <map>
#include <set>
#include <mutex>
#include <cstring>
#include <cstdint>
#include <iostream>
#include <chrono>

#include <sys/socket.h>
#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0
#endif
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>

extern "C" {
#include "deps/cJSON.h"
}

#include "rx_frame_info.hh"

inline void add_rx_frame_fields(cJSON* j, const RxFrameInfo& f) {
    cJSON_AddNumberToObject(j, "seq", (double)f.seq);
    cJSON_AddNumberToObject(j, "time", f.time);
    cJSON_AddNumberToObject(j, "snr", f.snr);
    cJSON_AddNumberToObject(j, "ber_pct", f.ber_pct);
    cJSON_AddNumberToObject(j, "level_db", f.level_db);
    cJSON_AddNumberToObject(j, "size", f.size);
    cJSON_AddBoolToObject(j, "reassembled", f.reassembled);
    cJSON_AddStringToObject(j, "modem", f.modem.c_str());
    cJSON_AddStringToObject(j, "mode", f.mode.c_str());
    if (f.modem == "ofdm") {
        cJSON_AddNumberToObject(j, "oper_mode", f.oper_mode);
        cJSON_AddStringToObject(j, "modulation", f.modulation.c_str());
        cJSON_AddStringToObject(j, "code_rate", f.code_rate.c_str());
        cJSON_AddStringToObject(j, "frame_size", f.frame_size.c_str());
    } else if (f.modem == "robust") {
        cJSON_AddNumberToObject(j, "robust_mode", f.robust_mode);
    } else if (f.modem == "mfsk") {
        cJSON_AddNumberToObject(j, "mfsk_mode", f.mfsk_mode);
    }
    cJSON_AddStringToObject(j, "callsign", f.callsign.c_str());
    cJSON_AddStringToObject(j, "callsign_source", f.callsign_source.c_str());
    if (f.probe) {
        cJSON_AddBoolToObject(j, "probe", 1);
        cJSON_AddStringToObject(j, "suggested_mode", f.probe_mode.c_str());
    }
    if (f.probe_reply) {
        cJSON_AddBoolToObject(j, "probe_reply", 1);
        cJSON_AddNumberToObject(j, "reported_snr", f.reported_snr);
        cJSON_AddStringToObject(j, "suggested_mode", f.probe_mode.c_str());
    }
}

// Base64 decode (RFC 4648)
inline std::vector<uint8_t> base64_decode(const char* input, size_t len) {
    static const uint8_t T[256] = {
        64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,
        64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,
        64,64,64,64,64,64,64,64,64,64,64,62,64,64,64,63,
        52,53,54,55,56,57,58,59,60,61,64,64,64,65,64,64,
        64, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,
        15,16,17,18,19,20,21,22,23,24,25,64,64,64,64,64,
        64,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,
        41,42,43,44,45,46,47,48,49,50,51,64,64,64,64,64,
        64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,
        64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,
        64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,
        64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,
        64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,
        64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,
        64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,
        64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,
    };
    std::vector<uint8_t> out;
    out.reserve(len * 3 / 4);
    uint32_t buf = 0;
    int bits = 0;
    for (size_t i = 0; i < len; i++) {
        uint8_t v = T[(uint8_t)input[i]];
        if (v >= 64) continue; // skip padding and invalid
        buf = (buf << 6) | v;
        bits += 6;
        if (bits >= 8) {
            bits -= 8;
            out.push_back((buf >> bits) & 0xFF);
        }
    }
    return out;
}

inline std::string base64_encode(const uint8_t* data, size_t len) {
    static const char* T =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string out;
    out.reserve((len + 2) / 3 * 4);
    size_t i = 0;
    for (; i + 2 < len; i += 3) {
        uint32_t v = ((uint32_t)data[i] << 16) | ((uint32_t)data[i + 1] << 8) | data[i + 2];
        out.push_back(T[(v >> 18) & 63]);
        out.push_back(T[(v >> 12) & 63]);
        out.push_back(T[(v >> 6) & 63]);
        out.push_back(T[v & 63]);
    }
    if (i < len) {
        uint32_t v = (uint32_t)data[i] << 16;
        if (i + 1 < len) v |= (uint32_t)data[i + 1] << 8;
        out.push_back(T[(v >> 18) & 63]);
        out.push_back(T[(v >> 12) & 63]);
        out.push_back(i + 1 < len ? T[(v >> 6) & 63] : '=');
        out.push_back('=');
    }
    return out;
}

inline std::string json_frame(cJSON* json) {
    char* str = cJSON_PrintUnformatted(json);
    if (!str) return std::string();
    uint32_t len = strlen(str);
    std::string out;
    out.reserve(4 + len);
    out.push_back((char)((len >> 24) & 0xFF));
    out.push_back((char)((len >> 16) & 0xFF));
    out.push_back((char)((len >> 8) & 0xFF));
    out.push_back((char)(len & 0xFF));
    out.append(str, len);
    cJSON_free(str);
    return out;
}

class ControlPort {
public:
    struct TNCInterface {
        std::function<cJSON*()> get_status;
        std::function<cJSON*()> get_config;
        std::function<bool(cJSON* params)> set_config;
        std::function<std::string(const std::string&)> rigctl_command;
        std::function<bool(const std::vector<uint8_t>&, int oper_mode)> tx_data;
        std::function<bool()> send_beacon;
        std::function<bool()> send_probe;
        std::function<bool()> reconnect_audio;
        std::function<std::vector<RxFrameInfo>(size_t limit, uint64_t since_seq)> rx_frame_history;

        std::function<cJSON*()> get_ui_state;
        std::function<bool(const std::string& text, std::string* applied)> set_ui_settings;
        std::function<cJSON*(const std::string& stream, uint64_t& cursor)> stream_poll;
        std::function<uint64_t(const std::string& stream)> stream_cursor_now;
        std::function<void(const std::string& stream, int delta)> stream_refcount;
        std::function<void(std::function<void(cJSON*)> done)> list_audio_devices;
        std::function<void(std::function<void(float)> done)> alc_tune;
        std::function<void()> reset_stats;
        std::function<void()> restart;
        std::function<void()> idle;
    };

    static constexpr int STREAM_TICK_MS = 100;
    static constexpr size_t MAX_SEND_BUF = 8u * 1024 * 1024;

    static const std::set<std::string>& known_streams() {
        static const std::set<std::string> s = {
            "status", "log", "packet", "message", "constellation", "scope", "settings"};
        return s;
    }

    ControlPort(int port, const std::string& bind_address, TNCInterface iface)
        : port_(port), bind_address_(bind_address), iface_(std::move(iface)),
          alive_(std::make_shared<std::atomic<bool>>(true)) {}

    ~ControlPort() {
        alive_->store(false);
        stop();
    }

    bool start() {
        server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
        if (server_fd_ < 0) {
            std::cerr << "control: Failed to create socket" << std::endl;
            return false;
        }

        int opt = 1;
        setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        if (inet_pton(AF_INET, bind_address_.c_str(), &addr.sin_addr) != 1) {
            std::cerr << "control: invalid bind address " << bind_address_ << std::endl;
            close(server_fd_);
            server_fd_ = -1;
            return false;
        }
        addr.sin_port = htons(port_);

        if (bind(server_fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            std::cerr << "control: Failed to bind to port " << port_ << std::endl;
            close(server_fd_);
            server_fd_ = -1;
            return false;
        }

        if (listen(server_fd_, 5) < 0) {
            std::cerr << "control: Failed to listen" << std::endl;
            close(server_fd_);
            server_fd_ = -1;
            return false;
        }

        fcntl(server_fd_, F_SETFL, O_NONBLOCK);

        running_ = true;
        thread_ = std::thread(&ControlPort::run, this);

        std::cerr << "Control port listening on " << bind_address_ << ":" << port_ << std::endl;
        return true;
    }

    void stop() {
        running_ = false;
        if (thread_.joinable()) {
            thread_.join();
        }
        if (server_fd_ >= 0) {
            close(server_fd_);
            server_fd_ = -1;
        }
    }

    // Push a config_changed event to all connected clients
    void notify_config_changed() {
        if (!iface_.get_config) return;
        cJSON* config = iface_.get_config();
        if (!config) return;

        cJSON* event = cJSON_CreateObject();
        cJSON_AddStringToObject(event, "event", "config_changed");
        cJSON_AddItemToObject(event, "config", config);
        broadcast_event(event);
        cJSON_Delete(event);
    }

    // Push per-frame receive stats to all connected clients
    void notify_rx_frame(const RxFrameInfo& info) {
        cJSON* event = cJSON_CreateObject();
        cJSON_AddStringToObject(event, "event", "rx_frame");
        add_rx_frame_fields(event, info);
        broadcast_event(event);
        cJSON_Delete(event);
    }

    void notify_ui_settings(const std::string& text, uint64_t except_id = 0) {
        cJSON* event = cJSON_CreateObject();
        cJSON_AddStringToObject(event, "event", "ui_settings");
        cJSON_AddStringToObject(event, "settings", text.c_str());
        publish("settings", event, except_id);
        cJSON_Delete(event);
    }

    // Push an event to all connected clients (thread-safe)
    void broadcast_event(cJSON* event) {
        std::string frame = json_frame(event);
        if (frame.empty()) return;
        std::lock_guard<std::recursive_mutex> lock(clients_mutex_);
        for (auto& client : clients_)
            queue_frame(client, frame);
    }

    void publish(const char* stream, cJSON* event, uint64_t except_id = 0) {
        std::string frame = json_frame(event);
        if (frame.empty()) return;
        std::lock_guard<std::recursive_mutex> lock(clients_mutex_);
        for (auto& client : clients_)
            if (client.id != except_id && client.streams.count(stream))
                queue_frame(client, frame);
    }

private:
    struct Client {
        uint64_t id;
        int fd;
        std::vector<uint8_t> recv_buf;
        std::string send_buf;
        size_t send_off = 0;
        bool dead = false;
        std::set<std::string> streams;
        std::map<std::string, uint64_t> cursors;

        Client(uint64_t id, int fd) : id(id), fd(fd) {}
    };

    void run() {
        auto last_tick = std::chrono::steady_clock::now();
        while (running_) {
            if (iface_.idle) iface_.idle();
            // Accept new connections
            struct sockaddr_in client_addr;
            socklen_t client_len = sizeof(client_addr);
            int client_fd = accept(server_fd_, (struct sockaddr*)&client_addr, &client_len);

            if (client_fd >= 0) {
                int flag = 1;
                setsockopt(client_fd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));
                fcntl(client_fd, F_SETFL, O_NONBLOCK);

                char ip_str[INET_ADDRSTRLEN];
                inet_ntop(AF_INET, &client_addr.sin_addr, ip_str, sizeof(ip_str));
                std::cerr << "control: Client connected from " << ip_str << std::endl;

                {
                    std::lock_guard<std::recursive_mutex> lock(clients_mutex_);
                    clients_.emplace_back(next_client_id_++, client_fd);
                }
            }

            // Poll clients
            std::vector<std::pair<uint64_t, std::string>> pending;
            {
                std::lock_guard<std::recursive_mutex> lock(clients_mutex_);
                for (auto& client : clients_) {
                    if (client.dead) continue;
                    uint8_t buf[4096];
                    ssize_t n = recv(client.fd, buf, sizeof(buf), MSG_DONTWAIT);

                    if (n > 0) {
                        client.recv_buf.insert(client.recv_buf.end(), buf, buf + n);
                        if (!process_recv_buf(client, pending))
                            client.dead = true;
                    } else if (n == 0 || (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK)) {
                        client.dead = true;
                    }
                }
            }
            for (auto& m : pending)
                handle_message(m.first, m.second);

            auto now = std::chrono::steady_clock::now();
            if (now - last_tick >= std::chrono::milliseconds(STREAM_TICK_MS)) {
                last_tick = now;
                stream_tick();
            }

            {
                std::lock_guard<std::recursive_mutex> lock(clients_mutex_);
                for (auto it = clients_.begin(); it != clients_.end();) {
                    flush_locked(*it);
                    if (it->dead) {
                        std::cerr << "control: Client disconnected" << std::endl;
                        release_client(*it);
                        close(it->fd);
                        it = clients_.erase(it);
                        continue;
                    }
                    ++it;
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        // Cleanup
        {
            std::lock_guard<std::recursive_mutex> lock(clients_mutex_);
            for (auto& client : clients_) {
                release_client(client);
                close(client.fd);
            }
            clients_.clear();
        }
    }

    void stream_tick() {
        if (!iface_.stream_poll) return;
        std::lock_guard<std::recursive_mutex> lock(clients_mutex_);
        for (auto& client : clients_) {
            if (client.dead) continue;
            for (const auto& name : client.streams) {
                cJSON* ev = iface_.stream_poll(name, client.cursors[name]);
                if (!ev) continue;
                queue_json(client, ev);
                cJSON_Delete(ev);
            }
        }
    }

    void release_client(Client& client) {
        if (iface_.stream_refcount)
            for (const auto& name : client.streams)
                iface_.stream_refcount(name, -1);
        client.streams.clear();
    }

    void queue_frame(Client& client, const std::string& frame) {
        if (client.dead) return;
        if (client.send_buf.size() - client.send_off + frame.size() > MAX_SEND_BUF) {
            std::cerr << "control: client not draining, dropping" << std::endl;
            client.dead = true;
            return;
        }
        client.send_buf += frame;
        flush_locked(client);
    }

    void queue_json(Client& client, cJSON* json) {
        std::string frame = json_frame(json);
        if (!frame.empty()) queue_frame(client, frame);
    }

    void flush_locked(Client& client) {
        while (!client.dead && client.send_off < client.send_buf.size()) {
            ssize_t n = ::send(client.fd, client.send_buf.data() + client.send_off,
                               client.send_buf.size() - client.send_off,
                               MSG_NOSIGNAL | MSG_DONTWAIT);
            if (n > 0) {
                client.send_off += (size_t)n;
                continue;
            }
            if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR))
                break;
            client.dead = true;
        }
        if (client.send_off >= client.send_buf.size()) {
            client.send_buf.clear();
            client.send_off = 0;
        } else if (client.send_off > 65536) {
            client.send_buf.erase(0, client.send_off);
            client.send_off = 0;
        }
    }

    Client* find_client(uint64_t id) {
        for (auto& client : clients_)
            if (client.id == id) return &client;
        return nullptr;
    }

    bool process_recv_buf(Client& client,
                          std::vector<std::pair<uint64_t, std::string>>& out) {
        // Messages are: [4-byte big-endian length][JSON bytes]
        while (client.recv_buf.size() >= 4) {
            uint32_t msg_len = ((uint32_t)client.recv_buf[0] << 24) |
                               ((uint32_t)client.recv_buf[1] << 16) |
                               ((uint32_t)client.recv_buf[2] << 8) |
                               ((uint32_t)client.recv_buf[3]);

            if (msg_len > 1024 * 1024) {
                // Sanity limit: 1MB
                std::cerr << "control: Message too large (" << msg_len << "), disconnecting" << std::endl;
                return false;
            }

            if (client.recv_buf.size() < 4 + msg_len) {
                break; // Need more data
            }

            // Extract JSON
            std::string json_str(client.recv_buf.begin() + 4,
                                 client.recv_buf.begin() + 4 + msg_len);
            client.recv_buf.erase(client.recv_buf.begin(),
                                  client.recv_buf.begin() + 4 + msg_len);

            out.emplace_back(client.id, std::move(json_str));
        }
        return true;
    }

    void handle_message(uint64_t id, const std::string& json_str) {
        cJSON* request = cJSON_Parse(json_str.c_str());
        if (!request) {
            send_error(id, "invalid JSON");
            return;
        }

        cJSON* cmd = cJSON_GetObjectItemCaseSensitive(request, "cmd");
        if (!cJSON_IsString(cmd) || !cmd->valuestring) {
            send_error(id, "missing 'cmd' field");
            cJSON_Delete(request);
            return;
        }

        const char* cmd_str = cmd->valuestring;

        if (strcmp(cmd_str, "get_status") == 0) {
            handle_get_status(id);
        } else if (strcmp(cmd_str, "get_config") == 0) {
            handle_get_config(id);
        } else if (strcmp(cmd_str, "set_config") == 0) {
            handle_set_config(id, request);
        } else if (strcmp(cmd_str, "rigctl") == 0) {
            handle_rigctl(id, request);
        } else if (strcmp(cmd_str, "tx") == 0) {
            handle_tx(id, request);
        } else if (strcmp(cmd_str, "send_beacon") == 0) {
            handle_send_beacon(id);
        } else if (strcmp(cmd_str, "send_probe") == 0) {
            handle_send_probe(id);
        } else if (strcmp(cmd_str, "reconnect_audio") == 0) {
            handle_reconnect_audio(id);
        } else if (strcmp(cmd_str, "rx_frame_history") == 0) {
            handle_rx_frame_history(id, request);
        } else if (strcmp(cmd_str, "subscribe") == 0) {
            handle_subscribe(id, request, true);
        } else if (strcmp(cmd_str, "unsubscribe") == 0) {
            handle_subscribe(id, request, false);
        } else if (strcmp(cmd_str, "get_ui_state") == 0) {
            handle_get_ui_state(id);
        } else if (strcmp(cmd_str, "set_ui_settings") == 0) {
            handle_set_ui_settings(id, request);
        } else if (strcmp(cmd_str, "list_audio_devices") == 0) {
            handle_list_audio_devices(id);
        } else if (strcmp(cmd_str, "alc_tune") == 0) {
            handle_alc_tune(id);
        } else if (strcmp(cmd_str, "reset_stats") == 0) {
            handle_reset_stats(id);
        } else if (strcmp(cmd_str, "restart") == 0) {
            handle_restart(id);
        } else {
            send_error(id, "unknown command");
        }

        cJSON_Delete(request);
    }

    void handle_get_status(uint64_t id) {
        if (!iface_.get_status) {
            send_error(id, "get_status not available");
            return;
        }
        cJSON* response = iface_.get_status();
        if (response) {
            cJSON_AddBoolToObject(response, "ok", 1);
            send_json(id, response);
            cJSON_Delete(response);
        } else {
            send_error(id, "internal error");
        }
    }

    void handle_get_config(uint64_t id) {
        if (!iface_.get_config) {
            send_error(id, "get_config not available");
            return;
        }
        cJSON* response = iface_.get_config();
        if (response) {
            cJSON_AddBoolToObject(response, "ok", 1);
            send_json(id, response);
            cJSON_Delete(response);
        } else {
            send_error(id, "internal error");
        }
    }

    void handle_set_config(uint64_t id, cJSON* request) {
        if (!iface_.set_config) {
            send_error(id, "set_config not available");
            return;
        }
        if (iface_.set_config(request)) {
            cJSON* response = cJSON_CreateObject();
            cJSON_AddBoolToObject(response, "ok", 1);
            send_json(id, response);
            cJSON_Delete(response);
            notify_config_changed();
        } else {
            send_error(id, "set_config failed");
        }
    }

    void handle_rigctl(uint64_t id, cJSON* request) {
        if (!iface_.rigctl_command) {
            send_error(id, "rigctl not available");
            return;
        }

        cJSON* command = cJSON_GetObjectItemCaseSensitive(request, "command");
        if (!cJSON_IsString(command) || !command->valuestring) {
            send_error(id, "missing 'command' field");
            return;
        }

        std::string result = iface_.rigctl_command(command->valuestring);

        cJSON* response = cJSON_CreateObject();
        cJSON_AddBoolToObject(response, "ok", 1);
        cJSON_AddStringToObject(response, "response", result.c_str());
        send_json(id, response);
        cJSON_Delete(response);
    }

    void handle_send_beacon(uint64_t id) {
        if (!iface_.send_beacon) {
            send_error(id, "send_beacon not available");
            return;
        }
        cJSON* response = cJSON_CreateObject();
        cJSON_AddBoolToObject(response, "ok", iface_.send_beacon() ? 1 : 0);
        send_json(id, response);
        cJSON_Delete(response);
    }

    void handle_send_probe(uint64_t id) {
        if (!iface_.send_probe) {
            send_error(id, "send_probe not available");
            return;
        }
        cJSON* response = cJSON_CreateObject();
        cJSON_AddBoolToObject(response, "ok", iface_.send_probe() ? 1 : 0);
        send_json(id, response);
        cJSON_Delete(response);
    }

    void handle_reconnect_audio(uint64_t id) {
        if (!iface_.reconnect_audio) {
            send_error(id, "reconnect_audio not available");
            return;
        }
        cJSON* response = cJSON_CreateObject();
        cJSON_AddBoolToObject(response, "ok", iface_.reconnect_audio() ? 1 : 0);
        send_json(id, response);
        cJSON_Delete(response);
    }

    void handle_rx_frame_history(uint64_t id, cJSON* request) {
        if (!iface_.rx_frame_history) {
            send_error(id, "rx_frame_history not available");
            return;
        }
        size_t limit = 64;
        uint64_t since_seq = 0;
        cJSON* item = cJSON_GetObjectItemCaseSensitive(request, "limit");
        if (cJSON_IsNumber(item) && item->valuedouble > 0)
            limit = (size_t)item->valuedouble;
        item = cJSON_GetObjectItemCaseSensitive(request, "since_seq");
        if (cJSON_IsNumber(item) && item->valuedouble > 0)
            since_seq = (uint64_t)item->valuedouble;

        auto frames = iface_.rx_frame_history(limit, since_seq);
        int64_t now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();

        cJSON* response = cJSON_CreateObject();
        cJSON_AddBoolToObject(response, "ok", 1);
        cJSON* arr = cJSON_AddArrayToObject(response, "frames");
        for (const auto& f : frames) {
            cJSON* j = cJSON_CreateObject();
            add_rx_frame_fields(j, f);
            cJSON_AddNumberToObject(j, "age_s", (double)(now_ms - f.steady_ms) / 1000.0);
            cJSON_AddItemToArray(arr, j);
        }
        send_json(id, response);
        cJSON_Delete(response);
    }

    void handle_tx(uint64_t id, cJSON* request) {
        if (!iface_.tx_data) {
            send_error(id, "tx not available");
            return;
        }

        cJSON* data_item = cJSON_GetObjectItemCaseSensitive(request, "data");
        if (!cJSON_IsString(data_item) || !data_item->valuestring) {
            send_error(id, "missing 'data' field (base64)");
            return;
        }

        auto payload = base64_decode(data_item->valuestring, strlen(data_item->valuestring));
        if (payload.empty()) {
            send_error(id, "empty or invalid base64 data");
            return;
        }

        // Per-packet oper_mode override: -1 means use default
        int oper_mode = -1;
        cJSON* mode_item = cJSON_GetObjectItemCaseSensitive(request, "oper_mode");
        if (cJSON_IsNumber(mode_item)) {
            oper_mode = mode_item->valueint;
        }

        if (iface_.tx_data(payload, oper_mode)) {
            cJSON* response = cJSON_CreateObject();
            cJSON_AddBoolToObject(response, "ok", 1);
            cJSON_AddNumberToObject(response, "size", payload.size());
            send_json(id, response);
            cJSON_Delete(response);
        } else {
            send_error(id, "tx failed");
        }
    }

    void handle_subscribe(uint64_t id, cJSON* request, bool on) {
        cJSON* arr = cJSON_GetObjectItemCaseSensitive(request, "streams");
        if (!cJSON_IsArray(arr)) {
            send_error(id, "missing 'streams' array");
            return;
        }
        cJSON* cursors = cJSON_GetObjectItemCaseSensitive(request, "cursors");
        std::lock_guard<std::recursive_mutex> lock(clients_mutex_);
        Client* client = find_client(id);
        if (!client) return;
        cJSON* item;
        cJSON_ArrayForEach(item, arr) {
            if (!cJSON_IsString(item) || !item->valuestring) continue;
            std::string name = item->valuestring;
            if (!known_streams().count(name)) {
                send_error(id, ("unknown stream: " + name).c_str());
                return;
            }
            bool have = client->streams.count(name) > 0;
            if (on && !have) {
                uint64_t cursor = 0;
                cJSON* c = cursors ? cJSON_GetObjectItemCaseSensitive(cursors, name.c_str())
                                   : nullptr;
                if (cJSON_IsNumber(c) && c->valuedouble >= 0)
                    cursor = (uint64_t)c->valuedouble;
                else if (iface_.stream_cursor_now)
                    cursor = iface_.stream_cursor_now(name);
                client->streams.insert(name);
                client->cursors[name] = cursor;
                if (iface_.stream_refcount) iface_.stream_refcount(name, +1);
            } else if (!on && have) {
                client->streams.erase(name);
                client->cursors.erase(name);
                if (iface_.stream_refcount) iface_.stream_refcount(name, -1);
            }
        }
        cJSON* response = cJSON_CreateObject();
        cJSON_AddBoolToObject(response, "ok", 1);
        cJSON* list = cJSON_AddArrayToObject(response, "streams");
        for (const auto& name : client->streams)
            cJSON_AddItemToArray(list, cJSON_CreateString(name.c_str()));
        send_json(id, response);
        cJSON_Delete(response);
    }

    void handle_get_ui_state(uint64_t id) {
        if (!iface_.get_ui_state) {
            send_error(id, "get_ui_state not available");
            return;
        }
        cJSON* response = iface_.get_ui_state();
        if (!response) {
            send_error(id, "internal error");
            return;
        }
        cJSON_AddBoolToObject(response, "ok", 1);
        send_json(id, response);
        cJSON_Delete(response);
    }

    void handle_set_ui_settings(uint64_t id, cJSON* request) {
        if (!iface_.set_ui_settings) {
            send_error(id, "set_ui_settings not available");
            return;
        }
        cJSON* text = cJSON_GetObjectItemCaseSensitive(request, "settings");
        if (!cJSON_IsString(text) || !text->valuestring) {
            send_error(id, "missing 'settings' field");
            return;
        }
        std::string applied;
        if (!iface_.set_ui_settings(text->valuestring, &applied)) {
            send_error(id, "set_ui_settings failed");
            return;
        }
        cJSON* response = cJSON_CreateObject();
        cJSON_AddBoolToObject(response, "ok", 1);
        cJSON_AddStringToObject(response, "settings", applied.c_str());
        send_json(id, response);
        cJSON_Delete(response);
        notify_config_changed();
        notify_ui_settings(applied, id);
    }

    void handle_list_audio_devices(uint64_t id) {
        if (!iface_.list_audio_devices) {
            send_error(id, "list_audio_devices not available");
            return;
        }
        auto alive = alive_;
        iface_.list_audio_devices([this, id, alive](cJSON* response) {
            if (!alive->load()) {
                if (response) cJSON_Delete(response);
                return;
            }
            if (!response) {
                send_error(id, "internal error");
                return;
            }
            cJSON_AddBoolToObject(response, "ok", 1);
            send_json(id, response);
            cJSON_Delete(response);
        });
    }

    void handle_alc_tune(uint64_t id) {
        if (!iface_.alc_tune) {
            send_error(id, "alc_tune not available");
            return;
        }
        auto alive = alive_;
        iface_.alc_tune([this, id, alive](float drive) {
            if (!alive->load()) return;
            cJSON* response = cJSON_CreateObject();
            cJSON_AddBoolToObject(response, "ok", drive >= 0.0f ? 1 : 0);
            cJSON_AddNumberToObject(response, "tx_drive", drive);
            send_json(id, response);
            cJSON_Delete(response);
        });
    }

    void handle_reset_stats(uint64_t id) {
        if (!iface_.reset_stats) {
            send_error(id, "reset_stats not available");
            return;
        }
        iface_.reset_stats();
        cJSON* response = cJSON_CreateObject();
        cJSON_AddBoolToObject(response, "ok", 1);
        send_json(id, response);
        cJSON_Delete(response);
    }

    void handle_restart(uint64_t id) {
        if (!iface_.restart) {
            send_error(id, "restart not available");
            return;
        }
        cJSON* response = cJSON_CreateObject();
        cJSON_AddBoolToObject(response, "ok", 1);
        send_json(id, response);
        cJSON_Delete(response);
        iface_.restart();
    }

    void send_json(uint64_t id, cJSON* json) {
        std::lock_guard<std::recursive_mutex> lock(clients_mutex_);
        Client* client = find_client(id);
        if (!client) return;
        queue_json(*client, json);
    }

    void send_error(uint64_t id, const char* error) {
        cJSON* response = cJSON_CreateObject();
        cJSON_AddBoolToObject(response, "ok", 0);
        cJSON_AddStringToObject(response, "error", error);
        send_json(id, response);
        cJSON_Delete(response);
    }

    int port_;
    std::string bind_address_;
    TNCInterface iface_;
    int server_fd_ = -1;
    std::atomic<bool> running_{false};
    std::thread thread_;
    std::recursive_mutex clients_mutex_;
    std::list<Client> clients_;
    uint64_t next_client_id_ = 1;
    std::shared_ptr<std::atomic<bool>> alive_;
};
