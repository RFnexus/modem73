#pragma once

#include <string>
#include <thread>
#include <atomic>
#include <functional>
#include <vector>
#include <list>
#include <mutex>
#include <cstring>
#include <iostream>

#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>

extern "C" {
#include "deps/cJSON.h"
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

class ControlPort {
public:
    struct TNCInterface {
        std::function<cJSON*()> get_status;
        std::function<cJSON*()> get_config;
        std::function<bool(cJSON* params)> set_config;
        std::function<std::string(const std::string&)> rigctl_command;
        std::function<bool(const std::vector<uint8_t>&, int oper_mode)> tx_data;
    };

    ControlPort(int port, const std::string& bind_address, TNCInterface iface)
        : port_(port), bind_address_(bind_address), iface_(std::move(iface)) {}

    ~ControlPort() {
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
        addr.sin_addr.s_addr = inet_addr(bind_address_.c_str());
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

    // Push an event to all connected clients (thread-safe)
    void broadcast_event(cJSON* event) {
        char* str = cJSON_PrintUnformatted(event);
        if (!str) return;

        uint32_t len = strlen(str);
        uint8_t header[4];
        header[0] = (len >> 24) & 0xFF;
        header[1] = (len >> 16) & 0xFF;
        header[2] = (len >> 8) & 0xFF;
        header[3] = len & 0xFF;

        std::lock_guard<std::recursive_mutex> lock(clients_mutex_);
        for (auto& client : clients_) {
            ::send(client.fd, header, 4, MSG_NOSIGNAL);
            ::send(client.fd, str, len, MSG_NOSIGNAL);
        }

        cJSON_free(str);
    }

private:
    struct Client {
        int fd;
        std::vector<uint8_t> recv_buf;

        Client(int fd) : fd(fd) {}
    };

    void run() {
        while (running_) {
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
                    clients_.emplace_back(client_fd);
                }
            }

            // Poll clients
            {
                std::lock_guard<std::recursive_mutex> lock(clients_mutex_);
                for (auto it = clients_.begin(); it != clients_.end();) {
                    uint8_t buf[4096];
                    ssize_t n = recv(it->fd, buf, sizeof(buf), MSG_DONTWAIT);

                    if (n > 0) {
                        it->recv_buf.insert(it->recv_buf.end(), buf, buf + n);
                        process_recv_buf(*it);
                    } else if (n == 0 || (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK)) {
                        std::cerr << "control: Client disconnected" << std::endl;
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
                close(client.fd);
            }
            clients_.clear();
        }
    }

    void process_recv_buf(Client& client) {
        // Messages are: [4-byte big-endian length][JSON bytes]
        while (client.recv_buf.size() >= 4) {
            uint32_t msg_len = ((uint32_t)client.recv_buf[0] << 24) |
                               ((uint32_t)client.recv_buf[1] << 16) |
                               ((uint32_t)client.recv_buf[2] << 8) |
                               ((uint32_t)client.recv_buf[3]);

            if (msg_len > 1024 * 1024) {
                // Sanity limit: 1MB
                std::cerr << "control: Message too large (" << msg_len << "), disconnecting" << std::endl;
                client.recv_buf.clear();
                return;
            }

            if (client.recv_buf.size() < 4 + msg_len) {
                break; // Need more data
            }

            // Extract JSON
            std::string json_str(client.recv_buf.begin() + 4,
                                 client.recv_buf.begin() + 4 + msg_len);
            client.recv_buf.erase(client.recv_buf.begin(),
                                  client.recv_buf.begin() + 4 + msg_len);

            handle_message(client.fd, json_str);
        }
    }

    void handle_message(int client_fd, const std::string& json_str) {
        cJSON* request = cJSON_Parse(json_str.c_str());
        if (!request) {
            send_error(client_fd, "invalid JSON");
            return;
        }

        cJSON* cmd = cJSON_GetObjectItemCaseSensitive(request, "cmd");
        if (!cJSON_IsString(cmd) || !cmd->valuestring) {
            send_error(client_fd, "missing 'cmd' field");
            cJSON_Delete(request);
            return;
        }

        const char* cmd_str = cmd->valuestring;

        if (strcmp(cmd_str, "get_status") == 0) {
            handle_get_status(client_fd);
        } else if (strcmp(cmd_str, "get_config") == 0) {
            handle_get_config(client_fd);
        } else if (strcmp(cmd_str, "set_config") == 0) {
            handle_set_config(client_fd, request);
        } else if (strcmp(cmd_str, "rigctl") == 0) {
            handle_rigctl(client_fd, request);
        } else if (strcmp(cmd_str, "tx") == 0) {
            handle_tx(client_fd, request);
        } else {
            send_error(client_fd, "unknown command");
        }

        cJSON_Delete(request);
    }

    void handle_get_status(int client_fd) {
        if (!iface_.get_status) {
            send_error(client_fd, "get_status not available");
            return;
        }
        cJSON* response = iface_.get_status();
        if (response) {
            cJSON_AddBoolToObject(response, "ok", 1);
            send_json(client_fd, response);
            cJSON_Delete(response);
        } else {
            send_error(client_fd, "internal error");
        }
    }

    void handle_get_config(int client_fd) {
        if (!iface_.get_config) {
            send_error(client_fd, "get_config not available");
            return;
        }
        cJSON* response = iface_.get_config();
        if (response) {
            cJSON_AddBoolToObject(response, "ok", 1);
            send_json(client_fd, response);
            cJSON_Delete(response);
        } else {
            send_error(client_fd, "internal error");
        }
    }

    void handle_set_config(int client_fd, cJSON* request) {
        if (!iface_.set_config) {
            send_error(client_fd, "set_config not available");
            return;
        }
        if (iface_.set_config(request)) {
            cJSON* response = cJSON_CreateObject();
            cJSON_AddBoolToObject(response, "ok", 1);
            send_json(client_fd, response);
            cJSON_Delete(response);
            notify_config_changed();
        } else {
            send_error(client_fd, "set_config failed");
        }
    }

    void handle_rigctl(int client_fd, cJSON* request) {
        if (!iface_.rigctl_command) {
            send_error(client_fd, "rigctl not available");
            return;
        }

        cJSON* command = cJSON_GetObjectItemCaseSensitive(request, "command");
        if (!cJSON_IsString(command) || !command->valuestring) {
            send_error(client_fd, "missing 'command' field");
            return;
        }

        std::string result = iface_.rigctl_command(command->valuestring);

        cJSON* response = cJSON_CreateObject();
        cJSON_AddBoolToObject(response, "ok", 1);
        cJSON_AddStringToObject(response, "response", result.c_str());
        send_json(client_fd, response);
        cJSON_Delete(response);
    }

    void handle_tx(int client_fd, cJSON* request) {
        if (!iface_.tx_data) {
            send_error(client_fd, "tx not available");
            return;
        }

        cJSON* data_item = cJSON_GetObjectItemCaseSensitive(request, "data");
        if (!cJSON_IsString(data_item) || !data_item->valuestring) {
            send_error(client_fd, "missing 'data' field (base64)");
            return;
        }

        auto payload = base64_decode(data_item->valuestring, strlen(data_item->valuestring));
        if (payload.empty()) {
            send_error(client_fd, "empty or invalid base64 data");
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
            send_json(client_fd, response);
            cJSON_Delete(response);
        } else {
            send_error(client_fd, "tx failed");
        }
    }

    void send_json(int client_fd, cJSON* json) {
        char* str = cJSON_PrintUnformatted(json);
        if (!str) return;

        uint32_t len = strlen(str);
        uint8_t header[4];
        header[0] = (len >> 24) & 0xFF;
        header[1] = (len >> 16) & 0xFF;
        header[2] = (len >> 8) & 0xFF;
        header[3] = len & 0xFF;

        // Send header + body (best effort, non-blocking)
        ::send(client_fd, header, 4, MSG_NOSIGNAL);
        ::send(client_fd, str, len, MSG_NOSIGNAL);

        cJSON_free(str);
    }

    void send_error(int client_fd, const char* error) {
        cJSON* response = cJSON_CreateObject();
        cJSON_AddBoolToObject(response, "ok", 0);
        cJSON_AddStringToObject(response, "error", error);
        send_json(client_fd, response);
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
};
