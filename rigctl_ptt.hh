#pragma once

#include <string>
#include <iostream>
#include <cstring>
#include <mutex>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <netdb.h>
#include <poll.h>

class RigctlPTT {
public:
    RigctlPTT(const std::string& host = "localhost", int port = 4532)
        : host_(host), port_(port) {}
    
    ~RigctlPTT() {
        disconnect();
    }
    
    bool connect() {
        std::lock_guard<std::mutex> lock(mutex_);
        return connect_locked();
    }

    void disconnect() {
        std::lock_guard<std::mutex> lock(mutex_);
        disconnect_locked();
    }


    
    bool set_ptt(bool on) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!connected_) {
            if (!connect_locked()) return false;
        }

        // T 1 (PTT on) or T 0 (PTT off)
        std::string cmd = on ? "T 1\n" : "T 0\n";

        if (send(sock_, cmd.c_str(), cmd.length(), 0) < 0) {
            std::cerr << "rigctl: Failed to send PTT command" << std::endl;
            disconnect_locked();
            return false;
        }

        // read response
        char response[256];
        int n = recv(sock_, response, sizeof(response) - 1, 0);
        if (n > 0) {
            response[n] = '\0';
            // rigctld returns RPRT 0 on success
            if (strstr(response, "RPRT 0") || n == 0) {
                ptt_on_ = on;
                std::cerr << "rigctl: PTT " << (on ? "ON" : "OFF") << std::endl;
                return true;
            } else {
                std::cerr << "rigctl: PTT command failed: " << response << std::endl;
                return false;
            }
        }
        // temp fallback
        ptt_on_ = on;
        return true;
    }

    // Send an arbitrary rigctld command and return the response
    std::string send_command(const std::string& cmd) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!connected_) {
            if (!connect_locked()) return "ERR: not connected";
        }

        std::string wire = cmd + "\n";
        if (::send(sock_, wire.c_str(), wire.length(), 0) < 0) {
            disconnect_locked();
            return "ERR: send failed";
        }

        // Read response with timeout
        std::string result;
        char buf[1024];
        struct pollfd pfd = {sock_, POLLIN, 0};

        while (true) {
            int ready = poll(&pfd, 1, 500);
            if (ready <= 0) break;
            int n = recv(sock_, buf, sizeof(buf) - 1, 0);
            if (n <= 0) break;
            buf[n] = '\0';
            result += buf;
            // If we got RPRT, that's the end of the response
            if (result.find("RPRT") != std::string::npos) break;
        }

        return result;
    }

    
    bool ptt_on() const { return ptt_on_; }
    bool is_connected() const { return connected_; }


private:
    bool connect_locked() {
        if (connected_) return true;

        sock_ = socket(AF_INET, SOCK_STREAM, 0);
        if (sock_ < 0) {
            std::cerr << "rigctl: Failed to create socket" << std::endl;
            return false;
        }

        struct hostent* server = gethostbyname(host_.c_str());
        if (!server) {
            std::cerr << "rigctl PTT: Can't connect to host " << host_ << std::endl;
            close(sock_);
            sock_ = -1;
            return false;
        }

        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        memcpy(&addr.sin_addr.s_addr, server->h_addr, server->h_length);
        addr.sin_port = htons(port_);

        if (::connect(sock_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            std::cerr << "rigctl: Can't connect to " << host_ << ":" << port_ << std::endl;
            close(sock_);
            sock_ = -1;
            return false;
        }

        connected_ = true;
        std::cerr << "rigctl: Connected to " << host_ << ":" << port_ << std::endl;
        return true;
    }

    void disconnect_locked() {
        if (sock_ >= 0) {
            close(sock_);
            sock_ = -1;
        }
        connected_ = false;
        ptt_on_ = false;
    }

    std::string host_;
    int port_;
    int sock_ = -1;
    bool connected_ = false;
    bool ptt_on_ = false;
    std::mutex mutex_;
};



class DummyPTT {
public:
    bool connect() { 
        std::cerr << "PTT: Using dummy PTT (no rigctld)" << std::endl;
        return true; 
    }
    void disconnect() {}
    bool set_ptt(bool on) { 
        ptt_on_ = on;
        std::cerr << "PTT: " << (on ? "ON" : "OFF") << " (dummy)" << std::endl;
        return true; 
    }
    bool ptt_on() const { return ptt_on_; }
    bool is_connected() const { return true; }
    
private:
    bool ptt_on_ = false;
};
