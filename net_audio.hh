#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <functional>
#include <string>
#include <thread>
#include <vector>

#include <sys/socket.h>
#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0
#endif
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
#include <poll.h>
#include <fcntl.h>
#include <errno.h>

class NetAudioLink {
public:
    static constexpr const char* PREFIX = "tcp:";
    static constexpr int POLL_MS = 20;
    static constexpr int CONNECT_TIMEOUT_MS = 3000;
    static constexpr int SEND_TIMEOUT_S = 2;

    using CaptureSink = std::function<void(const float*, int)>;
    using PlaybackSource = std::function<int(float*, int)>;
    using Logger = std::function<void(const std::string&)>;

    static bool is_net_device(const std::string& name) {
        return name.rfind(PREFIX, 0) == 0;
    }

    ~NetAudioLink() { close(); }

    std::string open(const std::string& spec, int sample_rate, CaptureSink capture,
                     PlaybackSource playback, Logger log) {
        close();
        std::string addr = spec.substr(strlen(PREFIX));
        size_t colon = addr.rfind(':');
        if (colon == std::string::npos || colon == 0 || colon + 1 >= addr.size())
            return "expected tcp:HOST:PORT, got '" + spec + "'";
        stalled_ = false;
        rx_frames_ = 0;
        tx_frames_ = 0;
        host_ = addr.substr(0, colon);
        port_ = std::atoi(addr.c_str() + colon + 1);
        if (port_ <= 0 || port_ > 65535)
            return "bad port in '" + spec + "'";
        rate_ = sample_rate;
        capture_ = std::move(capture);
        playback_ = std::move(playback);
        log_ = std::move(log);

        struct addrinfo hints;
        memset(&hints, 0, sizeof(hints));
        hints.ai_family = AF_UNSPEC;
        hints.ai_socktype = SOCK_STREAM;
        struct addrinfo* res = nullptr;
        if (getaddrinfo(host_.c_str(), std::to_string(port_).c_str(), &hints, &res) != 0 || !res)
            return "cannot resolve " + host_;
        int fd = -1;
        for (struct addrinfo* ai = res; ai && fd < 0; ai = ai->ai_next) {
            int s = socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
            if (s < 0) continue;
            fcntl(s, F_SETFL, O_NONBLOCK);
            int r = ::connect(s, ai->ai_addr, ai->ai_addrlen);
            if (r < 0 && errno == EINPROGRESS) {
                struct pollfd p = {s, POLLOUT, 0};
                int err = 0;
                socklen_t len = sizeof(err);
                if (poll(&p, 1, CONNECT_TIMEOUT_MS) > 0 &&
                    getsockopt(s, SOL_SOCKET, SO_ERROR, &err, &len) == 0 && err == 0)
                    r = 0;
            }
            if (r == 0) {
                fd = s;
            } else {
                ::close(s);
            }
        }
        freeaddrinfo(res);
        if (fd < 0)
            return "nothing listening on " + host_ + ":" + std::to_string(port_);
        int flags = fcntl(fd, F_GETFL, 0);
        fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);
        int one = 1;
        setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
        struct timeval tv = {SEND_TIMEOUT_S, 0};
        setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

        fd_ = fd;
        stop_ = false;
        credit_ = 0;
        alive_ = true;
        thread_ = std::thread(&NetAudioLink::pump, this);
        return std::string();
    }

    void close() {
        stop_ = true;
        if (fd_ >= 0) shutdown(fd_, SHUT_RDWR);
        if (thread_.joinable()) thread_.join();
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
        alive_ = false;
    }

    bool alive() const { return alive_.load(); }
    bool stalled() const { return stalled_.load(); }
    uint64_t rx_frames() const { return rx_frames_.load(); }
    uint64_t tx_frames() const { return tx_frames_.load(); }

    std::string describe() const { return host_ + ":" + std::to_string(port_); }

private:
    static int64_t now_ms() {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    }

    bool send_all(const uint8_t* p, size_t n) {
        size_t off = 0;
        while (off < n) {
            ssize_t r = ::send(fd_, p + off, n - off, MSG_NOSIGNAL);
            if (r > 0) {
                off += (size_t)r;
                continue;
            }
            if (r < 0 && errno == EINTR) continue;
            return false;
        }
        return true;
    }

    bool send_playback(int frames) {
        std::vector<float> out(frames);
        int real = playback_(out.data(), frames);
        std::vector<uint8_t> bytes(frames * 2);
        for (int i = 0; i < frames; i++) {
            float v = out[i];
            if (v > 1.0f) v = 1.0f;
            if (v < -1.0f) v = -1.0f;
            int16_t q = (int16_t)(v * 32767.0f);
            bytes[2 * i] = (uint8_t)(q & 0xFF);
            bytes[2 * i + 1] = (uint8_t)((q >> 8) & 0xFF);
        }
        (void)real;
        if (!send_all(bytes.data(), bytes.size())) return false;
        tx_frames_ += (uint64_t)frames;
        return true;
    }

    bool playback_pending() {
        float probe[1];
        return playback_(probe, 0) > 0;
    }

    void pump() {
        std::vector<uint8_t> buf(8192);
        std::vector<float> in;
        uint8_t carry = 0;
        bool have_carry = false;
        int64_t last_fallback = now_ms();
        bool stalled = false;
        while (!stop_) {
            struct pollfd p = {fd_, POLLIN, 0};
            int r = poll(&p, 1, POLL_MS);
            if (r < 0) {
                if (errno == EINTR) continue;
                break;
            }
            if (r == 0) {
                int64_t now = now_ms();
                int frames = (int)((now - last_fallback) * rate_ / 1000);
                last_fallback = now;
                if (frames > 0 && playback_pending()) {
                    if (!stalled && log_) log_("Network audio: no capture data, pacing transmit locally");
                    stalled = true;
                    stalled_ = true;
                    if (!send_playback(frames)) break;
                    credit_ += frames;
                }
                continue;
            }
            ssize_t n = recv(fd_, buf.data(), buf.size(), 0);
            if (n <= 0) {
                if (n < 0 && errno == EINTR) continue;
                break;
            }
            last_fallback = now_ms();
            stalled = false;
            stalled_ = false;
            size_t start = 0;
            in.clear();
            if (have_carry) {
                int16_t v = (int16_t)((uint16_t)carry | ((uint16_t)buf[0] << 8));
                in.push_back(v / 32768.0f);
                start = 1;
                have_carry = false;
            }
            size_t pairs = (n - start) / 2;
            for (size_t i = 0; i < pairs; i++) {
                int16_t v = (int16_t)((uint16_t)buf[start + 2 * i] | ((uint16_t)buf[start + 2 * i + 1] << 8));
                in.push_back(v / 32768.0f);
            }
            if ((n - start) % 2) {
                carry = buf[n - 1];
                have_carry = true;
            }
            int frames = (int)in.size();
            if (frames == 0) continue;
            capture_(in.data(), frames);
            rx_frames_ += (uint64_t)frames;
            int reply = frames;
            if (credit_ > 0) {
                int64_t use = credit_ < reply ? credit_ : reply;
                reply -= (int)use;
                credit_ -= use;
            }
            if (reply > 0 && !send_playback(reply)) break;
        }
        alive_ = false;
        if (!stop_ && log_) log_("(!) Network audio: connection to " + describe() + " lost");
    }

    int fd_ = -1;
    std::thread thread_;
    std::atomic<bool> alive_{false};
    std::atomic<bool> stop_{false};
    std::atomic<bool> stalled_{false};
    std::atomic<uint64_t> rx_frames_{0};
    std::atomic<uint64_t> tx_frames_{0};
    std::string host_;
    int port_ = 0;
    int rate_ = 48000;
    int64_t credit_ = 0;
    CaptureSink capture_;
    PlaybackSource playback_;
    Logger log_;
};
