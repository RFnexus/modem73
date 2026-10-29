#pragma once

// Network audio: raw PCM over a TCP socket instead of a sound card.
//
// Select it by giving the same "tcp:HOST:PORT" as both --input-device and
// --output-device. modem73 connects to HOST:PORT. The peer is the audio clock:
// it streams mono signed 16-bit little-endian capture samples at the modem's
// sample rate, and for every block it sends, modem73 answers with the same
// number of playback samples (silence while idle). This lets another program
// carry the modem's audio over a link that is not a sound card, such as a
// radio's Bluetooth audio channel.

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <functional>
#include <string>
#include <thread>
#include <vector>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#undef IN
#undef OUT
#else
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

class NetAudioLink {
public:
    using CaptureSink = std::function<void(const float*, size_t)>;
    using PlaybackSource = std::function<void(float*, size_t)>;

    static constexpr const char* PREFIX = "tcp:";

    static bool is_net_device(const std::string& device) {
        return device.rfind(PREFIX, 0) == 0;
    }

    ~NetAudioLink() { close(); }

    // Connect and start the pump thread. On failure returns false and sets err.
    bool open(const std::string& device, CaptureSink capture, PlaybackSource playback,
              std::string* err) {
        close();
        std::string spec = device.substr(std::string(PREFIX).size());
        size_t colon = spec.rfind(':');
        if (colon == std::string::npos || colon == 0 || colon + 1 == spec.size()) {
            if (err) *err = "expected tcp:HOST:PORT";
            return false;
        }
        std::string host = spec.substr(0, colon);
        std::string port = spec.substr(colon + 1);

        addrinfo hints{};
        hints.ai_family = AF_UNSPEC;
        hints.ai_socktype = SOCK_STREAM;
        addrinfo* res = nullptr;
        if (getaddrinfo(host.c_str(), port.c_str(), &hints, &res) != 0 || !res) {
            if (err) *err = "cannot resolve " + host;
            return false;
        }
        Socket s = INVALID;
        for (addrinfo* a = res; a; a = a->ai_next) {
            s = ::socket(a->ai_family, a->ai_socktype, a->ai_protocol);
            if (s == INVALID) continue;
            if (::connect(s, a->ai_addr, (int)a->ai_addrlen) == 0) break;
            close_socket(s);
            s = INVALID;
        }
        freeaddrinfo(res);
        if (s == INVALID) {
            if (err) *err = "nothing listening on " + host + ":" + port;
            return false;
        }
        int one = 1;
        setsockopt(s, IPPROTO_TCP, TCP_NODELAY, (const char*)&one, sizeof(one));

        sock_ = s;
        capture_ = std::move(capture);
        playback_ = std::move(playback);
        stop_.store(false);
        alive_.store(true);
        thread_ = std::thread([this] { pump(); });
        return true;
    }

    void close() {
        stop_.store(true);
        if (sock_ != INVALID) shutdown_socket(sock_);
        if (thread_.joinable()) thread_.join();
        if (sock_ != INVALID) close_socket(sock_);
        sock_ = INVALID;
        alive_.store(false);
    }

    // True while the peer is connected and the pump is running.
    bool alive() const { return alive_.load(); }

private:
#ifdef _WIN32
    using Socket = SOCKET;
    static constexpr Socket INVALID = INVALID_SOCKET;
    static void close_socket(Socket s) { closesocket(s); }
    static void shutdown_socket(Socket s) { ::shutdown(s, SD_BOTH); }
#else
    using Socket = int;
    static constexpr Socket INVALID = -1;
    static void close_socket(Socket s) { ::close(s); }
    static void shutdown_socket(Socket s) { ::shutdown(s, SHUT_RDWR); }
#endif

    bool send_all(const char* data, size_t len) {
#ifdef MSG_NOSIGNAL
        const int flags = MSG_NOSIGNAL;
#else
        const int flags = 0;
#endif
        while (len > 0) {
            int n = ::send(sock_, data, (int)len, flags);
            if (n <= 0) return false;
            data += n;
            len -= (size_t)n;
        }
        return true;
    }

    void pump() {
        std::vector<char> bytes(8192);
        size_t have = 0;
        std::vector<float> in;
        std::vector<float> out;
        std::vector<char> wire;
        while (!stop_.load()) {
            int n = ::recv(sock_, bytes.data() + have, (int)(bytes.size() - have), 0);
            if (n <= 0) break;
            have += (size_t)n;
            size_t frames = have / 2;
            if (frames == 0) continue;

            in.resize(frames);
            for (size_t i = 0; i < frames; i++) {
                int16_t v = (int16_t)((uint8_t)bytes[2 * i] | ((uint8_t)bytes[2 * i + 1] << 8));
                in[i] = v / 32768.0f;
            }
            if (have % 2) bytes[0] = bytes[have - 1];
            have %= 2;

            capture_(in.data(), frames);
            out.assign(frames, 0.0f);
            playback_(out.data(), frames);

            wire.resize(frames * 2);
            for (size_t i = 0; i < frames; i++) {
                float v = std::max(-1.0f, std::min(1.0f, out[i]));
                uint16_t u = (uint16_t)(int16_t)(v * 32767.0f);
                wire[2 * i] = (char)(u & 0xff);
                wire[2 * i + 1] = (char)(u >> 8);
            }
            if (!send_all(wire.data(), wire.size())) break;
        }
        alive_.store(false);
    }

    Socket sock_ = INVALID;
    std::thread thread_;
    std::atomic<bool> stop_{false};
    std::atomic<bool> alive_{false};
    CaptureSink capture_;
    PlaybackSource playback_;
};
