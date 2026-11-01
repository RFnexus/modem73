#pragma once

#include <string>
#include <vector>
#include <thread>
#include <memory>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <chrono>
#include <complex>
#include <cstring>
#include <cstdint>
#include <cmath>

#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
#include <poll.h>
#include <fcntl.h>
#include <errno.h>

#include "control_port.hh"
#include "tnc_ui_state.hh"

class AttachClient {
public:
    AttachClient(TNCUIState& state, const std::string& host, int port)
        : state_(state), host_(host), port_(port),
          target_(host + ":" + std::to_string(port)) {}

    ~AttachClient() { stop(); }

    bool start() {
        if (!connect_socket()) return false;
        running_ = true;
        reader_ = std::thread(&AttachClient::reader_loop, this);
        if (!sync_from_server()) {
            stop();
            return false;
        }
        bind_callbacks();
        ticker_ = std::thread(&AttachClient::ticker_loop, this);
        return true;
    }

    void stop() {
        running_ = false;
        if (reader_.joinable()) reader_.join();
        if (ticker_.joinable()) ticker_.join();
        close_socket();
    }

    bool connected() const { return fd_.load() >= 0; }
    const std::string& last_error() const { return last_error_; }

private:
    static int64_t now_ms() {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    }

    static double num(cJSON* j, const char* key, double def) {
        cJSON* it = cJSON_GetObjectItemCaseSensitive(j, key);
        return cJSON_IsNumber(it) ? it->valuedouble : def;
    }

    static bool flag(cJSON* j, const char* key, bool def) {
        cJSON* it = cJSON_GetObjectItemCaseSensitive(j, key);
        return cJSON_IsBool(it) ? cJSON_IsTrue(it) : def;
    }

    static std::string str(cJSON* j, const char* key) {
        cJSON* it = cJSON_GetObjectItemCaseSensitive(j, key);
        return cJSON_IsString(it) && it->valuestring ? it->valuestring : "";
    }

    static bool ok(cJSON* j) { return j && flag(j, "ok", false); }

    bool connect_socket() {
        struct addrinfo hints;
        memset(&hints, 0, sizeof(hints));
        hints.ai_family = AF_INET;
        hints.ai_socktype = SOCK_STREAM;
        struct addrinfo* res = nullptr;
        if (getaddrinfo(host_.c_str(), std::to_string(port_).c_str(), &hints, &res) != 0 || !res)
            return false;
        int fd = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
        if (fd < 0) {
            freeaddrinfo(res);
            return false;
        }
        fcntl(fd, F_SETFL, O_NONBLOCK);
        int r = ::connect(fd, res->ai_addr, res->ai_addrlen);
        freeaddrinfo(res);
        if (r < 0 && errno != EINPROGRESS) {
            close(fd);
            return false;
        }
        if (r < 0) {
            struct pollfd p = {fd, POLLOUT, 0};
            if (poll(&p, 1, 3000) <= 0) {
                close(fd);
                return false;
            }
            int err = 0;
            socklen_t len = sizeof(err);
            if (getsockopt(fd, SOL_SOCKET, SO_ERROR, &err, &len) < 0 || err != 0) {
                close(fd);
                return false;
            }
        }
        int flags = fcntl(fd, F_GETFL, 0);
        fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);
        int one = 1;
        setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
        struct timeval tv = {5, 0};
        setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
        fd_ = fd;
        state_.attach_connected = true;
        return true;
    }

    void close_socket() {
        int fd = fd_.exchange(-1);
        if (fd >= 0) close(fd);
        state_.attach_connected = false;
        scope_subscribed_ = false;
        {
            std::lock_guard<std::mutex> lk(resp_mutex_);
            if (resp_) {
                cJSON_Delete(resp_);
                resp_ = nullptr;
            }
            resp_gen_++;
        }
        resp_cv_.notify_all();
    }

    void on_disconnect() {
        if (fd_.load() < 0) return;
        close_socket();
        if (!lost_logged_.exchange(true))
            state_.add_log("(!) attach: connection to " + target_ + " lost, retrying");
    }

    bool send_frame(const std::string& frame) {
        std::lock_guard<std::mutex> lk(send_mutex_);
        int fd = fd_.load();
        if (fd < 0) return false;
        size_t off = 0;
        while (off < frame.size()) {
            ssize_t n = ::send(fd, frame.data() + off, frame.size() - off, MSG_NOSIGNAL);
            if (n > 0) {
                off += (size_t)n;
                continue;
            }
            if (n < 0 && errno == EINTR) continue;
            return false;
        }
        return true;
    }

    cJSON* request(cJSON* req, int timeout_ms = 5000) {
        std::lock_guard<std::mutex> call(call_mutex_);
        std::string frame = json_frame(req);
        cJSON_Delete(req);
        if (frame.empty() || fd_.load() < 0) return nullptr;
        uint64_t gen;
        {
            std::lock_guard<std::mutex> lk(resp_mutex_);
            if (resp_) {
                cJSON_Delete(resp_);
                resp_ = nullptr;
            }
            waiting_ = true;
            gen = resp_gen_;
        }
        if (!send_frame(frame)) {
            {
                std::lock_guard<std::mutex> lk(resp_mutex_);
                waiting_ = false;
            }
            on_disconnect();
            return nullptr;
        }
        std::unique_lock<std::mutex> lk(resp_mutex_);
        bool got = resp_cv_.wait_for(lk, std::chrono::milliseconds(timeout_ms),
                                     [&] { return resp_ != nullptr || resp_gen_ != gen; });
        waiting_ = false;
        cJSON* r = resp_;
        resp_ = nullptr;
        lk.unlock();
        if (r) return r;
        if (!got) on_disconnect();
        return nullptr;
    }

    cJSON* simple_request(const char* cmd, int timeout_ms = 5000) {
        cJSON* req = cJSON_CreateObject();
        cJSON_AddStringToObject(req, "cmd", cmd);
        return request(req, timeout_ms);
    }

    bool set_stream(const char* name, bool on) {
        cJSON* req = cJSON_CreateObject();
        cJSON_AddStringToObject(req, "cmd", on ? "subscribe" : "unsubscribe");
        cJSON* arr = cJSON_AddArrayToObject(req, "streams");
        cJSON_AddItemToArray(arr, cJSON_CreateString(name));
        cJSON* r = request(req);
        bool good = ok(r);
        if (r) cJSON_Delete(r);
        return good;
    }

    void reader_loop() {
        std::vector<uint8_t> buf(65536);
        std::vector<uint8_t> acc;
        int last_fd = -1;
        while (running_) {
            int fd = fd_.load();
            if (fd < 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }
            if (fd != last_fd) {
                acc.clear();
                last_fd = fd;
            }
            struct pollfd p = {fd, POLLIN, 0};
            int r = poll(&p, 1, 200);
            if (r <= 0 || fd != fd_.load()) continue;
            ssize_t n = recv(fd, buf.data(), buf.size(), 0);
            if (n <= 0) {
                if (n < 0 && (errno == EINTR || errno == EAGAIN)) continue;
                on_disconnect();
                continue;
            }
            acc.insert(acc.end(), buf.begin(), buf.begin() + n);
            while (acc.size() >= 4) {
                uint32_t len = ((uint32_t)acc[0] << 24) | ((uint32_t)acc[1] << 16) |
                               ((uint32_t)acc[2] << 8) | (uint32_t)acc[3];
                if (len > 16u * 1024 * 1024) {
                    on_disconnect();
                    break;
                }
                if (acc.size() < 4 + len) break;
                std::string js(acc.begin() + 4, acc.begin() + 4 + len);
                acc.erase(acc.begin(), acc.begin() + 4 + len);
                dispatch(js);
            }
        }
    }

    void dispatch(const std::string& js) {
        cJSON* j = cJSON_Parse(js.c_str());
        if (!j) return;
        cJSON* ev = cJSON_GetObjectItemCaseSensitive(j, "event");
        if (cJSON_IsString(ev) && ev->valuestring) {
            handle_event(ev->valuestring, j);
            cJSON_Delete(j);
            return;
        }
        std::lock_guard<std::mutex> lk(resp_mutex_);
        if (waiting_ && !resp_) {
            resp_ = j;
            resp_cv_.notify_all();
        } else {
            cJSON_Delete(j);
        }
    }

    void handle_event(const char* name, cJSON* j) {
        if (!strcmp(name, "status")) {
            apply_status(j);
        } else if (!strcmp(name, "log")) {
            cJSON* arr = cJSON_GetObjectItemCaseSensitive(j, "lines");
            cJSON* it;
            cJSON_ArrayForEach(it, arr) {
                if (cJSON_IsString(it) && it->valuestring)
                    state_.add_log_raw(it->valuestring);
            }
        } else if (!strcmp(name, "packet")) {
            cJSON* arr = cJSON_GetObjectItemCaseSensitive(j, "packets");
            cJSON* it;
            cJSON_ArrayForEach(it, arr) apply_packet(it);
        } else if (!strcmp(name, "message")) {
            cJSON* arr = cJSON_GetObjectItemCaseSensitive(j, "messages");
            cJSON* it;
            cJSON_ArrayForEach(it, arr) apply_message(it);
        } else if (!strcmp(name, "constellation")) {
            apply_constellation(j);
        } else if (!strcmp(name, "scope")) {
            apply_scope(j);
        } else if (!strcmp(name, "ui_settings")) {
            std::string text = str(j, "settings");
            if (!text.empty())
                state_.post_ui_task([text](TNCUIState& s) {
                    s.parse_settings_text(text);
                    s.update_modem_info();
                });
        }
    }

    void apply_status(cJSON* j) {
        state_.ptt_on = flag(j, "ptt_on", false);
        state_.transmitting = flag(j, "transmitting", false);
        state_.receiving = flag(j, "receiving", false);
        state_.dcd_active = flag(j, "dcd_active", false);
        state_.decoding_active = flag(j, "decoding_active", false);
        state_.client_count = (int)num(j, "client_count", 0);
        state_.tx_queue_size = (int)num(j, "tx_queue_size", 0);
        state_.rx_frame_count = (int)num(j, "rx_frame_count", 0);
        state_.tx_frame_count = (int)num(j, "tx_frame_count", 0);
        state_.rx_error_count = (int)num(j, "rx_error_count", 0);
        state_.last_rx_snr = (float)num(j, "last_rx_snr", 0);
        state_.last_rx_ber = (float)num(j, "last_rx_ber", -1);
        state_.sync_count = (int)num(j, "sync_count", 0);
        state_.preamble_errors = (int)num(j, "preamble_errors", 0);
        state_.symbol_errors = (int)num(j, "symbol_errors", 0);
        state_.erased_symbols = (int)num(j, "erased_symbols", 0);
        state_.crc_errors = (int)num(j, "crc_errors", 0);
        state_.total_tx_time = (float)num(j, "total_tx_time", 0);
        state_.channel_occupancy = (float)num(j, "channel_occupancy", 0);
        state_.csma_phase = (int)num(j, "csma_phase", 0);
        state_.csma_wait_ms = (int)num(j, "csma_wait_ms", 0);
        state_.csma_wait_need = (int)num(j, "csma_wait_need", 0);
        state_.csma_rank = (int)num(j, "csma_rank", -1);
        state_.csma_rank_n = (int)num(j, "csma_rank_n", 0);
        state_.csma_window_ms = (int)num(j, "csma_window_ms", 0);
        double sig_age = num(j, "wf_sig_age_ms", -1);
        state_.wf_sig_ms = sig_age >= 0 ? now_ms() - (int64_t)sig_age : 0;
        state_.remote_audio_level_db = (float)num(j, "audio_level_db", -100);
        state_.rigctl_connected = flag(j, "rigctl_connected", false);
        state_.audio_connected = flag(j, "audio_connected", false);
        state_.net_audio_configured = flag(j, "net_audio", false);
        state_.net_audio_connected = flag(j, "net_audio_connected", false);
        state_.net_audio_stalled = flag(j, "net_audio_stalled", false);
        state_.net_audio_rx = (uint64_t)num(j, "net_audio_rx_frames", 0);
        state_.net_audio_tx = (uint64_t)num(j, "net_audio_tx_frames", 0);
        state_.set_net_audio_peer(str(j, "net_audio_peer"));
        state_.ptt_failed = flag(j, "ptt_failed", false);
    }

    void apply_packet(cJSON* p) {
        state_.add_packet(flag(p, "is_tx", false), (int)num(p, "size", 0),
                          (float)num(p, "snr", 0), (float)num(p, "ber", -1),
                          str(p, "mode"), str(p, "callsign"),
                          (int)num(p, "age_ms", 0));
    }

    void apply_message(cJSON* m) {
        state_.add_message_raw(str(m, "time"), str(m, "from"), str(m, "text"),
                               flag(m, "outgoing", false));
    }

    void apply_constellation(cJSON* j) {
        cJSON* arr = cJSON_GetObjectItemCaseSensitive(j, "points");
        if (!cJSON_IsArray(arr)) return;
        std::vector<std::complex<float>> pts;
        cJSON* it = arr->child;
        while (it && it->next) {
            if (cJSON_IsNumber(it) && cJSON_IsNumber(it->next))
                pts.emplace_back((float)it->valuedouble, (float)it->next->valuedouble);
            it = it->next->next;
        }
        if (pts.empty()) return;
        state_.update_constellation(pts.data(), (int)pts.size(),
                                    (int)num(j, "mod_bits", 2), -1);
    }

    void apply_scope(cJSON* j) {
        std::string pcm = str(j, "pcm");
        if (pcm.empty()) return;
        auto bytes = base64_decode(pcm.c_str(), pcm.size());
        std::vector<float> samples(bytes.size() / 2);
        for (size_t i = 0; i < samples.size(); i++) {
            int16_t v = (int16_t)((uint16_t)bytes[2 * i] | ((uint16_t)bytes[2 * i + 1] << 8));
            samples[i] = v / 32767.0f;
        }
        state_.push_scope_decimated(samples.data(), (int)samples.size());
    }

    void apply_devices(cJSON* dev) {
        if (!dev) return;
        state_.available_input_devices.clear();
        state_.input_device_descriptions.clear();
        state_.available_output_devices.clear();
        state_.output_device_descriptions.clear();
        cJSON* it;
        cJSON_ArrayForEach(it, cJSON_GetObjectItemCaseSensitive(dev, "inputs")) {
            state_.available_input_devices.push_back(str(it, "id"));
            state_.input_device_descriptions.push_back(str(it, "desc"));
        }
        cJSON_ArrayForEach(it, cJSON_GetObjectItemCaseSensitive(dev, "outputs")) {
            state_.available_output_devices.push_back(str(it, "id"));
            state_.output_device_descriptions.push_back(str(it, "desc"));
        }
        if (state_.available_input_devices.empty()) {
            state_.available_input_devices.push_back("default");
            state_.input_device_descriptions.push_back("default - System Default");
        }
        if (state_.available_output_devices.empty()) {
            state_.available_output_devices.push_back("default");
            state_.output_device_descriptions.push_back("default - System Default");
        }
        state_.audio_input_index = 0;
        state_.audio_output_index = 0;
        for (size_t i = 0; i < state_.available_input_devices.size(); i++)
            if (state_.available_input_devices[i] == state_.audio_input_device) {
                state_.audio_input_index = (int)i;
                break;
            }
        for (size_t i = 0; i < state_.available_output_devices.size(); i++)
            if (state_.available_output_devices[i] == state_.audio_output_device) {
                state_.audio_output_index = (int)i;
                break;
            }
    }

    bool sync_from_server() {
        cJSON* r = simple_request("get_ui_state", 8000);
        if (!ok(r)) {
            last_error_ = r ? str(r, "error") : "no response";
            if (r) cJSON_Delete(r);
            return false;
        }
        std::string server_version = str(r, "version");
        std::string settings = str(r, "settings");
        std::shared_ptr<cJSON> devs(cJSON_DetachItemFromObjectCaseSensitive(r, "audio_devices"),
                                    cJSON_Delete);
        state_.post_ui_task([this, settings, devs](TNCUIState& s) {
            if (!settings.empty()) {
                s.parse_settings_text(settings);
                s.update_modem_info();
            }
            apply_devices(devs.get());
        });

        {
            std::lock_guard<std::mutex> lock(state_.log_mutex);
            state_.log_entries.clear();
        }
        cJSON* it;
        cJSON_ArrayForEach(it, cJSON_GetObjectItemCaseSensitive(r, "log")) {
            if (cJSON_IsString(it) && it->valuestring)
                state_.add_log_raw(it->valuestring);
        }
        {
            std::lock_guard<std::mutex> lock(state_.packets_mutex);
            state_.recent_packets.clear();
        }
        cJSON_ArrayForEach(it, cJSON_GetObjectItemCaseSensitive(r, "packets")) apply_packet(it);
        {
            std::lock_guard<std::mutex> lock(state_.messages_mutex);
            state_.messages.clear();
        }
        cJSON_ArrayForEach(it, cJSON_GetObjectItemCaseSensitive(r, "messages")) apply_message(it);
        state_.unread_messages = 0;
        if (!server_version.empty() && server_version != MODEM73_VERSION &&
            !version_warned_.exchange(true))
            state_.add_log("(!) attach: server runs modem73 " + server_version +
                           ", this TUI is " MODEM73_VERSION ", settings may not match");

        cJSON* status = cJSON_GetObjectItemCaseSensitive(r, "status");
        if (status) apply_status(status);

        double log_c = num(r, "log_total", 0);
        double pkt_c = num(r, "packet_total", 0);
        double msg_c = num(r, "message_total", 0);
        cJSON_Delete(r);

        cJSON* req = cJSON_CreateObject();
        cJSON_AddStringToObject(req, "cmd", "subscribe");
        cJSON* arr = cJSON_AddArrayToObject(req, "streams");
        for (const char* s : {"status", "log", "packet", "message", "constellation", "settings"})
            cJSON_AddItemToArray(arr, cJSON_CreateString(s));
        cJSON* cursors = cJSON_AddObjectToObject(req, "cursors");
        cJSON_AddNumberToObject(cursors, "log", log_c);
        cJSON_AddNumberToObject(cursors, "packet", pkt_c);
        cJSON_AddNumberToObject(cursors, "message", msg_c);
        r = request(req);
        bool good = ok(r);
        if (r) cJSON_Delete(r);
        scope_subscribed_ = false;
        return good;
    }

    void bind_callbacks() {
        state_.on_settings_changed = [this](TNCUIState& s) {
            cJSON* req = cJSON_CreateObject();
            cJSON_AddStringToObject(req, "cmd", "set_ui_settings");
            cJSON_AddStringToObject(req, "settings", s.serialize_settings().c_str());
            cJSON* r = request(req, 10000);
            if (ok(r)) {
                std::string applied = str(r, "settings");
                if (!applied.empty()) s.parse_settings_text(applied);
            } else {
                s.add_log("(!) attach: settings were not applied");
            }
            if (r) cJSON_Delete(r);
        };
        state_.on_send_data = [this](const std::vector<uint8_t>& data) {
            cJSON* req = cJSON_CreateObject();
            cJSON_AddStringToObject(req, "cmd", "tx");
            cJSON_AddStringToObject(req, "data",
                                    base64_encode(data.data(), data.size()).c_str());
            cJSON* r = request(req);
            if (!ok(r)) state_.add_log("(!) attach: send failed");
            if (r) cJSON_Delete(r);
        };
        state_.on_rigctl_command = [this](const std::string& cmd) -> std::string {
            cJSON* req = cJSON_CreateObject();
            cJSON_AddStringToObject(req, "cmd", "rigctl");
            cJSON_AddStringToObject(req, "command", cmd.c_str());
            cJSON* r = request(req);
            std::string out = ok(r) ? str(r, "response") : std::string();
            if (r) cJSON_Delete(r);
            return out;
        };
        state_.on_reconnect_audio = [this]() -> bool {
            cJSON* r = simple_request("reconnect_audio");
            bool good = ok(r);
            if (r) cJSON_Delete(r);
            return good;
        };
        state_.on_refresh_audio_devices = [this]() {
            cJSON* r = simple_request("list_audio_devices", 10000);
            if (ok(r)) apply_devices(r);
            if (r) cJSON_Delete(r);
        };
        state_.on_get_audio_level = [this]() -> float {
            return state_.remote_audio_level_db.load();
        };
        state_.on_restart_requested = [this]() {
            cJSON* r = simple_request("restart");
            if (ok(r))
                state_.add_log("attach: server restarting, waiting for it to come back");
            else
                state_.add_log("(!) attach: restart refused");
            if (r) cJSON_Delete(r);
        };
        state_.on_alc_tune = [this]() -> float {
            cJSON* r = simple_request("alc_tune", 60000);
            float drive = ok(r) ? (float)num(r, "tx_drive", -1) : -1.0f;
            if (r) cJSON_Delete(r);
            return drive;
        };
    }

    void ticker_loop() {
        while (running_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            int64_t now = now_ms();
            if (fd_.load() < 0) {
                if (now - last_connect_ms_ < 2000) continue;
                last_connect_ms_ = now;
                if (connect_socket() && sync_from_server()) {
                    lost_logged_ = false;
                    state_.add_log("attach: reconnected to " + target_);
                } else {
                    close_socket();
                }
                continue;
            }
            bool want_scope = state_.scope_active.load();
            if (want_scope != scope_subscribed_.load()) {
                if (set_stream("scope", want_scope)) scope_subscribed_ = want_scope;
            }
            if (state_.stats_reset_requested.exchange(false)) {
                cJSON* r = simple_request("reset_stats");
                if (r) cJSON_Delete(r);
            }
            if (state_.rig_poll_enabled.load() && now - last_rig_ms_ >= 500) {
                last_rig_ms_ = now;
                state_.poll_rig();
            }
        }
    }

    TNCUIState& state_;
    std::string host_;
    int port_;
    std::string target_;
    std::atomic<int> fd_{-1};
    std::atomic<bool> running_{false};
    std::thread reader_;
    std::thread ticker_;
    std::mutex call_mutex_;
    std::mutex send_mutex_;
    std::mutex resp_mutex_;
    std::condition_variable resp_cv_;
    cJSON* resp_ = nullptr;
    bool waiting_ = false;
    uint64_t resp_gen_ = 0;
    std::atomic<bool> scope_subscribed_{false};
    std::atomic<bool> lost_logged_{false};
    std::atomic<bool> version_warned_{false};
    std::string last_error_;
    int64_t last_rig_ms_ = 0;
    int64_t last_connect_ms_ = 0;
};
