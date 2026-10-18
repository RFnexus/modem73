#pragma once

#define MA_NO_DECODING
#define MA_NO_ENCODING
#define MA_NO_GENERATION
#define MA_NO_ENGINE
#define MA_NO_NODE_GRAPH
#include "deps/miniaudio.h"

#include <vector>
#include <string>
#include <iostream>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <algorithm>
#include <atomic>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <thread>
#include <chrono>

class MiniAudio {
public:
    static std::vector<std::pair<std::string, std::string>> list_capture_devices(const std::string& backend = "auto") {
        return list_devices_impl(true, backend);
    }

    static std::vector<std::pair<std::string, std::string>> list_playback_devices(const std::string& backend = "auto") {
        return list_devices_impl(false, backend);
    }
    // temp
    static std::vector<std::pair<std::string, std::string>> list_devices() {
        return list_capture_devices();
    }

    static bool valid_backend(const std::string& b) {
        return b == "auto" || b == "alsa" || b == "pulse" || b == "pulseaudio" || b == "jack";
    }

    MiniAudio(const std::string& capture_dev = "default",
              const std::string& playback_dev = "default",
              int sample_rate = 48000,
              const std::string& backend = "auto")
        : capture_device_id_(capture_dev), playback_device_id_(playback_dev),
          sample_rate_(sample_rate), backend_pref_(backend) {
        capture_buffer_.resize(CAPTURE_RING_SIZE, 0.0f);
        playback_buffer_.resize(RING_BUFFER_SIZE, 0.0f);
    }

    ~MiniAudio() {
        stopping_.store(true, std::memory_order_release);
        std::lock_guard<std::mutex> guard(lifecycle_mutex_);
        close_capture_locked();
        close_playback_locked();
        release_context_locked();
    }

    const std::string& capture_device() const { return capture_device_id_; }
    const std::string& playback_device() const { return playback_device_id_; }
    const std::string& backend_preference() const { return backend_pref_; }

    std::string backend_name() {
        std::lock_guard<std::mutex> guard(lifecycle_mutex_);
        return context_ ? ma_get_backend_name(context_->backend) : std::string("none");
    }

    void set_log_sink(std::function<void(const std::string&)> sink) {
        log_sink_ = std::move(sink);
    }

    void set_capture_device(const std::string& device) {
        std::lock_guard<std::mutex> guard(lifecycle_mutex_);
        close_capture_locked();
        capture_device_id_ = device;
    }

    void set_playback_device(const std::string& device) {
        std::lock_guard<std::mutex> guard(lifecycle_mutex_);
        close_playback_locked();
        playback_device_id_ = device;
    }

    void set_backend(const std::string& backend) {
        std::lock_guard<std::mutex> guard(lifecycle_mutex_);
        close_capture_locked();
        close_playback_locked();
        release_context_locked();
        backend_pref_ = backend;
    }

    bool open_playback(bool strict = false) {
        std::lock_guard<std::mutex> guard(lifecycle_mutex_);
        return open_playback_locked(strict);
    }

    bool open_capture(bool strict = false) {
        std::lock_guard<std::mutex> guard(lifecycle_mutex_);
        return open_capture_locked(strict);
    }

    void close_playback() {
        std::lock_guard<std::mutex> guard(lifecycle_mutex_);
        close_playback_locked();
    }

    void close_capture() {
        std::lock_guard<std::mutex> guard(lifecycle_mutex_);
        close_capture_locked();
    }

    void request_stop() {
        stopping_.store(true, std::memory_order_release);
    }

    int read(float* buffer, int frames) {
        if (stopping_.load(std::memory_order_acquire)) return -1;
        if (!capture_open_.load(std::memory_order_acquire)) return -1;
        std::shared_lock<std::shared_mutex> lock(capture_rw_, std::try_to_lock);
        if (!lock.owns_lock() || !capture_open_.load(std::memory_order_acquire)) return -1;

        int frames_read = 0;
        int timeout_ms = 1000;
        auto start = std::chrono::steady_clock::now();

        while (frames_read < frames) {
            if (stopping_.load(std::memory_order_acquire)) break;
            if (!capture_open_.load(std::memory_order_acquire)) break;
            size_t read_pos = capture_read_pos_.load();
            size_t write_pos = capture_write_pos_.load();
            size_t available = (write_pos - read_pos + CAPTURE_RING_SIZE) % CAPTURE_RING_SIZE;

            if (available > 0) {
                int to_read = std::min((int)available, frames - frames_read);
                for (int i = 0; i < to_read; i++) {
                    buffer[frames_read + i] = capture_buffer_[(read_pos + i) % CAPTURE_RING_SIZE];
                }
                capture_read_pos_ = (read_pos + to_read) % CAPTURE_RING_SIZE;
                frames_read += to_read;
                consecutive_read_failures_ = 0;
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                auto now = std::chrono::steady_clock::now();
                if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() > timeout_ms) {
                    consecutive_read_failures_++;
                    break;
                }
            }
        }

        return frames_read;
    }

    int write(const float* buffer, int frames) {
        if (stopping_.load(std::memory_order_acquire)) return -1;
        if (!playback_open_.load(std::memory_order_acquire)) return -1;
        std::shared_lock<std::shared_mutex> lock(playback_rw_, std::try_to_lock);
        if (!lock.owns_lock() || !playback_open_.load(std::memory_order_acquire)) return -1;

        int frames_written = 0;
        int timeout_ms = 1000;
        auto start = std::chrono::steady_clock::now();

        while (frames_written < frames) {
            if (stopping_.load(std::memory_order_acquire)) break;
            if (!playback_open_.load(std::memory_order_acquire)) break;
            size_t read_pos = playback_read_pos_.load();
            size_t write_pos = playback_write_pos_.load();
            size_t used = (write_pos - read_pos + RING_BUFFER_SIZE) % RING_BUFFER_SIZE;
            size_t available = RING_BUFFER_SIZE - 1 - used;

            if (available > 0) {
                int to_write = std::min((int)available, frames - frames_written);
                float g = tx_gain_.load();
                for (int i = 0; i < to_write; i++) {
                    playback_buffer_[(write_pos + i) % RING_BUFFER_SIZE] = buffer[frames_written + i] * g;
                }
                playback_write_pos_ = (write_pos + to_write) % RING_BUFFER_SIZE;
                frames_written += to_write;
                consecutive_write_failures_ = 0;
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                auto now = std::chrono::steady_clock::now();
                if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() > timeout_ms) {
                    consecutive_write_failures_++;
                    break;
                }
            }
        }

        return frames_written;
    }

    void set_tx_gain(float g) { tx_gain_.store(g); }

    // check audio status
    bool is_healthy() const {
        if (!capture_open_.load(std::memory_order_acquire) ||
            !playback_open_.load(std::memory_order_acquire))
            return false;
        if (capture_lost_.load(std::memory_order_acquire) ||
            playback_lost_.load(std::memory_order_acquire) ||
            rerouted_.load(std::memory_order_acquire))
            return false;
        if (consecutive_read_failures_.load() >= 3 ||
            consecutive_write_failures_.load() >= 3)
            return false;
        int64_t now = steady_ms();
        if (now - last_block_ms_.load(std::memory_order_relaxed) > HEARTBEAT_STALE_MS)
            return false;
        if (now - last_playback_ms_.load(std::memory_order_relaxed) > HEARTBEAT_STALE_MS)
            return false;
        return true;
    }

    std::string health_detail() const {
        if (!capture_open_.load()) return "capture closed";
        if (!playback_open_.load()) return "playback closed";
        if (capture_lost_.load()) return "capture device stopped";
        if (playback_lost_.load()) return "playback device stopped";
        if (rerouted_.load()) return "stream moved to another device";
        int64_t now = steady_ms();
        if (now - last_block_ms_.load() > HEARTBEAT_STALE_MS) return "no capture callbacks";
        if (now - last_playback_ms_.load() > HEARTBEAT_STALE_MS) return "no playback callbacks";
        if (consecutive_read_failures_.load() >= 3) return "capture read timeouts";
        if (consecutive_write_failures_.load() >= 3) return "playback write timeouts";
        return "ok";
    }

    // attempt to reconnect audio devices
    bool reconnect() {
        std::lock_guard<std::mutex> guard(lifecycle_mutex_);
        if (stopping_.load(std::memory_order_acquire)) return false;
        close_capture_locked();
        close_playback_locked();
        release_context_locked();

        consecutive_read_failures_ = 0;
        consecutive_write_failures_ = 0;
        capture_lost_.store(false);
        playback_lost_.store(false);
        rerouted_.store(false);

        bool ok = open_capture_locked(true) && open_playback_locked(true);
        if (!ok) {
            close_capture_locked();
            close_playback_locked();
        }
        return ok;
    }

    void write_silence(int frames) {
        std::vector<float> silence(frames, 0.0f);
        write(silence.data(), frames);
    }

    void drain_playback() {
        if (!playback_open_.load(std::memory_order_acquire)) return;

        int timeout_ms = 2000;
        auto start = std::chrono::steady_clock::now();

        while (true) {
            if (stopping_.load(std::memory_order_acquire)) return;
            if (!playback_open_.load(std::memory_order_acquire)) break;
            size_t read_pos = playback_read_pos_.load();
            size_t write_pos = playback_write_pos_.load();
            if (read_pos == write_pos) break;

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() > timeout_ms) {
                break;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }




    bool capture_alive() {
        return capture_open_.load(std::memory_order_acquire) &&
               block_seq_.load(std::memory_order_acquire) > 0 &&
               steady_ms() - last_block_ms_.load(std::memory_order_relaxed) <= 250;
    }

    float instant_level_db(int window_ms = 100) {
        if (!capture_open_.load(std::memory_order_acquire)) return 0.0f;
        if (steady_ms() - last_block_ms_.load(std::memory_order_relaxed) > 250)
            return 0.0f;
        uint64_t seq = block_seq_.load(std::memory_order_acquire);
        if (seq == 0) return 0.0f;
        uint64_t need = (uint64_t)sample_rate_ * window_ms / 1000;
        double energy = 0.0;
        uint64_t frames = 0;
        uint64_t count = seq < LEVEL_RING ? seq : LEVEL_RING;
        for (uint64_t k = 1; k <= count; k++) {
            uint64_t idx = (seq - k) % LEVEL_RING;
            uint32_t bf = block_frames_[idx].load(std::memory_order_relaxed);
            energy += block_msq_[idx].load(std::memory_order_relaxed) * bf;
            frames += bf;
            if (frames >= need) break;
        }
        if (frames == 0) return 0.0f;
        float rms = std::sqrt(energy / frames);
        if (rms < 1e-10f) return -100.0f;
        return 20.0f * std::log10(rms);
    }

    int sample_rate() const { return sample_rate_; }

private:
    static constexpr size_t RING_BUFFER_SIZE = 48000;
    static constexpr size_t CAPTURE_RING_SIZE = 8 * 48000;
    static constexpr int64_t HEARTBEAT_STALE_MS = 2000;
    static constexpr int UNINIT_TIMEOUT_MS = 3000;

    void log_msg(const std::string& msg) {
        if (log_sink_) log_sink_(msg);
        else std::cerr << msg << std::endl;
    }

    static std::vector<ma_backend> backend_order(const std::string& pref) {
#if defined(__linux__) && !defined(__ANDROID__)
        if (pref == "alsa")
            return {ma_backend_alsa, ma_backend_pulseaudio, ma_backend_jack};
        if (pref == "pulse" || pref == "pulseaudio")
            return {ma_backend_pulseaudio, ma_backend_alsa, ma_backend_jack};
        if (pref == "jack")
            return {ma_backend_jack, ma_backend_pulseaudio, ma_backend_alsa};
#else
        (void)pref;
#endif
        return {};
    }

    static ma_result init_context(ma_context* ctx, const std::string& pref, bool verbose) {
        ma_context_config cfg = ma_context_config_init();
        cfg.alsa.useVerboseDeviceEnumeration = verbose ? MA_TRUE : MA_FALSE;
        std::vector<ma_backend> order = backend_order(pref);
        return ma_context_init(order.empty() ? NULL : order.data(),
                               (ma_uint32)order.size(), &cfg, ctx);
    }

    static std::string one_line(const char* s) {
        std::string out;
        for (const char* p = s; *p; ++p) {
            if (*p == '\n' || *p == '\r') {
                bool has_sep = out.size() >= 3 && out.compare(out.size() - 3, 3, " / ") == 0;
                if (!out.empty() && !has_sep) out += " / ";
            } else {
                out += *p;
            }
        }
        while (out.size() >= 3 && out.compare(out.size() - 3, 3, " / ") == 0)
            out.erase(out.size() - 3);
        return out;
    }

    static std::string backend_id(const ma_context* ctx, const ma_device_info& info) {
        if (ctx->backend == ma_backend_alsa) return std::string(info.id.alsa);
        if (ctx->backend == ma_backend_pulseaudio) return std::string(info.id.pulse);
        return std::string();
    }

    static bool alsa_listed(const std::string& id) {
        if (id == "default" || id == "pipewire" || id == "pulse") return true;
        return id.rfind("hw:CARD=", 0) == 0 || id.rfind("plughw:CARD=", 0) == 0;
    }

    static bool looks_like_alsa_pcm(const std::string& s) {
        return s.find(':') != std::string::npos || s == "pipewire" || s == "pulse";
    }

    static std::vector<std::pair<std::string, std::string>> list_devices_impl(bool capture, const std::string& backend) {
        std::vector<std::pair<std::string, std::string>> result;
        result.push_back({"default", "default - System Default"});

        ma_context context;
        if (init_context(&context, backend, true) != MA_SUCCESS) return result;

        ma_device_info* playback_devices;
        ma_uint32 playback_count;
        ma_device_info* capture_devices;
        ma_uint32 capture_count;

        if (ma_context_get_devices(&context, &playback_devices, &playback_count,
                                   &capture_devices, &capture_count) != MA_SUCCESS) {
            ma_context_uninit(&context);
            return result;
        }

        ma_device_info* devices = capture ? capture_devices : playback_devices;
        ma_uint32 count = capture ? capture_count : playback_count;
        bool alsa = context.backend == ma_backend_alsa;

        for (ma_uint32 i = 0; i < count; i++) {
            std::string key = alsa ? std::string(devices[i].id.alsa) : std::string(devices[i].name);
            if (key.empty()) continue;
            if (alsa && !alsa_listed(key)) continue;
            std::string desc = one_line(devices[i].name);
            std::string label = alsa && !desc.empty() && desc != key ? key + " - " + desc : key;
            if (key == "default") {
                if (alsa && !desc.empty()) result[0].second = label;
                continue;
            }
            bool dup = std::any_of(result.begin(), result.end(),
                                   [&](const std::pair<std::string, std::string>& p) { return p.first == key; });
            if (dup) continue;
            result.push_back({key, label});
        }

        ma_context_uninit(&context);
        return result;
    }

    bool ensure_context() {
        if (context_) return true;

        ma_context* ctx = new ma_context;
        if (init_context(ctx, backend_pref_, false) != MA_SUCCESS) {
            delete ctx;
            log_msg("(!) Failed to initialize audio context");
            return false;
        }
        context_ = ctx;
        std::string name = ma_get_backend_name(ctx->backend);
        if (name != last_backend_logged_) {
            last_backend_logged_ = name;
            log_msg("Audio backend: " + name);
        }
        return true;
    }

    void release_context_locked() {
        if (!context_) return;
        if (context_abandoned_) {
            log_msg("(!) Leaving hung audio context behind");
        } else {
            ma_context_uninit(context_);
            delete context_;
        }
        context_ = nullptr;
        context_abandoned_ = false;
    }

    bool uninit_device_bounded(ma_device* dev) {
        auto done = std::make_shared<std::promise<void>>();
        std::future<void> fut = done->get_future();
        std::thread([dev, done]() {
            ma_device_stop(dev);
            ma_device_uninit(dev);
            delete dev;
            done->set_value();
        }).detach();
        return fut.wait_for(std::chrono::milliseconds(UNINIT_TIMEOUT_MS)) == std::future_status::ready;
    }

    bool find_device_id(bool capture, const std::string& selector, ma_device_id* out, bool* verbatim) {
        *verbatim = false;

        ma_device_info* playback_devices;
        ma_uint32 playback_count;
        ma_device_info* capture_devices;
        ma_uint32 capture_count;

        if (ma_context_get_devices(context_, &playback_devices, &playback_count,
                                   &capture_devices, &capture_count) == MA_SUCCESS) {
            ma_device_info* devices = capture ? capture_devices : playback_devices;
            ma_uint32 count = capture ? capture_count : playback_count;

            for (ma_uint32 i = 0; i < count; i++) {
                if (selector == devices[i].name) {
                    *out = devices[i].id;
                    return true;
                }
            }
            for (ma_uint32 i = 0; i < count; i++) {
                std::string id = backend_id(context_, devices[i]);
                if (!id.empty() && selector == id) {
                    *out = devices[i].id;
                    return true;
                }
            }
            if (!selector.empty() &&
                selector.find_first_not_of("0123456789") == std::string::npos) {
                int idx = std::atoi(selector.c_str());
                if (idx >= 0 && idx < (int)count) {
                    *out = devices[idx].id;
                    return true;
                }
            }
        }

        if (context_->backend != ma_backend_alsa) return false;

        ma_context verbose;
        if (init_context(&verbose, backend_pref_, true) == MA_SUCCESS) {
            bool found = false;
            if (ma_context_get_devices(&verbose, &playback_devices, &playback_count,
                                       &capture_devices, &capture_count) == MA_SUCCESS) {
                ma_device_info* devices = capture ? capture_devices : playback_devices;
                ma_uint32 count = capture ? capture_count : playback_count;
                for (ma_uint32 i = 0; i < count && !found; i++) {
                    if (selector == devices[i].id.alsa) {
                        *out = devices[i].id;
                        found = true;
                    }
                }
                for (ma_uint32 i = 0; i < count && !found; i++) {
                    if (selector == devices[i].name || selector == one_line(devices[i].name)) {
                        *out = devices[i].id;
                        found = true;
                    }
                }
            }
            ma_context_uninit(&verbose);
            if (found) return true;
        }

        if (looks_like_alsa_pcm(selector) && selector.size() < sizeof(out->alsa)) {
            std::memset(out, 0, sizeof(*out));
            std::memcpy(out->alsa, selector.c_str(), selector.size());
            *verbatim = true;
            return true;
        }

        return false;
    }

    std::string open_hint(const ma_device_id* id) const {
        if (!context_ || context_->backend != ma_backend_alsa || !id) return "";
        std::string s(id->alsa);
        if (s.rfind("hw:", 0) == 0 || s.rfind("plughw:", 0) == 0)
            return " - the card may be held by PipeWire or another program";
        return "";
    }

    bool open_playback_locked(bool strict) {
        if (playback_open_.load(std::memory_order_acquire)) return true;
        if (!ensure_context()) return false;

        ma_device_config config = ma_device_config_init(ma_device_type_playback);
        config.playback.format = ma_format_f32;
        config.playback.channels = 1;
        config.sampleRate = sample_rate_;
        config.dataCallback = playback_callback;
        config.notificationCallback = notification_callback;
        config.pUserData = this;
        config.periodSizeInFrames = 480;
        config.periods = 4;

        bool verbatim = false;
        bool resolved = false;
        if (playback_device_id_ != "default" && !playback_device_id_.empty()) {
            resolved = find_device_id(false, playback_device_id_, &stored_playback_id_, &verbatim);
            if (resolved) {
                config.playback.pDeviceID = &stored_playback_id_;
            } else if (strict) {
                log_msg("(!) Playback device '" + playback_device_id_ + "' not found");
                return false;
            } else {
                log_msg("(!) Playback device '" + playback_device_id_ +
                        "' not found, using system default");
            }
        }

        ma_device* dev = new ma_device;
        if (ma_device_init(context_, &config, dev) != MA_SUCCESS) {
            if (!(resolved && verbatim && !strict)) {
                delete dev;
                log_msg("(!) Failed to open playback device '" + playback_device_id_ + "'" +
                        open_hint(config.playback.pDeviceID));
                return false;
            }
            log_msg("(!) Playback device '" + playback_device_id_ +
                    "' could not be opened" + open_hint(config.playback.pDeviceID) +
                    ", using system default");
            config.playback.pDeviceID = nullptr;
            if (ma_device_init(context_, &config, dev) != MA_SUCCESS) {
                delete dev;
                log_msg("(!) Failed to open default playback device");
                return false;
            }
        }

        playback_lost_.store(false);
        last_playback_ms_.store(steady_ms(), std::memory_order_relaxed);
        playback_device_.store(dev, std::memory_order_release);
        if (ma_device_start(dev) != MA_SUCCESS) {
            playback_device_.store(nullptr, std::memory_order_release);
            log_msg("(!) Failed to start playback device");
            uninit_device_bounded(dev);
            return false;
        }

        playback_open_.store(true, std::memory_order_release);
        log_msg(std::string("Playback: ") + one_line(dev->playback.name));
        return true;
    }

    bool open_capture_locked(bool strict) {
        if (capture_open_.load(std::memory_order_acquire)) return true;
        if (!ensure_context()) return false;

        ma_device_config config = ma_device_config_init(ma_device_type_capture);
        config.capture.format = ma_format_f32;
        config.capture.channels = 1;
        config.sampleRate = sample_rate_;
        config.dataCallback = capture_callback;
        config.notificationCallback = notification_callback;
        config.pUserData = this;
        config.periodSizeInFrames = 480;
        config.periods = 4;
#ifdef __ANDROID__
        config.aaudio.inputPreset = ma_aaudio_input_preset_unprocessed;
        config.performanceProfile = ma_performance_profile_conservative;
#endif

        bool verbatim = false;
        bool resolved = false;
        if (capture_device_id_ != "default" && !capture_device_id_.empty()) {
            resolved = find_device_id(true, capture_device_id_, &stored_capture_id_, &verbatim);
            if (resolved) {
                config.capture.pDeviceID = &stored_capture_id_;
            } else if (strict) {
                log_msg("(!) Capture device '" + capture_device_id_ + "' not found");
                return false;
            } else {
                log_msg("(!) Capture device '" + capture_device_id_ +
                        "' not found, using system default");
            }
        }

        ma_device* dev = new ma_device;
        if (ma_device_init(context_, &config, dev) != MA_SUCCESS) {
            if (!(resolved && verbatim && !strict)) {
                delete dev;
                log_msg("(!) Failed to open capture device '" + capture_device_id_ + "'" +
                        open_hint(config.capture.pDeviceID));
                return false;
            }
            log_msg("(!) Capture device '" + capture_device_id_ +
                    "' could not be opened" + open_hint(config.capture.pDeviceID) +
                    ", using system default");
            config.capture.pDeviceID = nullptr;
            if (ma_device_init(context_, &config, dev) != MA_SUCCESS) {
                delete dev;
                log_msg("(!) Failed to open default capture device");
                return false;
            }
        }

        capture_lost_.store(false);
        last_block_ms_.store(steady_ms(), std::memory_order_relaxed);
        capture_device_.store(dev, std::memory_order_release);
        if (ma_device_start(dev) != MA_SUCCESS) {
            capture_device_.store(nullptr, std::memory_order_release);
            log_msg("(!) Failed to start capture device");
            uninit_device_bounded(dev);
            return false;
        }

        capture_open_.store(true, std::memory_order_release);
        log_msg(std::string("Capture: ") + one_line(dev->capture.name) +
                " (device " + std::to_string(dev->capture.internalSampleRate) + " Hz, " +
                std::to_string(dev->capture.internalChannels) + " ch, period " +
                std::to_string(dev->capture.internalPeriodSizeInFrames) + ")");
        return true;
    }

    void close_playback_locked() {
        bool was_open = playback_open_.exchange(false, std::memory_order_acq_rel);
        ma_device* dev = playback_device_.exchange(nullptr, std::memory_order_acq_rel);
        std::unique_lock<std::shared_mutex> lock(playback_rw_);
        if (dev) {
            if (!uninit_device_bounded(dev)) {
                context_abandoned_ = true;
                log_msg("(!) Playback device did not close within " +
                        std::to_string(UNINIT_TIMEOUT_MS / 1000) + "s, abandoning it");
            }
        }
        (void)was_open;
        playback_read_pos_ = 0;
        playback_write_pos_ = 0;
    }

    void close_capture_locked() {
        bool was_open = capture_open_.exchange(false, std::memory_order_acq_rel);
        ma_device* dev = capture_device_.exchange(nullptr, std::memory_order_acq_rel);
        std::unique_lock<std::shared_mutex> lock(capture_rw_);
        if (dev) {
            if (!uninit_device_bounded(dev)) {
                context_abandoned_ = true;
                log_msg("(!) Capture device did not close within " +
                        std::to_string(UNINIT_TIMEOUT_MS / 1000) + "s, abandoning it");
            }
        }
        (void)was_open;
        capture_read_pos_ = 0;
        capture_write_pos_ = 0;
    }

    static void notification_callback(const ma_device_notification* n) {
        MiniAudio* self = static_cast<MiniAudio*>(n->pDevice->pUserData);
        if (!self) return;
        bool is_capture = n->pDevice->type == ma_device_type_capture;
        ma_device* current = is_capture ? self->capture_device_.load(std::memory_order_acquire)
                                        : self->playback_device_.load(std::memory_order_acquire);
        if (n->pDevice != current) return;
        std::atomic<bool>& lost = is_capture ? self->capture_lost_ : self->playback_lost_;
        switch (n->type) {
        case ma_device_notification_type_stopped:
            lost.store(true, std::memory_order_release);
            break;
        case ma_device_notification_type_started:
            lost.store(false, std::memory_order_release);
            break;
        case ma_device_notification_type_rerouted:
            self->rerouted_.store(true, std::memory_order_release);
            break;
        default:
            break;
        }
    }

    static void playback_callback(ma_device* device, void* output, const void* input, ma_uint32 frame_count) {
        (void)input;
        MiniAudio* self = static_cast<MiniAudio*>(device->pUserData);
        float* out = static_cast<float*>(output);

        if (device != self->playback_device_.load(std::memory_order_acquire)) {
            for (ma_uint32 i = 0; i < frame_count; i++) out[i] = 0.0f;
            return;
        }

        size_t read_pos = self->playback_read_pos_.load();
        size_t write_pos = self->playback_write_pos_.load();
        size_t available = (write_pos - read_pos + RING_BUFFER_SIZE) % RING_BUFFER_SIZE;

        ma_uint32 to_read = std::min((ma_uint32)available, frame_count);

        for (ma_uint32 i = 0; i < to_read; i++) {
            out[i] = self->playback_buffer_[(read_pos + i) % RING_BUFFER_SIZE];
        }
        for (ma_uint32 i = to_read; i < frame_count; i++) {
            out[i] = 0.0f;
        }

        self->playback_read_pos_ = (read_pos + to_read) % RING_BUFFER_SIZE;
        self->last_playback_ms_.store(steady_ms(), std::memory_order_relaxed);
    }

    static void capture_callback(ma_device* device, void* output, const void* input, ma_uint32 frame_count) {
        (void)output;
        MiniAudio* self = static_cast<MiniAudio*>(device->pUserData);
        const float* in = static_cast<const float*>(input);

        if (device != self->capture_device_.load(std::memory_order_acquire)) return;

        size_t read_pos = self->capture_read_pos_.load();
        size_t write_pos = self->capture_write_pos_.load();
        size_t used = (write_pos - read_pos + CAPTURE_RING_SIZE) % CAPTURE_RING_SIZE;
        size_t available = CAPTURE_RING_SIZE - 1 - used;

        ma_uint32 to_write = std::min((ma_uint32)available, frame_count);

        for (ma_uint32 i = 0; i < to_write; i++) {
            self->capture_buffer_[(write_pos + i) % CAPTURE_RING_SIZE] = in[i];
        }

        self->capture_write_pos_ = (write_pos + to_write) % CAPTURE_RING_SIZE;

        float sum_sq = 0.0f;
        for (ma_uint32 i = 0; i < frame_count; i++) {
            sum_sq += in[i] * in[i];
        }
        self->capture_energy_.store(self->capture_energy_.load() + sum_sq);
        self->capture_samples_.store(self->capture_samples_.load() + frame_count);

        if (frame_count > 0) {
            uint64_t seq = self->block_seq_.load(std::memory_order_relaxed);
            self->block_msq_[seq % LEVEL_RING].store(sum_sq / frame_count,
                                                     std::memory_order_relaxed);
            self->block_frames_[seq % LEVEL_RING].store(frame_count,
                                                        std::memory_order_relaxed);
            self->block_seq_.store(seq + 1, std::memory_order_release);
            self->last_block_ms_.store(steady_ms(), std::memory_order_relaxed);
        }
    }

    static int64_t steady_ms() {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    }

    std::string capture_device_id_;
    std::string playback_device_id_;
    int sample_rate_;
    std::string backend_pref_;
    std::function<void(const std::string&)> log_sink_;

    ma_context* context_ = nullptr;
    bool context_abandoned_ = false;
    std::string last_backend_logged_;

    std::atomic<ma_device*> playback_device_{nullptr};
    std::atomic<ma_device*> capture_device_{nullptr};
    ma_device_id stored_capture_id_;
    ma_device_id stored_playback_id_;
    std::atomic<bool> playback_open_{false};
    std::atomic<float> tx_gain_{1.0f};
    std::atomic<bool> capture_open_{false};
    std::atomic<bool> stopping_{false};
    std::atomic<bool> capture_lost_{false};
    std::atomic<bool> playback_lost_{false};
    std::atomic<bool> rerouted_{false};

    std::mutex lifecycle_mutex_;
    std::shared_mutex capture_rw_;
    std::shared_mutex playback_rw_;

    std::vector<float> capture_buffer_;
    std::vector<float> playback_buffer_;
    std::atomic<size_t> capture_read_pos_{0};
    std::atomic<size_t> capture_write_pos_{0};
    std::atomic<size_t> playback_read_pos_{0};
    std::atomic<size_t> playback_write_pos_{0};

    std::atomic<int> consecutive_read_failures_{0};
    std::atomic<int> consecutive_write_failures_{0};

    // Running signal level for CSMA
    std::atomic<double> capture_energy_{0.0};
    std::atomic<uint64_t> capture_samples_{0};
    static constexpr size_t LEVEL_RING = 64;
    std::atomic<double> block_msq_[LEVEL_RING] = {};
    std::atomic<uint32_t> block_frames_[LEVEL_RING] = {};
    std::atomic<uint64_t> block_seq_{0};
    std::atomic<int64_t> last_block_ms_{0};
    std::atomic<int64_t> last_playback_ms_{0};
};
