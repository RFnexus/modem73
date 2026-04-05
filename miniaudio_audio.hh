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
#include <atomic>
#include <thread>
#include <chrono>

class MiniAudio {
public:
    static std::vector<std::pair<std::string, std::string>> list_capture_devices() {
        std::vector<std::pair<std::string, std::string>> result;
        
        ma_context context;
        if (ma_context_init(NULL, 0, NULL, &context) != MA_SUCCESS) {
            result.push_back({"default", "default - System Default"});
            return result;
        }
        
        ma_device_info* playback_devices;
        ma_uint32 playback_count;
        ma_device_info* capture_devices;
        ma_uint32 capture_count;
        
        if (ma_context_get_devices(&context, &playback_devices, &playback_count,
                                   &capture_devices, &capture_count) != MA_SUCCESS) {
            ma_context_uninit(&context);
            result.push_back({"default", "default - System Default"});
            return result;
        }
        
        result.push_back({"default", "default - System Default"});
        
        for (ma_uint32 i = 0; i < capture_count; i++) {
            std::string id = std::to_string(i);
            std::string name = capture_devices[i].name;
            std::string desc = id + " - " + name;
            result.push_back({id, desc});
        }
        
        ma_context_uninit(&context);
        return result;
    }
    
    static std::vector<std::pair<std::string, std::string>> list_playback_devices() {
        std::vector<std::pair<std::string, std::string>> result;
        
        ma_context context;
        if (ma_context_init(NULL, 0, NULL, &context) != MA_SUCCESS) {
            result.push_back({"default", "default - System Default"});
            return result;
        }
        
        ma_device_info* playback_devices;
        ma_uint32 playback_count;
        ma_device_info* capture_devices;
        ma_uint32 capture_count;
        
        if (ma_context_get_devices(&context, &playback_devices, &playback_count,
                                   &capture_devices, &capture_count) != MA_SUCCESS) {
            ma_context_uninit(&context);
            result.push_back({"default", "default - System Default"}); // ====!====
            return result;
        }
        
        result.push_back({"default", "default - System Default"});
        
        for (ma_uint32 i = 0; i < playback_count; i++) {
            std::string id = std::to_string(i);
            std::string name = playback_devices[i].name;
            std::string desc = id + " - " + name;
            result.push_back({id, desc});
        }
        
        ma_context_uninit(&context);
        return result;
    }
    // temp
    static std::vector<std::pair<std::string, std::string>> list_devices() {
        return list_capture_devices();
    }
    
    MiniAudio(const std::string& capture_dev = "default", 
              const std::string& playback_dev = "default",
              int sample_rate = 48000)
        : capture_device_id_(capture_dev), playback_device_id_(playback_dev), sample_rate_(sample_rate) {
        capture_buffer_.resize(RING_BUFFER_SIZE, 0.0f);
        playback_buffer_.resize(RING_BUFFER_SIZE, 0.0f);
    }
    
    ~MiniAudio() {
        close_capture();
        close_playback();
        
        if (context_initialized_) {
            ma_context_uninit(&context_);
            context_initialized_ = false;
        }
    }
    
    const std::string& capture_device() const { return capture_device_id_; }
    const std::string& playback_device() const { return playback_device_id_; }
    
    void set_capture_device(const std::string& device) {
        close_capture();
        capture_device_id_ = device;
    }
    
    void set_playback_device(const std::string& device) {
        close_playback();
        playback_device_id_ = device;
    }
    
    bool open_playback() {
        if (!ensure_context()) return false;
        
        ma_device_config config = ma_device_config_init(ma_device_type_playback);
        config.playback.format = ma_format_f32;
        config.playback.channels = 1;
        config.sampleRate = sample_rate_;
        config.dataCallback = playback_callback;
        config.pUserData = this;
        config.periodSizeInFrames = 480;
        config.periods = 4;
        
        if (playback_device_id_ != "default" && !playback_device_id_.empty()) {
            int dev_index = std::atoi(playback_device_id_.c_str());
            
            ma_device_info* playback_devices;
            ma_uint32 playback_count;
            ma_device_info* capture_devices;
            ma_uint32 capture_count;
            
            if (ma_context_get_devices(&context_, &playback_devices, &playback_count,
                                       &capture_devices, &capture_count) == MA_SUCCESS) {
                if (dev_index >= 0 && dev_index < (int)playback_count) {
                    stored_playback_id_ = playback_devices[dev_index].id;
                    config.playback.pDeviceID = &stored_playback_id_;
                }
            }
        }
        
        if (ma_device_init(&context_, &config, &playback_device_) != MA_SUCCESS) {
            std::cerr << "Failed to init playback device" << std::endl;
            return false;
        }
        
        if (ma_device_start(&playback_device_) != MA_SUCCESS) {
            std::cerr << "Failed to start playback device" << std::endl;
            ma_device_uninit(&playback_device_);
            return false;
        }
        
        playback_open_ = true;
        std::cerr << "Playback: " << playback_device_.playback.name << std::endl;
        return true;
    }
    
    bool open_capture() {
        if (!ensure_context()) return false;
        
        ma_device_config config = ma_device_config_init(ma_device_type_capture);
        config.capture.format = ma_format_f32;
        config.capture.channels = 1;
        config.sampleRate = sample_rate_;
        config.dataCallback = capture_callback;
        config.pUserData = this;
        config.periodSizeInFrames = 480;
        config.periods = 4;
        
        if (capture_device_id_ != "default" && !capture_device_id_.empty()) {
            int dev_index = std::atoi(capture_device_id_.c_str());
            
            ma_device_info* playback_devices;
            ma_uint32 playback_count;
            ma_device_info* capture_devices;
            ma_uint32 capture_count;
            
            if (ma_context_get_devices(&context_, &playback_devices, &playback_count,
                                       &capture_devices, &capture_count) == MA_SUCCESS) {
                if (dev_index >= 0 && dev_index < (int)capture_count) {
                    stored_capture_id_ = capture_devices[dev_index].id;
                    config.capture.pDeviceID = &stored_capture_id_;
                }
            }
        }
        
        if (ma_device_init(&context_, &config, &capture_device_) != MA_SUCCESS) {
            std::cerr << "Failed to initialize capture device" << std::endl;
            return false;
        }
        
        if (ma_device_start(&capture_device_) != MA_SUCCESS) {
            std::cerr << "Failed to start capture device" << std::endl;
            ma_device_uninit(&capture_device_);
            return false;
        }
        
        capture_open_ = true;
        std::cerr << "Capture: " << capture_device_.capture.name << std::endl;
        return true;
    }
    
    void close_playback() {
        if (playback_open_) {
            ma_device_uninit(&playback_device_);
            playback_open_ = false;
        }
        playback_read_pos_ = 0;
        playback_write_pos_ = 0;
    }
    
    void close_capture() {
        if (capture_open_) {
            ma_device_uninit(&capture_device_);
            capture_open_ = false;
        }
        capture_read_pos_ = 0;
        capture_write_pos_ = 0;
    }
    
    int read(float* buffer, int frames) {
        if (!capture_open_) return -1;
        
        int frames_read = 0;
        int timeout_ms = 1000;
        auto start = std::chrono::steady_clock::now();
        
        while (frames_read < frames) {
            size_t read_pos = capture_read_pos_.load();
            size_t write_pos = capture_write_pos_.load();
            size_t available = (write_pos - read_pos + RING_BUFFER_SIZE) % RING_BUFFER_SIZE;
            
            if (available > 0) {
                int to_read = std::min((int)available, frames - frames_read);
                for (int i = 0; i < to_read; i++) {
                    buffer[frames_read + i] = capture_buffer_[(read_pos + i) % RING_BUFFER_SIZE];
                }
                capture_read_pos_ = (read_pos + to_read) % RING_BUFFER_SIZE;
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
        if (!playback_open_) return -1;
        
        int frames_written = 0;
        int timeout_ms = 1000;
        auto start = std::chrono::steady_clock::now();
        
        while (frames_written < frames) {
            size_t read_pos = playback_read_pos_.load();
            size_t write_pos = playback_write_pos_.load();
            size_t used = (write_pos - read_pos + RING_BUFFER_SIZE) % RING_BUFFER_SIZE;
            size_t available = RING_BUFFER_SIZE - 1 - used;
            
            if (available > 0) {
                int to_write = std::min((int)available, frames - frames_written);
                for (int i = 0; i < to_write; i++) {
                    playback_buffer_[(write_pos + i) % RING_BUFFER_SIZE] = buffer[frames_written + i];
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
    
    // check audio status
    bool is_healthy() const {
        return capture_open_ && playback_open_ && 
               consecutive_read_failures_ < 3 && 
               consecutive_write_failures_ < 3;
    }
    
    // attempt to reconnect audio devices
    bool reconnect() {
        close_capture();
        close_playback();
        
        consecutive_read_failures_ = 0;
        consecutive_write_failures_ = 0;
        
        bool ok = true;
        if (!open_playback()) ok = false;
        if (!open_capture()) ok = false;
        
        return ok;
    }
    
    void write_silence(int frames) {
        std::vector<float> silence(frames, 0.0f);
        write(silence.data(), frames);
    }
    
    void drain_playback() {
        if (!playback_open_) return;
        
        int timeout_ms = 2000;
        auto start = std::chrono::steady_clock::now();
        
        while (true) {
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




    float measure_level(int duration_ms = 100) {
        (void)duration_ms;
        if (!capture_open_) return -100.0f;
        float rms = std::sqrt(capture_level_sum_.load() /
                              std::max(1.0f, (float)capture_level_count_.load()));
        if (rms < 1e-10f) return -100.0f;
        return 20.0f * std::log10(rms);
    }
    
    int sample_rate() const { return sample_rate_; }
    
private:
    static constexpr size_t RING_BUFFER_SIZE = 48000;
    
    bool ensure_context() {
        if (context_initialized_) return true;
        
        if (ma_context_init(NULL, 0, NULL, &context_) != MA_SUCCESS) {
            std::cerr << "Failed to initialize audio context" << std::endl;
            return false;
        }
        context_initialized_ = true;
        return true;
    }
    
    static void playback_callback(ma_device* device, void* output, const void* input, ma_uint32 frame_count) {
        (void)input;
        MiniAudio* self = static_cast<MiniAudio*>(device->pUserData);
        float* out = static_cast<float*>(output);
        
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
    }
    
    static void capture_callback(ma_device* device, void* output, const void* input, ma_uint32 frame_count) {
        (void)output;
        MiniAudio* self = static_cast<MiniAudio*>(device->pUserData);
        const float* in = static_cast<const float*>(input);
        
        size_t read_pos = self->capture_read_pos_.load();
        size_t write_pos = self->capture_write_pos_.load();
        size_t used = (write_pos - read_pos + RING_BUFFER_SIZE) % RING_BUFFER_SIZE;
        size_t available = RING_BUFFER_SIZE - 1 - used;
        
        ma_uint32 to_write = std::min((ma_uint32)available, frame_count);
        
        float sum_sq = 0.0f;
        for (ma_uint32 i = 0; i < to_write; i++) {
            self->capture_buffer_[(write_pos + i) % RING_BUFFER_SIZE] = in[i];
            sum_sq += in[i] * in[i];
        }

        self->capture_write_pos_ = (write_pos + to_write) % RING_BUFFER_SIZE;

        // Update running level for CSMA exponential moving average
        self->capture_level_sum_ = sum_sq;
        self->capture_level_count_ = to_write;
    }
    
    std::string capture_device_id_;
    std::string playback_device_id_;
    int sample_rate_;
    
    ma_context context_;
    bool context_initialized_ = false;
    
    ma_device playback_device_;
    ma_device capture_device_;
    ma_device_id stored_capture_id_;
    ma_device_id stored_playback_id_;
    bool playback_open_ = false;
    bool capture_open_ = false;
    
    std::vector<float> capture_buffer_;
    std::vector<float> playback_buffer_;
    std::atomic<size_t> capture_read_pos_{0};
    std::atomic<size_t> capture_write_pos_{0};
    std::atomic<size_t> playback_read_pos_{0};
    std::atomic<size_t> playback_write_pos_{0};
    
    int consecutive_read_failures_ = 0;
    int consecutive_write_failures_ = 0;

    // Running signal level for CSMA
    std::atomic<float> capture_level_sum_{0.0f};
    std::atomic<int> capture_level_count_{1};
};
