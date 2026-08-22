#pragma once

#ifdef WITH_HAMLIB
#include <string>
#include <mutex>

std::string hamlib_list_models();

class HamlibPTT {

public:
    ~HamlibPTT();

    bool open(int model, const std::string& device, int baud, std::string& err);

    bool set_ptt(bool on);
    
    bool get_freq(double& hz);
    std::string command(const std::string& cmd);
    bool is_connected() const { return connected_; }
    const std::string& last_error() const { return last_error_; }
    void close();

private:
    void close_locked();
    void* rig_ = nullptr;
    bool connected_ = false;
    std::mutex mutex_;
    std::string last_error_;
};

#endif
