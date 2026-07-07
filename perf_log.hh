// per-mode rx performance logger.
//
#pragma once

#include <string>
#include <map>
#include <vector>
#include <mutex>
#include <cstdio>
#include <ctime>
#include <cstdlib>
#include <sys/stat.h>

struct PerfModeStats {
    std::string mode;
    int frames = 0;
    int lost = 0;
    int last_seq = -1;
    float snr_min = 0, snr_max = 0, snr_sum = 0;
    double ber_sum = 0;
    int ber_n = 0;
    time_t last_seen = 0;

    float snr_avg() const { return frames ? snr_sum / frames : 0; }
    float ber_avg() const { return ber_n ? (float)(ber_sum / ber_n) : -1; }
};

class PerfLogger {
public:
    ~PerfLogger() {
        if (csv_)
            fclose(csv_);
    }

    static std::string default_path() {
        const char* home = getenv("HOME");
        return std::string(home ? home : ".") + "/.config/modem73/perf.csv";
    }

    void record(const std::string& mode, float snr, float ber_pct, int bytes,
                int seq = -1) {
        std::lock_guard<std::mutex> lock(m_);
        auto& a = stats_[mode];
        int lost_before = 0;
        if (seq >= 0) {
            if (a.last_seq >= 0 && seq > a.last_seq)
                lost_before = seq - a.last_seq - 1;
            // seq < last_seq means the far end restarted its counter
            a.last_seq = seq;
            a.lost += lost_before;
        }
        if (a.frames == 0) {
            a.mode = mode;
            a.snr_min = a.snr_max = snr;
        } else {
            if (snr < a.snr_min) a.snr_min = snr;
            if (snr > a.snr_max) a.snr_max = snr;
        }
        a.frames++;
        a.snr_sum += snr;
        if (ber_pct >= 0) {
            a.ber_sum += ber_pct;
            a.ber_n++;
        }
        a.last_seen = time(nullptr);
        total_++;
        if (csv_) {
            char ts[32];
            time_t now = time(nullptr);
            strftime(ts, sizeof(ts), "%Y-%m-%dT%H:%M:%S", localtime(&now));
            fprintf(csv_, "%s,%s,%.2f,%.3f,%d,%d,%d\n", ts, mode.c_str(),
                    snr, ber_pct, bytes, seq, lost_before);
            fflush(csv_);
        }
    }

    bool set_csv_enabled(bool on) {
        std::lock_guard<std::mutex> lock(m_);
        if (!on) {
            if (csv_) {
                fclose(csv_);
                csv_ = nullptr;
            }
            return true;
        }
        if (csv_)
            return true;
        path_ = default_path();
        const char* home = getenv("HOME");
        if (home) {
            mkdir((std::string(home) + "/.config").c_str(), 0755);
            mkdir((std::string(home) + "/.config/modem73").c_str(), 0755);
        }
        static const char* header = "time,mode,snr_db,ber_pct,bytes,seq,lost_before\n";
        struct stat st;
        bool fresh = stat(path_.c_str(), &st) != 0 || st.st_size == 0;
        if (!fresh) {
            char first[128] = {0};
            FILE* f = fopen(path_.c_str(), "r");
            if (f) {
                if (!fgets(first, sizeof(first), f))
                    first[0] = 0;
                fclose(f);
            }
            if (std::string(first) != header) {
                rename(path_.c_str(), (path_ + ".old").c_str());
                fresh = true;
            }
        }
        csv_ = fopen(path_.c_str(), "a");
        if (!csv_)
            return false;
        if (fresh)
            fputs(header, csv_);
        return true;
    }

    bool csv_enabled() const {
        std::lock_guard<std::mutex> lock(m_);
        return csv_ != nullptr;
    }

    std::string csv_path() const {
        std::lock_guard<std::mutex> lock(m_);
        return path_.empty() ? default_path() : path_;
    }

    int total() const {
        std::lock_guard<std::mutex> lock(m_);
        return total_;
    }

    void reset() {
        std::lock_guard<std::mutex> lock(m_);
        stats_.clear();
        total_ = 0;
    }

    std::vector<PerfModeStats> snapshot() const {
        std::lock_guard<std::mutex> lock(m_);
        std::vector<PerfModeStats> out;
        out.reserve(stats_.size());
        for (const auto& kv : stats_)
            out.push_back(kv.second);
        return out;
    }

private:
    mutable std::mutex m_;
    std::map<std::string, PerfModeStats> stats_;
    FILE* csv_ = nullptr;
    std::string path_;
    int total_ = 0;
};
