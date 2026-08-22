#ifdef WITH_HAMLIB
#include "hamlib_ptt.hh"
#include <hamlib/rig.h>
#include <cstdio>
#include <vector>
#include <cstdlib>
#include <cerrno>
#include <fcntl.h>
#include <poll.h>
#include <netdb.h>
#include <sys/socket.h>
#include <unistd.h>

static bool tcp_reachable(const std::string& host, const std::string& port, int timeout_ms) {
    struct addrinfo hints{}, *res = nullptr;

    hints.ai_family = AF_UNSPEC;

    hints.ai_socktype = SOCK_STREAM;

    if (getaddrinfo(host.c_str(), port.c_str(), &hints, &res) != 0 || !res) return false;
    bool ok = false;

    for (struct addrinfo* ai = res; ai && !ok; ai = ai->ai_next) {
        int fd = socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
        if (fd < 0) continue;
        fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);
        int rc = connect(fd, ai->ai_addr, ai->ai_addrlen);
        if (rc == 0) {
            ok = true;
        } else if (errno == EINPROGRESS) {
            struct pollfd pfd{fd, POLLOUT, 0};
            if (poll(&pfd, 1, timeout_ms) > 0) {
                int soerr = 0;
                socklen_t len = sizeof(soerr);
                if (getsockopt(fd, SOL_SOCKET, SO_ERROR, &soerr, &len) == 0 && soerr == 0) ok = true;
            }
        }
        ::close(fd);
    }
    freeaddrinfo(res);
    
    return ok;
}

std::string hamlib_list_models() {

    std::string out;
    rig_load_all_backends();
    rig_list_foreach([](const struct rig_caps* caps, void* data) -> int {
        auto* s = static_cast<std::string*>(data);
        *s += std::to_string(caps->rig_model) + "|" + (caps->mfg_name ? caps->mfg_name : "") + "|" +
              (caps->model_name ? caps->model_name : "") + "\n";
        return 1;
    }, &out);
    return out;

}

static std::string rprt(int rc) {

    return "RPRT " + std::to_string(rc) + "\n";

}

std::string HamlibPTT::command(const std::string& cmdline) {

    std::lock_guard<std::mutex> guard(mutex_);
    if (!rig_) return rprt(-RIG_EIO);
    RIG* rig = static_cast<RIG*>(rig_);
    std::string line = cmdline;
    while (!line.empty() && (line.back() == '\n' || line.back() == '\r')) line.pop_back();
    bool ext = !line.empty() && line[0] == '+';
    if (ext) line.erase(0, 1);
    std::vector<std::string> tok;
    size_t pos = 0;
    while (pos < line.size()) {
        while (pos < line.size() && line[pos] == ' ') pos++;
        size_t e = line.find(' ', pos);
        if (e == std::string::npos) e = line.size();
        if (e > pos) tok.push_back(line.substr(pos, e - pos));
        pos = e;
    }
    if (tok.empty()) return rprt(-RIG_EINVAL);
    const std::string& c = tok[0];
    std::string out;
    int rc = RIG_OK;


    if (c == "f") {

        freq_t f = 0;
        rc = rig_get_freq(rig, RIG_VFO_CURR, &f);
        if (ext) out += "get_freq:\n";
        out += (ext ? "Frequency: " : "") + std::to_string((long long)f) + "\n";
    } else if (c == "F" && tok.size() > 1) {
        rc = rig_set_freq(rig, RIG_VFO_CURR, (freq_t)atof(tok[1].c_str()));
        if (ext) out += "set_freq: " + tok[1] + "\n";
    } else if (c == "m") {
        rmode_t mode = RIG_MODE_NONE;
        pbwidth_t width = 0;
        rc = rig_get_mode(rig, RIG_VFO_CURR, &mode, &width);
        if (ext) out += "get_mode:\n";
        out += (ext ? "Mode: " : "") + std::string(rig_strrmode(mode)) + "\n";
        out += (ext ? "Passband: " : "") + std::to_string((long)width) + "\n";
    } else if (c == "M" && tok.size() > 1) {
        rmode_t mode = rig_parse_mode(tok[1].c_str());
        pbwidth_t width = tok.size() > 2 ? (pbwidth_t)atol(tok[2].c_str()) : RIG_PASSBAND_NOCHANGE;
        if (width == 0) width = RIG_PASSBAND_NOCHANGE;
        rc = rig_set_mode(rig, RIG_VFO_CURR, mode, width);
        if (ext) out += "set_mode: " + tok[1] + "\n";
    } else if (c == "l" && tok.size() > 1) {

        setting_t lvl = rig_parse_level(tok[1].c_str());
        value_t val;
        val.f = 0;
        rc = rig_get_level(rig, RIG_VFO_CURR, lvl, &val);
        if (ext) out += "get_level: " + tok[1] + "\n";
        char buf[32];
        if (RIG_LEVEL_IS_FLOAT(lvl)) snprintf(buf, sizeof(buf), "%f", val.f);
        else snprintf(buf, sizeof(buf), "%d", val.i);
        out += (ext ? "Level Value: " : "") + std::string(buf) + "\n";

    } else if (c == "L" && tok.size() > 2) {

        setting_t lvl = rig_parse_level(tok[1].c_str());
        value_t val;
        if (RIG_LEVEL_IS_FLOAT(lvl)) val.f = (float)atof(tok[2].c_str());
        else val.i = atoi(tok[2].c_str());
        rc = rig_set_level(rig, RIG_VFO_CURR, lvl, val);
        if (ext) out += "set_level: " + tok[1] + " " + tok[2] + "\n";

    } else if (c == "u" && tok.size() > 1) {

        int st = 0;
        rc = rig_get_func(rig, RIG_VFO_CURR, rig_parse_func(tok[1].c_str()), &st);
        if (ext) out += "get_func: " + tok[1] + "\n";
        out += (ext ? "Func Status: " : "") + std::to_string(st) + "\n";

    } else if (c == "U" && tok.size() > 2) {

        rc = rig_set_func(rig, RIG_VFO_CURR, rig_parse_func(tok[1].c_str()), atoi(tok[2].c_str()));
        if (ext) out += "set_func: " + tok[1] + " " + tok[2] + "\n";

    } else if (c == "G" && tok.size() > 1) {

        rc = rig_vfo_op(rig, RIG_VFO_CURR, rig_parse_vfo_op(tok[1].c_str()));
        if (ext) out += "vfo_op: " + tok[1] + "\n";

    } else if (c == "t") {

        ptt_t p = RIG_PTT_OFF;
        rc = rig_get_ptt(rig, RIG_VFO_CURR, &p);
        if (ext) out += "get_ptt:\n";
        out += (ext ? "PTT: " : "") + std::to_string((int)p) + "\n";
    } else if (c == "T" && tok.size() > 1) {

        rc = rig_set_ptt(rig, RIG_VFO_CURR, atoi(tok[1].c_str()) ? RIG_PTT_ON : RIG_PTT_OFF);
        if (ext) out += "set_ptt: " + tok[1] + "\n";

    } else {
        rc = -RIG_ENAVAIL;
    }
    if (rc != RIG_OK) return (ext ? out : std::string()) + rprt(rc);
    return out + rprt(0);
}



HamlibPTT::~HamlibPTT() { close(); } 



bool HamlibPTT::open(int model, const std::string& device, int baud, std::string& err) {

    std::lock_guard<std::mutex> guard(mutex_);
    close_locked();
    fprintf(stderr, "hamlib: init model %d\n", model);
    rig_set_debug(RIG_DEBUG_NONE);
    RIG* rig = rig_init(model);
    fprintf(stderr, "hamlib: rig_init %s\n", rig ? "ok" : "failed");
    if (!rig) {
        err = "unknown hamlib rig model " + std::to_string(model);
        return false;
    }
    if (!device.empty())
        rig_set_conf(rig, rig_token_lookup(rig, "rig_pathname"), device.c_str());
    if (baud > 0)
        rig_set_conf(rig, rig_token_lookup(rig, "serial_speed"), std::to_string(baud).c_str());
    size_t colon = device.rfind(':');
    bool looks_usb_key = device.size() > 10 && device[4] == ':' && device[9] == ':';
    if (colon != std::string::npos && device.find('/') == std::string::npos && !looks_usb_key) {
        std::string host = device.substr(0, colon);
        std::string port = device.substr(colon + 1);
        if (!tcp_reachable(host, port, 4000)) {
            err = "cannot reach " + device;
            rig_cleanup(rig);
            return false;
        }
    }

    rig_set_conf(rig, rig_token_lookup(rig, "ptt_type"), "RIG");

    rig_set_conf(rig, rig_token_lookup(rig, "timeout"), "2000");

    rig_set_conf(rig, rig_token_lookup(rig, "retry"), "1");

    fprintf(stderr, "hamlib: opening %s\n", device.c_str());

    int rc = rig_open(rig);

    fprintf(stderr, "hamlib: rig_open rc=%d\n", rc);

    if (rc != RIG_OK) {

        err = rigerror(rc);
        rig_cleanup(rig);
        return false;

    }
    rig_ = rig;
    connected_ = true;
    return true;
}

bool HamlibPTT::set_ptt(bool on) {

    std::lock_guard<std::mutex> guard(mutex_);
    if (!rig_) return false;
    int rc = rig_set_ptt(static_cast<RIG*>(rig_), RIG_VFO_CURR, on ? RIG_PTT_ON : RIG_PTT_OFF);
    if (rc != RIG_OK) {
        fprintf(stderr, "hamlib: set_ptt %d failed: %s\n", on ? 1 : 0, rigerror(rc));
        last_error_ = rigerror(rc);
        return false;
    }
    return true;

}

bool HamlibPTT::get_freq(double& hz) {

    std::lock_guard<std::mutex> guard(mutex_);
    if (!rig_) return false;
    freq_t f = 0;
    if (rig_get_freq(static_cast<RIG*>(rig_), RIG_VFO_CURR, &f) != RIG_OK) return false;
    hz = (double)f;
    return true;

}

void HamlibPTT::close() {

    std::lock_guard<std::mutex> guard(mutex_);
    close_locked();

}

void HamlibPTT::close_locked() {

    if (rig_) {
        rig_close(static_cast<RIG*>(rig_));
        rig_cleanup(static_cast<RIG*>(rig_));
        rig_ = nullptr;
    }
    connected_ = false;

}
#endif
