#pragma once

#include <string>
#include <vector>
#include <cstdint>
#include <cstdlib>
#include <cwchar>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cerrno>
#include <cstring>
#include <iostream>

#include <hidapi/hidapi.h>

class CM108PTT {
public:
    struct CM108Device {
        std::string path;     // hid_device_info::path
        uint16_t vid = 0;
        uint16_t pid = 0;
        std::string chip;     // friendly chip name from the variant table
        std::string product;  // USB product string 
        std::string serial;   // USB serial number 
    };

    CM108PTT() = default;

    ~CM108PTT() {
        close();
    }

    // All listed chips share the CM108 GPIO HID command set
    static std::vector<CM108Device> enumerate() {
        struct Variant { uint16_t vid, pid; const char* name; };
        static const Variant variants[] = {
            {0x0D8C, 0x0008, "CM108/109/119 (legacy)"},
            {0x0D8C, 0x0009, "CM108/109/119 (legacy)"},
            {0x0D8C, 0x000A, "CM108/109/119 (legacy)"},
            {0x0D8C, 0x000B, "CM108/109/119 (legacy)"},
            {0x0D8C, 0x000C, "CM108/109/119 (legacy)"},
            {0x0D8C, 0x000D, "CM108/109/119 (legacy)"},
            {0x0D8C, 0x000E, "CM108/109/119 (legacy)"},
            {0x0D8C, 0x000F, "CM108/109/119 (legacy)"},
            {0x0D8C, 0x0139, "CM108AH"},
            {0x0D8C, 0x013A, "CM119A"},
            {0x0D8C, 0x0013, "CM119B"},
            {0x0D8C, 0x0012, "CM108B"},
            {0x1209, 0x7388, "AIOC"},
        };

        std::vector<CM108Device> out;
        hid_init();
        hid_device_info* devs = hid_enumerate(0x0, 0x0);
        for (hid_device_info* d = devs; d; d = d->next) {
            for (const auto& v : variants) {
                if (d->vendor_id != v.vid || d->product_id != v.pid) continue;
                CM108Device e;
                e.path = d->path ? d->path : "";
                e.vid = d->vendor_id;
                e.pid = d->product_id;
                e.chip = v.name;
                e.product = wstr_to_utf8(d->product_string);
                e.serial = wstr_to_utf8(d->serial_number);
                out.push_back(std::move(e));
                break;
            }
        }
        hid_free_enumeration(devs);
        return out;
    }
    

    bool open(const int gpio, const std::string& selector = ""){


        gpio_ = gpio;
        auto devices = enumerate();
        if (devices.empty()) {
            std::cerr << "No CM108-compatible PTT device found" << std::endl;
            return false;
        }

        const CM108Device* chosen = nullptr;
        if (selector.empty()) {
            chosen = &devices.front();
        } else {
            for (const auto& d : devices) {
                if (d.serial == selector || d.path == selector) { chosen = &d; break; }
            }
            if (!chosen) {
                std::cerr << "CM108 device '" << selector << "' not found" << std::endl;
                return false;
            }
        }

        handle_ = hid_open_path(chosen->path.c_str());
        if (!handle_) {
            std::cerr << "Failed to open CM108 PTT (" << chosen->chip << ")" << std::endl;
            return false;
        }
        return true;
    }

    void close(){
        if (handle_) {
            set_ptt(false);
            hid_close(handle_);
            handle_ = nullptr;
        }
        hid_exit();
    }

    bool set_ptt(bool on){
        if (!handle_) return false;
        if (gpio_ < 1 || gpio_ > 4) return false;

        unsigned char buf[5];
        buf[0] = 0x00;
        buf[1] = 0x00;

        if (on){
            buf[2] = cm108_on_[gpio_-1];
            buf[3] = cm108_on_[gpio_-1];
        } else {
            buf[2] = 0x00;
            buf[3] = 0x00;
        }
        buf[4] = 0x00;

        res_ = hid_write(handle_, buf, 5);
        return res_ >= 0;
    }

private:
    static std::string wstr_to_utf8(const wchar_t* ws) {
        if (!ws) return {};
        char buf[256];
        size_t n = wcstombs(buf, ws, sizeof(buf) - 1);
        if (n == static_cast<size_t>(-1)) return {};
        buf[n] = '\0';
        return std::string(buf);
    }

    int res_ = 0;
    int gpio_ = 3;  // PTT control pin GPIOX, where X should be 1,2,3,4 - GPIO3 on most devices
    const int cm108_on_[4] = {0x01, 0x02, 0x04, 0x08};
    hid_device *handle_ = nullptr;
};
