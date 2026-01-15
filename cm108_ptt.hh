#pragma once

#include <string>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cerrno>
#include <cstring>
#include <iostream>

#include <hidapi/hidapi.h>

class CM108PTT {
public:
    CM108PTT() = default;

    ~CM108PTT() {
        close();
    }

    bool open(const int gpio){
        res_ = hid_init();
        gpio_ = gpio;
        handle_ = hid_open(0x0D8C, 0x013C, NULL);
        if (!handle_) {
            std::cerr << "Failed to open CM108 PTT via USB" << std::endl;
            hid_exit();
            return false;
        }  
        return true;
    }

    void close(){
        if (handle_) {
            hid_close(handle_);
            handle_ = nullptr;
        }
        hid_exit();
    }

    void set_ptt(bool on){
        if (!handle_) return;

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
    }

private:
    int res_ = 0;
    int gpio_ = 3;  // PTT control pin GPIOX, where X should be 1,2,3,4 - GPIO3 on most devices
    const int cm108_on_[4] = {0x01, 0x02, 0x04, 0x08};
    hid_device *handle_ = nullptr;
};
