#pragma once

#include <string>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <cerrno>
#include <cstring>

enum class PTTLine {
    DTR = 0,
    RTS = 1,
    BOTH = 2
};

class SerialPTT {
public:
    SerialPTT() = default;
    
    ~SerialPTT() {
        close();
    }

    
    
    bool open(const std::string& port, PTTLine line = PTTLine::RTS, 
              bool invert_dtr = false, bool invert_rts = false) {
        close();
        
        port_ = port;
        line_ = line;
        invert_dtr_ = invert_dtr;
        invert_rts_ = invert_rts;
        
        fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0) {
            last_error_ = std::string("Failed to open ") + port + ": " + strerror(errno);
            return false;
        }
        
        struct termios tty;
        if (tcgetattr(fd_, &tty) != 0) {
            last_error_ = "Failed to get terminal attributes";
            close();
            return false;
        }
        
        cfmakeraw(&tty);
        tcsetattr(fd_, TCSANOW, &tty);
        



        ptt_off();
        



        open_ = true;
        return true;
    }

    
    void close() {
        if (fd_ >= 0) {
            ptt_off();
            ::close(fd_);
            fd_ = -1;
        }
        open_ = false;
    }

    
    bool is_open() const { return open_; }
    
    const std::string& last_error() const { return last_error_; }
    
    bool ptt_on() {
        if (fd_ < 0) return false;

        int flags;
        if (ioctl(fd_, TIOCMGET, &flags) < 0) return false;
        




        if (line_ == PTTLine::DTR || line_ == PTTLine::BOTH) {
            if (invert_dtr_) {
                flags &= ~TIOCM_DTR;  // clear DTR 
            } else {
                flags |= TIOCM_DTR;   // set DTR 
            }
        }
        



        if (line_ == PTTLine::RTS || line_ == PTTLine::BOTH) {
            if (invert_rts_) {
                flags &= ~TIOCM_RTS;  // clear RTS
            } else {
                flags |= TIOCM_RTS;   // set RTS
            }
        }



        return ioctl(fd_, TIOCMSET, &flags) == 0;
    }

    bool ptt_off() {
        if (fd_ < 0) return true;

        int flags;
        if (ioctl(fd_, TIOCMGET, &flags) < 0) return false;
        


        if (line_ == PTTLine::DTR || line_ == PTTLine::BOTH) {
            if (invert_dtr_) {
                flags |= TIOCM_DTR;   // set DTR
            } else {
                flags &= ~TIOCM_DTR;  // clear DTR 
            }
        }
        


        if (line_ == PTTLine::RTS || line_ == PTTLine::BOTH) {
            if (invert_rts_) {
                flags |= TIOCM_RTS;   // set RTS
            } else {
                flags &= ~TIOCM_RTS;  // clear RTS
            }
        }



        return ioctl(fd_, TIOCMSET, &flags) == 0;
    }

    bool reconnect() {
        std::string saved_port = port_;
        PTTLine saved_line = line_;
        bool saved_invert_dtr = invert_dtr_;
        bool saved_invert_rts = invert_rts_;
        
        close();
        return open(saved_port, saved_line, saved_invert_dtr, saved_invert_rts);
    }


private:
    int fd_ = -1;
    bool open_ = false;
    std::string port_;
    PTTLine line_ = PTTLine::RTS;
    bool invert_dtr_ = false;
    bool invert_rts_ = false;
    std::string last_error_;
};
