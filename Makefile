CXX = g++
CC = gcc
CXXFLAGS = -std=c++17 -O3 -march=native -Wall -Wextra
LDFLAGS = -lpthread  -ltinfo -lncurses -ldl -lm -lhidapi-hidraw

# dependencies
AICODIX_DSP ?= ../dsp
AICODIX_CODE ?= ../code
MODEM_SRC ?= ../modem

INCLUDES = -I$(AICODIX_DSP) -I$(AICODIX_CODE) -I$(MODEM_SRC)

TARGET = modem73

SRCS = kiss_tnc.cc
HDRS = kiss_tnc.hh miniaudio_audio.hh rigctl_ptt.hh modem.hh tnc_ui.hh
OBJS = miniaudio.o

# defualt to build with UI, headless operations through --headless
UI_FLAGS = -DWITH_UI

.PHONY: all clean install debug help

all: $(TARGET)

miniaudio.o: miniaudio.c miniaudio.h
	$(CC) -c -O2 -o $@ miniaudio.c

$(TARGET): $(SRCS) $(HDRS) $(OBJS)
	$(CXX) $(CXXFLAGS) $(UI_FLAGS) $(INCLUDES) -o $@ $(SRCS) $(OBJS) $(LDFLAGS)

clean:
	rm -f $(TARGET) $(OBJS)

install: $(TARGET)
	install -m 755 $(TARGET) /usr/local/bin/
    $(shell cp 50-cm108-ptt.rules /etc/udev/rules.d/)

# Debug build
debug: CXXFLAGS = -std=c++17 -g -O0 -Wall -Wextra -DDEBUG
debug: $(TARGET)

# Help
help:
	@echo "MODEM73 makefile"
	@echo ""
	@echo "Targets:"
	@echo "  all      - Build modem"
	@echo "  clean    - Remove build"
	@echo "  install  - Install to /usr/local/bin"
	@echo "  debug    - Build with debug symbols"
	@echo ""
	@echo "Variables:"
	@echo "  AICODIX_DSP  - Path to aicodix/dsp (default: ../dsp)"
	@echo "  AICODIX_CODE - Path to aicodix/code (default: ../code)"
	@echo "  MODEM_SRC    - Path to modem source (default: ../modem)"
	@echo ""
	@echo "Example:"
	@echo "  make AICODIX_DSP=~/aicodix/dsp AICODIX_CODE=~/aicodix/code"
	@echo ""
	@echo "Runtime options:"
	@echo "  ./modem73            # Run with UI"
	@echo "  ./modem73  -h        # Run headless"
	@echo "  ./modem73  --headless"
