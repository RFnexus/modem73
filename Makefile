CXX = g++
CC = gcc
CXXFLAGS = -std=c++17 -O3 -march=native -Wall -Wextra
LDFLAGS = -lpthread  -ltinfo -lncurses -ldl -lm

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

# Optional CM108 PTT support requires libhidapi-dev
HIDAPI_CFLAGS := $(shell pkg-config --cflags hidapi-hidraw 2>/dev/null || pkg-config --cflags hidapi-libusb 2>/dev/null || pkg-config --cflags hidapi 2>/dev/null)
HIDAPI_LIBS := $(shell pkg-config --libs hidapi-hidraw 2>/dev/null || pkg-config --libs hidapi-libusb 2>/dev/null || pkg-config --libs hidapi 2>/dev/null)

ifneq ($(HIDAPI_LIBS),)
    $(info CM108 PTT support: enabled (found hidapi))
    CM108_FLAGS = -DWITH_CM108
    CXXFLAGS += $(HIDAPI_CFLAGS)
    LDFLAGS += $(HIDAPI_LIBS)
else
    $(info CM108 PTT support: disabled (install libhidapi-dev to enable))
    CM108_FLAGS =
endif

.PHONY: all clean install debug help

all: $(TARGET)

miniaudio.o: miniaudio.c miniaudio.h
	$(CC) -c -O2 -o $@ miniaudio.c

$(TARGET): $(SRCS) $(HDRS) $(OBJS)
	$(CXX) $(CXXFLAGS) $(UI_FLAGS) $(CM108_FLAGS) $(INCLUDES) -o $@ $(SRCS) $(OBJS) $(LDFLAGS)
ifneq ($(HIDAPI_LIBS),)
	@echo ""
	@echo "CM108 PTT support enabled. To allow non-root access, install udev rules:"
	@echo "  sudo cp misc/50-cm108-ptt.rules /etc/udev/rules.d/"
	@echo "  sudo udevadm control --reload-rules"
endif

clean:
	rm -f $(TARGET) $(OBJS)

install: $(TARGET)
	install -m 755 $(TARGET) /usr/local/bin/
ifneq ($(HIDAPI_LIBS),)
	@if [ -f misc/50-cm108-ptt.rules ]; then \
		cp misc/50-cm108-ptt.rules /etc/udev/rules.d/ 2>/dev/null || \
		echo "Note: Run 'sudo cp misc/50-cm108-ptt.rules /etc/udev/rules.d/' for CM108 udev rules"; \
	fi
endif

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
	@echo "Optional features:"
	@echo "  CM108 PTT    - Requires libhidapi-dev (auto-detected)"
	@echo ""
	@echo "Example:"
	@echo "  make AICODIX_DSP=~/aicodix/dsp AICODIX_CODE=~/aicodix/code"
	@echo ""
	@echo "Runtime options:"
	@echo "  ./modem73            # Run with UI"
	@echo "  ./modem73  -h        # Run headless"
	@echo "  ./modem73  --headless"
