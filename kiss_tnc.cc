#include "kiss_tnc_impl.hh"
#ifdef WITH_UI
#include "tnc_ui.hh"
#endif

void signal_handler(int /*sig*/) {
    std::cerr << "\nShutting down..." << std::endl;
    g_running = false;
}

static const char* const MOD_OPTS[] = {
    "BPSK", "QPSK", "8PSK", "QAM16", "QAM64", "QAM256", "QAM1024", "QAM4096"
};
static const int N_MOD = sizeof(MOD_OPTS) / sizeof(*MOD_OPTS);
static const char* const RATE_OPTS[] = {"1/2", "2/3", "3/4", "5/6", "1/4", "1/2x2", "1/4x2"};
static const int N_RATE = sizeof(RATE_OPTS) / sizeof(*RATE_OPTS);
static const char* const MODEM_TYPE_OPTS[] = {"ofdm", "mfsk", "robust"};
static const char* const CSMA_MODE_OPTS[] = {"threshold", "sync", "ranked"};
static const char* const CSMA_BAND_OPTS[] = {"hf", "vhf"};

// Load key=value settings from path into config when --config is passed
static bool apply_settings_file(const std::string& path, TNCConfig& config,
                                const std::set<std::string>& cli_set) {
    FILE* f = fopen(path.c_str(), "r");
    if (!f) return false;

    auto take = [&](const char* k) {
        return cli_set.find(k) == cli_set.end();
    };

    char line[512];
    while (fgets(line, sizeof(line), f)) {
        if (line[0] == '#' || line[0] == '\n') continue;
        char key[64], value[384];
        if (sscanf(line, "%63[^=]=%383[^\n]", key, value) != 2) continue;

        if (!strcmp(key, "callsign") && take(key)) config.callsign = value;
        else if (!strcmp(key, "modem_type") && take(key)) {
            int v = atoi(value);
            if (v >= 0 && v <= 2) config.modem_type = v;
        }
        else if (!strcmp(key, "mfsk_mode") && take(key)) {
            int v = atoi(value);
            if (v >= 0 && v <= 3) config.mfsk_mode = v;
        }
        else if (!strcmp(key, "robust_mode") && take(key)) {
            int v = atoi(value);
            if (v >= 0 && v < ROBUST_MODE_COUNT) config.robust_mode = v;
        }
        else if (!strcmp(key, "perf_log") && take(key)) config.perf_log = atoi(value) != 0;
        else if (!strcmp(key, "hamlib_model") && take(key)) config.hamlib_model = atoi(value);
        else if (!strcmp(key, "hamlib_device") && take(key)) config.hamlib_device = value;
        else if (!strcmp(key, "hamlib_baud") && take(key)) config.hamlib_baud = atoi(value);
        else if (!strcmp(key, "modulation") && take(key)) {
            int idx = atoi(value);
            if (idx >= 0 && idx < N_MOD) config.modulation = MOD_OPTS[idx];
        }
        else if (!strcmp(key, "code_rate") && take(key)) {
            int idx = atoi(value);
            if (idx >= 0 && idx < N_RATE) config.code_rate = RATE_OPTS[idx];
        }
        else if (!strcmp(key, "short_frame") && take("frame_size")) config.frame_size = atoi(value) != 0 ? 0 : 1;
        else if (!strcmp(key, "frame_size") && take(key)) {
            int v = atoi(value);
            if (v >= 0 && v <= 3) config.frame_size = v;
        }
        else if (!strcmp(key, "center_freq") && take(key)) config.center_freq = 1500;
        else if (!strcmp(key, "rx_filter_enabled") && take(key)) config.rx_filter_enabled = atoi(value) != 0;
        else if (!strcmp(key, "postamble") && take(key)) config.postamble = atoi(value) != 0;
        else if (!strcmp(key, "mfsk_rx_enabled") && take(key)) config.mfsk_rx_enabled = atoi(value) != 0;
        else if (!strcmp(key, "ofdm_rx_enabled") && take(key)) config.ofdm_rx_enabled = atoi(value) != 0;
        else if (!strcmp(key, "robust_rx_enabled") && take(key)) config.robust_rx_enabled = atoi(value) != 0;
        else if (!strcmp(key, "csma_enabled") && take(key)) config.csma_enabled = atoi(value) != 0;
        else if (!strcmp(key, "csma_sync_only") && take(key)) config.csma_sync_only = atoi(value) != 0;
        else if (!strcmp(key, "csma_fast_floor") && take(key)) config.csma_fast_floor = atoi(value) != 0;
        else if (!strcmp(key, "csma_ranked") && take(key)) config.csma_ranked = atoi(value) != 0;
        else if (!strcmp(key, "csma_band") && take(key)) config.csma_band = atoi(value) != 0 ? 1 : 0;
        else if (!strcmp(key, "carrier_threshold_db") && take(key)) {
            float v = atof(value);
            if (std::isfinite(v) && v >= -80.0f && v <= 0.0f)
                config.carrier_threshold_db = v;
        }
        else if (!strcmp(key, "slot_time_ms") && take(key)) config.slot_time_ms = atoi(value);
        else if (!strcmp(key, "csma_quiet_ms") && take(key)) config.csma_quiet_ms = atoi(value);
        else if (!strcmp(key, "csma_cw") && take(key)) config.csma_cw = atoi(value);
        else if (!strcmp(key, "csma_responder_dither") && take(key)) config.csma_responder_dither = atoi(value);
        else if (!strcmp(key, "csma_burst") && take(key)) config.csma_burst = atoi(value);
        else if (!strcmp(key, "tx_lead_tone") && take(key)) config.tx_lead_tone = atoi(value) != 0;
        else if (!strcmp(key, "p_persistence") && take(key)) config.p_persistence = atoi(value);
        else if (!strcmp(key, "fragmentation_enabled") && take(key)) config.fragmentation_enabled = atoi(value) != 0;
        else if (!strcmp(key, "tx_blanking_enabled") && take(key)) config.tx_blanking_enabled = atoi(value) != 0;
        else if (!strcmp(key, "tx_drive") && take(key)) {
            float v = (float)atof(value);
            if (std::isfinite(v) && v >= 0.05f && v <= 1.0f) config.tx_drive = v;
        }
        else if (!strcmp(key, "audio_input") && take(key)) config.audio_input_device = value;
        else if (!strcmp(key, "audio_output") && take(key)) config.audio_output_device = value;
        else if (!strcmp(key, "audio_device")) {
            if (take("audio_input")) config.audio_input_device = value;
            if (take("audio_output")) config.audio_output_device = value;
        }
        else if (!strcmp(key, "ptt_type") && take(key)) config.ptt_type = static_cast<PTTType>(atoi(value));
        else if (!strcmp(key, "vox_tone_freq") && take(key)) {
            int v = atoi(value);
            if (v >= 300 && v <= 3000) config.vox_tone_freq = v;
        }
        else if (!strcmp(key, "tx_delay_ms") && take(key)) {
            int v = atoi(value);
            if (v >= 250 && v <= 2500) config.tx_delay_ms = v;
        }
        else if (!strcmp(key, "beacon_interval_s") && take(key)) {
            int v = atoi(value);
            if (v >= 45 && v <= 90) config.beacon_interval_s = v;
        }
        else if (!strcmp(key, "vox_lead_ms") && take(key)) {
            int v = atoi(value);
            if (v >= 50 && v <= 2000) config.vox_lead_ms = v;
        }
        else if (!strcmp(key, "vox_tail_ms") && take(key)) {
            int v = atoi(value);
            if (v >= 50 && v <= 2000) config.vox_tail_ms = v;
        }
        else if (!strcmp(key, "com_port") && take(key)) config.com_port = value;
        else if (!strcmp(key, "com_ptt_line") && take(key)) {
            int v = atoi(value);
            if (v >= 0 && v <= 2) config.com_ptt_line = v;
        }
        else if (!strcmp(key, "com_invert_dtr") && take(key)) config.com_invert_dtr = atoi(value) != 0;
        else if (!strcmp(key, "com_invert_rts") && take(key)) config.com_invert_rts = atoi(value) != 0;
#ifdef WITH_CM108
        else if (!strcmp(key, "cm108_gpio") && take(key)) config.cm108_gpio = atoi(value);
        else if (!strcmp(key, "cm108_device") && take(key)) config.cm108_device = value;
#endif
        else if (!strcmp(key, "port") && take(key)) {
            int v = atoi(value);
            if (v >= 1 && v <= 65535) config.port = v;
        }
        else if (!strcmp(key, "bind_address") && take(key)) config.bind_address = value;
        else if (!strcmp(key, "control_bind_address") && take(key)) config.control_bind_address = value;
    }

    fclose(f);
    return true;
}

static bool arg_ieq(const std::string& a, const char* b) {
    size_t i = 0;
    for (; i < a.size() && b[i]; ++i)
        if (toupper((unsigned char)a[i]) != toupper((unsigned char)b[i]))
            return false;
    return i == a.size() && b[i] == '\0';
}

template<typename T, size_t N>
static int arg_index(const std::string& value, T (&names)[N]) {
    for (size_t i = 0; i < N; ++i)
        if (arg_ieq(value, names[i])) return (int)i;
    return -1;
}

template<typename T, size_t N>
static void print_arg_options(const char* flag, const std::string& value,
                              T (&names)[N]) {
    std::cerr << "Invalid " << flag << " '" << value << "', expected one of:";
    for (size_t i = 0; i < N; ++i)
        std::cerr << ' ' << names[i];
    std::cerr << std::endl;
}

static int clamp_arg(const char* flag, const char* value, int lo, int hi) {
    int v = std::atoi(value);
    int c = std::min(hi, std::max(lo, v));
    if (c != v)
        std::cerr << "Warning: " << flag << ' ' << v << " out of range "
                  << lo << ".." << hi << ", using " << c << std::endl;
    return c;
}

static float clamp_arg_f(const char* flag, const char* value, float lo, float hi) {
    float v = (float)std::atof(value);
    if (!std::isfinite(v)) v = lo;
    float c = std::min(hi, std::max(lo, v));
    if (c != v)
        std::cerr << "Warning: " << flag << ' ' << v << " out of range "
                  << lo << ".." << hi << ", using " << c << std::endl;
    return c;
}

void print_help(const char* prog) {
    std::cerr << "MODEM73\n\n"
              << "Usage: " << prog << " [options]\n"
              << "\nGeneral:\n"
#ifdef WITH_UI
              << "  -h, --headless          Run without the TUI\n"
#endif
              << "  -v, --verbose           Verbose output\n"
              << "      --debug             Log heard station IDs and population tracking\n"
              << "      --config [FILE]     Load options from FILE\n"
              << "                          (defaults to ~/.config/modem73/settings)\n"
              << "      --list-audio        List available audio devices and exit\n"
#ifdef WITH_CM108
              << "      --list-cm108        List CM108-compatible devices and exit\n"
#endif
              << "      --help              Show this help\n"
              << "\nNetwork:\n"
              << "  -p, --port PORT         KISS TCP port (default: 8001)\n"
              << "      --bind ADDR         KISS bind address (default: 0.0.0.0)\n"
              << "      --control-port PORT Control port (default: 8073, 0 to disable)\n"
              << "      --control-bind ADDR Control port bind address (default: 127.0.0.1)\n"
              << "\nAudio:\n"
              << "  -d, --device DEV        Audio device for both capture and playback\n"
              << "      --input-device DEV  Audio capture device\n"
              << "      --output-device DEV Audio playback device\n"
              << "      --tx-level PCT      TX audio output level, 5-100 (default: 100)\n"
              << "\nModem:\n"
              << "  -c, --callsign CALL     Callsign (default: N0CALL)\n"
              << "      --modem TYPE        ofdm, mfsk or robust (default: ofdm)\n"
              << "  -m, --modulation MOD    OFDM modulation, one of:\n"
              << "                          BPSK QPSK 8PSK QAM16 QAM64 QAM256 QAM1024 QAM4096\n"
              << "                          (default: QPSK)\n"
              << "  -r, --rate RATE         OFDM code rate, one of:\n"
              << "                          1/2 2/3 3/4 5/6 1/4 (default: 1/2)\n"
              << "                          \n"
              << "      --short             OFDM short frames\n"
              << "      --normal            OFDM normal frames (default)\n"
              << "      --long              OFDM long frames\n"
              << "      --micro             OFDM QB micro burst (WIP)\n"
              << "      --postamble         Send a postamble after each OFDM frame\n"
              << "      --no-postamble      Do not send a postamble (default)\n"
              << "      --mfsk-mode MODE    MFSK-8, MFSK-16, MFSK-32 or MFSK-32R\n"
              << "                          (implies --modem mfsk)\n"
              << "      --robust-mode MODE  RDM-1200 RDM-800 RDM-600 RDM-300 RDMN-300 RDMN-150\n"
              << "                          suffix S selects short frames (e.g. RDM-600S),\n"
              << "                          RDM-QB is the 32 B micro burst\n"
              << "                          (implies --modem robust)\n"
              << "      --no-rxfilter       Disable RX bandpass in front of the OFDM decoder\n"
              << "\nRX decoders:\n"
              << "      --no-mfsk-rx        Disable the 3 always-on MFSK RX decoders to save CPU\n"
              << "                          (ignored while an MFSK mode is selected for TX)\n"
              << "      --no-ofdm-rx        Disable the OFDM RX decoder to save CPU\n"
              << "                          (ignored while an OFDM mode is selected for TX)\n"
              << "      --no-robust-rx      Disable the 2 ROBUST (RDM) RX decoders to save CPU\n"
              << "                          (ignored while a ROBUST mode is selected for TX)\n"
              << "\nPTT:\n"
              << "      --ptt TYPE          PTT type: none, rigctl, vox, com"
#ifdef WITH_CM108
              << ", cm108"
#endif
              << " (default: rigctl)\n"
              << "      --rigctl HOST:PORT  Rigctld address (default: localhost:4532,\n"
              << "                          implies --ptt rigctl)\n"
              << "      --com-port PORT     Serial port for COM PTT (default: /dev/ttyUSB0)\n"
              << "      --hamlib-model N    Hamlib rig model number for HAMLIB PTT\n"
              << "      --hamlib-device DEV Serial port or host:port for HAMLIB PTT\n"
              << "      --hamlib-baud BAUD  Serial speed for HAMLIB PTT (0 = rig default)\n"
              << "      --com-line LINE     COM PTT line: dtr, rts, both, -dtr, -rts, -both\n"
              << "                          (prefix '-' inverts polarity; default: rts)\n"
              << "      --vox-freq HZ       VOX tone frequency (default: 1200)\n"
              << "      --vox-lead MS       VOX lead time in ms (default: 550)\n"
              << "      --vox-tail MS       VOX tail time in ms (default: 500)\n"
#ifdef WITH_CM108
              << "      --cm108-gpio N      CM108 GPIO pin for PTT (default: 3)\n"
              << "      --cm108-device SPEC CM108 device to use: serial or USB path\n"
              << "                          (default: first compatible device)\n"
#endif
              << "      --ptt-delay MS      PTT delay before TX (default: 50)\n"
              << "      --ptt-tail MS       PTT tail after TX (default: 50)\n"
              << "      --tx-delay MS       TXDelay ahead of the frame, 250-2500 (default: 500)\n"
              << "\nCSMA:\n"
              << "      --no-csma           Disable CSMA, transmit as soon as a packet is queued\n"
              << "      --csma-mode MODE    threshold, sync or ranked (default: threshold)\n"
              << "                          threshold: busy = any audio over --csma-threshold\n"
              << "                          sync:      busy = a real modem signal only\n"
              << "                          ranked:    sync plus stations take timed turns\n"
              << "      --csma-band BAND    Timing profile: hf or vhf (default: hf)\n"
              << "      --csma-preset NAME  bench, relaxed, moderate or busy; sets quiet, window,\n"
              << "                          slot, burst, dither and lead tone for the chosen band\n"
              << "      --csma-threshold DB Carrier sense threshold, threshold mode (default: -30)\n"
              << "      --csma-slot MS      Slot time in ms (default: 500)\n"
              << "      --csma-quiet MS     Idle time before contending (default: 0 = auto)\n"
              << "      --csma-cw N         Contention window in slots (default: 8)\n"
              << "      --csma-dither MS    Responder delay spread from callsign hash\n"
              << "                          (default: 250, 0 = off)\n"
              << "      --csma-burst N      Packets sent per channel acquisition, 1-4 (default: 2)\n"
              << "      --lead-tone         Send tone during TXDelay so others detect keyup (default)\n"
              << "      --no-lead-tone      Send silence during TXDelay instead\n"
              << "      --fast-floor        Drop the noise floor estimate quickly, sync mode (default)\n"
              << "      --no-fast-floor     Keep the slower noise floor estimate\n"
              << "      --beacon-interval S Presence tone interval in ranked mode, 45-90 (default: 45)\n"
              << "\nFragmentation:\n"
              << "      --frag              Enable packet fragmentation/reassembly\n"
              << "      --no-frag           Disable fragmentation (default)\n"
              << "\nTX blanking:\n"
              << "      --tx-blank          Suppress the decoder during TX (default always on with CSMA)\n"
              << "      --no-tx-blank       Disable TX blanking (only takes effect with --no-csma)\n"
              << "\nSettings are saved to ~/.config/modem73/settings\n";
}

int main(int argc, char** argv) {
    std::cerr << "MODEM73 build " << __DATE__ << " " << __TIME__ << std::endl;

    TNCConfig config;

    // Track which settings were explicitly set on CLI
    std::set<std::string> cli_set;
    bool cli_control_port = false;
    bool cli_config = false;

    // Parse arguments
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        
        if (arg == "--help") {
            print_help(argv[0]);
            return 0;
        } else if (arg == "--list-audio") {
            std::cout << "Input devices:\n";
            auto input_devices = MiniAudio::list_capture_devices();
            for (const auto& dev : input_devices) {
                std::cout << "  " << dev.second << "\n";
            }
            std::cout << "\nOutput devices:\n";
            auto output_devices = MiniAudio::list_playback_devices();
            for (const auto& dev : output_devices) {
                std::cout << "  " << dev.second << "\n";
            }
            return 0;
#ifdef WITH_CM108
        } else if (arg == "--list-cm108") {
            auto devices = CM108PTT::enumerate();
            if (devices.empty()) {
                std::cout << "No CM108-compatible devices found\n";
            } else {
                std::cout << "CM108-compatible devices:\n";
                for (const auto& d : devices) {
                    std::cout << "  " << d.chip;
                    if (!d.product.empty()) std::cout << " [" << d.product << "]";
                    std::cout << "\n    serial: " << (d.serial.empty() ? "(none)" : d.serial)
                              << "\n    path:   " << d.path << "\n";
                }
            }
            return 0;
#endif
        } else if (arg == "-v" || arg == "--verbose") {
            g_verbose = true;
        } else if (arg == "--debug") {
            g_debug = true;
        } else if (arg == "-h" || arg == "--headless") {
#ifdef WITH_UI
            g_use_ui = false;
#endif
        } else if ((arg == "-p" || arg == "--port") && i + 1 < argc) {
            config.port = std::min(65535, std::max(1, std::atoi(argv[++i])));
            cli_set.insert("port");
        } else if (arg == "--bind" && i + 1 < argc) {
            config.bind_address = argv[++i];
            cli_set.insert("bind_address");
        } else if (arg == "--control-bind" && i + 1 < argc) {
            config.control_bind_address = argv[++i];
            cli_set.insert("control_bind_address");
        } else if (arg == "--control-port" && i + 1 < argc) {
            config.control_port = std::atoi(argv[++i]);
            cli_control_port = true;
        } else if (arg == "--config") {
            cli_config = true;
            if (i + 1 < argc && argv[i + 1][0] != '-') {
                config.config_file = argv[++i];
            } else {
                const char* home = getenv("HOME");
                if (home) {
                    config.config_file = std::string(home) + "/.config/modem73/settings";
                }
            }
        } else if ((arg == "-d" || arg == "--device") && i + 1 < argc) {
            // Set both input and output to same device
            config.audio_input_device = argv[++i];
            config.audio_output_device = config.audio_input_device;
            cli_set.insert("audio_input");
            cli_set.insert("audio_output");
        } else if (arg == "--input-device" && i + 1 < argc) {
            config.audio_input_device = argv[++i];
            cli_set.insert("audio_input");
        } else if (arg == "--output-device" && i + 1 < argc) {
            config.audio_output_device = argv[++i];
            cli_set.insert("audio_output");
        } else if (arg == "--tx-level" && i + 1 < argc) {
            config.tx_drive = clamp_arg("--tx-level", argv[++i], 5, 100) / 100.0f;
            cli_set.insert("tx_drive");
        } else if ((arg == "-c" || arg == "--callsign") && i + 1 < argc) {
            config.callsign = argv[++i];
            for (auto& ch : config.callsign)
                ch = toupper((unsigned char)ch);
            cli_set.insert("callsign");
        } else if (arg == "--modem" && i + 1 < argc) {
            std::string value = argv[++i];
            int idx = arg_index(value, MODEM_TYPE_OPTS);
            if (idx < 0) {
                print_arg_options("--modem", value, MODEM_TYPE_OPTS);
                return 1;
            }
            config.modem_type = idx;
            cli_set.insert("modem_type");
        } else if (arg == "--mfsk-mode" && i + 1 < argc) {
            std::string value = argv[++i];
            int idx = arg_index(value, MFSK_MODE_NAMES);
            if (idx < 0) {
                print_arg_options("--mfsk-mode", value, MFSK_MODE_NAMES);
                return 1;
            }
            config.mfsk_mode = idx;
            config.modem_type = 1;
            cli_set.insert("mfsk_mode");
            cli_set.insert("modem_type");
        } else if (arg == "--robust-mode" && i + 1 < argc) {
            std::string value = argv[++i];
            int idx = arg_index(value, ROBUST_MODE_NAMES);
            if (idx < 0) {
                print_arg_options("--robust-mode", value, ROBUST_MODE_NAMES);
                return 1;
            }
            config.robust_mode = idx;
            config.modem_type = 2;
            cli_set.insert("robust_mode");
            cli_set.insert("modem_type");
        } else if ((arg == "-m" || arg == "--modulation") && i + 1 < argc) {
            std::string value = argv[++i];
            int idx = arg_index(value, MOD_OPTS);
            if (idx < 0) {
                print_arg_options("--modulation", value, MOD_OPTS);
                return 1;
            }
            config.modulation = MOD_OPTS[idx];
            cli_set.insert("modulation");
        } else if ((arg == "-r" || arg == "--rate") && i + 1 < argc) {
            std::string value = argv[++i];
            int idx = arg_index(value, RATE_OPTS);
            if (idx < 0) {
                print_arg_options("--rate", value, RATE_OPTS);
                return 1;
            }
            config.code_rate = RATE_OPTS[idx];
            cli_set.insert("code_rate");
        } else if (arg == "--postamble") {
            config.postamble = true;
            cli_set.insert("postamble");
        } else if (arg == "--no-postamble") {
            config.postamble = false;
            cli_set.insert("postamble");
        } else if (arg == "--short") {
            config.frame_size = 0;
            cli_set.insert("frame_size");
        } else if (arg == "--normal") {
            config.frame_size = 1;
            cli_set.insert("frame_size");
        } else if (arg == "--long") {
            config.frame_size = 2;
            cli_set.insert("frame_size");
        } else if (arg == "--micro") {
            config.frame_size = 3;
            cli_set.insert("frame_size");
        } else if (arg == "--no-rxfilter") {
            config.rx_filter_enabled = false;
            cli_set.insert("rx_filter_enabled");
        } else if (arg == "--rigctl" && i + 1 < argc) {
            config.ptt_type = PTTType::RIGCTL;
            cli_set.insert("ptt_type");
            std::string hostport = argv[++i];
            size_t colon = hostport.find(':');
            if (colon != std::string::npos) {
                config.rigctl_host = hostport.substr(0, colon);
                config.rigctl_port = std::atoi(hostport.substr(colon + 1).c_str());
            } else {
                config.rigctl_host = hostport;
            }
        } else if (arg == "--hamlib-model" && i + 1 < argc) {
            config.hamlib_model = atoi(argv[++i]);
            cli_set.insert("hamlib_model");
        } else if (arg == "--hamlib-device" && i + 1 < argc) {
            config.hamlib_device = argv[++i];
            cli_set.insert("hamlib_device");
        } else if (arg == "--hamlib-baud" && i + 1 < argc) {
            config.hamlib_baud = atoi(argv[++i]);
            cli_set.insert("hamlib_baud");
        } else if (arg == "--com-port" && i + 1 < argc) {
            config.com_port = argv[++i];
            cli_set.insert("com_port");
        } else if (arg == "--com-line" && i + 1 < argc) {
            std::string line = argv[++i];
            bool invert_specified = false;
            if (line == "dtr") {
                config.com_ptt_line = 0;
            } else if (line == "rts") {
                config.com_ptt_line = 1;
            } else if (line == "both") {
                config.com_ptt_line = 2;
            } else if (line == "-dtr") {
                config.com_ptt_line = 0;
                config.com_invert_dtr = true;
                config.com_invert_rts = false;
                invert_specified = true;
            } else if (line == "-rts") {
                config.com_ptt_line = 1;
                config.com_invert_dtr = false;
                config.com_invert_rts = true;
                invert_specified = true;
            } else if (line == "-both") {
                config.com_ptt_line = 2;
                config.com_invert_dtr = true;
                config.com_invert_rts = true;
                invert_specified = true;
            } else {
                std::cerr << "Unknown COM PTT line: " << line
                          << " (use dtr, rts, both, -dtr, -rts, -both)\n";
                return 1;
            }
            cli_set.insert("com_ptt_line");
            if (invert_specified) {
                cli_set.insert("com_invert_dtr");
                cli_set.insert("com_invert_rts");
            }
        } else if (arg == "--ptt" && i + 1 < argc) {
            cli_set.insert("ptt_type");
            std::string ptt_type = argv[++i];
            if (ptt_type == "none") config.ptt_type = PTTType::NONE;
            else if (ptt_type == "rigctl") config.ptt_type = PTTType::RIGCTL;
            else if (ptt_type == "vox") config.ptt_type = PTTType::VOX;
            else if (ptt_type == "com") config.ptt_type = PTTType::COM;
#ifdef WITH_CM108
            else if (ptt_type == "cm108") config.ptt_type = PTTType::CM108;
#endif
            else {
                std::cerr << "Unknown PTT type: " << ptt_type << " (use none, rigctl, vox, com"
#ifdef WITH_CM108
                          << ", cm108"
#endif
                          << ")\n";
                return 1;
            }
        } else if (arg == "--vox-freq" && i + 1 < argc) {
            config.vox_tone_freq = std::min(3000, std::max(300, std::atoi(argv[++i])));
            cli_set.insert("vox_tone_freq");
        } else if (arg == "--vox-lead" && i + 1 < argc) {
            config.vox_lead_ms = std::min(2000, std::max(50, std::atoi(argv[++i])));
            cli_set.insert("vox_lead_ms");
        } else if (arg == "--vox-tail" && i + 1 < argc) {
            config.vox_tail_ms = std::min(2000, std::max(50, std::atoi(argv[++i])));
            cli_set.insert("vox_tail_ms");
#ifdef WITH_CM108
        } else if (arg == "--cm108-gpio" && i + 1 < argc) {
            config.cm108_gpio = std::atoi(argv[++i]);
            cli_set.insert("cm108_gpio");
        } else if (arg == "--cm108-device" && i + 1 < argc) {
            config.cm108_device = argv[++i];
            cli_set.insert("cm108_device");
#endif
        } else if (arg == "--ptt-delay" && i + 1 < argc) {
            config.ptt_delay_ms = clamp_arg("--ptt-delay", argv[++i], 0, 2000);
            cli_set.insert("ptt_delay_ms");
        } else if (arg == "--ptt-tail" && i + 1 < argc) {
            config.ptt_tail_ms = clamp_arg("--ptt-tail", argv[++i], 0, 2000);
            cli_set.insert("ptt_tail_ms");
        } else if (arg == "--tx-delay" && i + 1 < argc) {
            config.tx_delay_ms = clamp_arg("--tx-delay", argv[++i], 250, 2500);
            cli_set.insert("tx_delay_ms");
        } else if (arg == "--no-mfsk-rx") {
            config.mfsk_rx_enabled = false;
            cli_set.insert("mfsk_rx_enabled");
        } else if (arg == "--no-ofdm-rx") {
            config.ofdm_rx_enabled = false;
            cli_set.insert("ofdm_rx_enabled");
        } else if (arg == "--no-robust-rx") {
            config.robust_rx_enabled = false;
            cli_set.insert("robust_rx_enabled");
        } else if (arg == "--no-csma") {
            config.csma_enabled = false;
            cli_set.insert("csma_enabled");
        } else if (arg == "--csma-mode" && i + 1 < argc) {
            std::string value = argv[++i];
            int idx = arg_index(value, CSMA_MODE_OPTS);
            if (idx < 0) {
                print_arg_options("--csma-mode", value, CSMA_MODE_OPTS);
                return 1;
            }
            config.csma_sync_only = idx >= 1;
            config.csma_ranked = idx == 2;
            cli_set.insert("csma_sync_only");
            cli_set.insert("csma_ranked");
        } else if (arg == "--csma-band" && i + 1 < argc) {
            std::string value = argv[++i];
            int idx = arg_index(value, CSMA_BAND_OPTS);
            if (idx < 0) {
                print_arg_options("--csma-band", value, CSMA_BAND_OPTS);
                return 1;
            }
            config.csma_band = idx;
            cli_set.insert("csma_band");
        } else if (arg == "--csma-preset" && i + 1 < argc) {
            std::string value = argv[++i];
            int idx = -1;
            for (int p = 0; p < CSMA_PRESET_COUNT; ++p)
                if (arg_ieq(value, CSMA_PRESETS[0][p].name)) { idx = p; break; }
            if (idx < 0) {
                std::cerr << "Invalid --csma-preset '" << value << "', expected one of:";
                for (int p = 0; p < CSMA_PRESET_COUNT; ++p)
                    std::cerr << ' ' << CSMA_PRESETS[0][p].name;
                std::cerr << std::endl;
                return 1;
            }
            // presets are per band, so --csma-band must come first to take effect
            const CsmaPreset& preset = CSMA_PRESETS[config.csma_band & 1][idx];
            config.csma_quiet_ms = preset.quiet_ms;
            config.csma_cw = preset.cw;
            config.slot_time_ms = preset.slot_ms;
            config.csma_burst = preset.burst;
            config.csma_responder_dither = preset.dither;
            config.tx_lead_tone = preset.lead_tone;
            cli_set.insert("csma_quiet_ms");
            cli_set.insert("csma_cw");
            cli_set.insert("slot_time_ms");
            cli_set.insert("csma_burst");
            cli_set.insert("csma_responder_dither");
            cli_set.insert("tx_lead_tone");
        } else if (arg == "--csma-threshold" && i + 1 < argc) {
            config.carrier_threshold_db = clamp_arg_f("--csma-threshold", argv[++i], -80.0f, 0.0f);
            cli_set.insert("carrier_threshold_db");
        } else if (arg == "--csma-slot" && i + 1 < argc) {
            config.slot_time_ms = clamp_arg("--csma-slot", argv[++i], 50, 5000);
            cli_set.insert("slot_time_ms");
        } else if (arg == "--csma-quiet" && i + 1 < argc) {
            config.csma_quiet_ms = clamp_arg("--csma-quiet", argv[++i], 0, 10000);
            cli_set.insert("csma_quiet_ms");
        } else if (arg == "--csma-cw" && i + 1 < argc) {
            config.csma_cw = clamp_arg("--csma-cw", argv[++i], 2, 32);
            cli_set.insert("csma_cw");
        } else if (arg == "--csma-dither" && i + 1 < argc) {
            config.csma_responder_dither = clamp_arg("--csma-dither", argv[++i], 0, 3000);
            cli_set.insert("csma_responder_dither");
        } else if (arg == "--csma-burst" && i + 1 < argc) {
            config.csma_burst = clamp_arg("--csma-burst", argv[++i], 1, 4);
            cli_set.insert("csma_burst");
        } else if (arg == "--lead-tone") {
            config.tx_lead_tone = true;
            cli_set.insert("tx_lead_tone");
        } else if (arg == "--no-lead-tone") {
            config.tx_lead_tone = false;
            cli_set.insert("tx_lead_tone");
        } else if (arg == "--fast-floor") {
            config.csma_fast_floor = true;
            cli_set.insert("csma_fast_floor");
        } else if (arg == "--no-fast-floor") {
            config.csma_fast_floor = false;
            cli_set.insert("csma_fast_floor");
        } else if (arg == "--beacon-interval" && i + 1 < argc) {
            config.beacon_interval_s = clamp_arg("--beacon-interval", argv[++i], 45, 90);
            cli_set.insert("beacon_interval_s");
        } else if (arg == "--frag") {
            config.fragmentation_enabled = true;
            cli_set.insert("fragmentation_enabled");
        } else if (arg == "--no-frag") {
            config.fragmentation_enabled = false;
            cli_set.insert("fragmentation_enabled");
        } else if (arg == "--tx-blank") {
            config.tx_blanking_enabled = true;
            cli_set.insert("tx_blanking_enabled");
        } else if (arg == "--no-tx-blank") {
            config.tx_blanking_enabled = false;
            cli_set.insert("tx_blanking_enabled");
        // deprecated, kept so existing scripts still start; hidden from --help
        } else if ((arg == "-f" || arg == "--freq") && i + 1 < argc) {
            ++i;
            std::cerr << "Warning: " << arg
                      << " is deprecated, center frequency is fixed at 1500 Hz" << std::endl;
        } else if (arg == "--csma-persist" && i + 1 < argc) {
            ++i;
            std::cerr << "Warning: --csma-persist is deprecated and ignored,"
                         " use --csma-cw and --csma-slot" << std::endl;
        } else if (arg == "--no-rigctl") {
            config.ptt_type = PTTType::NONE;
            cli_set.insert("ptt_type");
            std::cerr << "Warning: --no-rigctl is deprecated, use --ptt none" << std::endl;
        } else {
            std::cerr << "Unknown option: " << arg << std::endl;
            print_help(argv[0]);
            return 1;
        }
    }


    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    signal(SIGPIPE, SIG_IGN);

    if (!g_use_ui) {
        std::string settings_path;
        if (cli_config && !config.config_file.empty()) {
            settings_path = config.config_file;
        } else {
            const char* home = getenv("HOME");
            if (home) settings_path = std::string(home) + "/.config/modem73/settings";
        }
        if (!settings_path.empty()) {
            if (apply_settings_file(settings_path, config, cli_set)) {
                std::cerr << "Loaded settings from " << settings_path << std::endl;
            } else if (cli_config) {
                std::cerr << "Could not read config file: " << settings_path << std::endl;
            }
        }
    }

#ifdef WITH_UI
    TNCUIState ui_state;
    if (g_use_ui) {
        g_ui_state = &ui_state;
        
        // Set up config file path
        const char* home = getenv("HOME");
        if (home) {
            std::string config_dir = std::string(home) + "/.config/modem73";
            mkdir(config_dir.c_str(), 0755);
            ui_state.config_file = cli_config && !config.config_file.empty()
                                       ? config.config_file
                                       : config_dir + "/settings";
            ui_state.presets_file = config_dir + "/presets";
            
            auto input_devices = MiniAudio::list_capture_devices();
            for (const auto& dev : input_devices) {
                ui_state.available_input_devices.push_back(dev.first);
                ui_state.input_device_descriptions.push_back(dev.second);
            }
            if (ui_state.available_input_devices.empty()) {
                ui_state.available_input_devices.push_back("default");
                ui_state.input_device_descriptions.push_back("default - System Default");
            }
            
            auto output_devices = MiniAudio::list_playback_devices();
            for (const auto& dev : output_devices) {
                ui_state.available_output_devices.push_back(dev.first);
                ui_state.output_device_descriptions.push_back(dev.second);
            }
            if (ui_state.available_output_devices.empty()) {
                ui_state.available_output_devices.push_back("default");
                ui_state.output_device_descriptions.push_back("default - System Default");
            }
            
            // Try to load saved settings
            if (ui_state.load_settings()) {
                // Apply loaded settings to config
                if (!cli_set.count("callsign"))
                    config.callsign = ui_state.callsign;
                if (!cli_set.count("modem_type"))
                    config.modem_type = ui_state.modem_type_index;
                if (!cli_set.count("mfsk_mode"))
                    config.mfsk_mode = ui_state.mfsk_mode_index;
                if (!cli_set.count("robust_mode"))
                    config.robust_mode = ui_state.robust_mode_index;
                if (!cli_set.count("tx_drive"))
                    config.tx_drive = ui_state.tx_drive;
                if (!cli_set.count("center_freq"))
                    config.center_freq = ui_state.center_freq;
                if (!cli_set.count("modulation"))
                    config.modulation = MODULATION_OPTIONS[ui_state.modulation_index];
                if (!cli_set.count("code_rate"))
                    config.code_rate = CODE_RATE_OPTIONS[ui_state.code_rate_index];
                if (!cli_set.count("frame_size"))
                    config.frame_size = ui_state.frame_size;
                if (!cli_set.count("postamble"))
                    config.postamble = ui_state.postamble;
                if (!cli_set.count("csma_enabled"))
                    config.csma_enabled = ui_state.csma_enabled;
                if (!cli_set.count("csma_sync_only"))
                    config.csma_sync_only = ui_state.csma_sync_only;
                if (!cli_set.count("csma_fast_floor"))
                    config.csma_fast_floor = ui_state.csma_fast_floor;
                if (!cli_set.count("csma_ranked"))
                    config.csma_ranked = ui_state.csma_ranked;
                if (!cli_set.count("csma_band"))
                    config.csma_band = ui_state.csma_band;
                if (!cli_set.count("carrier_threshold_db"))
                    config.carrier_threshold_db = ui_state.carrier_threshold_db;
                if (!cli_set.count("slot_time_ms"))
                    config.slot_time_ms = ui_state.slot_time_ms;
                if (!cli_set.count("csma_quiet_ms"))
                    config.csma_quiet_ms = ui_state.csma_quiet_ms;
                if (!cli_set.count("csma_cw"))
                    config.csma_cw = ui_state.csma_cw;
                if (!cli_set.count("csma_responder_dither"))
                    config.csma_responder_dither = ui_state.csma_responder_dither;
                if (!cli_set.count("csma_burst"))
                    config.csma_burst = ui_state.csma_burst;
                if (!cli_set.count("tx_lead_tone"))
                    config.tx_lead_tone = ui_state.tx_lead_tone;
                if (!cli_set.count("p_persistence"))
                    config.p_persistence = ui_state.p_persistence;
                if (!cli_set.count("fragmentation_enabled"))
                    config.fragmentation_enabled = ui_state.fragmentation_enabled;
                if (!cli_set.count("tx_blanking_enabled"))
                    config.tx_blanking_enabled = ui_state.tx_blanking_enabled;
                if (!cli_set.count("ofdm_rx_enabled"))
                    config.ofdm_rx_enabled = ui_state.ofdm_rx_enabled;
                if (!cli_set.count("robust_rx_enabled"))
                    config.robust_rx_enabled = ui_state.robust_rx_enabled;
                if (!cli_set.count("mfsk_rx_enabled"))
                    config.mfsk_rx_enabled = ui_state.mfsk_rx_enabled;
                bool devices_migrated = false;
                if (!ui_state.audio_input_device.empty() &&
                    ui_state.audio_input_device.find_first_not_of("0123456789") == std::string::npos) {
                    size_t legacy_idx = ui_state.audio_input_device.size() < 6
                        ? std::stoul(ui_state.audio_input_device) + 1 : (size_t)-1;
                    if (legacy_idx < ui_state.available_input_devices.size()) {
                        ui_state.audio_input_device = ui_state.available_input_devices[legacy_idx];
                        devices_migrated = true;
                    }
                }
                if (!ui_state.audio_output_device.empty() &&
                    ui_state.audio_output_device.find_first_not_of("0123456789") == std::string::npos) {
                    size_t legacy_idx = ui_state.audio_output_device.size() < 6
                        ? std::stoul(ui_state.audio_output_device) + 1 : (size_t)-1;
                    if (legacy_idx < ui_state.available_output_devices.size()) {
                        ui_state.audio_output_device = ui_state.available_output_devices[legacy_idx];
                        devices_migrated = true;
                    }
                }
                if (devices_migrated) {
                    ui_state.save_settings();
                    std::cerr << "Migrated audio device settings to device names" << std::endl;
                }
                // Audio devices
                if (!cli_set.count("audio_input"))
                    config.audio_input_device = ui_state.audio_input_device;
                if (!cli_set.count("audio_output"))
                    config.audio_output_device = ui_state.audio_output_device;
                // PTT settings
                if (!cli_set.count("ptt_type"))
                    config.ptt_type = static_cast<PTTType>(ui_state.ptt_type_index);
                if (!cli_set.count("vox_tone_freq"))
                    config.vox_tone_freq = ui_state.vox_tone_freq;
                if (!cli_set.count("vox_lead_ms"))
                    config.vox_lead_ms = ui_state.vox_lead_ms;
                if (!cli_set.count("tx_delay_ms"))
                    config.tx_delay_ms = ui_state.tx_delay_ms;
                if (!cli_set.count("beacon_interval_s"))
                    config.beacon_interval_s = ui_state.beacon_interval_s;
                if (!cli_set.count("vox_tail_ms"))
                    config.vox_tail_ms = ui_state.vox_tail_ms;

                // COM PTT settings
                if (!cli_set.count("com_port"))
                    config.com_port = ui_state.com_port;
                if (!cli_set.count("hamlib_model"))
                    config.hamlib_model = ui_state.hamlib_model;
                if (!cli_set.count("hamlib_device"))
                    config.hamlib_device = ui_state.hamlib_device;
                if (!cli_set.count("hamlib_baud"))
                    config.hamlib_baud = ui_state.hamlib_baud;
                if (!cli_set.count("com_ptt_line"))
                    config.com_ptt_line = ui_state.com_ptt_line;
                if (!cli_set.count("com_invert_dtr"))
                    config.com_invert_dtr = ui_state.com_invert_dtr;
                if (!cli_set.count("com_invert_rts"))
                    config.com_invert_rts = ui_state.com_invert_rts;

#ifdef WITH_CM108
                // CM108 PTT settings

                if (!cli_set.count("cm108_gpio"))
                    config.cm108_gpio = ui_state.cm108_gpio;

                if (!cli_set.count("cm108_device"))
                    config.cm108_device = ui_state.cm108_device;
                
#endif

                // Network settings
                if (!cli_set.count("port"))
                    config.port = ui_state.port;
                if (!cli_control_port)
                    config.control_port = ui_state.control_port;
                if (!cli_set.count("bind_address"))
                    config.bind_address = ui_state.bind_address;
                if (!cli_set.count("control_bind_address"))
                    config.control_bind_address = ui_state.control_bind_address;

                // Find audio device indices
                for (size_t i = 0; i < ui_state.available_input_devices.size(); i++) {
                    if (ui_state.available_input_devices[i] == ui_state.audio_input_device) {
                        ui_state.audio_input_index = i;
                        break;
                    }
                }
                for (size_t i = 0; i < ui_state.available_output_devices.size(); i++) {
                    if (ui_state.available_output_devices[i] == ui_state.audio_output_device) {
                        ui_state.audio_output_index = i;
                        break;
                    }
                }
                
                std::cerr << "Loaded settings from " << ui_state.config_file << std::endl;
            } else {

                ui_state.callsign = config.callsign;
                ui_state.center_freq = config.center_freq;
                ui_state.csma_enabled = config.csma_enabled;
                ui_state.csma_sync_only = config.csma_sync_only;
                ui_state.csma_fast_floor = config.csma_fast_floor;
                ui_state.csma_ranked = config.csma_ranked;
                ui_state.csma_band = config.csma_band;
                ui_state.carrier_threshold_db = config.carrier_threshold_db;
                ui_state.slot_time_ms = config.slot_time_ms;
                ui_state.csma_quiet_ms = config.csma_quiet_ms;
                ui_state.csma_cw = config.csma_cw;
                ui_state.csma_responder_dither = config.csma_responder_dither;
                ui_state.csma_burst = config.csma_burst;
                ui_state.tx_lead_tone = config.tx_lead_tone;
                ui_state.p_persistence = config.p_persistence;
                ui_state.frame_size = config.frame_size;
                ui_state.postamble = config.postamble;
                ui_state.fragmentation_enabled = config.fragmentation_enabled;
                ui_state.tx_blanking_enabled = config.tx_blanking_enabled;
                ui_state.ofdm_rx_enabled = config.ofdm_rx_enabled;
                ui_state.robust_rx_enabled = config.robust_rx_enabled;
                ui_state.mfsk_rx_enabled = config.mfsk_rx_enabled;
                // Audio devices
                ui_state.audio_input_device = config.audio_input_device;
                ui_state.audio_output_device = config.audio_output_device;




                // PTT settings
                ui_state.ptt_type_index = static_cast<int>(config.ptt_type);
                ui_state.vox_tone_freq = config.vox_tone_freq;
                ui_state.vox_lead_ms = config.vox_lead_ms;
                ui_state.vox_tail_ms = config.vox_tail_ms;
                ui_state.tx_delay_ms = config.tx_delay_ms;
                ui_state.beacon_interval_s = config.beacon_interval_s;
                // COM PTT settings
                ui_state.com_port = config.com_port;
                ui_state.com_ptt_line = config.com_ptt_line;
                ui_state.com_invert_dtr = config.com_invert_dtr;
                ui_state.com_invert_rts = config.com_invert_rts;
#ifdef WITH_CM108
                // CM108 PTT settings
                ui_state.cm108_gpio = config.cm108_gpio;
                ui_state.cm108_device = config.cm108_device;
#endif
                // Network settings
                ui_state.port = config.port;
                ui_state.control_port = config.control_port;
                ui_state.bind_address = config.bind_address;
                ui_state.control_bind_address = config.control_bind_address;

                // Find modulation index
                for (size_t i = 0; i < MODULATION_OPTIONS.size(); ++i) {
                    if (MODULATION_OPTIONS[i] == config.modulation) {
                        ui_state.modulation_index = i;
                        break;
                    }
                }
                
                // Find code rate index
                for (size_t i = 0; i < CODE_RATE_OPTIONS.size(); ++i) {
                    if (CODE_RATE_OPTIONS[i] == config.code_rate) {
                        ui_state.code_rate_index = i;
                        break;
                    }
                }
            }
        }
        
        ui_state.callsign = config.callsign;
        ui_state.center_freq = config.center_freq;
        ui_state.modem_type_index = config.modem_type;
        ui_state.mfsk_mode_index = config.mfsk_mode;
        ui_state.robust_mode_index = config.robust_mode;
        ui_state.frame_size = config.frame_size;
        ui_state.postamble = config.postamble;
        ui_state.csma_enabled = config.csma_enabled;
        ui_state.csma_sync_only = config.csma_sync_only;
        ui_state.csma_fast_floor = config.csma_fast_floor;
        ui_state.csma_ranked = config.csma_ranked;
        ui_state.csma_band = config.csma_band;
        ui_state.carrier_threshold_db = config.carrier_threshold_db;
        ui_state.slot_time_ms = config.slot_time_ms;
        ui_state.csma_quiet_ms = config.csma_quiet_ms;
        ui_state.csma_cw = config.csma_cw;
        ui_state.csma_responder_dither = config.csma_responder_dither;
        ui_state.csma_burst = config.csma_burst;
        ui_state.tx_lead_tone = config.tx_lead_tone;
        ui_state.p_persistence = config.p_persistence;
        ui_state.tx_drive = config.tx_drive;
        ui_state.audio_input_device = config.audio_input_device;
        ui_state.audio_output_device = config.audio_output_device;
        ui_state.com_port = config.com_port;
        ui_state.com_ptt_line = config.com_ptt_line;
        ui_state.com_invert_dtr = config.com_invert_dtr;
        ui_state.com_invert_rts = config.com_invert_rts;
#ifdef WITH_CM108
        ui_state.cm108_gpio = config.cm108_gpio;
        ui_state.cm108_device = config.cm108_device;
#endif
        ui_state.port = config.port;
        ui_state.control_port = config.control_port;
        ui_state.bind_address = config.bind_address;
        ui_state.control_bind_address = config.control_bind_address;
        for (size_t i = 0; i < MODULATION_OPTIONS.size(); ++i) {
            if (MODULATION_OPTIONS[i] == config.modulation) {
                ui_state.modulation_index = i;
                break;
            }
        }
        for (size_t i = 0; i < CODE_RATE_OPTIONS.size(); ++i) {
            if (CODE_RATE_OPTIONS[i] == config.code_rate) {
                ui_state.code_rate_index = i;
                break;
            }
        }
        for (size_t i = 0; i < ui_state.available_input_devices.size(); i++) {
            if (ui_state.available_input_devices[i] == ui_state.audio_input_device) {
                ui_state.audio_input_index = i;
                break;
            }
        }
        for (size_t i = 0; i < ui_state.available_output_devices.size(); i++) {
            if (ui_state.available_output_devices[i] == ui_state.audio_output_device) {
                ui_state.audio_output_index = i;
                break;
            }
        }

        // Set PTT info for display
        ui_state.ptt_type_index = static_cast<int>(config.ptt_type);
        ui_state.rigctl_host = config.rigctl_host;
        ui_state.rigctl_port = config.rigctl_port;
        ui_state.vox_tone_freq = config.vox_tone_freq;
        ui_state.vox_lead_ms = config.vox_lead_ms;
        ui_state.vox_tail_ms = config.vox_tail_ms;
        ui_state.tx_delay_ms = config.tx_delay_ms;
        ui_state.beacon_interval_s = config.beacon_interval_s;
        



        ui_state.load_presets();
        
        // Sync fragmentation setting from command line to UI
        ui_state.fragmentation_enabled = config.fragmentation_enabled;
        ui_state.tx_blanking_enabled = config.tx_blanking_enabled;
        ui_state.ofdm_rx_enabled = config.ofdm_rx_enabled;
        ui_state.robust_rx_enabled = config.robust_rx_enabled;
        ui_state.mfsk_rx_enabled = config.mfsk_rx_enabled;

        ui_state.update_modem_info();
        
        // Set up stop callback
        ui_state.on_stop_requested = []() {
            g_running = false;
        };
    }
#endif
    
    if (!valid_bind_address(config.bind_address)) {
        std::cerr << "Error: invalid bind address '" << config.bind_address << "'" << std::endl;
        return 1;
    }
    if (!valid_bind_address(config.control_bind_address)) {
        std::cerr << "Error: invalid control bind address '"
                  << config.control_bind_address << "'" << std::endl;
        return 1;
    }

    while (!check_port_available(config.bind_address, config.port)) {
        std::cerr << "Error: Port " << config.port << " is already in use or cannot be bound" << std::endl;
        std::cerr << "Another instance of modem73 may be running, or another application is using this port." << std::endl;
        
        if (!g_use_ui) {
            std::cerr << "Use --port to specify a different port." << std::endl;
            return 1;
        }
        
        std::cerr << "\nEnter a different port number (or 'q' to quit): ";
        std::string input;
        if (!std::getline(std::cin, input) || input.empty() || input == "q" || input == "Q") {
            std::cerr << "Exiting." << std::endl;
            return 1;
        }
        
        try {
            int new_port = std::stoi(input);
            if (new_port < 1 || new_port > 65535) {
                std::cerr << "Invalid port number. Must be between 1 and 65535." << std::endl;
                continue;
            }
            config.port = new_port;
#ifdef WITH_UI
            if (g_use_ui) {
                ui_state.port = new_port;
            }
#endif
            std::cerr << "Trying port " << config.port << "..." << std::endl;
        } catch (const std::exception&) {
            std::cerr << "Invalid input. Please enter a number." << std::endl;
        }
    }
    
    while (config.control_port > 0 && !check_port_available(config.control_bind_address, config.control_port)) {
        std::cerr << "Error: Control port " << config.control_port << " is already in use" << std::endl;

        if (!g_use_ui) {
            std::cerr << "Use --control-port to specify a different port." << std::endl;
            return 1;
        }

        std::cerr << "\nEnter a different control port (or 'q' to quit, 0 to disable): ";
        std::string input;
        if (!std::getline(std::cin, input) || input.empty() || input == "q" || input == "Q") {
            std::cerr << "Exiting." << std::endl;
            return 1;
        }

        try {
            int new_port = std::stoi(input);
            if (new_port < 0 || new_port > 65535) {
                std::cerr << "Invalid port number. Must be 0-65535." << std::endl;
                continue;
            }
            config.control_port = new_port;
            if (new_port == 0)
                std::cerr << "Control port disabled." << std::endl;
            else
                std::cerr << "Trying control port " << config.control_port << "..." << std::endl;
        } catch (const std::exception&) {
            std::cerr << "Invalid input. Please enter a number." << std::endl;
        }
    }

    config.center_freq = 1500;
    if (config.csma_enabled)
        config.tx_blanking_enabled = true;

    try {
        KISSTNC tnc(config);

        // Set up control port
        std::unique_ptr<ControlPort> ctrl;
        if (config.control_port > 0) {
            ControlPort::TNCInterface ctrl_iface;

            ctrl_iface.get_status = [&tnc, &ui_state]() -> cJSON* {
                cJSON* j = cJSON_CreateObject();
                auto stats = tnc.get_decoder_stats();
                {
                    TNCConfig c = tnc.get_config();
                    cJSON_AddNumberToObject(j, "net_bps_estimate",
                        net_bps_estimate(c.csma_enabled, c.csma_quiet_ms, c.csma_cw,
                                         c.slot_time_ms, c.csma_burst, c.tx_lead_tone,
                                         c.tx_delay_ms, ui_state.airtime_seconds,
                                         ui_state.mtu_bytes));
                }

                // Channel state
                const char* state = "idle";
                if (tnc.is_transmitting()) state = "tx";
                else if (tnc.is_receiving()) state = "rx";
                cJSON_AddStringToObject(j, "channel_state", state);

                cJSON_AddBoolToObject(j, "ptt_on", tnc.is_transmitting());
                cJSON_AddNumberToObject(j, "tx_queue", (double)tnc.tx_queue_depth());
                cJSON_AddNumberToObject(j, "rx_frame_count", stats.sync_count - stats.preamble_errors - stats.crc_errors);
                cJSON_AddNumberToObject(j, "tx_frame_count", 0); // TODO: add tx counter to KISSTNC
                cJSON_AddNumberToObject(j, "rx_error_count", stats.preamble_errors + stats.crc_errors);
                cJSON_AddNumberToObject(j, "sync_count", stats.sync_count);
                cJSON_AddNumberToObject(j, "preamble_errors", stats.preamble_errors);
                cJSON_AddNumberToObject(j, "symbol_errors", stats.symbol_errors);
                cJSON_AddNumberToObject(j, "erased_symbols", stats.erased_symbols);
                cJSON_AddNumberToObject(j, "crc_errors", stats.crc_errors);
                cJSON_AddNumberToObject(j, "last_snr", stats.last_snr);
                cJSON_AddNumberToObject(j, "last_ber", stats.last_ber);
                cJSON_AddNumberToObject(j, "ber_ema", stats.ber_ema);
                cJSON_AddNumberToObject(j, "client_count", tnc.get_client_count());
                cJSON_AddBoolToObject(j, "rigctl_connected", tnc.is_rigctl_connected());
                cJSON_AddBoolToObject(j, "audio_connected", tnc.is_audio_healthy());
                cJSON_AddNumberToObject(j, "population", tnc.channel_population());
                cJSON_AddNumberToObject(j, "occupancy_pct", tnc.channel_occupancy());

                return j;
            };

            ctrl_iface.get_config = [&tnc]() -> cJSON* {
                cJSON* j = cJSON_CreateObject();
                TNCConfig cfg = tnc.get_config();

                cJSON_AddStringToObject(j, "callsign", cfg.callsign.c_str());
                cJSON_AddNumberToObject(j, "modem_type", cfg.modem_type);
                cJSON_AddNumberToObject(j, "mfsk_mode", cfg.mfsk_mode);
                cJSON_AddNumberToObject(j, "robust_mode", cfg.robust_mode);
                if (cfg.modem_type == 1) {
                    cJSON_AddStringToObject(j, "modulation",
                        MFSK_MODE_NAMES[cfg.mfsk_mode < 4 ? cfg.mfsk_mode : 0]);
                } else if (cfg.modem_type == 2) {
                    cJSON_AddStringToObject(j, "modulation",
                        ROBUST_MODE_NAMES[cfg.robust_mode >= 0 &&
                            cfg.robust_mode < ROBUST_MODE_COUNT ? cfg.robust_mode : 0]);
                } else {
                    cJSON_AddStringToObject(j, "modulation", cfg.modulation.c_str());
                }
                cJSON_AddStringToObject(j, "code_rate", cfg.code_rate.c_str());
                cJSON_AddBoolToObject(j, "short_frame", cfg.frame_size == 0);
                cJSON_AddNumberToObject(j, "frame_size", cfg.frame_size);
                cJSON_AddBoolToObject(j, "postamble", cfg.postamble);
                cJSON_AddNumberToObject(j, "center_freq", cfg.center_freq);
                cJSON_AddNumberToObject(j, "payload_size", tnc.get_payload_size());
                cJSON_AddBoolToObject(j, "csma_enabled", cfg.csma_enabled);
                cJSON_AddBoolToObject(j, "csma_sync_only", cfg.csma_sync_only);
                cJSON_AddBoolToObject(j, "csma_fast_floor", cfg.csma_fast_floor);
                cJSON_AddBoolToObject(j, "csma_ranked", cfg.csma_ranked);
                cJSON_AddNumberToObject(j, "beacon_interval_s", cfg.beacon_interval_s);
                cJSON_AddNumberToObject(j, "csma_band", cfg.csma_band);
                cJSON_AddNumberToObject(j, "carrier_threshold_db", cfg.carrier_threshold_db);
                cJSON_AddNumberToObject(j, "p_persistence", cfg.p_persistence);
                cJSON_AddNumberToObject(j, "slot_time_ms", cfg.slot_time_ms);
                cJSON_AddNumberToObject(j, "csma_quiet_ms", cfg.csma_quiet_ms);
                cJSON_AddNumberToObject(j, "csma_cw", cfg.csma_cw);
                cJSON_AddNumberToObject(j, "csma_responder_dither", cfg.csma_responder_dither);
                cJSON_AddNumberToObject(j, "csma_burst", cfg.csma_burst);
                cJSON_AddBoolToObject(j, "tx_lead_tone", cfg.tx_lead_tone);
                cJSON_AddNumberToObject(j, "tx_drive", cfg.tx_drive);
                cJSON_AddBoolToObject(j, "tx_blanking_enabled", cfg.tx_blanking_enabled);
                cJSON_AddBoolToObject(j, "fragmentation_enabled", cfg.fragmentation_enabled);
                cJSON_AddBoolToObject(j, "mfsk_rx_enabled", cfg.mfsk_rx_enabled);
                cJSON_AddBoolToObject(j, "ofdm_rx_enabled", cfg.ofdm_rx_enabled);
                cJSON_AddBoolToObject(j, "robust_rx_enabled", cfg.robust_rx_enabled);

                return j;
            };

            ctrl_iface.set_config = [&tnc](cJSON* params) -> bool {
                TNCConfig new_config = tnc.get_config();

                cJSON* item;
                if ((item = cJSON_GetObjectItemCaseSensitive(params, "modem_type")) && cJSON_IsNumber(item)
                    && item->valueint >= 0 && item->valueint <= 2)
                    new_config.modem_type = item->valueint;
                if ((item = cJSON_GetObjectItemCaseSensitive(params, "mfsk_mode")) && cJSON_IsNumber(item)
                    && item->valueint >= 0 && item->valueint <= 3)
                    new_config.mfsk_mode = item->valueint;
                if ((item = cJSON_GetObjectItemCaseSensitive(params, "robust_mode")) && cJSON_IsNumber(item)
                    && item->valueint >= 0 && item->valueint < ROBUST_MODE_COUNT)
                    new_config.robust_mode = item->valueint;
                if ((item = cJSON_GetObjectItemCaseSensitive(params, "callsign")) && cJSON_IsString(item)) {
                    if (!ModemConfig::valid_callsign(item->valuestring)) {
                        ui_log(std::string("(!) Control port: invalid callsign '") +
                               item->valuestring + "' (A-Z 0-9 / only, 1-9 chars)");
                        return false;
                    }
                    new_config.callsign = item->valuestring;
                }
                if ((item = cJSON_GetObjectItemCaseSensitive(params, "modulation")) && cJSON_IsString(item))
                    new_config.modulation = item->valuestring;
                if ((item = cJSON_GetObjectItemCaseSensitive(params, "code_rate")) && cJSON_IsString(item))
                    new_config.code_rate = item->valuestring;
                if ((item = cJSON_GetObjectItemCaseSensitive(params, "short_frame")) && cJSON_IsBool(item))
                    new_config.frame_size = cJSON_IsTrue(item) ? 0 : 1;
                if ((item = cJSON_GetObjectItemCaseSensitive(params, "frame_size")) && cJSON_IsNumber(item)
                    && item->valueint >= 0 && item->valueint <= 2)
                    new_config.frame_size = item->valueint;
                if ((item = cJSON_GetObjectItemCaseSensitive(params, "center_freq")) && cJSON_IsNumber(item))
                    new_config.center_freq = 1500;
                if ((item = cJSON_GetObjectItemCaseSensitive(params, "postamble")) && cJSON_IsBool(item))
                    new_config.postamble = cJSON_IsTrue(item);
                if ((item = cJSON_GetObjectItemCaseSensitive(params, "csma_enabled")) && cJSON_IsBool(item))
                    new_config.csma_enabled = cJSON_IsTrue(item);
                if ((item = cJSON_GetObjectItemCaseSensitive(params, "csma_sync_only")) && cJSON_IsBool(item))
                    new_config.csma_sync_only = cJSON_IsTrue(item);
                if ((item = cJSON_GetObjectItemCaseSensitive(params, "csma_fast_floor")) && cJSON_IsBool(item))
                    new_config.csma_fast_floor = cJSON_IsTrue(item);
                if ((item = cJSON_GetObjectItemCaseSensitive(params, "csma_ranked")) && cJSON_IsBool(item))
                    new_config.csma_ranked = cJSON_IsTrue(item);
                if ((item = cJSON_GetObjectItemCaseSensitive(params, "beacon_interval_s")) && cJSON_IsNumber(item)
                    && item->valueint >= 45 && item->valueint <= 90)
                    new_config.beacon_interval_s = item->valueint;
                if ((item = cJSON_GetObjectItemCaseSensitive(params, "csma_band")) && cJSON_IsNumber(item))
                    new_config.csma_band = item->valueint != 0 ? 1 : 0;
                if ((item = cJSON_GetObjectItemCaseSensitive(params, "carrier_threshold_db")) && cJSON_IsNumber(item))
                    new_config.carrier_threshold_db = (float)item->valuedouble;
                if ((item = cJSON_GetObjectItemCaseSensitive(params, "p_persistence")) && cJSON_IsNumber(item))
                    new_config.p_persistence = item->valueint;
                if ((item = cJSON_GetObjectItemCaseSensitive(params, "slot_time_ms")) && cJSON_IsNumber(item))
                    new_config.slot_time_ms = item->valueint;
                if ((item = cJSON_GetObjectItemCaseSensitive(params, "csma_quiet_ms")) && cJSON_IsNumber(item))
                    new_config.csma_quiet_ms = item->valueint;
                if ((item = cJSON_GetObjectItemCaseSensitive(params, "csma_cw")) && cJSON_IsNumber(item))
                    new_config.csma_cw = item->valueint;
                if ((item = cJSON_GetObjectItemCaseSensitive(params, "csma_responder_dither")) && cJSON_IsNumber(item))
                    new_config.csma_responder_dither = item->valueint;
                if ((item = cJSON_GetObjectItemCaseSensitive(params, "csma_burst")) && cJSON_IsNumber(item))
                    new_config.csma_burst = item->valueint;
                if ((item = cJSON_GetObjectItemCaseSensitive(params, "tx_lead_tone")) && cJSON_IsBool(item))
                    new_config.tx_lead_tone = cJSON_IsTrue(item);
                if ((item = cJSON_GetObjectItemCaseSensitive(params, "tx_drive")) && cJSON_IsNumber(item)
                    && item->valuedouble >= 0.05 && item->valuedouble <= 1.0)
                    new_config.tx_drive = (float)item->valuedouble;
                if ((item = cJSON_GetObjectItemCaseSensitive(params, "tx_blanking_enabled")) && cJSON_IsBool(item))
                    new_config.tx_blanking_enabled = cJSON_IsTrue(item);
                if ((item = cJSON_GetObjectItemCaseSensitive(params, "fragmentation_enabled")) && cJSON_IsBool(item))
                    new_config.fragmentation_enabled = cJSON_IsTrue(item);
                if ((item = cJSON_GetObjectItemCaseSensitive(params, "mfsk_rx_enabled")) && cJSON_IsBool(item))
                    new_config.mfsk_rx_enabled = cJSON_IsTrue(item);
                if ((item = cJSON_GetObjectItemCaseSensitive(params, "ofdm_rx_enabled")) && cJSON_IsBool(item))
                    new_config.ofdm_rx_enabled = cJSON_IsTrue(item);
                if ((item = cJSON_GetObjectItemCaseSensitive(params, "robust_rx_enabled")) && cJSON_IsBool(item))
                    new_config.robust_rx_enabled = cJSON_IsTrue(item);

                auto rejected = tnc.update_config(new_config);

#ifdef WITH_UI
                // Sync config back to TUI state so the UI reflects changes.
                // Rejected fields keep whatever the TNC actually kept.
                TNCConfig applied = tnc.get_config();
                if (g_ui_state) {
                    g_ui_state->callsign = applied.callsign;
                    g_ui_state->modem_type_index = new_config.modem_type;
                    g_ui_state->mfsk_mode_index = new_config.mfsk_mode;
                    g_ui_state->robust_mode_index = new_config.robust_mode;
                    g_ui_state->center_freq = new_config.center_freq;
                    g_ui_state->frame_size = new_config.frame_size;
                    g_ui_state->postamble = new_config.postamble;
                    g_ui_state->csma_enabled = new_config.csma_enabled;
                    g_ui_state->csma_sync_only = new_config.csma_sync_only;
                    g_ui_state->beacon_interval_s = new_config.beacon_interval_s;
                    g_ui_state->carrier_threshold_db = new_config.carrier_threshold_db;
                    g_ui_state->p_persistence = new_config.p_persistence;
                    g_ui_state->tx_drive = applied.tx_drive;
                    g_ui_state->slot_time_ms = new_config.slot_time_ms;
                    g_ui_state->tx_blanking_enabled = new_config.tx_blanking_enabled || new_config.csma_enabled;
                    g_ui_state->fragmentation_enabled = new_config.fragmentation_enabled;
                    g_ui_state->ofdm_rx_enabled = new_config.ofdm_rx_enabled;
                    g_ui_state->robust_rx_enabled = new_config.robust_rx_enabled;
                    g_ui_state->mfsk_rx_enabled = new_config.mfsk_rx_enabled;

                    // Map modulation string back to index
                    for (size_t i = 0; i < MODULATION_OPTIONS.size(); i++) {
                        if (MODULATION_OPTIONS[i] == applied.modulation) {
                            g_ui_state->modulation_index = i;
                            break;
                        }
                    }
                    // Map code rate string back to index
                    for (size_t i = 0; i < CODE_RATE_OPTIONS.size(); i++) {
                        if (CODE_RATE_OPTIONS[i] == applied.code_rate) {
                            g_ui_state->code_rate_index = i;
                            break;
                        }
                    }

                    g_ui_state->update_modem_info();
                }
#endif
                return rejected.empty();
            };

            ctrl_iface.send_beacon = [&tnc]() -> bool {
                return tnc.queue_beacon();
            };

            ctrl_iface.rigctl_command = [&tnc](const std::string& cmd) -> std::string {
                return tnc.rigctl_command(cmd);
            };

            ctrl_iface.tx_data = [&tnc](const std::vector<uint8_t>& data, int oper_mode) -> bool {
                tnc.queue_data_ex(data, oper_mode);
                return true;
            };

            ctrl = std::make_unique<ControlPort>(config.control_port, config.control_bind_address, ctrl_iface);
            ctrl->start();

            tnc.rx_stats_callback = [&ctrl](float snr, float ber_pct, float level_db) {
                if (ctrl) ctrl->notify_rx_frame(snr, ber_pct, level_db);
            };
        }

#ifdef WITH_UI
        if (g_use_ui) {
            ui_state.perf_logger = &tnc.perf_log_;
            ui_state.on_settings_changed = [&tnc, &ctrl](TNCUIState& state) {
                TNCConfig new_config = tnc.get_config();
                new_config.modem_type = state.modem_type_index;
                new_config.mfsk_mode = state.mfsk_mode_index;
                new_config.robust_mode = state.robust_mode_index;
                new_config.callsign = state.callsign;
                new_config.center_freq = 1500;
                new_config.modulation = MODULATION_OPTIONS[state.modulation_index];
                new_config.code_rate = CODE_RATE_OPTIONS[state.code_rate_index];
                new_config.frame_size = state.frame_size;
                new_config.postamble = state.postamble;
                new_config.csma_enabled = state.csma_enabled;
                new_config.csma_sync_only = state.csma_sync_only;
                new_config.csma_fast_floor = state.csma_fast_floor;
                new_config.csma_ranked = state.csma_ranked;
                new_config.csma_band = state.csma_band;
                new_config.carrier_threshold_db = state.carrier_threshold_db;
                new_config.p_persistence = state.p_persistence;
                new_config.slot_time_ms = state.slot_time_ms;
                new_config.csma_quiet_ms = state.csma_quiet_ms;
                new_config.csma_cw = state.csma_cw;
                new_config.csma_responder_dither = state.csma_responder_dither;
                new_config.csma_burst = state.csma_burst;
                new_config.tx_lead_tone = state.tx_lead_tone;
                new_config.fragmentation_enabled = state.fragmentation_enabled;
                new_config.tx_blanking_enabled = state.tx_blanking_enabled;
                new_config.ofdm_rx_enabled = state.ofdm_rx_enabled;
                new_config.robust_rx_enabled = state.robust_rx_enabled;
                new_config.mfsk_rx_enabled = state.mfsk_rx_enabled;
                new_config.tx_drive = state.tx_drive;
                new_config.audio_input_device = state.audio_input_device;
                new_config.audio_output_device = state.audio_output_device;
                // PTT settings
                new_config.ptt_type = static_cast<PTTType>(state.ptt_type_index);
                new_config.vox_tone_freq = state.vox_tone_freq;
                new_config.vox_lead_ms = state.vox_lead_ms;
                new_config.vox_tail_ms = state.vox_tail_ms;
                new_config.tx_delay_ms = state.tx_delay_ms;
                new_config.beacon_interval_s = state.beacon_interval_s;
                // COM PTT settings
                new_config.com_port = state.com_port;
                new_config.hamlib_model = state.hamlib_model;
                new_config.hamlib_device = state.hamlib_device;
                new_config.hamlib_baud = state.hamlib_baud;
                new_config.com_ptt_line = state.com_ptt_line;
                new_config.com_invert_dtr = state.com_invert_dtr;
                new_config.com_invert_rts = state.com_invert_rts;

                tnc.update_config(new_config);
                if (ctrl) ctrl->notify_config_changed();
            };
            
            // Set up send data callback for UTILS tab
            ui_state.on_send_data = [&tnc](const std::vector<uint8_t>& data) {
                tnc.queue_data(data);
            };
            
            // Set up audio reconnect callback
            ui_state.on_reconnect_audio = [&tnc]() -> bool {
                return tnc.reconnect_audio();
            };

            ui_state.on_get_audio_level = [&tnc]() -> float {
                return tnc.get_audio_level();
            };

            ui_state.on_alc_tune = [&tnc]() -> float {
                return tnc.alc_auto_tune();
            };

            ui_state.on_rigctl_command = [&tnc](const std::string& cmd) -> std::string {
                return tnc.rigctl_command(cmd);
            };

            // Run TNC in background thread
            std::thread tnc_thread([&tnc]() {
                try {
                    tnc.run();
                } catch (const std::exception& e) {
                    tnc.unkey();
                    ui_log(std::string("FATAL: ") + e.what());
                    g_fatal_error = e.what();
                    g_running = false;
                }
            });
            
            // Status update thread 
            std::thread status_thread([&tnc, &ui_state]() {
                while (g_running) {
                    ui_state.rigctl_connected = tnc.is_rigctl_connected();
                    ui_state.audio_connected = tnc.is_audio_healthy();
                    if (ui_state.rig_poll_enabled.load()) {
                        ui_state.poll_rig();
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                }
            });
            
            TNCUI ui(ui_state);
            ui.run();
            
            // cleanup
            g_running = false;
            status_thread.join();
            tnc_thread.join();

            for (int i = 0; i < 100 && ui_state.alc_tune_running.load(); ++i)
                std::this_thread::sleep_for(std::chrono::milliseconds(100));

        } else {
            tnc.run();
        }
#else
        tnc.run();
#endif
        if (ctrl) ctrl->stop();
    } catch (const std::exception& e) {
        std::cerr << "error " << e.what() << std::endl;
        return 1;
    }

    if (!g_fatal_error.empty()) {
        std::cerr << "error " << g_fatal_error << std::endl;
        return 1;
    }
    return 0;
}
