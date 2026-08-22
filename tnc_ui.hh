#pragma once

#include <locale.h>
#include <ncurses.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <arpa/inet.h>
#include <glob.h>
#include "tnc_ui_state.hh"

class TNCUI {
public:
    TNCUI(TNCUIState& state) : state_(state) {}
    
    ~TNCUI() {
        if (initialized_) {
            endwin();
        }

        if (saved_stderr_ >= 0) {
            dup2(saved_stderr_, STDERR_FILENO);
            close(saved_stderr_);
        }
    }
    
    void run() {
        // set locale LC_ALL for Unicode character support,  
        setlocale(LC_ALL, "");
        

        saved_stderr_ = dup(STDERR_FILENO);
        int devnull = open("/dev/null", O_WRONLY);
        if (devnull >= 0) {
            dup2(devnull, STDERR_FILENO);
            close(devnull);
        }
        
        initscr();
        initialized_ = true;
        cbreak();
        noecho();
        keypad(stdscr, TRUE);
        nodelay(stdscr, TRUE);
        curs_set(0);
        set_escdelay(25);
        
        mousemask(ALL_MOUSE_EVENTS | REPORT_MOUSE_POSITION, NULL);
        mouseinterval(0);
        
        if (has_colors()) {
            start_color();
            use_default_colors();
            init_pair(1, COLOR_GREEN, -1);    // RX/good 
            init_pair(2, COLOR_RED, -1);      // TX/error
            init_pair(3, COLOR_YELLOW, -1);   // Warning 
            init_pair(4, COLOR_CYAN, -1);     // Important 
            init_pair(5, COLOR_WHITE, -1);    // Normal 
            init_pair(6, COLOR_MAGENTA, -1);  // Special

            if (COLORS >= 256 && COLOR_PAIRS > WF_PAIR_BASE + 31) {
                static const short grad[31] = {
                    16, 17, 18, 19, 20, 21, 27, 33, 39, 45, 51,
                    50, 49, 48, 47, 46, 82, 118, 154, 190, 226,
                    220, 214, 208, 202, 196, 197, 198, 199, 200, 201};
                for (int i = 0; i < 31; i++)
                    init_pair(WF_PAIR_BASE + i, COLOR_BLACK, grad[i]);
                wf_pairs_ = 31;
            } else if (COLOR_PAIRS > WF_PAIR_BASE + 7) {
                static const short grad[7] = {COLOR_BLACK, COLOR_BLUE, COLOR_CYAN,
                                              COLOR_GREEN, COLOR_YELLOW, COLOR_RED,
                                              COLOR_MAGENTA};
                for (int i = 0; i < 7; i++)
                    init_pair(WF_PAIR_BASE + i, COLOR_BLACK, grad[i]);
                wf_pairs_ = 7;
            }
            if (COLOR_PAIRS > WF_PAIR_BASE + 32) {
                init_pair(WF_PAIR_BASE + 32, COLOR_WHITE, COLOR_RED);
                wf_sig_ok_ = true;
            }
        }
        
        running_ = true;
        
        while (running_ && g_running) {
            int ch = getch();
            if (ch != ERR) {
                handle_input(ch);
            }
            tick_auto_send();
            tick_level_history();
            draw();
            if (rx_off_dialog_field_ >= 0)
                draw_rx_off_dialog();
            if (lan_dialog_)
                draw_lan_dialog();
            refresh();
            std::this_thread::sleep_for(std::chrono::milliseconds(33));
        }
        
        endwin();
        initialized_ = false;
        

        if (saved_stderr_ >= 0) {
            dup2(saved_stderr_, STDERR_FILENO);
            close(saved_stderr_);
            saved_stderr_ = -1;
        }
    }
    
private:
    enum Field {
        FIELD_CALLSIGN = 0,
        FIELD_CONFIG_TOKEN,
        FIELD_MODEM_TYPE,
        FIELD_MODULATION,
        FIELD_CODERATE,
        FIELD_FRAMESIZE,
        FIELD_POSTAMBLE,
        FIELD_MFSK_MODE,
        FIELD_ROBUST_MODE,
        FIELD_ROBUST_MTU,
        FIELD_FREQ,
        FIELD_CSMA,
        FIELD_CSMA_MODE,
        FIELD_CSMA_HELP,
        FIELD_THRESHOLD,
        FIELD_CSMA_BAND,
        FIELD_CSMA_PRESET,
        FIELD_CSMA_ADV,
        FIELD_CSMA_QUIET,
        FIELD_CSMA_CW,
        FIELD_LEAD_TONE,
        FIELD_RESP_DITHER,
        FIELD_CSMA_BURST,
        FIELD_FAST_FLOOR,
        FIELD_BEACON_INT,
        FIELD_CSMA_INFO,
        FIELD_FRAGMENTATION,
        FIELD_TX_BLANKING,
        FIELD_RX_OFDM,
        FIELD_RX_ROBUST,
        FIELD_RX_MFSK,
        FIELD_AUDIO_INPUT,
        FIELD_AUDIO_OUTPUT,
        FIELD_TX_LEVEL,
        FIELD_PTT_TYPE,
        FIELD_VOX_FREQ,
        FIELD_VOX_LEAD,
        FIELD_VOX_TAIL,
        FIELD_COM_PORT,
        FIELD_COM_LINE,
        FIELD_COM_INVERT,
        FIELD_HAMLIB_MODEL,
        FIELD_HAMLIB_DEVICE,
        FIELD_HAMLIB_BAUD,
#ifdef WITH_CM108
        FIELD_CM108_GPIO,
        FIELD_CM108_DEVICE,
#endif
        FIELD_TX_DELAY,
        FIELD_NET_PORT,
        FIELD_CONTROL_PORT,
        FIELD_LAN_MODE,
        FIELD_PRESET,
        FIELD_COUNT
    };

    enum RigField {
        RIG_FIELD_FREQ = 0,
        RIG_FIELD_STEP,
        RIG_FIELD_PRESET,
        RIG_FIELD_MODE,
        RIG_FIELD_POWER,
        RIG_FIELD_DRIVE,
        RIG_FIELD_TUNER,
        RIG_FIELD_TUNE,
        RIG_FIELD_COUNT
    };

    bool rig_ui() const {
#ifdef WITH_HAMLIB
        if (state_.ptt_type_index == 5) return true;
#endif
        return state_.ptt_type_index == 1;
    }

    int tab_count() const {
        return rig_ui() ? 6 : 5;
    }

    bool rig_should_skip(int field) const {
        if (state_.rig_tuner_supported.load() == 0) {
            if (field == RIG_FIELD_TUNER || field == RIG_FIELD_TUNE) return true;
        }
        return false;
    }

    void handle_input(int ch) {
        if (lan_dialog_) {
            if (ch == 'y' || ch == 'Y' || ch == '\n' || ch == KEY_ENTER) {
                state_.bind_address = "0.0.0.0";
                state_.control_bind_address = "0.0.0.0";
                state_.save_settings();
                state_.add_log("LAN mode enabled (restart to apply)");
                lan_dialog_ = false;
            } else if (ch == 'n' || ch == 'N' || ch == 27) {
                lan_dialog_ = false;
            }
            return;
        }
        if (rx_off_dialog_field_ >= 0) {
            if (ch == 'y' || ch == 'Y' || ch == '\n' || ch == KEY_ENTER) {
                bool* flag = rx_dialog_flag();
                if (flag) {
                    *flag = false;
                    apply_settings();
                    state_.add_log(std::string("(!) ") + rx_dialog_name() +
                                   " RX decoding disabled");
                    if (rx_field_modem(rx_off_dialog_field_) == state_.modem_type_index)
                        state_.add_log(std::string("(!) ") + rx_dialog_name() +
                                       " decoder stays on while it is the TX mode");
                }
                rx_off_dialog_field_ = -1;
            } else if (ch == 'n' || ch == 'N' || ch == 27 || ch == 'q') {
                rx_off_dialog_field_ = -1;
            }
            return;
        }
        if (ch == KEY_MOUSE) {
            MEVENT event;
            if (getmouse(&event) == OK) {
                handle_mouse(event);
            }
            return;
        }
        
        if (ch == KEY_F(1)) {
            show_help_ = !show_help_;
            return;
        }
        
        if (show_help_) {
            show_help_ = false;
            return;
        }
        if (show_csma_help_) {
            show_csma_help_ = false;
            return;
        }
        if (show_sync_help_) {
            show_sync_help_ = false;
            return;
        }

        switch (ch) {
            case 'q':
            case 'Q':
                if (state_.on_stop_requested) {
                    state_.on_stop_requested();
                }
                running_ = false;
                break;
                
            case '\t':
                current_tab_ = (current_tab_ + 1) % tab_count();
                break;

            case KEY_BTAB:  // shift tab prev
                current_tab_ = (current_tab_ + tab_count() - 1) % tab_count();
                break;
                
            case KEY_UP:
            case 'k':
                if (current_tab_ == 1) {
                    do {
                        current_field_ = (current_field_ + FIELD_COUNT - 1) % FIELD_COUNT;
                    } while (should_skip_field(current_field_));
                } else if (current_tab_ == 2) {
                    if (log_scroll_ > 0) {
                        log_scroll_--;
                        log_follow_ = false;
                    }
                } else if (current_tab_ == 3) {
                    if (utils_selection_ == 0 && utils_scroll_ > 0) {
                        utils_scroll_--;
                    } else {
                        utils_selection_ = (utils_selection_ + utils_visible_slots() - 1)
                                         % utils_visible_slots();
                        utils_ensure_visible();
                    }
                } else if (current_tab_ == 5) {
                    do {
                        rig_field_ = (rig_field_ + RIG_FIELD_COUNT - 1) % RIG_FIELD_COUNT;
                    } while (rig_should_skip(rig_field_));
                } else if (current_tab_ == 0) {
                    recent_sel_ = recent_sel_ <= 0 ? 0 : recent_sel_ - 1;
                }
                break;
                
            case KEY_DOWN:
            case 'j':
                if (current_tab_ == 1) {
                    do {
                        current_field_ = (current_field_ + 1) % FIELD_COUNT;
                    } while (should_skip_field(current_field_));
                } else if (current_tab_ == 2) {
                    log_scroll_++;
                } else if (current_tab_ == 3) {
                    if (utils_selection_ == utils_visible_slots() - 1 &&
                        utils_scroll_ < utils_max_scroll_) {
                        utils_scroll_++;
                    } else {
                        utils_selection_ = (utils_selection_ + 1) % utils_visible_slots();
                        utils_ensure_visible();
                    }
                } else if (current_tab_ == 5) {
                    do {
                        rig_field_ = (rig_field_ + 1) % RIG_FIELD_COUNT;
                    } while (rig_should_skip(rig_field_));
                } else if (current_tab_ == 0) {
                    int n = (int)state_.get_recent_packets().size();
                    if (recent_sel_ < 0)
                        recent_sel_ = 0;
                    else if (recent_sel_ < n - 1)
                        recent_sel_++;
                }
                break;
                
            case KEY_LEFT:
            case 'h':
                if (current_tab_ == 1) {
                    if (current_field_ == FIELD_PRESET) {
                        if (!state_.presets.empty()) {
                            state_.selected_preset--;
                            if (state_.selected_preset < 0) {
                                state_.selected_preset = state_.presets.size() - 1;
                            }
                        }
                    } else if (current_field_ >= FIELD_MODEM_TYPE && current_field_ != FIELD_PRESET) {
                        adjust_field(-1);
                    }
                } else if (current_tab_ == 3 && (utils_selection_ == 0 || utils_selection_ == 1)) {
                    int step = 1;
                    if (state_.random_data_size >= 1000) step = 100;
                    else if (state_.random_data_size >= 100) step = 10;
                    state_.random_data_size = std::max(1, state_.random_data_size - step);
                } else if (current_tab_ == 5) {
                    adjust_rig_field(-1);
                }
                break;
                
            case KEY_RIGHT:
            case 'l':
                if (current_tab_ == 1) {
                    if (current_field_ == FIELD_PRESET) {
                        if (!state_.presets.empty()) {
                            state_.selected_preset++;
                            if (state_.selected_preset >= (int)state_.presets.size()) {
                                state_.selected_preset = 0;
                            }
                        }
                    } else if (current_field_ >= FIELD_MODEM_TYPE && current_field_ != FIELD_PRESET) {
                        adjust_field(1);
                    }
                } else if (current_tab_ == 3 && (utils_selection_ == 0 || utils_selection_ == 1)) {
                    int step = 1;
                    if (state_.random_data_size >= 1000) step = 100;
                    else if (state_.random_data_size >= 100) step = 10;
                    int max_size = state_.fragmentation_enabled ? 65535 : state_.mtu_bytes;
                    state_.random_data_size = std::min(max_size, state_.random_data_size + step);
                } else if (current_tab_ == 5) {
                    adjust_rig_field(1);
                }
                break;
                
            case KEY_PPAGE:
                if (current_tab_ == 2) {
                    log_scroll_ = std::max(0, log_scroll_ - 10);
                    log_follow_ = false;
                } else if (current_tab_ == 3) utils_scroll_ = std::max(0, utils_scroll_ - 5);
                break;

            case KEY_NPAGE:
                if (current_tab_ == 2) log_scroll_ += 10;
                else if (current_tab_ == 3) utils_scroll_ += 5;
                break;
                
            case KEY_HOME:
                if (current_tab_ == 2) {
                    log_scroll_ = 0;
                    log_follow_ = false;
                }
                break;
                
            case KEY_END:
                if (current_tab_ == 2) log_scroll_ = 999999;
                break;
                
            case '\n':
            case KEY_ENTER:
                if (current_tab_ == 1) {
                    if (current_field_ == FIELD_CONFIG_TOKEN) {
                        open_config_token();
                    } else if (current_field_ == FIELD_CSMA_INFO) {
                        show_csma_help_ = true;
                    } else if (current_field_ == FIELD_CSMA_HELP) {
                        show_sync_help_ = true;
                    } else if (current_field_ == FIELD_CSMA_ADV) {
                        state_.csma_advanced_open = !state_.csma_advanced_open;
                    } else if (current_field_ == FIELD_CALLSIGN) {


                        edit_text_field(FIELD_CALLSIGN);


                    } else if (current_field_ == FIELD_NET_PORT) {


                        edit_text_field(FIELD_NET_PORT);

                    } else if (current_field_ == FIELD_HAMLIB_MODEL) {
                        show_hamlib_model_dialog();
                    } else if (current_field_ == FIELD_HAMLIB_DEVICE) {
                        show_serial_port_dialog(FIELD_HAMLIB_DEVICE);
                    } else if (current_field_ == FIELD_HAMLIB_BAUD) {
                        adjust_field(1);
                    } else if (current_field_ == FIELD_LAN_MODE) {
                        adjust_field(1);
                    } else if (current_field_ == FIELD_CONTROL_PORT) {

                        edit_text_field(FIELD_CONTROL_PORT);

                    } else if (current_field_ == FIELD_COM_PORT) {


                        show_com_port_dialog();

                    } else if (current_field_ == FIELD_PTT_TYPE) {

                        show_ptt_type_dialog();

#ifdef WITH_CM108
                    } else if (current_field_ == FIELD_CM108_GPIO) {
                        edit_text_field(FIELD_CM108_GPIO);
#endif

                    } else if (current_field_ == FIELD_AUDIO_INPUT) {


                        show_device_select_dialog(true);  


                    } else if (current_field_ == FIELD_AUDIO_OUTPUT) {


                        show_device_select_dialog(false);

#ifdef WITH_CM108
                    } else if (current_field_ == FIELD_CM108_DEVICE) {

                        show_cm108_device_dialog();
#endif

                    } else if (current_field_ == FIELD_PRESET) {


                        load_selected_preset();


                    }

                } else if (current_tab_ == 3) {


                    handle_utils_action();


                } else if (current_tab_ == 5) {

                    rig_enter_action();

                }
                break;

            // Preset field
            case 's': //
                if (current_tab_ == 1 && current_field_ == FIELD_PRESET) {

                    save_preset_dialog();

                } else if (current_tab_ == 5 && rig_field_ == RIG_FIELD_PRESET) {

                    save_freq_preset_dialog();

                }
                break;

            case KEY_DC:
            case 'x':
                if (current_tab_ == 1 && current_field_ == FIELD_PRESET) {

                    delete_selected_preset();

                } else if (current_tab_ == 5 && rig_field_ == RIG_FIELD_PRESET) {

                    delete_selected_freq_preset();

                }
                break;
                
            // utils
            case '1':

                if (current_tab_ == 3) {

                    utils_selection_ = 0;
                    handle_utils_action();

                }
                break;

            case '2':

                if (current_tab_ == 3) {

                    utils_selection_ = 1;
                    handle_utils_action();

                }
                break;

            case '3':

                if (current_tab_ == 3) {

                    utils_selection_ = 2;
                    handle_utils_action();

                }
                break;

            case '4':

                if (current_tab_ == 3) {

                    utils_selection_ = 3;
                    handle_utils_action();

                }
                break;

            case '5':

                if (current_tab_ == 3) {

                    utils_selection_ = 4;
                    handle_utils_action();

                }
                break;

            case '6':

                if (current_tab_ == 3) {

                    utils_selection_ = 5;
                    handle_utils_action();

                }
                break;

            case '7':

                if (current_tab_ == 3) {

                    utils_selection_ = 6;
                    handle_utils_action();

                }
                break;

            case '8':

                if (current_tab_ == 3) {

                    state_.utils_testing_open = true;
                    utils_selection_ = UTILS_TOP_ACTIONS + 1;
                    handle_utils_action();

                }
                break;

        }
    }
    
    void handle_mouse(MEVENT& event) {
        int rows, cols;
        getmaxyx(stdscr, rows, cols);
        (void)rows;  
        
        if (event.bstate & BUTTON1_CLICKED || event.bstate & BUTTON1_PRESSED) {
            // Tab clicks
            if (event.y == 2) {
                int ntabs = tab_count();
                int tab_width = (cols - 4) / ntabs;
                if (event.x >= 2 && tab_width > 0) {
                    int idx = (event.x - 2) / tab_width;
                    if (idx >= ntabs) idx = ntabs - 1;
                    current_tab_ = idx;
                }
            }
            
            if (current_tab_ == 1 && event.x < cols/2 - 2) {
                int logical = event.y - 4 + config_scroll_;
                int field = -1;
                for (int f = 0; f < FIELD_COUNT; f++) {
                    if (should_skip_field(f)) continue;
                    if (config_field_row(f) == logical) { field = f; break; }
                }
                if (field == FIELD_CONFIG_TOKEN)
                    open_config_token();
                if (field == FIELD_CSMA_INFO)
                    show_csma_help_ = true;
                if (field == FIELD_CSMA_HELP)
                    show_sync_help_ = true;
                
                if (field >= 0 && field < FIELD_COUNT) {
                    current_field_ = field;
                    
                    // Handle clicks on interactive elements
                    if (field == FIELD_MODEM_TYPE) {
                        int idx = modem_tab_at(event.x, cols);
                        if (idx >= 0) {
                            if (idx != state_.modem_type_index) {
                                state_.modem_type_index = idx;
                                apply_settings();
                            }
                        } else if (event.x >= 18) {
                            adjust_field(event.x < 22 ? -1 : 1);
                        }
                    } else if (field == FIELD_PRESET) {
                        // Click on preset - determine action by position
                        if (event.x >= 15 && event.x < 18 && !state_.presets.empty()) {
                            // Left arrow
                            state_.selected_preset--;
                            if (state_.selected_preset < 0)
                                state_.selected_preset = state_.presets.size() - 1;
                        } else if (event.x >= 18 && event.x < 28 && !state_.presets.empty()) {
                            // Name area - load on click
                            load_selected_preset();
                        } else if (event.x >= 28 && event.x < 34 && !state_.presets.empty()) {
                            // Right arrow area
                            state_.selected_preset++;
                            if (state_.selected_preset >= (int)state_.presets.size())
                                state_.selected_preset = 0;
                        }
                    } else if (event.x >= 18) {
                        // Value area clicks for other fields
                        if (field == FIELD_CALLSIGN) {
                            edit_text_field(field);
                        } else if (field == FIELD_PTT_TYPE) {
                            show_ptt_type_dialog();
                        } else if (field >= FIELD_MODULATION) {
                            if (event.x < 22) adjust_field(-1);
                            else adjust_field(1);
                        }
                    }
                }
            }
        }
        
        // Scroll wheel in log
        if (current_tab_ == 2) {
            if (event.bstate & BUTTON4_PRESSED) {
                if (log_scroll_ > 0) {
                    log_scroll_--;
                    log_follow_ = false;
                }
            } else if (event.bstate & BUTTON5_PRESSED) {
                log_scroll_++;
            }
        }

        if (current_tab_ == 3) {
            if (event.bstate & BUTTON4_PRESSED) {
                if (utils_scroll_ > 0) utils_scroll_--;
            } else if (event.bstate & BUTTON5_PRESSED) {
                utils_scroll_++;
            }
        }
    }
    
    // line input at (y, x). returns true on enter, false if cancelled with esc.
    // buf must hold at least max_len + 1 bytes.
    bool prompt_input(int y, int x, char* buf, int max_len) {
        curs_set(1);
        nodelay(stdscr, FALSE);

        int len = (int)strlen(buf);
        bool accepted = false;

        while (true) {
            mvhline(y, x, ' ', max_len);
            mvaddnstr(y, x, buf, len);
            move(y, x + len);
            refresh();

            int ch = getch();
            if (ch == 27) {
                break;
            } else if (ch == '\n' || ch == KEY_ENTER) {
                accepted = true;
                break;
            } else if (ch == KEY_BACKSPACE || ch == 127 || ch == 8) {
                if (len > 0) buf[--len] = '\0';
            } else if (ch >= 32 && ch < 127 && len < max_len) {
                buf[len++] = (char)ch;
                buf[len] = '\0';
            }
        }

        nodelay(stdscr, TRUE);
        curs_set(0);
        return accepted;
    }

    void edit_text_field(int field) {
        int col = 16;
        int max_len;

        if (field == FIELD_CALLSIGN) {
            max_len = 9;
        } else if (field == FIELD_HAMLIB_DEVICE) {
            int rows, cols;
            getmaxyx(stdscr, rows, cols);
            max_len = std::max(20, std::min(40, cols / 2 - 2 - col - 1));
        } else if (field == FIELD_COM_PORT) {
            max_len = 20;
#ifdef WITH_CM108
        } else if (field == FIELD_CM108_GPIO) {
            max_len = 1;
#endif
        } else if (field == FIELD_NET_PORT) {
            max_len = 5;
        } else if (field == FIELD_CONTROL_PORT) {
            max_len = 5;
        } else {
            return; // not text editable
        }

        int row = 4 + config_field_row(field) - config_scroll_;
        if (row < 4) return;
        
        // Clear the value area
        move(row, col);
        for (int i = 0; i < 20; i++) addch(' ');

        char buf[64] = {0};
        if (!prompt_input(row, col, buf, max_len)) return;

        if (strlen(buf) > 0) {
            if (field == FIELD_CALLSIGN) {
                for (char* p = buf; *p; p++) *p = toupper(*p);
                if (!ModemConfig::valid_callsign(buf)) {
                    state_.add_log(std::string("(!) Invalid callsign '") + buf +
                                   "' (A-Z 0-9 / only, 1-9 chars), keeping " +
                                   state_.callsign);
                    return;
                }
                state_.callsign = buf;
                apply_settings();
            } else if (field == FIELD_HAMLIB_DEVICE) {
                if (state_.hamlib_device != buf) {
                    state_.hamlib_device = buf;
                    state_.add_log("(!) Rig device changed, restart required");
                    apply_settings();
                }
            } else if (field == FIELD_COM_PORT) {
                state_.com_port = buf;
                state_.add_log("(!) COM port changed, restart required");
                apply_settings();
#ifdef WITH_CM108
            } else if (field == FIELD_CM108_GPIO) {
                try {
                    int gpio = std::stoi(buf);
                    if (gpio >= 1 && gpio <= 4) {
                        state_.cm108_gpio = gpio;
                        apply_settings();
                    }
                } catch (...) {}
#endif
            } else if (field == FIELD_NET_PORT) {
                try {
                    int port = std::stoi(buf);
                    if (port >= 1024 && port <= 65535) {
                        state_.port = port;
                        state_.add_log("(!) KISS port changed, restart required");
                        apply_settings();
                    }
                } catch (...) {}
            } else if (field == FIELD_CONTROL_PORT) {
                try {
                    int port = std::stoi(buf);
                    if (port >= 1024 && port <= 65535) {
                        state_.control_port = port;
                        state_.add_log("(!) Control port changed, restart required");
                        apply_settings();
                    }
                } catch (...) {}
            }
        }
    }

    void compose_message() {
        int rows, cols;
        getmaxyx(stdscr, rows, cols);
        (void)cols;

        move(rows - 3, 0);
        clrtoeol();
        attron(A_BOLD);
        mvaddstr(rows - 3, 2, "Test msg> ");
        attroff(A_BOLD);

        char buf[TNCUIState::MAX_MESSAGE_CHARS + 1] = {0};
        if (!prompt_input(rows - 3, 12, buf, TNCUIState::MAX_MESSAGE_CHARS)) return;

        std::string text(buf);
        if (text.empty()) return;

        std::string payload = "M73:" + state_.callsign + ":" + text;
        if (!state_.fragmentation_enabled && (int)payload.size() > state_.mtu_bytes) {
            state_.add_log("MSG too long for mode (" + std::to_string(payload.size()) +
                           " > " + std::to_string(state_.mtu_bytes) + "B), enable fragmentation");
            return;
        }
        if (state_.on_send_data) {
            state_.on_send_data(std::vector<uint8_t>(payload.begin(), payload.end()));
            state_.add_message(state_.callsign, text, true);
        }
    }

    int csma_mode() const {
        if (!state_.csma_sync_only) return 0;
        return state_.csma_ranked ? 2 : 1;
    }

    void set_csma_mode(int m) {
        state_.csma_sync_only = (m >= 1);
        state_.csma_ranked = (m == 2);
        if (m == 2 && !state_.tx_lead_tone) {
            state_.tx_lead_tone = true;
            state_.add_log("Ranked: lead tone enabled");
        }
    }

    ConfigToken::Profile profile_from_state() {
        ConfigToken::Profile p;
        p.modem_type = state_.modem_type_index;
        p.mfsk_mode = state_.mfsk_mode_index;
        p.robust_mode = state_.robust_mode_index;
        p.modulation = state_.modulation_index;
        p.code_rate = state_.code_rate_index;
        p.frame_size = state_.frame_size;
        p.postamble = state_.postamble ? 1 : 0;
        p.center_freq = state_.center_freq;
        p.csma_enabled = state_.csma_enabled ? 1 : 0;
        p.csma_mode = csma_mode();
        p.csma_band = state_.csma_band;
        p.fast_floor = state_.csma_fast_floor ? 1 : 0;
        p.lead_tone = state_.tx_lead_tone ? 1 : 0;
        p.threshold_db = (int)lroundf(state_.carrier_threshold_db);
        p.quiet_ms = state_.csma_quiet_ms;
        p.cw = state_.csma_cw;
        p.slot_ms = state_.slot_time_ms;
        p.burst = state_.csma_burst;
        p.dither_ms = state_.csma_responder_dither;
        p.p_persistence = state_.p_persistence;
        p.fragmentation = state_.fragmentation_enabled ? 1 : 0;
        p.tx_blanking = state_.tx_blanking_enabled ? 1 : 0;
        p.rx_ofdm = state_.ofdm_rx_enabled ? 1 : 0;
        p.rx_robust = state_.robust_rx_enabled ? 1 : 0;
        p.rx_mfsk = state_.mfsk_rx_enabled ? 1 : 0;
        p.vox_freq = state_.vox_tone_freq;
        p.vox_lead_ms = state_.vox_lead_ms;
        p.vox_tail_ms = state_.vox_tail_ms;
        return p;
    }

    void apply_profile(const ConfigToken::Profile& p) {
        state_.modem_type_index = std::clamp(p.modem_type, 0, 2);
        state_.mfsk_mode_index = std::clamp(p.mfsk_mode, 0, 3);
        state_.robust_mode_index = std::clamp(p.robust_mode, 0, ROBUST_MODE_COUNT - 1);
        state_.modulation_index = std::clamp(p.modulation, 0, 7);
        state_.code_rate_index = std::clamp(p.code_rate, 0, 4);
        state_.frame_size = std::clamp(p.frame_size, 0, 3);
        state_.postamble = p.postamble != 0;
        state_.center_freq = std::clamp(p.center_freq, 300, 2700);
        state_.clamp_micro();
        state_.csma_enabled = p.csma_enabled != 0;
        state_.tx_lead_tone = p.lead_tone != 0;
        state_.csma_band = std::clamp(p.csma_band, 0, 1);
        set_csma_mode(std::clamp(p.csma_mode, 0, 2));
        state_.csma_fast_floor = p.fast_floor != 0;
        state_.carrier_threshold_db = (float)std::clamp(p.threshold_db, -63, 0);
        state_.csma_quiet_ms = std::clamp(p.quiet_ms, 0, 3100);
        state_.csma_cw = std::clamp(p.cw, 2, 15);
        state_.slot_time_ms = std::clamp(p.slot_ms, 50, 1550);
        state_.csma_burst = std::clamp(p.burst, 1, 7);
        state_.csma_responder_dither = std::clamp(p.dither_ms, 0, 750);
        state_.p_persistence = std::clamp(p.p_persistence, 0, 255);
        state_.fragmentation_enabled = p.fragmentation != 0;
        state_.tx_blanking_enabled = p.tx_blanking != 0;
        state_.ofdm_rx_enabled = p.rx_ofdm != 0;
        state_.robust_rx_enabled = p.rx_robust != 0;
        state_.mfsk_rx_enabled = p.rx_mfsk != 0;
        state_.vox_tone_freq = std::clamp(p.vox_freq, 300, 3000);
        state_.vox_lead_ms = std::clamp(p.vox_lead_ms, 0, 310);
        state_.vox_tail_ms = std::clamp(p.vox_tail_ms, 0, 310);
        state_.update_modem_info();
        apply_settings();
    }

    void open_config_token() {
        int rows, cols;
        getmaxyx(stdscr, rows, cols);
        std::string mine = ConfigToken::encode(profile_from_state());
        int w = 56, h = 14;
        int x0 = (cols - w) / 2, y0 = (rows - h) / 2;
        if (x0 < 0) x0 = 0;
        if (y0 < 0) y0 = 0;
        attron(COLOR_PAIR(4));
        for (int y = y0; y < y0 + h && y < rows; y++)
            mvhline(y, x0, ' ', w);
        mvhline(y0, x0, ACS_HLINE, w);
        mvhline(y0 + h - 1, x0, ACS_HLINE, w);
        mvvline(y0, x0, ACS_VLINE, h);
        mvvline(y0, x0 + w - 1, ACS_VLINE, h);
        mvaddch(y0, x0, ACS_ULCORNER);
        mvaddch(y0, x0 + w - 1, ACS_URCORNER);
        mvaddch(y0 + h - 1, x0, ACS_LLCORNER);
        mvaddch(y0 + h - 1, x0 + w - 1, ACS_LRCORNER);
        attron(A_BOLD);
        mvaddstr(y0, x0 + 3, " CONFIG TOKEN ");
        attroff(A_BOLD);
        attroff(COLOR_PAIR(4));

        int y = y0 + 2, lx = x0 + 2;
        mvaddstr(y++, lx, "Share this so others match your setup:");
        y++;
        attron(A_BOLD);
        mvaddstr(y++, lx + 1, mine.c_str());
        attroff(A_BOLD);
        y++;
        attron(A_DIM);
        mvaddstr(y++, lx, "Mouse mode is on, so hold Shift and drag to select.");
        mvaddstr(y++, lx, "Copy with Ctrl+Shift+C, paste with Ctrl+Shift+V.");
        mvaddstr(y++, lx, "Right-click also pastes in Windows Terminal.");
        attroff(A_DIM);
        y++;
        mvaddstr(y++, lx, "Paste one to apply, or leave blank:");

        char buf[160] = {0};
        if (!prompt_input(y, lx + 1, buf, 44) || strlen(buf) == 0)
            return;

        ConfigToken::Profile p = profile_from_state();
        std::string err;
        ConfigToken::Result res;
        if (!ConfigToken::decode(buf, &p, &err, &res)) {
            state_.add_log("(!) Config token: " + err);
            return;
        }
        apply_profile(p);
        state_.add_log("Config token applied, " + std::to_string(res.applied) +
                       " settings changed");
        if (res.extra_bits > 0)
            state_.add_log("(!) Token also has newer settings this build ignores");
    }

    bool should_skip_field(int field) {
        if (field == FIELD_FREQ) return true;
        
        // TX blanking is forced on while CSMA is enabled
        if (state_.csma_enabled && field == FIELD_TX_BLANKING) return true;
        // Hide OFDM-only fields when in MFSK mode
        if (state_.modem_type_index != 0) {
            if (field == FIELD_MODULATION || field == FIELD_CODERATE ||
                field == FIELD_FRAMESIZE || field == FIELD_POSTAMBLE)
                return true;
        }
        if (state_.modem_type_index != 1 && field == FIELD_MFSK_MODE) return true;
        if (state_.modem_type_index != 2 &&
            (field == FIELD_ROBUST_MODE || field == FIELD_ROBUST_MTU)) return true;
        // RIGCTL has its own TX Drive on the RIG tab, so don't offer it twice
        if (rig_ui() && field == FIELD_TX_LEVEL) return true;
        if (state_.ptt_type_index != 2) {  // not VOX
            if (field == FIELD_VOX_FREQ || field == FIELD_VOX_LEAD || field == FIELD_VOX_TAIL) {
                return true;
            }
        }
        if (state_.ptt_type_index != 3) {  // not COM
            if (field == FIELD_COM_PORT || field == FIELD_COM_LINE || field == FIELD_COM_INVERT) {
                return true;
            }
        }
        if (state_.ptt_type_index != 5) {
            if (field == FIELD_HAMLIB_MODEL || field == FIELD_HAMLIB_DEVICE || field == FIELD_HAMLIB_BAUD)
                return true;
        }
#ifdef WITH_CM108
        if (state_.ptt_type_index != 4) {  // not CM108
            if (field == FIELD_CM108_GPIO || field == FIELD_CM108_DEVICE) {
                return true;
            }
        }
#endif
        if (!state_.csma_advanced_open) {
            if (field == FIELD_CSMA_QUIET || field == FIELD_CSMA_CW ||
                field == FIELD_LEAD_TONE || field == FIELD_RESP_DITHER ||
                field == FIELD_CSMA_BURST || field == FIELD_FAST_FLOOR ||
                field == FIELD_BEACON_INT)
                return true;
        }
        {
            int m = csma_mode();
            if (m != 0 && (field == FIELD_THRESHOLD || field == FIELD_CSMA_CW))
                return true;
            if (m != 1 && field == FIELD_FAST_FLOOR)
                return true;
            if (m != 2 && field == FIELD_BEACON_INT)
                return true;
            if (m == 2 && (field == FIELD_CSMA_QUIET ||
                           field == FIELD_LEAD_TONE ||
                           field == FIELD_RESP_DITHER ||
                           field == FIELD_CSMA_BAND ||
                           field == FIELD_CSMA_PRESET))
                return true;
        }
        return false;
    }
    

    // must mirror draw_config's order/spacing exactly -- a drifted map
    // scrolls the selection off-screen and breaks mouse click mapping
    int config_field_row(int field) const {
        int row = 0;
        row++;
        if (field == FIELD_CALLSIGN) return row;
        row++;
        if (field == FIELD_CONFIG_TOKEN) return row;
        row++;
        if (field == FIELD_MODEM_TYPE) return row;
        row++;
        if (state_.modem_type_index == 0) {
            if (field == FIELD_MODULATION) return row;
            row++;
            if (field == FIELD_CODERATE) return row;
            row++;
            if (field == FIELD_FRAMESIZE) return row;
            row++;
            if (field == FIELD_POSTAMBLE) return row;
            row++;
        } else if (state_.modem_type_index == 1) {
            if (field == FIELD_MFSK_MODE) return row;
            row++;
        } else {
            if (field == FIELD_ROBUST_MODE) return row;
            row++;
            if (field == FIELD_ROBUST_MTU) return row;
            row++;
        }
        row++;
        row++;
        if (field == FIELD_CSMA) return row;
        row++;
        if (field == FIELD_CSMA_MODE) return row;
        row++;
        if (field == FIELD_CSMA_HELP) return row;
        row++;
        if (csma_mode() == 0) {
            if (field == FIELD_THRESHOLD) return row;
            row++;
            row++;
        }
        if (csma_mode() != 2) {
            if (field == FIELD_CSMA_BAND) return row;
            row++;
            if (field == FIELD_CSMA_PRESET) return row;
            row++;
        }
        if (field == FIELD_CSMA_ADV) return row;
        row++;
        if (state_.csma_advanced_open) {
            int m = csma_mode();
            if (m != 2) {
                if (field == FIELD_CSMA_QUIET) return row;
                row++;
            }
            if (m == 0) {
                if (field == FIELD_CSMA_CW) return row;
                row++;
            }
            if (m != 2) {
                if (field == FIELD_LEAD_TONE) return row;
                row++;
                if (field == FIELD_RESP_DITHER) return row;
                row++;
            }
            if (field == FIELD_CSMA_BURST) return row;
            row++;
            if (m == 1) {
                if (field == FIELD_FAST_FLOOR) return row;
                row++;
            }
            if (m == 2) {
                if (field == FIELD_BEACON_INT) return row;
                row++;
            }
        }
        if (field == FIELD_CSMA_INFO) return row;
        row += 2;
        row += 2;
        if (field == FIELD_FRAGMENTATION) return row;
        row += 2;
        if (!state_.csma_enabled) {
            row++;
            if (field == FIELD_TX_BLANKING) return row;
            row += 2;
        }
        row++;
        if (field == FIELD_RX_OFDM) return row;
        row++;
        if (field == FIELD_RX_ROBUST) return row;
        row++;
        if (field == FIELD_RX_MFSK) return row;
        row += 2;
        row++;
        if (field == FIELD_AUDIO_INPUT) return row;
        row++;
        if (field == FIELD_AUDIO_OUTPUT) return row;
        row++;
        if (field == FIELD_TX_LEVEL) return row;
        row++;
        if (field == FIELD_PTT_TYPE) return row;
        row++;
        if (state_.ptt_type_index == 2) {
            if (field == FIELD_VOX_FREQ) return row;
            row++;
            if (field == FIELD_VOX_LEAD) return row;
            row++;
            if (field == FIELD_VOX_TAIL) return row;
            row++;
        }
        if (state_.ptt_type_index == 3) {
            if (field == FIELD_COM_PORT) return row;
            row++;
            if (field == FIELD_COM_LINE) return row;
            row++;
            if (field == FIELD_COM_INVERT) return row;
            row++;
        }
        if (state_.ptt_type_index == 5) {
            if (field == FIELD_HAMLIB_MODEL) return row;
            row++;
            if (field == FIELD_HAMLIB_DEVICE) return row;
            row++;
            if (field == FIELD_HAMLIB_BAUD) return row;
            row++;
        }
#ifdef WITH_CM108
        if (state_.ptt_type_index == 4) {
            if (field == FIELD_CM108_GPIO) return row;
            row++;
            if (field == FIELD_CM108_DEVICE) return row;
            row++;
        }
#endif
        if (field == FIELD_TX_DELAY) return row;
        row++;
        row++;
        row++;
        if (field == FIELD_NET_PORT) return row;
        row++;
        if (field == FIELD_CONTROL_PORT) return row;
        row++;
        if (field == FIELD_LAN_MODE) return row;
        row += 2;
        if (field == FIELD_PRESET) return row;
        return row;
    }

    int csma_preset_match() const {
        for (int i = 0; i < CSMA_PRESET_COUNT; ++i) {
            const CsmaPreset& p = CSMA_PRESETS[state_.csma_band & 1][i];
            if (state_.csma_quiet_ms == p.quiet_ms &&
                state_.csma_cw == p.cw &&
                state_.slot_time_ms == p.slot_ms &&
                state_.csma_burst == p.burst &&
                state_.csma_responder_dither == p.dither &&
                state_.tx_lead_tone == p.lead_tone)
                return i;
        }
        return -1;
    }

    void csma_apply_preset(int idx) {
        const CsmaPreset& p = CSMA_PRESETS[state_.csma_band & 1][idx];
        state_.csma_quiet_ms = p.quiet_ms;
        state_.csma_cw = p.cw;
        state_.slot_time_ms = p.slot_ms;
        state_.csma_burst = p.burst;
        state_.csma_responder_dither = p.dither;
        state_.tx_lead_tone = p.lead_tone;
        state_.add_log(std::string("CSMA preset: ") +
                       CSMA_BAND_NAMES[state_.csma_band & 1] + " " + p.name);
    }

    void adjust_field(int delta) {
        switch (current_field_) {
            case FIELD_MODEM_TYPE:
                state_.modem_type_index = (state_.modem_type_index + delta + 3) % 3;
                state_.update_modem_info();
                break;
            case FIELD_ROBUST_MODE: {
                int n = (int)ROBUST_MODE_OPTIONS.size();
                int base = (robust_base_index(state_.robust_mode_index) + delta + n) % n;
                state_.robust_mode_index = robust_mode_of(base,
                    RobustParams::is_short((RobustMode)state_.robust_mode_index));
                state_.update_modem_info();
                break;
            }
            case FIELD_ROBUST_MTU:
                state_.robust_mode_index = robust_mode_of(
                    robust_base_index(state_.robust_mode_index),
                    !RobustParams::is_short((RobustMode)state_.robust_mode_index));
                state_.update_modem_info();
                break;
            case FIELD_MFSK_MODE:
                state_.mfsk_mode_index = (state_.mfsk_mode_index + delta + 4) % 4;
                state_.update_modem_info();
                break;
            case FIELD_MODULATION:
                state_.modulation_index = (state_.modulation_index + delta + 8) % 8;
                state_.clamp_micro();
                break;
            case FIELD_CODERATE:
                do {
                    state_.code_rate_index = (state_.code_rate_index + delta +
                        (int)CODE_RATE_OPTIONS.size()) % (int)CODE_RATE_OPTIONS.size();
                } while (state_.code_rate_index >= 5);
                state_.clamp_micro();
                break;
            case FIELD_FRAMESIZE:
                do {
                    state_.frame_size = (state_.frame_size + delta + 4) % 4;
                } while (state_.frame_size == 3 && !state_.micro_allowed());
                break;
            case FIELD_POSTAMBLE:
                state_.postamble = !state_.postamble;
                break;
            case FIELD_HAMLIB_BAUD: {
                static const int bauds[] = {0, 4800, 9600, 19200, 38400, 57600, 115200};
                int n = (int)(sizeof(bauds) / sizeof(bauds[0]));
                int cur = 0;
                for (int i = 0; i < n; i++) if (bauds[i] == state_.hamlib_baud) cur = i;
                state_.hamlib_baud = bauds[(cur + delta + n) % n];
                break;
            }
            case FIELD_LAN_MODE: {
                bool lan = state_.bind_address == "0.0.0.0" &&
                           state_.control_bind_address == "0.0.0.0";
                if (lan) {
                    state_.bind_address = "127.0.0.1";
                    state_.control_bind_address = "127.0.0.1";
                    state_.add_log("LAN mode disabled (restart to apply)");
                } else {
                    lan_dialog_ = true;
                }
                break;
            }
            case FIELD_CSMA:
                state_.csma_enabled = !state_.csma_enabled;
                if (state_.csma_enabled) state_.tx_blanking_enabled = true;
                break;
            case FIELD_THRESHOLD:
                state_.carrier_threshold_db += delta * 2;
                state_.carrier_threshold_db = std::max(-80.0f, std::min(0.0f, state_.carrier_threshold_db));
                break;
            case FIELD_CSMA_MODE:
                set_csma_mode((csma_mode() + delta % 3 + 3) % 3);
                break;
            case FIELD_CSMA_BAND: {
                int matched = csma_preset_match();
                state_.csma_band = (state_.csma_band + delta + 2) % 2;
                if (matched >= 0)
                    csma_apply_preset(matched);
                break;
            }
            case FIELD_CSMA_PRESET: {
                int idx = csma_preset_match();
                idx = idx < 0 ? (delta > 0 ? 0 : CSMA_PRESET_COUNT - 1)
                              : (idx + delta + CSMA_PRESET_COUNT) % CSMA_PRESET_COUNT;
                csma_apply_preset(idx);
                break;
            }
            case FIELD_CSMA_ADV:
                state_.csma_advanced_open = !state_.csma_advanced_open;
                return;
            case FIELD_CSMA_QUIET:
                state_.csma_quiet_ms += delta * 250;
                state_.csma_quiet_ms = std::max(0, std::min(10000, state_.csma_quiet_ms));
                break;
            case FIELD_CSMA_CW:
                state_.csma_cw += delta;
                state_.csma_cw = std::max(2, std::min(32, state_.csma_cw));
                break;
            case FIELD_LEAD_TONE:
                state_.tx_lead_tone = !state_.tx_lead_tone;
                break;
            case FIELD_RESP_DITHER:
                state_.csma_responder_dither += delta * 100;
                state_.csma_responder_dither = std::max(0, std::min(3000, state_.csma_responder_dither));
                break;
            case FIELD_CSMA_BURST:
                state_.csma_burst += delta;
                state_.csma_burst = std::max(1, std::min(4, state_.csma_burst));
                break;
            case FIELD_FAST_FLOOR:
                state_.csma_fast_floor = !state_.csma_fast_floor;
                break;
            case FIELD_BEACON_INT:
                state_.beacon_interval_s += delta * 15;
                state_.beacon_interval_s = std::max(45, std::min(90, state_.beacon_interval_s));
                break;
            case FIELD_FRAGMENTATION:
                state_.fragmentation_enabled = !state_.fragmentation_enabled;
                state_.update_modem_info();  // Update random_data_size limits
                state_.add_log("(!) Fragmentation changed, restart required");
                break;
            case FIELD_TX_BLANKING:
                state_.tx_blanking_enabled = !state_.tx_blanking_enabled;
                apply_settings();
                state_.add_log(std::string("TX blanking ") + (state_.tx_blanking_enabled ? "enabled" : "disabled"));
                break;
            case FIELD_RX_OFDM:
                toggle_rx_decoder(state_.ofdm_rx_enabled, "OFDM");
                break;
            case FIELD_RX_ROBUST:
                toggle_rx_decoder(state_.robust_rx_enabled, "ROBUST");
                break;
            case FIELD_RX_MFSK:
                toggle_rx_decoder(state_.mfsk_rx_enabled, "MFSK");
                break;
            case FIELD_AUDIO_INPUT:
                break;
            case FIELD_AUDIO_OUTPUT:
                break;
            case FIELD_PTT_TYPE:
                break;
            case FIELD_VOX_FREQ:
                state_.vox_tone_freq += delta * 100;
                state_.vox_tone_freq = std::max(300, std::min(2500, state_.vox_tone_freq));
                break;
            case FIELD_TX_DELAY:
                state_.tx_delay_ms += delta * 50;
                state_.tx_delay_ms = std::max(250, std::min(2500, state_.tx_delay_ms));
                break;
            case FIELD_VOX_LEAD:
                state_.vox_lead_ms += delta * 50;
                state_.vox_lead_ms = std::max(50, std::min(2000, state_.vox_lead_ms));
                break;
            case FIELD_VOX_TAIL:
                state_.vox_tail_ms += delta * 50;
                state_.vox_tail_ms = std::max(50, std::min(2000, state_.vox_tail_ms));
                break;
            case FIELD_TX_LEVEL: {
                int pct = (int)lround(state_.tx_drive.load() * 100) + delta * 5;
                state_.tx_drive = std::max(5, std::min(100, pct)) / 100.0f;
                break;
            }
            case FIELD_COM_PORT:
                break;
            case FIELD_COM_LINE:
                state_.com_ptt_line = (state_.com_ptt_line + delta + 3) % 3;
                break;
            case FIELD_COM_INVERT:
                if (delta > 0) {
                    if (!state_.com_invert_dtr && !state_.com_invert_rts) {
                        state_.com_invert_dtr = true;
                    } else if (state_.com_invert_dtr && !state_.com_invert_rts) {
                        state_.com_invert_dtr = false;
                        state_.com_invert_rts = true;
                    } else if (!state_.com_invert_dtr && state_.com_invert_rts) {
                        state_.com_invert_dtr = true;
                    } else {
                        state_.com_invert_dtr = false;
                        state_.com_invert_rts = false;
                    }
                } else {
                    if (!state_.com_invert_dtr && !state_.com_invert_rts) {
                        state_.com_invert_dtr = true;
                        state_.com_invert_rts = true;
                    } else if (state_.com_invert_dtr && state_.com_invert_rts) {
                        state_.com_invert_dtr = false;
                    } else if (!state_.com_invert_dtr && state_.com_invert_rts) {
                        state_.com_invert_rts = false;
                        state_.com_invert_dtr = true;
                    } else {
                        state_.com_invert_dtr = false;
                    }
                }
                break;
            case FIELD_NET_PORT:
                state_.port += delta * 1;
                state_.port = std::max(1024, std::min(65535, state_.port));
                break;
            default:
                return;
        }
        apply_settings();
    }
    
    void apply_settings() {

        state_.update_modem_info();
        

        if (state_.on_settings_changed) {
            state_.on_settings_changed(state_);
        }
        
        
        state_.save_settings();
    }
    
    struct HamlibModel { int id; std::string label; };

    const std::vector<HamlibModel>& hamlib_models() {
        static std::vector<HamlibModel> models;
        static bool loaded = false;
        if (!loaded) {
            loaded = true;
#ifdef WITH_HAMLIB
            std::string raw = hamlib_list_models();
            size_t pos = 0;
            while (pos < raw.size()) {
                size_t e = raw.find('\n', pos);
                if (e == std::string::npos) e = raw.size();
                std::string line = raw.substr(pos, e - pos);
                pos = e + 1;
                size_t a = line.find('|'), b = line.rfind('|');
                if (a == std::string::npos || b == a) continue;
                models.push_back({atoi(line.substr(0, a).c_str()),
                                  line.substr(a + 1, b - a - 1) + " " + line.substr(b + 1)});
            }
            std::sort(models.begin(), models.end(),
                      [](const HamlibModel& x, const HamlibModel& y) { return x.label < y.label; });
#endif
        }
        return models;
    }

    std::string hamlib_model_label(int id) {
        for (const auto& m : hamlib_models())
            if (m.id == id) return m.label;
        return "model " + std::to_string(id);
    }

    void show_hamlib_model_dialog() {
        const auto& models = hamlib_models();
        if (models.empty()) {
            state_.add_log("Hamlib: no rig list (built without Hamlib?)");
            return;
        }
        int rows, cols;
        getmaxyx(stdscr, rows, cols);
        int dialog_w = std::min(cols - 4, 50);
        int dialog_h = std::min(rows - 4, 20);
        int dialog_x = (cols - dialog_w) / 2;
        int dialog_y = (rows - dialog_h) / 2;
        int visible = dialog_h - 4;
        std::string filter;
        int selection = 0, scroll = 0;
        nodelay(stdscr, FALSE);
        while (true) {
            std::vector<int> shown;
            for (int i = 0; i < (int)models.size(); i++) {
                if (filter.empty()) { shown.push_back(i); continue; }
                std::string l = models[i].label, f = filter;
                for (auto& c : l) c = (char)tolower((unsigned char)c);
                for (auto& c : f) c = (char)tolower((unsigned char)c);
                if (l.find(f) != std::string::npos) shown.push_back(i);
            }
            if (selection >= (int)shown.size()) selection = std::max(0, (int)shown.size() - 1);
            if (selection < scroll) scroll = selection;
            if (selection >= scroll + visible) scroll = selection - visible + 1;
            for (int y = dialog_y; y < dialog_y + dialog_h; y++) {
                move(y, dialog_x);
                for (int x = 0; x < dialog_w; x++) addch(' ');
            }
            attron(COLOR_PAIR(4) | A_BOLD);
            draw_box(dialog_y, dialog_x, dialog_h, dialog_w);
            mvaddstr(dialog_y, dialog_x + 2, " Hamlib Rig ");
            attroff(COLOR_PAIR(4) | A_BOLD);
            mvaddnstr(dialog_y + 1, dialog_x + 2, ("Filter: " + filter).c_str(), dialog_w - 4);
            for (int i = 0; i < visible && scroll + i < (int)shown.size(); i++) {
                const auto& m = models[shown[scroll + i]];
                int y = dialog_y + 2 + i;
                bool sel = scroll + i == selection;
                if (sel) attron(COLOR_PAIR(4) | A_BOLD);
                std::string line = (sel ? "> " : "  ") + std::to_string(m.id) + " " + m.label;
                mvaddnstr(y, dialog_x + 1, line.c_str(), dialog_w - 2);
                if (sel) attroff(COLOR_PAIR(4) | A_BOLD);
            }
            attron(A_DIM);
            mvaddstr(dialog_y + dialog_h - 1, dialog_x + 2, " type to filter  Enter=OK  Esc=Cancel ");
            attroff(A_DIM);
            refresh();
            int ch = getch();
            if (ch == 27) break;
            if ((ch == '\n' || ch == KEY_ENTER) && !shown.empty()) {
                state_.hamlib_model = models[shown[selection]].id;
                state_.add_log("Hamlib rig: " + models[shown[selection]].label + " (restart to apply)");
                apply_settings();
                break;
            }
            if (ch == KEY_UP && selection > 0) selection--;
            else if (ch == KEY_DOWN && selection + 1 < (int)shown.size()) selection++;
            else if (ch == KEY_NPAGE) selection = std::min((int)shown.size() - 1, selection + visible);
            else if (ch == KEY_PPAGE) selection = std::max(0, selection - visible);
            else if ((ch == KEY_BACKSPACE || ch == 127 || ch == 8) && !filter.empty()) filter.pop_back();
            else if (ch >= 32 && ch < 127 && filter.size() < 30) { filter += (char)ch; selection = 0; }
        }
        nodelay(stdscr, TRUE);
    }

    void show_ptt_type_dialog() {
        int rows, cols;
        getmaxyx(stdscr, rows, cols);

        static const char* const descriptions[] = {
            "NONE   - PTT disabled (over the air)",
            "RIGCTL - Hamlib rigctld (network)",
            "VOX    - Tone-keyed VOX",
            "COM    - Serial port DTR/RTS",
            "CM108  - USB HID GPIO",
            "HAMLIB - Hamlib direct (serial or network rig)",
        };
        std::vector<int> items = {0, 1, 2, 3};
#ifdef WITH_CM108
        items.push_back(4);
#endif
#ifdef WITH_HAMLIB
        items.push_back(5);
#endif
        int count = (int)items.size();
        int selection = 0;
        for (int i = 0; i < count; i++) if (items[i] == state_.ptt_type_index) selection = i;

        int dialog_w = std::min(cols - 4, 42);
        int dialog_h = count + 3;
        int dialog_x = (cols - dialog_w) / 2;
        int dialog_y = (rows - dialog_h) / 2;

        nodelay(stdscr, FALSE);

        while (true) {
            for (int y = dialog_y; y < dialog_y + dialog_h; y++) {
                move(y, dialog_x);
                for (int x = 0; x < dialog_w; x++) addch(' ');
            }

            attron(COLOR_PAIR(4) | A_BOLD);
            draw_box(dialog_y, dialog_x, dialog_h, dialog_w);
            const char* title = " PTT Type ";
            mvaddstr(dialog_y, dialog_x + (dialog_w - (int)strlen(title)) / 2, title);
            attroff(COLOR_PAIR(4) | A_BOLD);

            for (int i = 0; i < count; i++) {
                int y = dialog_y + 1 + i;
                mvhline(y, dialog_x + 1, ' ', dialog_w - 2);

                if (i == selection) {
                    attron(COLOR_PAIR(4) | A_BOLD);
                    mvaddstr(y, dialog_x + 1, "> ");
                } else {
                    mvaddstr(y, dialog_x + 1, "  ");
                }

                int idx = items[i];
                std::string desc = (idx < (int)(sizeof(descriptions) / sizeof(descriptions[0]))) ?
                    descriptions[idx] : PTT_TYPE_OPTIONS[idx];
                int max_len = dialog_w - 4;
                if ((int)desc.length() > max_len) {
                    desc = desc.substr(0, max_len - 2) + "..";
                }
                addstr(desc.c_str());

                if (i == selection) {
                    attroff(COLOR_PAIR(4) | A_BOLD);
                }
            }

            attron(A_DIM);
            mvaddstr(dialog_y + dialog_h - 1, dialog_x + 2, " Enter=OK  Esc=Cancel ");
            attroff(A_DIM);

            refresh();

            int ch = getch();

            if (ch == 27 || ch == 'q') {
                break;
            } else if (ch == '\n' || ch == KEY_ENTER) {
                if (items[selection] != state_.ptt_type_index) {
                    state_.ptt_type_index = items[selection];
                    state_.add_log("PTT: " + PTT_TYPE_OPTIONS[items[selection]]);
                    apply_settings();
                }
                break;
            } else if (ch == KEY_UP || ch == 'k') {
                if (selection > 0) selection--;
            } else if (ch == KEY_DOWN || ch == 'j') {
                if (selection < count - 1) selection++;
            }
        }

        nodelay(stdscr, TRUE);
    }

    void show_com_port_dialog() {
        show_serial_port_dialog(FIELD_COM_PORT);
    }

    void show_serial_port_dialog(int field) {
        bool hamlib = (field == FIELD_HAMLIB_DEVICE);
        std::string& target = hamlib ? state_.hamlib_device : state_.com_port;
        std::vector<std::string> ports;
        std::vector<std::string> labels;
        const char* patterns[] = {"/dev/serial/by-id/*", "/dev/ttyUSB*", "/dev/ttyACM*"};
        for (const char* pat : patterns) {
            glob_t g;
            if (glob(pat, 0, nullptr, &g) == 0) {
                for (size_t i = 0; i < g.gl_pathc; i++) {
                    ports.push_back(g.gl_pathv[i]);
                    labels.push_back(g.gl_pathv[i]);
                }
            }
            globfree(&g);
        }
        ports.push_back("");
        labels.push_back(hamlib ? "[ Type manually / host:port ]" : "[ Type manually ]");

        int rows, cols;
        getmaxyx(stdscr, rows, cols);
        int dialog_w = std::min(cols - 4, 66);
        int max_visible = std::min((int)ports.size(), 12);
        int dialog_h = max_visible + 3;
        int dialog_x = (cols - dialog_w) / 2;
        int dialog_y = (rows - dialog_h) / 2;

        int selection = 0;
        for (size_t i = 0; i < ports.size(); i++) {
            if (ports[i] == target) { selection = i; break; }
        }
        int scroll_offset = 0;
        if (selection >= max_visible) scroll_offset = selection - max_visible + 1;

        nodelay(stdscr, FALSE);

        bool manual = false;
        while (true) {
            for (int y = dialog_y; y < dialog_y + dialog_h; y++) {
                move(y, dialog_x);
                for (int x = 0; x < dialog_w; x++) addch(' ');
            }
            attron(COLOR_PAIR(4) | A_BOLD);
            draw_box(dialog_y, dialog_x, dialog_h, dialog_w);
            const char* title = hamlib ? " Rig Device " : " Serial Port ";
            mvaddstr(dialog_y, dialog_x + (dialog_w - strlen(title)) / 2, title);
            attroff(COLOR_PAIR(4) | A_BOLD);

            int visible_count = std::min((int)ports.size() - scroll_offset, max_visible);
            for (int i = 0; i < visible_count; i++) {
                int idx = scroll_offset + i;
                int y = dialog_y + 1 + i;
                mvhline(y, dialog_x + 1, ' ', dialog_w - 2);
                if (idx == selection) {
                    attron(COLOR_PAIR(4) | A_BOLD);
                    mvaddstr(y, dialog_x + 1, "> ");
                } else {
                    mvaddstr(y, dialog_x + 1, "  ");
                }
                std::string label = labels[idx];
                int max_len = dialog_w - 4;
                if ((int)label.length() > max_len)
                    label = label.substr(0, max_len - 2) + "..";
                addstr(label.c_str());
                if (idx == selection) attroff(COLOR_PAIR(4) | A_BOLD);
            }

            attron(A_DIM);
            if (scroll_offset > 0)
                mvaddstr(dialog_y, dialog_x + dialog_w - 3, "^");
            if (scroll_offset + max_visible < (int)ports.size())
                mvaddstr(dialog_y + dialog_h - 1, dialog_x + dialog_w - 3, "v");
            mvaddstr(dialog_y + dialog_h - 1, dialog_x + 2, " Enter=OK  Esc=Cancel ");
            attroff(A_DIM);

            refresh();
            int ch = getch();

            if (ch == 27 || ch == 'q') {
                break;
            } else if (ch == '\n' || ch == KEY_ENTER) {
                if (selection >= 0 && selection < (int)ports.size()) {
                    if (ports[selection].empty()) {
                        manual = true;
                    } else if (ports[selection] != target) {
                        target = ports[selection];
                        state_.add_log(hamlib ? "(!) Rig device changed, restart required"
                                              : "(!) COM port changed, restart required");
                        apply_settings();
                    }
                }
                break;
            } else if (ch == KEY_UP || ch == 'k') {
                if (selection > 0) {
                    selection--;
                    if (selection < scroll_offset) scroll_offset = selection;
                }
            } else if (ch == KEY_DOWN || ch == 'j') {
                if (selection < (int)ports.size() - 1) {
                    selection++;
                    if (selection >= scroll_offset + max_visible)
                        scroll_offset = selection - max_visible + 1;
                }
            }
        }

        nodelay(stdscr, TRUE);
        if (manual) edit_text_field(field);
    }

    void show_device_select_dialog(bool is_input) {
        int rows, cols;
        getmaxyx(stdscr, rows, cols);
        
        const std::vector<std::string>& devices = is_input ? 
            state_.available_input_devices : state_.available_output_devices;
        const std::vector<std::string>& descriptions = is_input ?
            state_.input_device_descriptions : state_.output_device_descriptions;
        int& current_index = is_input ? state_.audio_input_index : state_.audio_output_index;
        std::string& current_device = is_input ? state_.audio_input_device : state_.audio_output_device;
        
        if (devices.empty()) {
            state_.add_log("No audio devices found");
            return;
        }
        
        // Dialog dimensions
        int dialog_w = std::min(cols - 4, 58);
        int max_visible = std::min((int)devices.size(), 12);
        int dialog_h = max_visible + 3;
        int dialog_x = (cols - dialog_w) / 2;
        int dialog_y = (rows - dialog_h) / 2;
        
        int selection = current_index;
        int scroll_offset = 0;
        
        if (selection >= max_visible) {
            scroll_offset = selection - max_visible + 1;
        }
        
        nodelay(stdscr, FALSE);
        
        while (true) {
            // Clear dialog area
            for (int y = dialog_y; y < dialog_y + dialog_h; y++) {
                move(y, dialog_x);
                for (int x = 0; x < dialog_w; x++) addch(' ');
            }
            
            // Draw box
            attron(COLOR_PAIR(4) | A_BOLD);
            draw_box(dialog_y, dialog_x, dialog_h, dialog_w);
            attroff(COLOR_PAIR(4) | A_BOLD);
            
            // Title
            const char* title = is_input ? " Input Device " : " Output Device ";
            attron(COLOR_PAIR(4) | A_BOLD);
            mvaddstr(dialog_y, dialog_x + (dialog_w - strlen(title)) / 2, title);
            attroff(COLOR_PAIR(4) | A_BOLD);
            
            // Draw device list
            int visible_count = std::min((int)devices.size() - scroll_offset, max_visible);
            for (int i = 0; i < visible_count; i++) {
                int dev_idx = scroll_offset + i;
                int y = dialog_y + 1 + i;
                
                mvhline(y, dialog_x + 1, ' ', dialog_w - 2);
                
                if (dev_idx == selection) {
                    attron(COLOR_PAIR(4) | A_BOLD);
                    mvaddstr(y, dialog_x + 1, "> ");
                } else {
                    mvaddstr(y, dialog_x + 1, "  ");
                }
                
                std::string desc = (dev_idx < (int)descriptions.size()) ? 
                    descriptions[dev_idx] : devices[dev_idx];
                int max_len = dialog_w - 4;
                if ((int)desc.length() > max_len) {
                    desc = desc.substr(0, max_len - 2) + "..";
                }
                addstr(desc.c_str());
                
                if (dev_idx == selection) {
                    attroff(COLOR_PAIR(4) | A_BOLD);
                }
            }
            
            // Scroll indicators
            if (scroll_offset > 0) {
                attron(A_DIM);
                mvaddstr(dialog_y, dialog_x + dialog_w - 3, "^");
                attroff(A_DIM);
            }
            if (scroll_offset + max_visible < (int)devices.size()) {
                attron(A_DIM);
                mvaddstr(dialog_y + dialog_h - 1, dialog_x + dialog_w - 3, "v");
                attroff(A_DIM);
            }
            
            // Help
            attron(A_DIM);
            mvaddstr(dialog_y + dialog_h - 1, dialog_x + 2, " Enter=OK  Esc=Cancel ");
            mvaddstr(dialog_y + dialog_h - 1, dialog_x + dialog_w - 15, "(needs restart)");
            attroff(A_DIM);
            
            refresh();
            
            int ch = getch();
            
            if (ch == 27 || ch == 'q') {
                break;
            } else if (ch == '\n' || ch == KEY_ENTER) {
                if (selection >= 0 && selection < (int)devices.size()) {
                    current_index = selection;
                    current_device = devices[selection];
                    state_.add_log(std::string(is_input ? "In: " : "Out: ") + 
                                   descriptions[selection] + " (restart to apply)");
                    apply_settings();
                }
                break;
            } else if (ch == KEY_UP || ch == 'k') {
                if (selection > 0) {
                    selection--;
                    if (selection < scroll_offset) scroll_offset = selection;
                }
            } else if (ch == KEY_DOWN || ch == 'j') {
                if (selection < (int)devices.size() - 1) {
                    selection++;
                    if (selection >= scroll_offset + max_visible) {
                        scroll_offset = selection - max_visible + 1;
                    }
                }
            } else if (ch == KEY_PPAGE) {
                selection = std::max(0, selection - max_visible);
                scroll_offset = std::max(0, scroll_offset - max_visible);
            } else if (ch == KEY_NPAGE) {
                selection = std::min((int)devices.size() - 1, selection + max_visible);
                if (selection >= scroll_offset + max_visible) {
                    scroll_offset = selection - max_visible + 1;
                }
            }
        }
        
        nodelay(stdscr, TRUE);
    }

#ifdef WITH_CM108
    void show_cm108_device_dialog() {
        int rows, cols;
        getmaxyx(stdscr, rows, cols);

        auto devices = CM108PTT::enumerate();



        std::vector<std::string> values;
        std::vector<std::string> descriptions;
        
        values.push_back("");
        descriptions.push_back("Auto (first compatible device)");
        for (const auto& d : devices) {
            std::string desc = d.chip;
            if (!d.product.empty()) desc += " - " + d.product;
            if (!d.serial.empty()) {
                values.push_back(d.serial);
                desc += " (sn " + d.serial + ")";
            } else {
                values.push_back(d.path);
                desc += " (" + d.path + ")";
            }
            descriptions.push_back(desc);
        }

        int selection = 0;
        for (int i = 1; i < (int)values.size(); i++) {
            if (values[i] == state_.cm108_device) { selection = i; break; }
        }





        if (selection == 0 && !state_.cm108_device.empty()) {
            values.push_back(state_.cm108_device);
            descriptions.push_back(state_.cm108_device + " (not connected)");
            selection = (int)values.size() - 1; 
        }


        int dialog_w = std::min(cols - 4, 58);
        int max_visible = std::min((int)values.size(), 12);
        int dialog_h = max_visible + 3;
        int dialog_x = (cols - dialog_w) / 2;
        int dialog_y = (rows - dialog_h) / 2;

        int scroll_offset = 0;
        if (selection >= max_visible) {
            scroll_offset = selection - max_visible + 1;
        }


        nodelay(stdscr, FALSE);

        while (true) {
            for (int y = dialog_y; y < dialog_y + dialog_h; y++) {
                move(y, dialog_x);
                for (int x = 0; x < dialog_w; x++) addch(' ');
            }

            attron(COLOR_PAIR(4) | A_BOLD);
            draw_box(dialog_y, dialog_x, dialog_h, dialog_w);
            attroff(COLOR_PAIR(4) | A_BOLD);

            const char* title = " CM108 Device ";
            attron(COLOR_PAIR(4) | A_BOLD);
            mvaddstr(dialog_y, dialog_x + (dialog_w - strlen(title)) / 2, title);
            attroff(COLOR_PAIR(4) | A_BOLD);

            int visible_count = std::min((int)values.size() - scroll_offset, max_visible);
            for (int i = 0; i < visible_count; i++) {
                int dev_idx = scroll_offset + i;
                int y = dialog_y + 1 + i;

                mvhline(y, dialog_x + 1, ' ', dialog_w - 2);

                if (dev_idx == selection) {
                    attron(COLOR_PAIR(4) | A_BOLD);
                    mvaddstr(y, dialog_x + 1, "> ");
                } else {
                    mvaddstr(y, dialog_x + 1, "  ");
                }

                std::string desc = descriptions[dev_idx];
                int max_len = dialog_w - 4;
                if ((int)desc.length() > max_len) {
                    desc = desc.substr(0, max_len - 2) + "..";
                }
                addstr(desc.c_str());

                if (dev_idx == selection) {
                    attroff(COLOR_PAIR(4) | A_BOLD);
                }
            }

            if (scroll_offset > 0) {
                attron(A_DIM);
                mvaddstr(dialog_y, dialog_x + dialog_w - 3, "^");
                attroff(A_DIM);
            }
            if (scroll_offset + max_visible < (int)values.size()) {
                attron(A_DIM);
                mvaddstr(dialog_y + dialog_h - 1, dialog_x + dialog_w - 3, "v");
                attroff(A_DIM);
            }

            attron(A_DIM);
            mvaddstr(dialog_y + dialog_h - 1, dialog_x + 2, " Enter=OK  Esc=Cancel ");
            mvaddstr(dialog_y + dialog_h - 1, dialog_x + dialog_w - 15, "(needs restart)");
            attroff(A_DIM);

            refresh();

            int ch = getch();

            if (ch == 27 || ch == 'q') {
                break;
            } else if (ch == '\n' || ch == KEY_ENTER) {
                if (selection >= 0 && selection < (int)values.size()) {
                    state_.cm108_device = values[selection];
                    state_.add_log("CM108: " + descriptions[selection] + " (restart to apply)");
                    apply_settings();
                }
                break;
            } else if (ch == KEY_UP || ch == 'k') {
                if (selection > 0) {
                    selection--;
                    if (selection < scroll_offset) scroll_offset = selection;
                }
            } else if (ch == KEY_DOWN || ch == 'j') {
                if (selection < (int)values.size() - 1) {
                    selection++;
                    if (selection >= scroll_offset + max_visible) {
                        scroll_offset = selection - max_visible + 1;
                    }
                }
            } else if (ch == KEY_PPAGE) {
                selection = std::max(0, selection - max_visible);
                scroll_offset = std::max(0, scroll_offset - max_visible);
            } else if (ch == KEY_NPAGE) {
                selection = std::min((int)values.size() - 1, selection + max_visible);
                if (selection >= scroll_offset + max_visible) {
                    scroll_offset = selection - max_visible + 1;
                }
            }
        }

        nodelay(stdscr, TRUE);
    }
#endif

    void save_preset_dialog() {
        int rows, cols;
        getmaxyx(stdscr, rows, cols);
        
        // Check if we have room for more presets
        if (state_.presets.size() >= TNCUIState::MAX_PRESETS) {
            state_.add_log("Cannot save: maximum presets reached");
            return;
        }
        
        int dialog_w = 40;
        int dialog_h = 5;
        int dialog_x = (cols - dialog_w) / 2;
        int dialog_y = (rows - dialog_h) / 2;
        
        attron(A_BOLD);
        draw_box(dialog_y, dialog_x, dialog_h, dialog_w);
        attroff(A_BOLD);
        
        mvaddstr(dialog_y, dialog_x + 2, " Save Preset ");
        mvaddstr(dialog_y + 2, dialog_x + 2, "Name: ");

        char buf[32] = {0};
        if (!prompt_input(dialog_y + 2, dialog_x + 8, buf, 24)) return;

        if (strlen(buf) > 0) {
            // replace any commas with underscores, commas are the delimiter
            for (char* p = buf; *p; p++) {
                if (*p == ',') *p = '_';
            }
            
            if (state_.create_preset(buf)) {
                state_.selected_preset = state_.presets.size() - 1;
                state_.add_log("Preset saved: " + std::string(buf));
            } else {
                state_.add_log("Failed to save preset");
            }
        }
    }
    
    void load_selected_preset() {
        if (state_.selected_preset < 0 || state_.selected_preset >= (int)state_.presets.size()) {
            state_.add_log("No preset selected");
            return;
        }
        
        if (state_.apply_preset(state_.selected_preset)) {
            state_.loaded_preset_index = state_.selected_preset; 
            apply_settings();
            state_.add_log("Loaded preset: " + state_.presets[state_.selected_preset].name);
        }
    }
    
    void delete_selected_preset() {
        if (state_.selected_preset < 0 || state_.selected_preset >= (int)state_.presets.size()) {
            state_.add_log("No preset selected");
            return;
        }
        
        std::string name = state_.presets[state_.selected_preset].name;
        int deleted_index = state_.selected_preset;
        if (state_.delete_preset(state_.selected_preset)) {
            state_.add_log("Deleted preset: " + name);
            if (state_.loaded_preset_index == deleted_index) {
                state_.loaded_preset_index = -1;  
            } else if (state_.loaded_preset_index > deleted_index) {
                state_.loaded_preset_index--;  
            }
        }
    }
    
    void draw_box(int y, int x, int h, int w) {
        // corners
        mvaddch(y, x, ACS_ULCORNER);
        mvaddch(y, x + w - 1, ACS_URCORNER);
        mvaddch(y + h - 1, x, ACS_LLCORNER);
        mvaddch(y + h - 1, x + w - 1, ACS_LRCORNER);
        
        // horizontal lines
        mvhline(y, x + 1, ACS_HLINE, w - 2);
        mvhline(y + h - 1, x + 1, ACS_HLINE, w - 2);
        
        // vertical lines
        mvvline(y + 1, x, ACS_VLINE, h - 2);
        mvvline(y + 1, x + w - 1, ACS_VLINE, h - 2);
    }
    
    void draw_hline(int y, int x, int w, bool connect_left = false, bool connect_right = false) {
        mvaddch(y, x, connect_left ? ACS_LTEE : ACS_HLINE);
        mvhline(y, x + 1, ACS_HLINE, w - 2);
        mvaddch(y, x + w - 1, connect_right ? ACS_RTEE : ACS_HLINE);
    }
    
    void draw() {
        frame_counter_++;
        update_calibration();

        if (current_tab_ >= tab_count()) current_tab_ = 0;
        {
            int64_t now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
            if (state_.ptt_on.load() || state_.transmitting.load())
                last_ptt_seen_ms_ = now_ms;
            state_.rig_poll_enabled = (current_tab_ == 5) || (current_tab_ == 0) ||
                (last_ptt_seen_ms_ > 0 && now_ms - last_ptt_seen_ms_ < 3000);
            state_.scope_active = (current_tab_ == 4);
            if (now_ms - occ_hist_ms_ >= 5000) {
                occ_hist_ms_ = now_ms;
                occ_hist_[occ_hist_pos_] = state_.channel_occupancy.load();
                occ_hist_pos_ = (occ_hist_pos_ + 1) % OCC_HIST_SIZE;
                if (occ_hist_n_ < OCC_HIST_SIZE)
                    occ_hist_n_++;
            }
        }
        if (current_tab_ == 5 && rig_should_skip(rig_field_)) rig_field_ = RIG_FIELD_FREQ;

        int rows, cols;
        getmaxyx(stdscr, rows, cols);
        erase();
        

        attron(A_DIM);
        draw_box(0, 0, rows, cols);
        attroff(A_DIM);
        
        // title
        mvaddstr(0, 2, " ");
        attron(A_DIM);
        addstr("/ / / ");
        attroff(A_DIM);
        attron(A_BOLD);
        addstr("MODEM73");
        attroff(A_BOLD);
        attron(A_DIM);
        addstr(" v" MODEM73_VERSION);
        attroff(A_DIM);
        addstr(" ");
        
        // PTT status 
        attron(A_DIM);
        addch(ACS_VLINE);
        attroff(A_DIM);
        if (state_.ptt_on || state_.transmitting) {
            attron(COLOR_PAIR(2) | A_BOLD);
            addstr(" TX ");
            attroff(COLOR_PAIR(2) | A_BOLD);
        } else {
            attron(COLOR_PAIR(1) | A_BOLD);
            addstr(" RX ");
            attroff(COLOR_PAIR(1) | A_BOLD);
        }
        attron(A_DIM);
        addch(ACS_VLINE);
        attroff(A_DIM);

        // high-SWR warning chip, visible from every tab
        if (state_.swr_warn_value.load() > 0.0f) {
            attron(COLOR_PAIR(2) | A_BOLD);
            printw(" !SWR %.1f ", state_.swr_warn_value.load());
            attroff(COLOR_PAIR(2) | A_BOLD);
            attron(A_DIM);
            addch(ACS_VLINE);
            attroff(A_DIM);
        }

        // Mode
        addstr(" ");
        attron(A_BOLD);
        addstr(state_.callsign.c_str());
        attroff(A_BOLD);
        if (state_.modem_type_index == 1) {
            printw(" %s",
                   MFSK_MODE_OPTIONS[state_.mfsk_mode_index].c_str());
        } else if (state_.modem_type_index == 2) {
            printw(" %s %s",
                   ROBUST_MODE_OPTIONS[robust_base_index(state_.robust_mode_index)].c_str(),
                   state_.robust_mode_index == 12 
                       ? "30B" 
                       : (RobustParams::is_short((RobustMode)state_.robust_mode_index) ? "170B" : "510B"));
        } else {
            printw(" %s %s %s ",
                   MODULATION_OPTIONS[state_.modulation_index].c_str(),
                   CODE_RATE_OPTIONS[state_.code_rate_index].c_str(),
                   state_.frame_size == 0 ? "S" : state_.frame_size == 2 ? "L"
                 : state_.frame_size == 3 ? "U" : "N");
        }
        
        // Stats 
        {
            auto now = std::chrono::system_clock::now();
            auto t = std::chrono::system_clock::to_time_t(now);
            struct tm lt;
            localtime_r(&t, &lt);
            attron(A_BOLD);
            mvprintw(0, cols - 31, "%02d:%02d:%02d", lt.tm_hour, lt.tm_min, lt.tm_sec);
            attroff(A_BOLD);
            attron(A_DIM);
            addstr(" |");
            attroff(A_DIM);
        }

        int rx = cols - 20;
        attron(COLOR_PAIR(1) | A_BOLD);
        mvprintw(0, rx, "%d", state_.rx_frame_count.load());
        attroff(COLOR_PAIR(1) | A_BOLD);
        attron(A_DIM);
        addstr("v ");   // temp
        attroff(A_DIM);
        attron(COLOR_PAIR(2) | A_BOLD);
        printw("%d", state_.tx_frame_count.load());
        attroff(COLOR_PAIR(2) | A_BOLD);
        attron(A_DIM);
        addstr("^ ");  // temp 
        attroff(A_DIM);
        printw(" %d", state_.client_count.load());
        attron(A_DIM);
        addstr("c ");
        attroff(A_DIM);
        

        // Tab bar
        attron(A_DIM);
        draw_hline(1, 0, cols, true, true);
        attroff(A_DIM);
        

        // Tabs
        char utils_tab[24];
        int unread = state_.unread_messages.load();
        if (unread > 0)
            snprintf(utils_tab, sizeof(utils_tab), "UTILS(%d)", unread);
        else

            snprintf(utils_tab, sizeof(utils_tab), "UTILS");
        bool log_err = state_.log_unread_error.load() && current_tab_ != 2;
        const char* log_tab = log_err ? "LOG (!)" : "LOG";
        const char* tabs[] = {"STATUS", "CONFIG", log_tab, utils_tab, "SCOPE", "RIG"};
        int ntabs = tab_count();
        int tab_width = (cols - 4) / ntabs;

        for (int i = 0; i < ntabs; i++) {
            int tx = 2 + i * tab_width;

            if (i == current_tab_) {
                attron(A_BOLD);
                mvaddch(2, tx, '>');
                printw(" %s", tabs[i]);
                attroff(A_BOLD);
            } else {
                if (i == 3 && unread > 0) attron(COLOR_PAIR(4) | A_BOLD);
                else if (i == 2 && log_err) attron(COLOR_PAIR(3) | A_BOLD);
                else attron(A_DIM);
                mvprintw(2, tx, "  %s", tabs[i]);
                if (i == 3 && unread > 0) attroff(COLOR_PAIR(4) | A_BOLD);
                else if (i == 2 && log_err) attroff(COLOR_PAIR(3) | A_BOLD);
                else attroff(A_DIM);
            }
        }
        
        // Content separator
        attron(A_DIM);
        draw_hline(3, 0, cols, true, true);
        attroff(A_DIM);
        
        // Content area
        int content_y = 4;
        int content_h = rows - 6;
        
        if (current_tab_ == 0) {
            draw_status(content_y, content_h, cols);
        } else if (current_tab_ == 1) {
            draw_config(content_y, content_h, cols);
        } else if (current_tab_ == 2) {
            draw_log(content_y, content_h, cols);
        } else if (current_tab_ == 3) {
            draw_utils(content_y, content_h, cols);
        } else if (current_tab_ == 4) {
            draw_scope(content_y, content_h, cols);
        } else {
            draw_rig(content_y, content_h, cols);
        }
        
        // Footer
        attron(A_DIM);
        draw_hline(rows - 2, 0, cols, true, true);
        
        if (current_tab_ == 1) {
            mvaddstr(rows - 1, 2, " ^/v nav  </> adjust  Enter edit  s save  x del  F1 help  Q quit ");
        } else if (current_tab_ == 2) {
            mvaddstr(rows - 1, 2, " ^/v scroll  PgUp/Dn page  F1 help  Q quit ");
        } else if (current_tab_ == 3) {
            mvaddstr(rows - 1, 2, " 1-7 select  Enter run  F1 help  Q quit ");
        } else if (current_tab_ == 5) {
            mvaddstr(rows - 1, 2, " ^/v nav  </> adjust  Enter set/tune  F1 help  Q quit ");
        } else if (current_tab_ == 4) {
            mvaddstr(rows - 1, 2, " Tab switch  F1 help  Q quit ");
        } else {
            mvaddstr(rows - 1, 2, " Tab switch  F1 help  Q quit ");
        }
        attroff(A_DIM);
        
        if (show_help_) {
            draw_help(rows, cols);
        }
        if (show_csma_help_) {
            draw_csma_help(rows, cols);
        }
        if (show_sync_help_) {
            draw_sync_only_help(rows, cols);
        }
    }
    
    void draw_status(int y, int h, int cols) {
        int c1 = 3;
        int c2 = 18;
        int c3 = cols / 2 + 2;
        int c4 = cols / 2 + 17;
        

        attron(A_DIM);
        mvaddstr(y, c1, "SIGNAL");
        attroff(A_DIM);
        y++;
        

        mvaddstr(y, c1, "Carrier");
        float lvl = state_.carrier_level_db.load();
        bool busy = lvl > state_.carrier_threshold_db;
        move(y, c2);
        if (busy) {
            attron(COLOR_PAIR(4) | A_BOLD);  
            printw("%6.1f dB", lvl);
            attroff(COLOR_PAIR(4) | A_BOLD);
        } else {
            attron(COLOR_PAIR(1) | A_BOLD);  
            printw("%6.1f dB", lvl);
            attroff(COLOR_PAIR(1) | A_BOLD);
        }
        y++;
        
        //  Meter
        mvaddstr(y, c1, "Level");
        move(y, c2);
        draw_level_meter(lvl, state_.carrier_threshold_db, 20);
        y++;
        
        mvaddstr(y, c1, "Threshold");
        mvprintw(y, c2, "%6.0f dB", state_.carrier_threshold_db);
        y++;
        
        mvaddstr(y, c1, "Last SNR");
        float snr = state_.last_rx_snr.load();
        if (snr > 10.0f) {
            attron(COLOR_PAIR(1) | A_BOLD);  
        } else if (snr > 5.0f) {
            attron(COLOR_PAIR(3) | A_BOLD);  
        }
        mvprintw(y, c2, "%6.1f dB", snr);
        attroff(COLOR_PAIR(1) | A_BOLD);
        attroff(COLOR_PAIR(3) | A_BOLD);
        y++;
        
        // SNR history
        mvaddstr(y, c1, "SNR Hist");
        move(y, c2);
        draw_snr_chart(20);
        y += 2;

        mvaddstr(y, c1, "BER");
        {
            float ber_pct = state_.last_rx_ber.load() * 100.0f;
            if (ber_pct < 0) {
                attron(A_DIM);
                mvaddstr(y, c2, "  ---");
                attroff(A_DIM);
            } else if (state_.modem_type_index == 2) {
                if (ber_pct < 18.0f) {
                    mvprintw(y, c2, "%5.2f%%", ber_pct);
                } else {
                    attron(COLOR_PAIR(2) | A_BOLD);
                    mvprintw(y, c2, "%5.2f%%", ber_pct);
                    attroff(COLOR_PAIR(2) | A_BOLD);
                }
            } else if (ber_pct < 3.0f) {
                attron(COLOR_PAIR(1) | A_BOLD);
                mvprintw(y, c2, "%5.2f%%", ber_pct);
                attroff(COLOR_PAIR(1) | A_BOLD);
            } else if (ber_pct < 13.0f) {
                attron(COLOR_PAIR(3) | A_BOLD);
                mvprintw(y, c2, "%5.2f%%", ber_pct);
                attroff(COLOR_PAIR(3) | A_BOLD);
            } else {
                attron(COLOR_PAIR(2) | A_BOLD);
                mvprintw(y, c2, "%5.2f%%", ber_pct);
                attroff(COLOR_PAIR(2) | A_BOLD);
            }
        }
        y++;

        mvaddstr(y, c1, "Heard");
        {
            auto pkts = state_.get_recent_packets();
            const TNCUIState::PacketInfo* last = nullptr;
            for (auto it = pkts.rbegin(); it != pkts.rend(); ++it) {
                if (!it->is_tx) {
                    last = &*it;
                    break;
                }
            }
            move(y, c2);
            if (!last) {
                attron(A_DIM);
                addstr("  ---");
                attroff(A_DIM);
            } else {
                auto secs = std::chrono::duration_cast<std::chrono::seconds>(
                    std::chrono::steady_clock::now() - last->timestamp).count();
                attron(A_BOLD);
                printw("%s", last->mode.empty() ? "?" : last->mode.c_str());
                attroff(A_BOLD);
                printw(" %s %.0fdB ",
                       last->callsign.empty() ? "-" : last->callsign.c_str(),
                       last->snr);
                attron(A_DIM);
                if (secs < 60)
                    printw("%llds", (long long)secs);
                else if (secs < 3600)
                    printw("%lldm", (long long)(secs / 60));
                else
                    printw("%lldh", (long long)(secs / 3600));
                attroff(A_DIM);
            }
        }
        y++;

        attron(A_DIM);
        mvaddstr(y, c1, "CSMA");
        attroff(A_DIM);
        y++;
        
        mvaddstr(y, c1, "Status");
        move(y, c2);
        if (state_.csma_enabled) {
            attron(COLOR_PAIR(1) | A_BOLD);
            addstr("ON");
            attroff(COLOR_PAIR(1) | A_BOLD);
        } else {
            attron(COLOR_PAIR(3) | A_BOLD);
            addstr("OFF");
            attroff(COLOR_PAIR(3) | A_BOLD);
        }
        {
            bool dcd = state_.dcd_active.load();
            if (state_.csma_sync_only ? dcd : (busy || dcd)) {
                attron(COLOR_PAIR(3) | A_BOLD);
                addstr("  BUSY");
                attroff(COLOR_PAIR(3) | A_BOLD);
                if (dcd) {
                    attron(A_DIM);
                    addstr(" (sync)");
                    attroff(A_DIM);
                }
            }
        }
        y++;

        mvaddstr(y, c1, "Occupancy");
        {
            float occ = state_.channel_occupancy.load() * 100.0f;
            int opair = occ >= 70.0f ? 2 : occ >= 30.0f ? 3 : 1;
            move(y, c2);
            attron(COLOR_PAIR(opair) | A_BOLD);
            printw("%5.0f%%", occ);
            attroff(COLOR_PAIR(opair) | A_BOLD);
            attron(A_DIM);
            addstr(" (30s)");
            attroff(A_DIM);
        }
        y++;

        mvaddstr(y, c1, "Occ Hist");
        move(y, c2);
        draw_occ_chart(20);
        y++;

        mvaddstr(y, c1, "Quiet");
        if (state_.csma_ranked && state_.csma_sync_only &&
            state_.csma_rank.load() >= 0) {
            mvprintw(y, c2, "%d ms (ranked)", RANKED_QUIET_DISPLAY_MS);
        } else if (state_.csma_quiet_ms > 0) {
            mvprintw(y, c2, "%d ms", state_.csma_quiet_ms);
        } else {
            int q = (int)(state_.airtime_seconds * 1000.0f) / 4;
            if (q < 300) q = 300;
            if (q > 3500) q = 3500;
            mvprintw(y, c2, "auto ~%d ms", q);
        }
        y++;

        mvaddstr(y, c1, "Window");
        {
            int w = state_.csma_window_ms.load();
            int s = std::max(1, state_.slot_time_ms);
            int rk = state_.csma_rank.load();
            if (state_.csma_ranked && rk >= 0 && state_.csma_rank_n.load() > 0) {
                mvprintw(y, c2, "%d turns (%d ms)", state_.csma_rank_n.load(), w);
            } else if (w > 0) {
                mvprintw(y, c2, "%d slots (%d ms)", w / s, w);
            } else {
                mvprintw(y, c2, "%d slots", state_.csma_cw);
            }
        }
        y++;

        mvaddstr(y, c1, "Slot");
        mvprintw(y, c2, "%d ms", state_.slot_time_ms);
        y++;

        mvaddstr(y, c1, "TX");
        {
            int phase = state_.csma_phase.load();
            int q = state_.tx_queue_size.load();
            move(y, c2);
            if (state_.transmitting.load()) {
                attron(COLOR_PAIR(2) | A_BOLD);
                addstr("sending");
                attroff(COLOR_PAIR(2) | A_BOLD);
            } else if (phase == 1) {
                attron(COLOR_PAIR(3));
                addstr("deferring (RX)");
                attroff(COLOR_PAIR(3));
            } else if (phase == 2) {
                attron(COLOR_PAIR(3));
                printw("quiet %d/%d ms", state_.csma_wait_ms.load(),
                       state_.csma_wait_need.load());
                attroff(COLOR_PAIR(3));
            } else if (phase == 3) {
                attron(COLOR_PAIR(4));
                {
                    int rk = state_.csma_rank.load();
                    int rn = state_.csma_rank_n.load();
                    if (rk >= rn && rk >= 0)
                        printw("yielding %d ms", state_.csma_wait_ms.load());
                    else if (rk >= 0)
                        printw("turn %d/%d in %d ms", rk + 1, rn,
                               state_.csma_wait_ms.load());
                    else
                        printw("contending %d ms", state_.csma_wait_ms.load());
                }
                attroff(COLOR_PAIR(4));
            } else {
                attron(A_DIM);
                addstr("idle");
                attroff(A_DIM);
            }
            if (q > 0) {
                attron(A_DIM);
                printw("  %d queued ~%ds", q,
                       (int)(q * state_.airtime_seconds + 0.5f));
                attroff(A_DIM);
            }
        }
        y += 2;

        mvaddstr(y, c1, "Rig");
        {
            move(y, c2);
            if (!state_.rigctl_connected.load()) {
                attron(A_DIM);
                addstr("  ---");
                attroff(A_DIM);
            } else {
                attron(A_BOLD);
                printw("%s", format_freq(state_.rig_freq_hz.load()).c_str());
                attroff(A_BOLD);
                std::string rmode = state_.get_rig_mode();
                if (!rmode.empty())
                    printw(" %s", rmode.c_str());
                float pwr = state_.rig_power_level.load();
                if (pwr >= 0)
                    printw(" %d%%", (int)lround(pwr * 100));
                float swr = state_.rig_meter_values[RIG_METER_SWR].load();
                if (!std::isnan(swr)) {
                    int spair = swr < 1.5f ? 1 : swr < SWR_WARN_THRESHOLD ? 3 : 2;
                    attron(COLOR_PAIR(spair));
                    printw(" SWR %.1f", swr);
                    attroff(COLOR_PAIR(spair));
                }
            }
        }


        y = 4;
        attron(A_DIM);
        mvaddstr(y, c3, "ACTIVITY");
        attroff(A_DIM);
        

        int graph_width = cols - c3 - 4;
        int graph_height = 6;
        draw_signal_graph(y + 1, c3, graph_width, graph_height);
        
        y += graph_height + 2;
        

        attron(COLOR_PAIR(4));
        mvaddstr(y, c3, ">>> STATS");
        attroff(COLOR_PAIR(4));
        y++;
        
        mvaddstr(y, c3, "RX");
        attron(COLOR_PAIR(1) | A_BOLD);
        mvprintw(y, c4, "%d", state_.rx_frame_count.load());
        attroff(COLOR_PAIR(1) | A_BOLD);
            
        //         addstr("  ");
        addstr("  ");
        attroff(A_BOLD);
        addstr("TX");
        attron(COLOR_PAIR(2) | A_BOLD);
        printw(" %d", state_.tx_frame_count.load());
        attroff(COLOR_PAIR(2) | A_BOLD);
        
        addstr("  ");
        int syncs = state_.sync_count.load();
        int total_errors = state_.preamble_errors.load() + 
                          state_.symbol_errors.load() + 
                          state_.crc_errors.load() +
                          state_.rx_error_count.load();
        if (state_.modem_type_index == 1) {
        } else if (syncs > 0) {
            float err_pct = 100.0f * total_errors / syncs;
            addstr("Err");
            if (total_errors == 0) {
                attron(COLOR_PAIR(1));
                printw(" 0/%d", syncs);
                attroff(COLOR_PAIR(1));
            } else if (err_pct < 20.0f) {
                attron(COLOR_PAIR(3));
                printw(" %d/%d", total_errors, syncs);
                attroff(COLOR_PAIR(3));
            } else {
                attron(COLOR_PAIR(2));
                printw(" %d/%d", total_errors, syncs);
                attroff(COLOR_PAIR(2));
            }
            attron(A_DIM);
            printw(" (%.0f%%)", err_pct);
            attroff(A_DIM);
        } else {
            attron(A_DIM);
            addstr("Err 0/0");
            attroff(A_DIM);
        }
        y++;
        

        mvaddstr(y, c3, "Clients");
        int clients = state_.client_count.load();
        if (clients > 0) {
            attron(COLOR_PAIR(4) | A_BOLD);
            mvprintw(y, c4, "%d", clients);
            attroff(COLOR_PAIR(4) | A_BOLD);
        } else {
            attron(A_DIM);
            mvprintw(y, c4, "%d", clients);
            attroff(A_DIM);
        }
        
        addstr("  Queue");
        printw(" %d", state_.tx_queue_size.load());
        

        y += 2;
        draw_recent_packets(y, c3, cols - c3 - 2, h - (y - 4) - 2);
    }
    
    void draw_recent_packets(int y, int x, int /* width */, int max_lines) {
        auto packets = state_.get_recent_packets();
        
        if (packets.empty()) {
            attron(A_DIM);
            mvaddstr(y, x, "Waiting for packets...");
            attroff(A_DIM);
            return;
        }
        
        attron(A_DIM);
        mvaddstr(y, x, "RECENT");
        attroff(A_DIM);
        y++;

        int sel = recent_sel_;
        if (sel >= (int)packets.size())
            sel = (int)packets.size() - 1;
        int limit = max_lines;
        if (sel >= 0 && limit > 2)
            limit -= 2;

        int lines = 0;
        for (auto it = packets.rbegin(); it != packets.rend() && lines < limit; ++it, ++lines) {
            const auto& pkt = *it;

            move(y + lines, x);
            if (lines == sel) {
                attron(A_BOLD);
                addstr("> ");
                attroff(A_BOLD);
            } else {
                addstr("  ");
            }

            if (pkt.is_tx) {
                attron(COLOR_PAIR(2) | A_BOLD);
                addstr("TX ");
                attroff(COLOR_PAIR(2) | A_BOLD);
            } else {
                attron(COLOR_PAIR(1) | A_BOLD);
                addstr("RX ");
                attroff(COLOR_PAIR(1) | A_BOLD);
            }
            
            attron(A_BOLD);
            printw("%4d", pkt.size);
            attroff(A_BOLD);
            attron(A_DIM);
            addstr("B ");
            attroff(A_DIM);
            


            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - pkt.timestamp).count();
            if (elapsed < 60) {
                printw("%2lds", elapsed);
            } else {
                printw("%2ldm", elapsed / 60);
            }
            


            // SNR
            if (!pkt.is_tx && pkt.snr > 0) {
                attron(COLOR_PAIR(4) | A_BOLD);
                printw(" %.0fdB", pkt.snr);
                attroff(COLOR_PAIR(4) | A_BOLD);
            }
            // BER
            if (!pkt.is_tx && pkt.ber >= 0) {
                float ber_pct = pkt.ber;
                bool robust = pkt.mode.rfind("RDM", 0) == 0;
                if (robust) {
                    if (ber_pct >= 18.0f)
                        attron(COLOR_PAIR(2));
                } else if (ber_pct < 1.0f) {
                    attron(COLOR_PAIR(1));
                } else if (ber_pct < 5.0f) {
                    attron(COLOR_PAIR(3));
                } else {
                    attron(COLOR_PAIR(2));
                }
                printw(" %.1f%%", ber_pct);
                attroff(COLOR_PAIR(1));
                attroff(COLOR_PAIR(2));
                attroff(COLOR_PAIR(3));
            }
        }

        if (sel >= 0 && sel < (int)packets.size()) {
            const auto& sp = *(packets.rbegin() + sel);
            move(y + lines + 1, x);
            attron(A_BOLD);
            printw("%s", sp.mode.empty() ? "?" : sp.mode.c_str());
            attroff(A_BOLD);
            printw("  %s", sp.callsign.empty() ? "-" : sp.callsign.c_str());
            if (!sp.is_tx && sp.snr > 0) {
                attron(COLOR_PAIR(4) | A_BOLD);
                printw("  %.1fdB", sp.snr);
                attroff(COLOR_PAIR(4) | A_BOLD);
            }
            if (!sp.is_tx && sp.ber >= 0) {
                bool robust = sp.mode.rfind("RDM", 0) == 0;
                bool bad = robust ? sp.ber >= 18.0f : sp.ber >= 5.0f;
                if (bad)
                    attron(COLOR_PAIR(2));
                printw("  BER %.1f%%", sp.ber);
                if (bad)
                    attroff(COLOR_PAIR(2));
            }
            attron(A_DIM);
            printw("  %dB", sp.size);
            attroff(A_DIM);
        }
    }
    
    void draw_level_meter(float level_db, float threshold_db, int width) {
        float min_db = -80.0f;
        float max_db = 0.0f;
        
        int level_pos = (int)((level_db - min_db) / (max_db - min_db) * width);
        int thresh_pos = (int)((threshold_db - min_db) / (max_db - min_db) * width);
        
        level_pos = std::max(0, std::min(width, level_pos));
        thresh_pos = std::max(0, std::min(width - 1, thresh_pos));
        
        attron(A_DIM);
        addch('[');
        attroff(A_DIM);
        
        for (int i = 0; i < width; i++) {
            if (i < level_pos) {
                if (i >= thresh_pos) {
                    attron(COLOR_PAIR(4) | A_BOLD);  
                    addch('=');
                    attroff(COLOR_PAIR(4) | A_BOLD);
                } else if (i >= width * 2 / 3) {
                    attron(COLOR_PAIR(3) | A_BOLD);  
                    addch('=');
                    attroff(COLOR_PAIR(3) | A_BOLD);
                } else {
                    attron(COLOR_PAIR(1) | A_BOLD); 
                    addch('=');
                    attroff(COLOR_PAIR(1) | A_BOLD);
                }
            } else if (i == thresh_pos) {
                attron(A_DIM);
                addch('|');  
                attroff(A_DIM);
            } else {
                attron(A_DIM);
                addch('-');
                attroff(A_DIM);
            }
        }
        
        attron(A_DIM);
        addch(']');
        attroff(A_DIM);
    }
    
    void draw_occ_chart(int width) {
        if (occ_hist_n_ == 0) {
            attron(A_DIM);
            addstr("[no data]");
            attroff(A_DIM);
            return;
        }
        int count = std::min(occ_hist_n_, width);
        attron(A_DIM);
        for (int i = count; i < width; i++)
            addch('.');
        attroff(A_DIM);
        for (int i = 0; i < count; i++) {
            int idx = (occ_hist_pos_ - count + i + 2 * OCC_HIST_SIZE) % OCC_HIST_SIZE;
            float v = occ_hist_[idx];
            char ch = v > 0.875f ? '#' : v > 0.7f ? '=' : v > 0.5f ? '+'
                    : v > 0.3f ? ':' : v > 0.15f ? '-' : v > 0.03f ? '.' : '_';
            int pair = v >= 0.7f ? 2 : v >= 0.3f ? 3 : 1;
            attron(COLOR_PAIR(pair));
            addch(ch);
            attroff(COLOR_PAIR(pair));
        }
    }

    void draw_snr_chart(int width) {
        auto history = state_.get_snr_history();
        
        if (history.empty()) {
            attron(A_DIM);
            addstr("[no data]");
            attroff(A_DIM);
            return;
        }
        
        // 
        float min_snr = 0.0f;
        float max_snr = 30.0f;
        

        int display_count = std::min((int)history.size(), width);
        int start_idx = (int)history.size() - display_count;
        
        for (int i = 0; i < display_count; i++) {
            float snr = history[start_idx + i];
            float normalized = (snr - min_snr) / (max_snr - min_snr);
            normalized = std::max(0.0f, std::min(1.0f, normalized));
            

            char ch;
            if (normalized > 0.875f) ch = '#';
            else if (normalized > 0.75f) ch = '=';
            else if (normalized > 0.625f) ch = '+';
            else if (normalized > 0.5f) ch = ':';
            else if (normalized > 0.375f) ch = '-';
            else if (normalized > 0.25f) ch = '.';
            else if (normalized > 0.125f) ch = '_';
            else ch = ' ';
            

            if (snr > 15.0f) {
                attron(COLOR_PAIR(1) | A_BOLD);  
            } else if (snr > 8.0f) {
                attron(COLOR_PAIR(3) | A_BOLD);  
            } else if (snr > 3.0f) {
                attron(COLOR_PAIR(4));           
            } else {
                attron(COLOR_PAIR(2));           
            }
            addch(ch);
            attroff(COLOR_PAIR(1) | A_BOLD);
            attroff(COLOR_PAIR(2));
            attroff(COLOR_PAIR(3) | A_BOLD);
            attroff(COLOR_PAIR(4));
        }
        

        attron(A_DIM);
        for (int i = display_count; i < width; i++) {
            addch('.');
        }
        attroff(A_DIM);
    }
    
    void draw_signal_graph(int y, int x, int width, int height) {
        auto history = state_.get_level_history();
        auto dcd_hist = state_.get_level_dcd_history();
        auto tone_hist = state_.get_level_tone_history();


        float min_db = -80.0f;
        float max_db = 0.0f;
        float thresh = state_.carrier_threshold_db;


        const char* blocks[] = {" ", ".", ":", "|", "#"};

        for (int row = 0; row < height; row++) {
            move(y + row, x);

            float row_min = max_db - (max_db - min_db) * (row + 1) / height;
            float row_max = max_db - (max_db - min_db) * row / height;

            for (int col = 0; col < width; col++) {
                int hist_idx = col * TNCUIState::LEVEL_HISTORY_SIZE / width;
                if (hist_idx >= (int)history.size()) hist_idx = history.size() - 1;

                float level = history[hist_idx];
                // magenta marks samples taken while the confirmed sync DCD
                // held: the segment of the waterfall carrying a real frame
                int pair = tone_hist[hist_idx] ? 2
                         : dcd_hist[hist_idx] ? 6 : level > thresh ? 4 : 1;

                if (level >= row_max) {
                    attron(COLOR_PAIR(pair) | A_BOLD);
                    addch(ACS_BLOCK);
                    attroff(COLOR_PAIR(pair) | A_BOLD);
                } else if (level > row_min) {
                    float frac = (level - row_min) / (row_max - row_min);
                    int idx = (int)(frac * 4);
                    if (idx > 4) idx = 4;
                    if (idx < 0) idx = 0;

                    attron(COLOR_PAIR(pair));
                    addstr(blocks[idx]);
                    attroff(COLOR_PAIR(pair));
                } else {
                    addch(' ');
                }
            }
        }
        
        int thresh_row = (int)((max_db - thresh) / (max_db - min_db) * height);
        if (thresh_row >= 0 && thresh_row < height) {
            attron(A_DIM | COLOR_PAIR(3));
            for (int col = 0; col < width; col += 2) {
                mvaddch(y + thresh_row, x + col, '-');
            }
            attroff(A_DIM | COLOR_PAIR(3));
        }
    }
    
    void draw_config(int y, int h, int cols) {
        int c1 = 3;      
        int c2 = 16;     
        int divider = cols/2 - 2;  
        int c3 = cols/2 + 1; 
        int start_y = y;
        int visible_rows = h - 2;  
        

        


        if (current_tab_ == 1) {
            int field_row = config_field_row(current_field_);
            if (field_row < config_scroll_ + 2) {
                config_scroll_ = std::max(0, field_row - 2);
            } else if (field_row > config_scroll_ + visible_rows - 3) {
                config_scroll_ = field_row - visible_rows + 3;
            }
        }

        
        int scroll = config_scroll_;
        int row = 0;  
        
        auto visible_y = [&](int logical_row) -> int {
            int screen_row = logical_row - scroll;
            if (screen_row < 0 || screen_row >= visible_rows) return -1;
            return start_y + screen_row;
        };
        
//
        attron(A_DIM);
        for (int r = start_y; r < start_y + visible_rows; r++) {
            mvaddch(r, divider, ACS_VLINE);
        }

        attroff(A_DIM);
        
        int dy = visible_y(row);
        if (dy >= 0) {
            attron(A_DIM);
            mvaddstr(dy, c1, "MODEM");
            attroff(A_DIM);
        }
        row++;
        
        dy = visible_y(row);
        if (dy >= 0) draw_field(dy, c1, c2, "Callsign", FIELD_CALLSIGN, state_.callsign, true);
        row++;

        dy = visible_y(row);
        if (dy >= 0) {
            bool sel_t = (current_field_ == FIELD_CONFIG_TOKEN);
            if (sel_t) attron(A_BOLD); else attron(A_DIM);
            mvprintw(dy, c1, "%s[#] Config Token (copy or paste)", sel_t ? "> " : "  ");
            if (sel_t) attroff(A_BOLD); else attroff(A_DIM);
        }
        row++;

        dy = visible_y(row);
        if (dy >= 0) draw_modem_tabs(dy, c1, c2, divider);
        row++;

        if (state_.modem_type_index == 0) {
            // OFDM fields
            dy = visible_y(row);
            if (dy >= 0) draw_selector_field(dy, c1, c2, "Modulation", FIELD_MODULATION,
                               MODULATION_OPTIONS[state_.modulation_index]);
            row++;

            dy = visible_y(row);
            if (dy >= 0) draw_selector_field(dy, c1, c2, "Code Rate", FIELD_CODERATE,
                               CODE_RATE_OPTIONS[state_.code_rate_index]);
            row++;

            dy = visible_y(row);
            if (dy >= 0) draw_selector_field(dy, c1, c2, "Frame Size", FIELD_FRAMESIZE,
                               state_.frame_size == 0 ? "SHORT" : state_.frame_size == 2 ? "LONG"
                             : state_.frame_size == 3 ? "MICRO" : "NORMAL");
            row++;

            dy = visible_y(row);
            if (dy >= 0) draw_toggle_field(dy, c1, c2, "Postamble", FIELD_POSTAMBLE, state_.postamble);
            row++;
        } else if (state_.modem_type_index == 1) {
            // MFSK field
            dy = visible_y(row);
            if (dy >= 0) draw_selector_field(dy, c1, c2, "MFSK Mode", FIELD_MFSK_MODE,
                               MFSK_MODE_OPTIONS[state_.mfsk_mode_index]);
            row++;
        } else {
            dy = visible_y(row);
            if (dy >= 0) draw_selector_field(dy, c1, c2, "RDM Mode", FIELD_ROBUST_MODE,
                               ROBUST_MODE_OPTIONS[robust_base_index(state_.robust_mode_index)]);
            row++;
            dy = visible_y(row);
            if (dy >= 0) draw_selector_field(dy, c1, c2, "Frame", FIELD_ROBUST_MTU,
                               ROBUST_MTU_OPTIONS[robust_mtu_index(state_.robust_mode_index)]);
            row++;
        }

        row++;

        dy = visible_y(row);
        if (dy >= 0) {
            attron(A_DIM);
            mvaddstr(dy, c1, "CSMA");
            mvaddnstr(dy, c1 + 6, "waits for a clear channel before transmitting",
                      std::max(0, divider - (c1 + 6) - 1));
            attroff(A_DIM);
        }
        row++;
        
        dy = visible_y(row);
        if (dy >= 0) draw_toggle_field(dy, c1, c2, "Enabled", FIELD_CSMA, state_.csma_enabled);
        row++;

        dy = visible_y(row);
        if (dy >= 0) {
            static const char* MODE_NAMES[3] = {"THRESHOLD", "SYNC", "RANKED"};
            static const char* MODE_HINT[3] = {
                "busy = any audio over Threshold",
                "busy = a real modem signal only",
                "SYNC plus stations take turns"};
            int m = csma_mode();
            draw_selector_field(dy, c1, c2, "Mode", FIELD_CSMA_MODE, MODE_NAMES[m]);
            attron(A_DIM);
            mvaddstr(dy, c2 + 12, MODE_HINT[m]);
            attroff(A_DIM);
        }
        row++;

        dy = visible_y(row);
        if (dy >= 0) {
            bool sel_h = (current_field_ == FIELD_CSMA_HELP);
            if (sel_h) attron(A_BOLD); else attron(A_DIM);
            mvprintw(dy, c1 + 2, "%s[?] Which one should I use?", sel_h ? "> " : "  ");
            if (sel_h) attroff(A_BOLD); else attroff(A_DIM);
        }
        row++;

        if (csma_mode() == 0) {
            dy = visible_y(row);
            if (dy >= 0) {
                char thresh_buf[32];
                snprintf(thresh_buf, sizeof(thresh_buf), "%.0f dB", state_.carrier_threshold_db);
                draw_selector_field(dy, c1, c2, "Threshold", FIELD_THRESHOLD, thresh_buf);
                float lvl = state_.carrier_level_db.load();
                if (lvl > state_.carrier_threshold_db) {
                    attron(COLOR_PAIR(4) | A_BOLD);
                } else {
                    attron(A_DIM);
                }
                mvprintw(dy, c2 + 9, "%.0f", lvl);
                attroff(COLOR_PAIR(4) | A_BOLD);
                attroff(A_DIM);
            }
            row++;

            dy = visible_y(row);
            if (dy >= 0) {
                mvaddstr(dy, c1, "Level");
                move(dy, c2);
                float lvl = state_.carrier_level_db.load();
                draw_level_meter(lvl, state_.carrier_threshold_db, 14);
            }
            row++;
        }

        if (csma_mode() != 2) {
        dy = visible_y(row);
        if (dy >= 0) draw_selector_field(dy, c1, c2, "Band", FIELD_CSMA_BAND,
                                         CSMA_BAND_NAMES[state_.csma_band & 1]);
        row++;

        dy = visible_y(row);
        if (dy >= 0) {
            int pidx = csma_preset_match();
            draw_selector_field(dy, c1, c2, "Preset", FIELD_CSMA_PRESET,
                                pidx >= 0 ? CSMA_PRESETS[state_.csma_band & 1][pidx].name
                                          : "CUSTOM");
        }
        row++;
        }

        dy = visible_y(row);
        if (dy >= 0) {
            bool sel_adv = (current_field_ == FIELD_CSMA_ADV);
            if (sel_adv) attron(A_BOLD); else attron(A_DIM);
            mvprintw(dy, c1, "%s[%c] Advanced", sel_adv ? "> " : "  ",
                     state_.csma_advanced_open ? '-' : '+');
            if (sel_adv) attroff(A_BOLD); else attroff(A_DIM);
        }
        row++;

        if (state_.csma_advanced_open) {
            int m = csma_mode();
            if (m != 2) {
            dy = visible_y(row);
            if (dy >= 0) {
                char quiet_buf[32];
                if (state_.csma_quiet_ms > 0)
                    snprintf(quiet_buf, sizeof(quiet_buf), "%d ms", state_.csma_quiet_ms);
                else
                    snprintf(quiet_buf, sizeof(quiet_buf), "AUTO");
                draw_selector_field(dy, c1 + 2, c2, "Quiet", FIELD_CSMA_QUIET, quiet_buf);
            }
            row++;
            }

            if (m == 0) {
                dy = visible_y(row);
                if (dy >= 0) {
                    char cw_buf[32];
                    snprintf(cw_buf, sizeof(cw_buf), "%d x %dms", state_.csma_cw, state_.slot_time_ms);
                    draw_selector_field(dy, c1 + 2, c2, "Window", FIELD_CSMA_CW, cw_buf);
                }
                row++;
            }

            if (m != 2) {
            dy = visible_y(row);
            if (dy >= 0) draw_toggle_field(dy, c1 + 2, c2, "Lead Tone", FIELD_LEAD_TONE, state_.tx_lead_tone);
            row++;

            dy = visible_y(row);
            if (dy >= 0) {
                char dith_buf[32];
                if (state_.csma_responder_dither > 0)
                    snprintf(dith_buf, sizeof(dith_buf), "%d ms", state_.csma_responder_dither);
                else
                    snprintf(dith_buf, sizeof(dith_buf), "OFF");
                draw_selector_field(dy, c1 + 2, c2, "Reply Offset", FIELD_RESP_DITHER, dith_buf);
            }
            row++;
            }

            dy = visible_y(row);
            if (dy >= 0) {
                char burst_buf[32];
                if (state_.csma_burst > 1)
                    snprintf(burst_buf, sizeof(burst_buf), "%d pkts", state_.csma_burst);
                else
                    snprintf(burst_buf, sizeof(burst_buf), "OFF");
                draw_selector_field(dy, c1 + 2, c2, "Burst", FIELD_CSMA_BURST, burst_buf);
            }
            row++;

            if (m == 1) {
                dy = visible_y(row);
                if (dy >= 0) {
                    draw_toggle_field(dy, c1 + 2, c2, "Fast Floor", FIELD_FAST_FLOOR,
                                      state_.csma_fast_floor);
                    attron(A_DIM);
                    mvaddstr(dy, c2 + 8, "only if all stations run 2.3>");
                    attroff(A_DIM);
                }
                row++;
            }
            if (m == 2) {
                dy = visible_y(row);
                if (dy >= 0) {
                    char bi_buf[16];
                    snprintf(bi_buf, sizeof(bi_buf), "%d s", state_.beacon_interval_s);
                    draw_selector_field(dy, c1 + 2, c2, "Presence Ivl", FIELD_BEACON_INT, bi_buf);
                }
                row++;
            }
        }

        dy = visible_y(row);
        if (dy >= 0) {
            bool sel_info = (current_field_ == FIELD_CSMA_INFO);
            if (sel_info) attron(A_BOLD); else attron(A_DIM);
            mvprintw(dy, c1, "%s[?] Info", sel_info ? "> " : "  ");
            if (sel_info) attroff(A_BOLD); else attroff(A_DIM);
        }
        row += 2;
        
        // Fragmentation section
        dy = visible_y(row);
        if (dy >= 0) {
            attron(A_DIM);
            mvaddstr(dy, c1, "FRAGMENTATION");
            mvaddstr(dy, c1 + 14, "(restart)");
            attroff(A_DIM);
        }
        row++;
        
        row++;
        
        dy = visible_y(row);
        if (dy >= 0) draw_toggle_field(dy, c1, c2, "Enabled", FIELD_FRAGMENTATION, state_.fragmentation_enabled);
        row += 2;
        
        if (!state_.csma_enabled) {
            dy = visible_y(row);
            if (dy >= 0) {
                attron(A_DIM);
                mvaddstr(dy, c1, "TX BLANKING");
                attroff(A_DIM);
            }
            row++;

            dy = visible_y(row);
            if (dy >= 0) draw_toggle_field(dy, c1, c2, "Enabled", FIELD_TX_BLANKING, state_.tx_blanking_enabled);
            row += 2;
        }

        dy = visible_y(row);
        if (dy >= 0) {
            attron(A_DIM);
            mvaddstr(dy, c1, "RX DECODERS");
            attroff(A_DIM);
        }
        row++;

        dy = visible_y(row);
        if (dy >= 0) {
            draw_toggle_field(dy, c1, c2, "OFDM", FIELD_RX_OFDM, state_.ofdm_rx_enabled);
            if (!state_.ofdm_rx_enabled && state_.modem_type_index == 0)
                draw_rx_forced(dy, c2);
        }
        row++;

        dy = visible_y(row);
        if (dy >= 0) {
            draw_toggle_field(dy, c1, c2, "ROBUST", FIELD_RX_ROBUST, state_.robust_rx_enabled);
            if (!state_.robust_rx_enabled && state_.modem_type_index == 2)
                draw_rx_forced(dy, c2);
        }
        row++;

        dy = visible_y(row);
        if (dy >= 0) {
            draw_toggle_field(dy, c1, c2, "MFSK", FIELD_RX_MFSK, state_.mfsk_rx_enabled);
            if (!state_.mfsk_rx_enabled && state_.modem_type_index == 1)
                draw_rx_forced(dy, c2);
        }
        row += 2;

        // Audio / ptt

        dy = visible_y(row);
        if (dy >= 0) {
            attron(A_DIM);
            mvaddstr(dy, c1, "AUDIO/PTT");
            mvaddstr(dy, c1 + 10, "(restart)");
            attroff(A_DIM);
        }
        row++;
        
        // Audio input device
        dy = visible_y(row);
        if (dy >= 0) {
            std::string dev_display = state_.audio_input_device;
            if (dev_display.length() > 12) {
                dev_display = dev_display.substr(0, 11) + "~";
            }
            draw_field(dy, c1, c2, "Input", FIELD_AUDIO_INPUT, dev_display, true);
            if (!state_.audio_connected.load() && current_field_ != FIELD_AUDIO_INPUT) {
                attron(COLOR_PAIR(2) | A_BOLD);
                mvaddstr(dy, c1, "Input");
                attroff(COLOR_PAIR(2) | A_BOLD);
            }
        }
        row++;
        
        // Audio output device
        dy = visible_y(row);
        if (dy >= 0) {
            std::string dev_display = state_.audio_output_device;
            if (dev_display.length() > 12) {
                dev_display = dev_display.substr(0, 11) + "~";
            }
            draw_field(dy, c1, c2, "Output", FIELD_AUDIO_OUTPUT, dev_display, true);
            if (!state_.audio_connected.load() && current_field_ != FIELD_AUDIO_OUTPUT) {
                attron(COLOR_PAIR(2) | A_BOLD);
                mvaddstr(dy, c1, "Output");
                attroff(COLOR_PAIR(2) | A_BOLD);
            }
        }
        row++;

        dy = visible_y(row);
        if (dy >= 0) {
            if (!rig_ui()) {
                char lvl_buf[24];
                snprintf(lvl_buf, sizeof(lvl_buf), "%d%%",
                         (int)lround(state_.tx_drive.load() * 100));
                draw_selector_field(dy, c1, c2, "TX Level", FIELD_TX_LEVEL, lvl_buf);
                attron(A_DIM);
                mvaddnstr(dy, c2 + 10, "Soundcard output level, applies live",
                          std::max(0, divider - (c2 + 10) - 1));
                attroff(A_DIM);
            } else {
                attron(A_DIM);
                mvaddstr(dy, c1, "TX Level");
                mvaddnstr(dy, c2 + 10, "TX audio level and controls under RIG tab",
                          std::max(0, divider - (c2 + 10) - 1));
                attroff(A_DIM);
            }
        }
        row++;

        dy = visible_y(row);
        if (dy >= 0) {
            draw_field(dy, c1, c2, "PTT", FIELD_PTT_TYPE,
                       PTT_TYPE_OPTIONS[state_.ptt_type_index], true);
            bool ptt_err = state_.ptt_failed.load() ||
                           (rig_ui() && !state_.rigctl_connected.load());
            if (ptt_err) {
                if (current_field_ != FIELD_PTT_TYPE) {
                    attron(COLOR_PAIR(2) | A_BOLD);
                    mvaddstr(dy, c1, "PTT");
                    attroff(COLOR_PAIR(2) | A_BOLD);
                }
                attron(COLOR_PAIR(2) | A_BOLD);
                mvaddstr(dy, c2 + 18, "!key failed");
                attroff(COLOR_PAIR(2) | A_BOLD);
            }
        }
        row++;
        
        if (state_.ptt_type_index == 2) {  // VOX
            dy = visible_y(row);
            if (dy >= 0) {
                char vox_freq_buf[32];
                snprintf(vox_freq_buf, sizeof(vox_freq_buf), "%d Hz", state_.vox_tone_freq);
                draw_selector_field(dy, c1, c2, "VOX Tone", FIELD_VOX_FREQ, vox_freq_buf);
            }
            row++;
            
            dy = visible_y(row);
            if (dy >= 0) {
                char vox_lead_buf[32];
                snprintf(vox_lead_buf, sizeof(vox_lead_buf), "%d ms", state_.vox_lead_ms);
                draw_selector_field(dy, c1, c2, "VOX Lead", FIELD_VOX_LEAD, vox_lead_buf);
            }
            row++;

            dy = visible_y(row);
            if (dy >= 0) {
                char vox_tail_buf[32];
                snprintf(vox_tail_buf, sizeof(vox_tail_buf), "%d ms", state_.vox_tail_ms);
                draw_selector_field(dy, c1, c2, "VOX Tail", FIELD_VOX_TAIL, vox_tail_buf);
            }
            row++;
        }
        
        if (state_.ptt_type_index == 3) {  // COM
            dy = visible_y(row);
            if (dy >= 0) {
                std::string port_display = state_.com_port;
                if (port_display.length() > 14) {
                    port_display = port_display.substr(0, 13) + "~";
                }
                draw_field(dy, c1, c2, "COM Port", FIELD_COM_PORT, port_display, true);
            }
            row++;
            
            dy = visible_y(row);
            if (dy >= 0) draw_selector_field(dy, c1, c2, "PTT Line", FIELD_COM_LINE,
                               PTT_LINE_OPTIONS[state_.com_ptt_line]);
            row++;
            
            dy = visible_y(row);
            if (dy >= 0) {
                std::string invert_str;
                if (!state_.com_invert_dtr && !state_.com_invert_rts) {
                    invert_str = "NORMAL";
                } else if (state_.com_invert_dtr && !state_.com_invert_rts) {
                    invert_str = "INV DTR";
                } else if (!state_.com_invert_dtr && state_.com_invert_rts) {
                    invert_str = "INV RTS";
                } else {
                    invert_str = "INV BOTH";
                }
                draw_selector_field(dy, c1, c2, "Invert", FIELD_COM_INVERT, invert_str);
            }
            row++;
        }
        if (state_.ptt_type_index == 5) {
            dy = visible_y(row);
            if (dy >= 0) {
                std::string m = state_.hamlib_model > 0 ? hamlib_model_label(state_.hamlib_model) : "select";
                if (m.length() > 22) m = m.substr(0, 21) + "~";
                draw_field(dy, c1, c2, "Rig", FIELD_HAMLIB_MODEL, m, true);
            }
            row++;
            dy = visible_y(row);
            if (dy >= 0) {
                std::string d = state_.hamlib_device.empty() ? "none" : state_.hamlib_device;
                if (d.length() > 22) d = d.substr(0, 21) + "~";
                draw_field(dy, c1, c2, "Rig Device", FIELD_HAMLIB_DEVICE, d, true);
            }
            row++;
            dy = visible_y(row);
            if (dy >= 0) draw_selector_field(dy, c1, c2, "Rig Baud", FIELD_HAMLIB_BAUD,
                                             state_.hamlib_baud > 0 ? std::to_string(state_.hamlib_baud) : std::string("default"));
            row++;
        }
#ifdef WITH_CM108
        if (state_.ptt_type_index == 4) {  // CM108
            dy = visible_y(row);
            if (dy >= 0) {
                char cm108_gpio_buf[32];
                snprintf(cm108_gpio_buf, sizeof(cm108_gpio_buf), "%d", state_.cm108_gpio);
                draw_field(dy, c1, c2, "GPIO Pin", FIELD_CM108_GPIO, cm108_gpio_buf, true);
            }
            row++;

            dy = visible_y(row);
            if (dy >= 0) {
                std::string dev_display = state_.cm108_device.empty() ? "Auto" : state_.cm108_device;
                if (dev_display.length() > 12) {
                    dev_display = dev_display.substr(0, 11) + "~";
                }
                draw_field(dy, c1, c2, "Device", FIELD_CM108_DEVICE, dev_display, true);
            }
            row++;
        }
#endif
        dy = visible_y(row);
        if (dy >= 0) {
            char txd_buf[24];
            snprintf(txd_buf, sizeof(txd_buf), "%d ms", state_.tx_delay_ms);
            draw_selector_field(dy, c1, c2, "TX Delay", FIELD_TX_DELAY, txd_buf);
        }
        row++;
        row++;
        
        // Network section
        dy = visible_y(row);
        if (dy >= 0) {
            attron(A_DIM);
            mvaddstr(dy, c1, "NETWORK");
            mvaddstr(dy, c1 + 8, "(restart)");
            attroff(A_DIM);
        }
        row++;
        
        dy = visible_y(row);
        if (dy >= 0) {
            char port_buf[32];
            snprintf(port_buf, sizeof(port_buf), "%d", state_.port);
            draw_field(dy, c1, c2, "KISS Port", FIELD_NET_PORT, port_buf, true);
        }
        row++;

        dy = visible_y(row);
        if (dy >= 0) {
            char cport_buf[32];
            snprintf(cport_buf, sizeof(cport_buf), "%d", state_.control_port);
            draw_field(dy, c1, c2, "Control Port", FIELD_CONTROL_PORT, cport_buf, true);
        }
        row++;

        dy = visible_y(row);
        if (dy >= 0) {
            bool lan = state_.bind_address == "0.0.0.0" &&
                       state_.control_bind_address == "0.0.0.0";
            draw_field(dy, c1, c2, "LAN Mode", FIELD_LAN_MODE, lan ? "ON" : "OFF", true);
        }
        row += 2;
        
        //  Preset section
        dy = visible_y(row);
        if (dy >= 0) {
            attron(A_DIM);
            mvaddstr(dy, c1, "PRESET");
            attroff(A_DIM);
        }
        row++;
        
        dy = visible_y(row);
        if (dy >= 0) {
            bool sel = (current_field_ == FIELD_PRESET);
            if (sel) {
                attron(A_BOLD);
                mvaddch(dy, c1 - 2, '>');
                mvaddstr(dy, c1, "Load");
                attroff(A_BOLD);
            } else {
                attron(A_DIM);
                mvaddstr(dy, c1, "Load");
                attroff(A_DIM);
            }
            
            move(dy, c2);
            if (state_.presets.empty()) {
                attron(A_DIM);
                addstr("(none)");
                attroff(A_DIM);
            } else {
                if (sel) attron(COLOR_PAIR(4) | A_BOLD);
                addstr("< ");
                if (state_.selected_preset >= 0 && state_.selected_preset < (int)state_.presets.size()) {
                    printw("%-10s", state_.presets[state_.selected_preset].name.c_str());
                }
                addstr(" >");
                if (sel) attroff(COLOR_PAIR(4) | A_BOLD);
            }
            
            int hint_y = visible_y(row + 1);
            if (sel && hint_y >= 0) {
                attron(A_DIM);
                mvaddstr(hint_y, c1, "Enter=load s=save x=del");
                attroff(A_DIM);
            }
        }
        
        // Info / stas
        y = start_y;
        
        attron(COLOR_PAIR(4) | A_BOLD);
        mvaddstr(y, c3, "MODEM INFO");
        attroff(COLOR_PAIR(4) | A_BOLD);
        y++;
        
        mvprintw(y, c3, "Payload %d B", state_.mtu_bytes);
        attron(COLOR_PAIR(4) | A_BOLD);
        if (state_.bitrate_bps >= 1000) {
            printw("  %.1f kb/s", state_.bitrate_bps / 1000.0f);
        } else {
            printw("  %d b/s", state_.bitrate_bps);
        }
        attroff(COLOR_PAIR(4) | A_BOLD);
        y++;
        
        mvprintw(y, c3, "Frame %.2fs", state_.airtime_seconds);
        float tx_time = state_.total_tx_time.load();
        printw("  TX ");
        if (tx_time < 60) printw("%.0fs", tx_time);
        else printw("%.1fm", tx_time / 60.0f);
        if (state_.csma_enabled) {
            int net = net_bps_estimate(true, state_.csma_quiet_ms, state_.csma_cw,
                                       state_.slot_time_ms, state_.csma_burst,
                                       state_.tx_lead_tone, state_.tx_delay_ms,
                                       state_.airtime_seconds, state_.mtu_bytes);
            attron(A_DIM);
            printw("  net ~%d b/s", net);
            attroff(A_DIM);
        }
        y++;




        {
            bool robust = (state_.modem_type_index == 2);
            bool hf_ok = robust || (state_.modem_type_index == 1) ||
                         (state_.modulation_index <= 2); // BPSK, QPSK, 8PSK
            mvaddstr(y, c3, "Band  ");
            if (hf_ok) {
                attron(COLOR_PAIR(3) | A_BOLD);
                addstr(robust ? "HF" : "HF/VHF");
                attroff(COLOR_PAIR(3) | A_BOLD);
            } else {
                attron(A_DIM);
                addstr("HF/VHF");
                attroff(A_DIM);
            }
            addstr("  ");
            if (!hf_ok) {
                attron(COLOR_PAIR(3) | A_BOLD);
                addstr("VHF/UHF");
                attroff(COLOR_PAIR(3) | A_BOLD);
            } else {
                attron(A_DIM);
                addstr("VHF/UHF");
                attroff(A_DIM);
            }
        }
        y += 2;
        
        // Right side, for audio / ptt status

        attron(COLOR_PAIR(4) | A_BOLD);
        mvaddstr(y, c3, "AUDIO/PTT");
        attroff(COLOR_PAIR(4) | A_BOLD);
        
        // Audio connection status
        if (state_.audio_connected.load()) {
            attron(COLOR_PAIR(1) | A_BOLD);
            addstr(" OK");
            attroff(COLOR_PAIR(1) | A_BOLD);
        } else {
            attron(COLOR_PAIR(2) | A_BOLD);
            addstr(" DISCONNECTED");
            attroff(COLOR_PAIR(2) | A_BOLD);
        }
        y++;
        

        // Show input device
        mvaddstr(y, c3, "In: ");
        {
            std::string dev_short = state_.audio_input_device;
            if (dev_short.length() > 14) dev_short = dev_short.substr(0, 13) + "~";
            if (state_.audio_connected.load()) {
                attron(A_DIM);
                addstr(dev_short.c_str());
                attroff(A_DIM);
            } else {
                attron(COLOR_PAIR(2));
                addstr(dev_short.c_str());
                attroff(COLOR_PAIR(2));
            }
        }
        y++;
        
        // Show output device
        mvaddstr(y, c3, "Out:");
        {
            std::string dev_short = state_.audio_output_device;
            if (dev_short.length() > 14) dev_short = dev_short.substr(0, 13) + "~";
            if (state_.audio_connected.load()) {
                attron(A_DIM);
                addstr(dev_short.c_str());
                attroff(A_DIM);
            } else {
                attron(COLOR_PAIR(2));
                addstr(dev_short.c_str());
                attroff(COLOR_PAIR(2));
            }
        }

        y++;
        
        mvaddstr(y, c3, "PTT: ");
        addstr(PTT_TYPE_OPTIONS[state_.ptt_type_index].c_str());
        if (rig_ui()) {
            if (state_.rigctl_connected.load()) {
                attron(COLOR_PAIR(1) | A_BOLD);
                addstr(" OK");
                attroff(COLOR_PAIR(1) | A_BOLD);
            } else {
                attron(COLOR_PAIR(2) | A_BOLD);
                addstr(" --");
                attroff(COLOR_PAIR(2) | A_BOLD);
            }
        }
        if (state_.ptt_on.load()) {
            attron(COLOR_PAIR(2) | A_BOLD);
            addstr(" TX");
            attroff(COLOR_PAIR(2) | A_BOLD);
        }
        y += 2;
        


        attron(COLOR_PAIR(4) | A_BOLD);
        mvaddstr(y, c3, "NETWORK");
        attroff(COLOR_PAIR(4) | A_BOLD);
        y++;
        
        mvprintw(y, c3, "Port: %d", state_.port);
        printw("  ");
        attron(COLOR_PAIR(4));
        printw("%dc", state_.client_count.load());
        attroff(COLOR_PAIR(4));
        y += 2;
        



        if (state_.selected_preset >= 0 && state_.selected_preset < (int)state_.presets.size()) {
            const auto& p = state_.presets[state_.selected_preset];
            attron(COLOR_PAIR(4) | A_BOLD);
            mvaddstr(y, c3, "PRESET");
            attroff(COLOR_PAIR(4) | A_BOLD);
            attron(A_DIM);
            printw(" %s", p.name.c_str());
            attroff(A_DIM);
            y++;
            
            if (p.modem_type_index == 1) {
                mvprintw(y, c3, "%s", MFSK_MODE_OPTIONS[p.mfsk_mode_index].c_str());
            } else if (p.modem_type_index == 2) {
                mvprintw(y, c3, "%s", ROBUST_MODE_OPTIONS[p.robust_mode_index].c_str());
            } else {
                mvprintw(y, c3, "%s %s %s",
                         MODULATION_OPTIONS[p.modulation_index].c_str(),
                         CODE_RATE_OPTIONS[p.code_rate_index].c_str(),
                         p.frame_size == 0 ? "S" : p.frame_size == 2 ? "L"
                       : p.frame_size == 3 ? "U" : "N");
            }
            y++;
            
            mvaddstr(y, c3, "PTT ");
            addstr(PTT_TYPE_OPTIONS[p.ptt_type_index].c_str());
            if (p.ptt_type_index == 2) {
                printw(" %dHz", p.vox_tone_freq);
            }
            y++;
            
            mvaddstr(y, c3, "CSMA ");
            if (p.csma_enabled) {
                attron(COLOR_PAIR(1) | A_BOLD);
                addstr("ON");
                attroff(COLOR_PAIR(1) | A_BOLD);
            } else {
                addstr("OFF");
            }
            y++;
            
            if (current_field_ == FIELD_PRESET) {
                if (state_.selected_preset == state_.loaded_preset_index) {
                    attron(COLOR_PAIR(1) | A_BOLD);
                    mvaddstr(y, c3, "/// loaded");
                    attroff(COLOR_PAIR(1) | A_BOLD);
                } else {
                    bool blink_on = (frame_counter_ / 15) % 2 == 0;
                    if (blink_on) {
                        attron(COLOR_PAIR(4) | A_BOLD);
                        mvaddstr(y, c3, "/// ENTER TO LOAD");
                        attroff(COLOR_PAIR(4) | A_BOLD);
                    }
                }
            }
        }
    }
    
    void draw_field(int y, int c1, int c2, const char* label, int field,
                    const std::string& value, bool editable) {
        draw_field_ex(y, c1, c2, label, field == current_field_, value, editable);
    }

    void draw_field_ex(int y, int c1, int c2, const char* label, bool sel,
                       const std::string& value, bool editable) {

        if (sel) {
            attron(A_BOLD);
            mvaddch(y, c1 - 2, '>');
            mvaddstr(y, c1, label);
            attroff(A_BOLD);
            move(y, c2);
            attron(COLOR_PAIR(4) | A_BOLD);
            addstr(value.c_str());
            attroff(COLOR_PAIR(4) | A_BOLD);
            if (editable) {
                attron(A_DIM);
                addstr("  [enter]");
                attroff(A_DIM);
            }
        } else {
            attron(A_DIM);
            mvaddstr(y, c1, label);
            attroff(A_DIM);
            mvaddstr(y, c2, value.c_str());
        }
    }
    
    void draw_selector_field(int y, int c1, int c2, const char* label, int field,
                             const std::string& value) {
        draw_selector_field_ex(y, c1, c2, label, field == current_field_, value);
    }

    void draw_selector_field_ex(int y, int c1, int c2, const char* label, bool sel,
                                const std::string& value) {

        if (sel) {
            attron(A_BOLD);
            mvaddch(y, c1 - 2, '>');
            mvaddstr(y, c1, label);
            attroff(A_BOLD);
            move(y, c2);
            attron(A_DIM);
            addstr("<");
            attroff(A_DIM);
            attron(COLOR_PAIR(4) | A_BOLD);
            printw(" %s ", value.c_str());
            attroff(COLOR_PAIR(4) | A_BOLD);
            attron(A_DIM);
            addstr(">");
            attroff(A_DIM);
        } else {
            attron(A_DIM);
            mvaddstr(y, c1, label);
            attroff(A_DIM);
            mvprintw(y, c2, "  %s", value.c_str());
        }
    }

    // which modem family tab is on screen
    int modem_tab_at(int x, int cols) {
        int c2 = 16, divider = cols / 2 - 2, total = 0;
        for (const auto& o : MODEM_TYPE_OPTIONS)
            total += (int)o.size() + 2;
        if (c2 + total > divider)
            return -1;
        int cx = c2;
        for (int i = 0; i < (int)MODEM_TYPE_OPTIONS.size(); ++i) {
            int w = (int)MODEM_TYPE_OPTIONS[i].size() + 2;
            if (x >= cx && x < cx + w)
                return i;
            cx += w;
        }
        return -1;
    }

    void draw_modem_tabs(int y, int c1, int c2, int right_limit) {
        bool sel = (FIELD_MODEM_TYPE == current_field_);
        int total = 0;
        for (const auto& o : MODEM_TYPE_OPTIONS)
            total += (int)o.size() + 2;
        if (c2 + total > right_limit) {
            draw_selector_field(y, c1, c2, "Modem", FIELD_MODEM_TYPE,
                                MODEM_TYPE_OPTIONS[state_.modem_type_index]);
            return;
        }
        if (sel) {
            attron(A_BOLD);
            mvaddch(y, c1 - 2, '>');
        }
        mvaddstr(y, c1, "Modem");
        if (sel) attroff(A_BOLD);
        move(y, c2);
        for (int i = 0; i < (int)MODEM_TYPE_OPTIONS.size(); ++i) {
            bool active = (i == state_.modem_type_index);
            if (active) {
                attron(COLOR_PAIR(6) | A_BOLD | A_REVERSE);
                printw(" %s ", MODEM_TYPE_OPTIONS[i].c_str());
                attroff(COLOR_PAIR(6) | A_BOLD | A_REVERSE);
            } else {
                attron(A_DIM);
                printw(" %s ", MODEM_TYPE_OPTIONS[i].c_str());
                attroff(A_DIM);
            }
        }
    }

    void draw_toggle_field(int y, int c1, int c2, const char* label, int field, bool value) {
        bool sel = (field == current_field_);
        
        if (sel) {
            attron(A_BOLD);
            mvaddch(y, c1 - 2, '>');
            mvaddstr(y, c1, label);
            attroff(A_BOLD);
            move(y, c2);
            attron(A_DIM);
            addstr("<");
            attroff(A_DIM);
            if (value) {
                attron(COLOR_PAIR(1) | A_BOLD);
                addstr(" ON ");
                attroff(COLOR_PAIR(1) | A_BOLD);
            } else {
                attron(COLOR_PAIR(3) | A_BOLD);
                addstr(" OFF ");
                attroff(COLOR_PAIR(3) | A_BOLD);
            }
            attron(A_DIM);
            addstr(">");
            attroff(A_DIM);
        } else {
            attron(A_DIM);
            mvaddstr(y, c1, label);
            attroff(A_DIM);
            move(y, c2);
            if (value) {
                attron(COLOR_PAIR(1));
                addstr("  ON");
                attroff(COLOR_PAIR(1));
            } else {
                attron(COLOR_PAIR(3));
                addstr("  OFF");
                attroff(COLOR_PAIR(3));
            }
        }
    }
    
    void draw_log(int y, int h, int cols) {
        // Decode stats header
        int c1 = 3;
        attron(A_DIM);
        mvaddstr(y, c1, "DECODE STATS");
        attroff(A_DIM);
        y++;
        
        int syncs = state_.sync_count.load();
        int pre_err = state_.preamble_errors.load();
        int sym_err = state_.symbol_errors.load();
        int crc_err = state_.crc_errors.load();
        int unframe_err = state_.rx_error_count.load();
        int decoded = state_.rx_frame_count.load();
        
        mvaddstr(y, c1, "Syncs");
        attron(COLOR_PAIR(4));
        printw(" %d", syncs);
        attroff(COLOR_PAIR(4));
        
        addstr("  Decoded");
        attron(COLOR_PAIR(1));
        printw(" %d", decoded);
        attroff(COLOR_PAIR(1));
        
        addstr("  CRC Fail");
        if (crc_err > 0) attron(COLOR_PAIR(2));
        printw(" %d", crc_err);
        if (crc_err > 0) attroff(COLOR_PAIR(2));
        
        addstr("  Seed Err");
        if (sym_err > 0) attron(COLOR_PAIR(2));
        printw(" %d", sym_err);
        if (sym_err > 0) attroff(COLOR_PAIR(2));

        int erased = state_.erased_symbols.load();
        addstr("  Erased");
        if (erased > 0) attron(COLOR_PAIR(3));
        printw(" %d", erased);
        if (erased > 0) attroff(COLOR_PAIR(3));

        addstr("  Pre Err");
        if (pre_err > 0) attron(COLOR_PAIR(2));
        printw(" %d", pre_err);
        if (pre_err > 0) attroff(COLOR_PAIR(2));
        
        if (unframe_err > 0) {
            addstr("  Unframe");
            attron(COLOR_PAIR(2));
            printw(" %d", unframe_err);
            attroff(COLOR_PAIR(2));
        }
        
        y += 2;
        h -= 3;
        
        auto log = state_.get_log();
        int visible = h - 1;
        int max_scroll = std::max(0, (int)log.size() - visible);
        if (log_follow_)
            log_scroll_ = max_scroll;
        else
            log_scroll_ = std::min(log_scroll_, max_scroll);
        if (log_scroll_ >= max_scroll)
            log_follow_ = true;

        int text_width = cols - 5;

        for (int i = 0; i < visible && (log_scroll_ + i) < (int)log.size(); i++) {
            const std::string& line = log[log_scroll_ + i];

            int pair = 0;
            bool bold = false;
            if (line.size() > 12 && line.compare(10, 3, "(!)") == 0) { pair = 3; bold = true; }
            else if (line.find("TX:") != std::string::npos) { pair = 2; bold = true; }
            else if (line.find("RX:") != std::string::npos) { pair = 1; bold = true; }
            else if (line.find("CSMA") != std::string::npos) pair = 3;
            else if (line.find("error") != std::string::npos || 
                     line.find("Error") != std::string::npos ||
                     line.find("failed") != std::string::npos) pair = 2;
            else if (line.find("Client") != std::string::npos) pair = 4;
            
            if (pair) attron(COLOR_PAIR(pair));
            if (bold) attron(A_BOLD);
            
            if ((int)line.length() > text_width) {
                mvprintw(y + i, 2, "%.*s...", text_width - 3, line.c_str());
            } else {
                mvprintw(y + i, 2, "%s", line.c_str());
            }
            
            if (bold) attroff(A_BOLD);
            if (pair) attroff(COLOR_PAIR(pair));
        }

        state_.log_unread_error = false;

        // scrollbar based on dims
        if ((int)log.size() > visible && visible > 2) {
            int sb_height = visible;
            int thumb_size = std::max(1, sb_height * visible / (int)log.size());
            int thumb_pos = max_scroll > 0 ? log_scroll_ * (sb_height - thumb_size) / max_scroll : 0;
            
            for (int i = 0; i < sb_height; i++) {
                if (i >= thumb_pos && i < thumb_pos + thumb_size) {
                    mvaddch(y + i, cols - 2, ACS_BLOCK);
                } else {
                    attron(A_DIM);
                    mvaddch(y + i, cols - 2, ACS_VLINE);
                    attroff(A_DIM);
                }
            }
        }
    }
    
    void draw_constellation(int y, int x, int height, int width) {
        // Height and width are separate to account for terminal character aspect ratio
        if (height < 5) height = 5;
        if (width < 9) width = 9;
        
        // Draw border using ACS characters
        attron(A_DIM);
        mvaddch(y, x, ACS_ULCORNER);
        mvaddch(y, x + width + 1, ACS_URCORNER);
        mvaddch(y + height + 1, x, ACS_LLCORNER);
        mvaddch(y + height + 1, x + width + 1, ACS_LRCORNER);
        for (int i = 1; i <= width; ++i) {
            mvaddch(y, x + i, ACS_HLINE);
            mvaddch(y + height + 1, x + i, ACS_HLINE);
        }
        for (int i = 1; i <= height; ++i) {
            mvaddch(y + i, x, ACS_VLINE);
            mvaddch(y + i, x + width + 1, ACS_VLINE);
        }
        
        // Axis labels
        mvaddstr(y - 1, x + width/2 - 1, "+Im");  // Top center (positive imaginary)
        mvaddstr(y + height + 2, x + width/2 - 1, "-Im");  // Bottom center (negative imaginary)
        mvaddstr(y + height/2, x - 4, "-Re");  // Left (negative real)
        mvaddstr(y + height/2, x + width + 3, "+Re");  // Right (positive real)
        attroff(A_DIM);
        
        // Draw key to the right of constellation
        int key_x = x + width + 8;
        int key_y = y + 1;
        
        attron(A_DIM);
        mvaddstr(key_y, key_x, "DENSITY");
        attroff(A_DIM);
        key_y += 1;
        
        // Show density scale with colors
        attron(COLOR_PAIR(1) | A_BOLD);
        mvaddstr(key_y++, key_x, "@ # * High");
        attroff(COLOR_PAIR(1) | A_BOLD);
        
        attron(COLOR_PAIR(3));
        mvaddstr(key_y++, key_x, "+ = - Med");
        attroff(COLOR_PAIR(3));
        
        attron(A_DIM);
        mvaddstr(key_y++, key_x, ": .   Low");
        attroff(A_DIM);
        
        key_y++;
        attron(A_DIM);
        mvaddstr(key_y++, key_x, "AXES");
        attroff(A_DIM);
        mvaddstr(key_y++, key_x, "Re: In-phase");
        mvaddstr(key_y++, key_x, "Im: Quadrature");
        
        // Check for stale data (10 second timeout)
        auto now = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        bool stale = (now - state_.constellation_update_time.load()) > 10000;
        
        if (!state_.constellation_valid.load() || stale) {
            // No data - show placeholder
            attron(A_DIM);
            int mid_y = height / 2;
            int mid_x = (width - 9) / 2;  // "No signal" is 9 chars
            mvaddstr(y + 1 + mid_y, x + 1 + mid_x, "No signal");
            attroff(A_DIM);
            return;
        }
        
        // Density characters (space to full block)
        const char* density_chars = " .:-=+*#@";
        const int num_chars = 9;
        
        std::lock_guard<std::mutex> lock(state_.constellation_mutex);
        
        // Find peak density for normalization
        int peak = 1;
        for (size_t i = 0; i < state_.constellation_density.size(); ++i) {
            if (state_.constellation_density[i] > peak) {
                peak = state_.constellation_density[i];
            }
        }
        
        // Scale factors to map grid to display (separate for x and y)
        int grid_size = TNCUIState::CONSTELLATION_GRID;
        float scale_y = (float)grid_size / height;
        float scale_x = (float)grid_size / width;
        
        // Draw constellation points
        for (int dy = 0; dy < height; ++dy) {
            for (int dx = 0; dx < width; ++dx) {
                // Map display coords to grid coords
                int gx = (int)(dx * scale_x);
                int gy = (int)(dy * scale_y);
                
                // Ensure at least 1 grid cell per display cell (prevents striping)
                int gx_end = std::max(gx + 1, std::min((int)((dx + 1) * scale_x), grid_size));
                int gy_end = std::max(gy + 1, std::min((int)((dy + 1) * scale_y), grid_size));

                int density = 0;
                for (int sy = gy; sy < gy_end; ++sy) {
                    for (int sx = gx; sx < gx_end; ++sx) {
                        int d = state_.constellation_density[sy * grid_size + sx];
                        if (d > density) density = d;
                    }
                }
                
                // Map density to character
                int char_idx = (density * (num_chars - 1)) / peak;
                if (density > 0 && char_idx == 0) char_idx = 1;
                char_idx = std::min(char_idx, num_chars - 1);
                
                // Apply color based on density
                if (char_idx >= 6) {
                    attron(COLOR_PAIR(1) | A_BOLD);  // Green = high density (good)
                } else if (char_idx >= 3) {
                    attron(COLOR_PAIR(3));           // Yellow = medium
                } else if (char_idx >= 1) {
                    attron(A_DIM);                   // Dim = low density
                }
                
                mvaddch(y + 1 + dy, x + 1 + dx, density_chars[char_idx]);
                
                attroff(COLOR_PAIR(1) | A_BOLD);
                attroff(COLOR_PAIR(3));
                attroff(A_DIM);
            }
        }
        
        // Draw center crosshair (only if cell is empty)
        int mid_y = height / 2;
        int mid_x = width / 2;
        int mid_gx = (int)(mid_x * scale_x);
        int mid_gy = (int)(mid_y * scale_y);
        bool center_empty = (mid_gx < grid_size && mid_gy < grid_size &&
                            state_.constellation_density[mid_gy * grid_size + mid_gx] == 0);
        if (center_empty) {
            attron(A_DIM);
            mvaddch(y + 1 + mid_y, x + 1 + mid_x, '+');
            attroff(A_DIM);
        }
        
        // Show modulation name in top-right of box
        const char* mod_name = "";
        switch (state_.constellation_mod_bits) {
            case 1: mod_name = "BPSK"; break;
            case 2: mod_name = "QPSK"; break;
            case 3: mod_name = "8PSK"; break;
            case 4: mod_name = "QAM16"; break;
            case 6: mod_name = "QAM64"; break;
            case 8: mod_name = "QAM256"; break;
            case 10: mod_name = "QAM1024"; break;
            case 12: mod_name = "QAM4096"; break;
        }
        if (mod_name[0]) {
            int name_len = strlen(mod_name);
            attron(A_DIM);
            mvaddstr(y, x + width + 1 - name_len, mod_name);
            attroff(A_DIM);
        }
    }
    
    void update_waterfall() {

        int64_t now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        if (now_ms - wf_row_ms_ < WF_ROW_MS)
            return;

        static constexpr int SEGS = 6;
        static constexpr int HOP = WF_FFT / 2;
        static constexpr int SPAN = WF_FFT + HOP * (SEGS - 1);
        float tdom[SPAN];
        {
            std::lock_guard<std::mutex> lock(state_.wf_mutex);
            if (state_.wf_written == wf_seen_)
                return;
            wf_seen_ = state_.wf_written;
            int p = state_.wf_wpos;
            for (int i = 0; i < SPAN; i++)
                tdom[i] = state_.wf_ring[(p + TNCUIState::WF_RING - SPAN + i)
                                         % TNCUIState::WF_RING];
        }



        wf_row_ms_ = now_ms;

        static const auto window = [] {
            std::array<float, WF_FFT> w{};
            for (int i = 0; i < WF_FFT; i++)
                w[i] = 0.5f - 0.5f * std::cos(2.0f * (float)M_PI * i / WF_FFT);
            return w;
        }();

        float pwr[WF_BINS] = {};
        float seg[WF_FFT];
        std::complex<float> fdom[WF_FFT / 2 + 1];
        for (int s = 0; s < SEGS; s++) {
            const float* src = tdom + s * HOP;
            for (int i = 0; i < WF_FFT; i++)
                seg[i] = src[i] * window[i];
            wf_fft_(fdom, seg);
            for (int i = 0; i < WF_BINS; i++)
                pwr[i] += std::norm(fdom[i]);
        }

        float db[WF_BINS];
        for (int i = 0; i < WF_BINS; i++) {
            float p3 = 2.0f * pwr[i] + pwr[i > 0 ? i - 1 : 0] +
                       pwr[i < WF_BINS - 1 ? i + 1 : i];
            db[i] = 10.0f * std::log10(p3 + 1e-12f);
        }

        float sorted[WF_BINS];
        std::copy(db, db + WF_BINS, sorted);
        std::nth_element(sorted, sorted + WF_BINS / 2, sorted + WF_BINS);
        float med = sorted[WF_BINS / 2];
        if (med < -105.0f)
            return;
        if (wf_rows_.empty())
            wf_floor_db_ = med;
        else if (med < wf_floor_db_)
            wf_floor_db_ += (med - wf_floor_db_) * 0.5f;
        else
            wf_floor_db_ += (med - wf_floor_db_) * 0.02f;

        int64_t sig = state_.wf_sig_ms.load();
        if (sig != wf_sig_seen_) {
            wf_sig_seen_ = sig;
            for (auto& rw : wf_rows_)
                if (rw.ms > sig - 420 && rw.ms <= sig + 60)
                    rw.sig = true;
        }
        WFRow nr;
        nr.ms = now_ms;
        nr.sig = wf_sig_seen_ != 0 && now_ms > wf_sig_seen_ - 420 &&
                 now_ms <= wf_sig_seen_ + 60;
        for (int i = 0; i < WF_BINS; i++) {
            float v = (db[i] - wf_floor_db_ + 6.0f) * (255.0f / 50.0f);
            nr.bins[i] = (uint8_t)std::max(0.0f, std::min(255.0f, v));
        }
        wf_rows_.push_front(nr);
        if ((int)wf_rows_.size() > WF_MAX_ROWS)
            wf_rows_.pop_back();
    }

    void draw_waterfall(int y, int x, int h, int w) {

        // frequency axis

        attron(A_DIM);
        mvhline(y, x, ACS_HLINE, w);
        for (int f = 0; f <= 3000; f += 500) {
            int cx = x + (int)((int64_t)f * (w - 1) / 3000);
            if (f % 1000) {
                mvaddch(y, cx, '+');
            } else if (f == 0) {
                mvaddch(y, cx, '0');
            } else {
                char lbl[8];
                snprintf(lbl, sizeof(lbl), "%dk", f / 1000);
                if (cx + (int)strlen(lbl) > x + w)
                    cx = x + w - (int)strlen(lbl);
                mvaddstr(y, cx, lbl);
            }
        }
        attroff(A_DIM);
        int cf = state_.center_freq;
        if (cf > 0 && cf <= 3000) {
            attron(COLOR_PAIR(4) | A_BOLD);
            mvaddch(y, x + (int)((int64_t)cf * (w - 1) / 3000), 'v');
            attroff(COLOR_PAIR(4) | A_BOLD);
        }

        if (wf_rows_.empty()) {
            attron(A_DIM);
            mvaddstr(y + 1, x, "(waiting for audio)");
            attroff(A_DIM);
            return;
        }

        int nrows = std::min((int)wf_rows_.size(), h);
        int fb0 = std::max(0, (cf - 110) * WF_BINS / 3000);
        int fb1 = std::min(WF_BINS - 1, (cf + 110) * WF_BINS / 3000);
        for (int r = 0; r < nrows; r++) {
            const auto& row = wf_rows_[r];
            move(y + 1 + r, x);
            int cur = -1;
            for (int col = 0; col < w; col++) {
                int b0 = col * WF_BINS / w;
                int b1 = std::max(b0 + 1, (col + 1) * WF_BINS / w);
                int q = 0;
                for (int b = b0; b < b1 && b < WF_BINS; b++)
                    q = std::max(q, (int)row.bins[b]);
                bool mark = row.sig && b0 <= fb1 && b1 > fb0;
                if (wf_pairs_ > 0) {
                    int attr = mark && wf_sig_ok_
                        ? (COLOR_PAIR(WF_PAIR_BASE + 32) | A_BOLD)
                        : COLOR_PAIR(WF_PAIR_BASE + q * wf_pairs_ / 256);
                    if (attr != cur) {
                        if (cur != -1) attroff(cur);
                        attron(attr);
                        cur = attr;
                    }
                    addch(' ');
                } else {
                    static const char ramp[] = " .:-=+*xX";
                    addch(mark ? 'S' : ramp[q * 9 / 256]);
                }
            }
            if (cur != -1) attroff(cur);
        }
    }

    void draw_scope(int y, int h, int cols) {
        int c1 = 3;
        int bottom = y + h;  // first row past the scope area

        attron(COLOR_PAIR(4) | A_BOLD);
        mvaddstr(y, c1, "[ CONSTELLATION ]");
        attroff(COLOR_PAIR(4) | A_BOLD);
        y += 2;
        
        // Reserve space for signal info (4 lines at bottom) and margins
        int available_h = h - 10;  // Extra for axis labels
        int available_w = cols - 28;  // Space for key on right + axis labels
        
        // Terminal chars are ~2:1 aspect ratio (taller than wide)
        // For visually square display: width should be ~2x height
        int const_height = available_h;
        if (const_height > 10) const_height = 10;  // leftover rows go to the waterfall
        int const_width = const_height * 2;  // 2:1 aspect ratio compensation
        
        // Clamp to available width
        if (const_width > available_w) {
            const_width = available_w;
            const_height = const_width / 2;
        }
        
        // Minimum sizes
        if (const_height < 9) const_height = 9;
        if (const_width < 17) const_width = 17;
        
        if (const_height >= 9) {
            // Offset to leave room for left axis label
            int x_offset = 6;
            draw_constellation(y + 1, x_offset, const_height, const_width);  // +1 for top axis label
            y += const_height + 5;  // Extra for axis labels
        } else {
            y += 2;
        }
        
        // Show signal info below constellation
        attron(A_DIM);
        mvaddstr(y, c1, "SIGNAL INFO");
        attroff(A_DIM);
        y++;
        
        // Last SNR
        mvaddstr(y, c1, "Last SNR:");
        float snr = state_.last_rx_snr.load();
        if (snr > 10.0f) {
            attron(COLOR_PAIR(1) | A_BOLD);
        } else if (snr > 5.0f) {
            attron(COLOR_PAIR(3) | A_BOLD);
        }
        mvprintw(y, c1 + 12, "%6.1f dB", snr);
        attroff(COLOR_PAIR(1) | A_BOLD);
        attroff(COLOR_PAIR(3) | A_BOLD);
        
        // Modulation
        const char* mod_name = "";
        switch (state_.constellation_mod_bits) {
            case 1: mod_name = "BPSK"; break;
            case 2: mod_name = "QPSK"; break;
            case 3: mod_name = "8PSK"; break;
            case 4: mod_name = "QAM16"; break;
            case 6: mod_name = "QAM64"; break;
            case 8: mod_name = "QAM256"; break;
            case 10: mod_name = "QAM1024"; break;
            case 12: mod_name = "QAM4096"; break;
        }
        mvaddstr(y, c1 + 28, "Modulation:");
        mvaddstr(y, c1 + 42, mod_name[0] ? mod_name : "---");
        y++;
        
        // Carrier level
        mvaddstr(y, c1, "Carrier:");
        float lvl = state_.carrier_level_db.load();
        bool busy = lvl > state_.carrier_threshold_db;
        if (busy) {
            attron(COLOR_PAIR(4) | A_BOLD);
        }
        mvprintw(y, c1 + 12, "%6.1f dB", lvl);
        attroff(COLOR_PAIR(4) | A_BOLD);
        
        // RX/TX counts
        mvaddstr(y, c1 + 28, "RX:");
        attron(COLOR_PAIR(1));
        printw(" %d", state_.rx_frame_count.load());
        attroff(COLOR_PAIR(1));
        addstr("  TX:");
        attron(COLOR_PAIR(2));
        printw(" %d", state_.tx_frame_count.load());
        attroff(COLOR_PAIR(2));

        y += 2;
        int wf_h = bottom - (y + 1);  // history rows under the axis line
        if (wf_h < 3) {
            state_.scope_active = false;  // too small; skip audio capture too
            return;
        }
        update_waterfall();
        draw_waterfall(y, c1, wf_h, cols - c1 - 3);
    }
    
    void draw_utils(int y, int h, int cols) {
        int c1 = 3;
        int c2 = cols / 2 + 2;
        int start_y = y;

        auto msgs = state_.get_messages();
        int show = std::min((int)msgs.size(), 4);

        int total_rows = 10 + utils_visible_slots() + std::max(1, show);
        int right_rows = 0;
        if (state_.perf_logger)
            right_rows = 5 + std::max(1, (int)state_.perf_logger->snapshot().size());
        right_rows += 2 + (int)state_.get_recent_packets().size();
        total_rows = std::max(total_rows, right_rows);
        int max_scroll = std::max(0, total_rows - h);
        utils_max_scroll_ = max_scroll;
        if (utils_scroll_ > max_scroll) utils_scroll_ = max_scroll;
        if (utils_scroll_ < 0) utils_scroll_ = 0;
        int row = 0;
        auto vy = [&](int logical_row) -> int {
            int screen_row = logical_row - utils_scroll_;
            if (screen_row < 0 || screen_row >= h) return -1;
            return start_y + screen_row;
        };

        int rn = 0;
        int rw = std::max(0, cols - c2 - 2);
        if (state_.perf_logger) {
            auto rrow = [&](int r) -> int { return vy(r); };
            int r = 0, dy2;
            char rbuf[256];
            auto rput = [&](int ry2, const char* s) {
                mvaddnstr(ry2, c2, s, rw);
            };
            if ((dy2 = rrow(r++)) >= 0) {
                attron(COLOR_PAIR(4) | A_BOLD);
                rput(dy2, "[ RX PERFORMANCE ]");
                attroff(COLOR_PAIR(4) | A_BOLD);
            }
            if ((dy2 = rrow(r++)) >= 0) {
                attron(A_DIM);
                snprintf(rbuf, sizeof(rbuf), "%-13s %4s %4s %6s %6s %6s %6s",
                         "MODE", "N", "LOST", "SNRav", "SNRmn", "SNRmx", "BERav");
                rput(dy2, rbuf);
                attroff(A_DIM);
            }
            auto rows = state_.perf_logger->snapshot();
            for (const auto& a : rows) {
                if ((dy2 = rrow(r++)) < 0)
                    break;
                char mode[14];
                snprintf(mode, sizeof(mode), "%s", a.mode.c_str());
                if (a.ber_avg() >= 0)
                    snprintf(rbuf, sizeof(rbuf),
                             "%-13s %4d %4d %5.1f  %5.1f  %5.1f  %5.2f%%",
                             mode, a.frames, a.lost, a.snr_avg(), a.snr_min,
                             a.snr_max, a.ber_avg());
                else
                    snprintf(rbuf, sizeof(rbuf),
                             "%-13s %4d %4d %5.1f  %5.1f  %5.1f      -",
                             mode, a.frames, a.lost, a.snr_avg(), a.snr_min,
                             a.snr_max);
                rput(dy2, rbuf);
            }
            if (rows.empty() && (dy2 = rrow(r++)) >= 0) {
                attron(A_DIM);
                rput(dy2, "(no frames decoded yet)");
                attroff(A_DIM);
            }
            r++;
            if ((dy2 = rrow(r++)) >= 0) {
                attron(A_DIM);
                if (state_.perf_logger->csv_enabled()) {
                    const std::string& p = state_.perf_logger->csv_path();
                    size_t slash = p.find_last_of('/');
                    const char* base = slash == std::string::npos
                        ? p.c_str() : p.c_str() + slash + 1;
                    snprintf(rbuf, sizeof(rbuf), "CSV: ON  %d rows  %s",
                             state_.perf_logger->total(), base);
                } else {
                    snprintf(rbuf, sizeof(rbuf), "CSV: OFF (%d frames tracked)",
                             state_.perf_logger->total());
                }
                rput(dy2, rbuf);
                attroff(A_DIM);
            }
            rn = r;
        }

        int dy = vy(row);
        if (dy >= 0) {
            attron(COLOR_PAIR(4) | A_BOLD);
            mvaddstr(dy, c1, "[ ACTIONS ]");
            attroff(COLOR_PAIR(4) | A_BOLD);
        }
        row++;

        const char* actions[] = {
            "Send Test Pattern",
            "Send Random Data",
            "Send Ping",
            "Clear Stats",
            "Auto Threshold",
            "Compose Message",
            "Auto Send",
            "Perf Log CSV",
            "Reset Perf Stats",
            "Auto Alternate"
        };

        int nslots = utils_visible_slots();
        for (int slot = 0; slot < nslots; slot++) {
            dy = vy(row);
            row++;
            if (dy < 0) continue;
            bool sel = (utils_selection_ == slot);
            if (slot == UTILS_TOP_ACTIONS) {
                if (sel) attron(A_BOLD); else attron(A_DIM);
                mvprintw(dy, c1, "%s[%c] TESTING", sel ? "> " : "  ",
                         state_.utils_testing_open ? '-' : '+');
                if (sel) attroff(A_BOLD); else attroff(A_DIM);
                if (!state_.utils_testing_open) {
                    attron(A_DIM);
                    printw("  send tools and mode roster");
                    attroff(A_DIM);
                }
                continue;
            }
            int i = utils_slot_action(slot);
            const char* label = i >= 10 ? ALT_MODES[i - 10].label : actions[i];
            const char* indent = i >= 10 ? "   " : "";
            if (sel) {
                attron(A_BOLD);
                mvprintw(dy, c1, "> %d. %s%s", i + 1, indent, label);
                attroff(A_BOLD);
            } else {
                attron(A_DIM);
                mvprintw(dy, c1, "  %d. %s%s", i + 1, indent, label);
                attroff(A_DIM);
            }
            if (i >= 10) {
                bool on = (state_.alt_mode_mask >> (i - 10)) & 1;
                if (on) {
                    attron(COLOR_PAIR(1) | A_BOLD);
                    printw("  [x]");
                    attroff(COLOR_PAIR(1) | A_BOLD);
                } else {
                    printw("  [ ]");
                }
                if (auto_alt_enabled_ && alt_index_ == i - 10) {
                    attron(COLOR_PAIR(4) | A_BOLD);
                    printw("  <TX");
                    attroff(COLOR_PAIR(4) | A_BOLD);
                }
            }
            if (i == 9) {
                if (auto_alt_enabled_) {
                    attron(COLOR_PAIR(1) | A_BOLD);
                    printw("  [x]");
                    attroff(COLOR_PAIR(1) | A_BOLD);
                    attron(A_DIM);
                    printw(" cycling %d modes @ 25%% duty", __builtin_popcount(state_.alt_mode_mask));
                    attroff(A_DIM);
                } else {
                    printw("  [ ]");
                    attron(A_DIM);
                    printw(" cycle the checked modes below");
                    attroff(A_DIM);
                }
            }
            if (i == 4 && calibrating_threshold_ && sel) {
                int elapsed = (frame_counter_ - calibration_start_frame_) / 30;
                attron(COLOR_PAIR(4) | A_BOLD);
                printw("  [%ds...]", 3 - elapsed);
                attroff(COLOR_PAIR(4) | A_BOLD);
            }
            if (i == 7 && state_.perf_logger) {
                if (state_.perf_logger->csv_enabled()) {
                    attron(COLOR_PAIR(1) | A_BOLD);
                    printw("  [x]");
                    attroff(COLOR_PAIR(1) | A_BOLD);
                } else {
                    printw("  [ ]");
                }
            }
            if (i == 6) {
                if (auto_send_enabled_) {
                    attron(COLOR_PAIR(1) | A_BOLD);
                    printw("  [x]");
                    attroff(COLOR_PAIR(1) | A_BOLD);
                    attron(A_DIM);
                    if (state_.transmitting.load()) {
                        printw(" lorem %dB, TX...", state_.mtu_bytes);
                    } else {
                        float period = auto_send_period();
                        float wait = period - std::chrono::duration<float>(
                            std::chrono::steady_clock::now() - auto_send_last_).count();
                        if (wait < 0) wait = 0;
                        printw(" lorem %dB, next in %.0fs", state_.mtu_bytes, wait);
                    }
                    attroff(A_DIM);
                } else {
                    if (sel) attron(A_BOLD); else attron(A_DIM);
                    printw("  [ ]");
                    if (sel) attroff(A_BOLD); else attroff(A_DIM);
                    attron(A_DIM);
                    printw(" lorem %dB @ 25%% duty", state_.mtu_bytes);
                    attroff(A_DIM);
                }
            }
        }

        row++;

        dy = vy(row);
        if (dy >= 0) {
            attron(COLOR_PAIR(4) | A_BOLD);
            mvaddstr(dy, c1, "[ TEST INFO ]");
            attroff(COLOR_PAIR(4) | A_BOLD);
        }
        row++;

        dy = vy(row);
        if (dy >= 0) {
            attron(A_DIM);
            mvaddstr(dy, c1, "MTU");
            attroff(A_DIM);
            mvprintw(dy, c1 + 14, "%d bytes", state_.mtu_bytes);
            if (state_.fragmentation_enabled) {
                attron(COLOR_PAIR(4));
                printw(" [FRAG]");
                attroff(COLOR_PAIR(4));
            }
        }
        row++;

        dy = vy(row);
        if (dy >= 0) {
            bool size_selected = (utils_selection_ == 0 || utils_selection_ == 1);
            if (size_selected) {
                attron(A_BOLD | COLOR_PAIR(4));
            } else {
                attron(A_DIM);
            }
            mvaddstr(dy, c1, "Test Size");
            if (size_selected) {
                attroff(A_BOLD | COLOR_PAIR(4));
                mvprintw(dy, c1 + 14, "< %d bytes >", state_.random_data_size);
            } else {
                attroff(A_DIM);
                mvprintw(dy, c1 + 14, "%d bytes", state_.random_data_size);
            }
            if (state_.fragmentation_enabled && state_.random_data_size > state_.mtu_bytes) {
                int data_per_frag = state_.mtu_bytes - 5;  // 5-byte fragment header
                int num_frags = (state_.random_data_size + data_per_frag - 1) / data_per_frag;
                attron(COLOR_PAIR(3));
                printw(" (%d frags)", num_frags);
                attroff(COLOR_PAIR(3));
            }
        }
        row++;

        dy = vy(row);
        if (dy >= 0) {
            attron(A_DIM);
            mvaddstr(dy, c1, "Pattern");
            attroff(A_DIM);
            mvaddstr(dy, c1 + 14, "0x55 (alternating)");
        }
        row++;

        dy = vy(row);
        if (dy >= 0) {
            attron(A_DIM);
            mvaddstr(dy, c1, "Frames Sent");
            attroff(A_DIM);
            mvprintw(dy, c1 + 14, "%d", state_.tx_frame_count.load());
        }
        row++;

        row++;
        state_.unread_messages = 0;
        dy = vy(row);
        if (dy >= 0) {
            attron(COLOR_PAIR(4) | A_BOLD);
            mvaddstr(dy, c1, "[ MESSAGES ]");
            attroff(COLOR_PAIR(4) | A_BOLD);
        }
        row++;
        dy = vy(row);
        if (dy >= 0) {
            attron(A_DIM);
            mvaddstr(dy, c1, "Use this for testing only");
            attroff(A_DIM);
        }
        row++;
        {
            int width = cols / 2 - c1 - 2;
            if (width < 20) width = 20;
            if (msgs.empty()) {
                dy = vy(row);
                row++;
                if (dy >= 0) {
                    attron(A_DIM);
                    mvaddstr(dy, c1, "(no messages)");
                    attroff(A_DIM);
                }
            }
            for (int i = (int)msgs.size() - show; i < (int)msgs.size(); i++) {
                dy = vy(row);
                row++;
                if (dy < 0) continue;
                const auto& m = msgs[i];
                std::string line = m.time + " " + (m.outgoing ? "-> " : "<- ") + m.from + ": " + m.text;
                int pair = m.outgoing ? 2 : 1;
                attron(COLOR_PAIR(pair));
                mvaddnstr(dy, c1, line.c_str(), width);
                attroff(COLOR_PAIR(pair));
            }
        }

        if (rn > 0) rn++;
        int ry = vy(rn++);
        if (ry >= 0) {
            attron(COLOR_PAIR(4) | A_BOLD);
            mvaddnstr(ry, c2, "[ RECENT ACTIVITY ]", rw);
            attroff(COLOR_PAIR(4) | A_BOLD);
        }

        auto packets = state_.get_recent_packets();
        for (int i = 0; i < (int)packets.size(); i++) {
            ry = vy(rn++);
            if (ry < 0) continue;
            const auto& pkt = packets[i];
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now() - pkt.timestamp).count();

            if (pkt.is_tx) {
                attron(COLOR_PAIR(2) | A_BOLD);
                mvaddnstr(ry, c2, "TX", rw);
                attroff(COLOR_PAIR(2) | A_BOLD);
            } else {
                attron(COLOR_PAIR(1) | A_BOLD);
                mvaddnstr(ry, c2, "RX", rw);
                attroff(COLOR_PAIR(1) | A_BOLD);
            }

            char pbuf[64];
            if (rw > 3) {
                snprintf(pbuf, sizeof(pbuf), "%4dB", pkt.size);
                mvaddnstr(ry, c2 + 3, pbuf, rw - 3);
            }

            attron(A_DIM);
            if (rw > 10) {
                if (elapsed < 60)
                    snprintf(pbuf, sizeof(pbuf), "%llds ago", (long long)elapsed);
                else
                    snprintf(pbuf, sizeof(pbuf), "%lldm ago", (long long)(elapsed / 60));
                mvaddnstr(ry, c2 + 10, pbuf, rw - 10);
            }
            attroff(A_DIM);

            if (!pkt.is_tx && pkt.snr > 0 && rw > 20) {
                attron(COLOR_PAIR(4) | A_BOLD);
                snprintf(pbuf, sizeof(pbuf), "%.0fdB", pkt.snr);
                mvaddnstr(ry, c2 + 20, pbuf, rw - 20);
                attroff(COLOR_PAIR(4) | A_BOLD);
            }
        }
        
        if (packets.empty()) {
            attron(A_DIM);
            mvaddstr(ry, c2, "No recent packets");
            attroff(A_DIM);
        }
    }
    
    void handle_utils_action() {
        int action = utils_slot_action(utils_selection_);
        if (action < 0) {
            state_.utils_testing_open = !state_.utils_testing_open;
            return;
        }
        switch (action) {
            case 0: {
                if (state_.on_send_data) {
                    std::vector<uint8_t> data(state_.random_data_size, 0x55);
                    state_.on_send_data(data);
                    state_.add_log("Sent test pattern (" + std::to_string(state_.random_data_size) + " bytes)");
                }
                break;
            }
            case 1: {
                if (state_.on_send_data) {
                    std::vector<uint8_t> data(state_.random_data_size);
                    std::random_device rd;
                    std::mt19937 gen(rd());
                    std::uniform_int_distribution<> dis(0, 255);
                    for (auto& b : data) b = dis(gen);
                    state_.on_send_data(data);
                    state_.add_log("Sent random data (" + std::to_string(state_.random_data_size) + " bytes)");
                }
                break;
            }
            case 2: {
                if (state_.on_send_data) {
                    std::string ping = "PING:" + state_.callsign;
                    std::vector<uint8_t> data(ping.begin(), ping.end());
                    state_.on_send_data(data);
                    state_.add_log("Sent ping");
                }
                break;
            }
            case 3: {
                // Clear stats
                state_.rx_frame_count = 0;
                state_.tx_frame_count = 0;
                state_.rx_error_count = 0;
                state_.sync_count = 0;
                state_.preamble_errors = 0;
                state_.symbol_errors = 0;
                state_.erased_symbols = 0;
                state_.crc_errors = 0;
                state_.stats_reset_requested = true;
                state_.total_tx_time = 0;
                state_.add_log("Stats cleared");
                break;
            }
            case 4: {
                // Auto Threshold 
                if (!calibrating_threshold_) {
                    calibrating_threshold_ = true;
                    calibration_start_frame_ = frame_counter_;
                    calibration_max_level_ = -100.0f;
                    state_.add_log("Calibrating threshold...");
                }
                break;
            }
            case 5: {
                compose_message();
                break;
            }
            case 6: {
                auto_send_enabled_ = !auto_send_enabled_;
                if (auto_send_enabled_) {
                    auto_alt_enabled_ = false;
                    auto_send_last_ = std::chrono::steady_clock::now() -
                        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                            std::chrono::duration<float>(auto_send_period()));
                    state_.add_log("Auto send ON (lorem, 25% duty)");
                } else {
                    state_.add_log("Auto send OFF");
                }
                break;
            }
            case 7: {
                if (state_.perf_logger) {
                    bool on = !state_.perf_logger->csv_enabled();
                    state_.perf_logger->set_csv_enabled(on);
                    state_.add_log(on ? "Perf CSV log ON: " + state_.perf_logger->csv_path()
                                      : "Perf CSV log OFF");
                }
                break;
            }
            case 8: {
                if (state_.perf_logger) {
                    state_.perf_logger->reset();
                    state_.add_log("Perf stats reset");
                }
                break;
            }
            case 9: {
                if (!auto_alt_enabled_ && state_.alt_mode_mask == 0) {
                    state_.add_log("Auto alternate: check some modes below first");
                    break;
                }
                auto_alt_enabled_ = !auto_alt_enabled_;
                if (auto_alt_enabled_) {
                    auto_send_enabled_ = false;
                    auto_send_last_ = std::chrono::steady_clock::now() -
                        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                            std::chrono::duration<float>(auto_send_period()));
                    state_.add_log("Auto alternate ON (" +
                        std::to_string(__builtin_popcount(state_.alt_mode_mask)) +
                        " modes, 25% duty)");
                } else {
                    state_.add_log("Auto alternate OFF");
                }
                break;
            }
            default: {
                if (action >= 10 &&
                    action < 10 + ALT_MODE_COUNT) {
                    int bit = action - 10;
                    state_.alt_mode_mask ^= 1 << bit;
                    state_.save_settings();
                }
                break;
            }
        }
    }

    void apply_alt_mode(int idx) {
        const AltMode& m = ALT_MODES[idx];
        state_.modem_type_index = m.modem_type;
        if (m.modem_type == 0) {
            state_.modulation_index = m.modulation;
            state_.code_rate_index = m.code_rate;
            state_.frame_size = m.frame_size;
        } else if (m.modem_type == 1) {
            state_.mfsk_mode_index = m.mfsk_mode;
        } else {
            state_.robust_mode_index = m.robust_mode;
        }
        state_.update_modem_info();
        if (state_.on_settings_changed)
            state_.on_settings_changed(state_);
    }

    // one frame on air, three frame-times idle: 25% duty cycle keeps the
    // finals safe on rigs rated for 50% digital operation.
    float auto_send_period() const {
        float p = state_.airtime_seconds * 4.0f;
        return p > 4.0f ? p : 4.0f;
    }

    std::vector<uint8_t> lorem_payload(int n) {
        static const char LOREM[] =
            "Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do "
            "eiusmod tempor incididunt ut labore et dolore magna aliqua. Ut "
            "enim ad minim veniam, quis nostrud exercitation ullamco laboris "
            "nisi ut aliquip ex ea commodo consequat. Duis aute irure dolor "
            "in reprehenderit in voluptate velit esse cillum dolore eu "
            "fugiat nulla pariatur. ";
        const int L = (int)sizeof(LOREM) - 1;
        if (n < 1) n = 1;
        int off = std::uniform_int_distribution<int>(0, L - 1)(auto_send_rng_);
        std::vector<uint8_t> out(n);
        for (int i = 0; i < n; i++)
            out[i] = (uint8_t)LOREM[(off + i) % L];
        return out;
    }

    static constexpr int UTILS_TOP_ACTIONS = 6;
    int utils_visible_slots() const {
        return state_.utils_testing_open ? UTILS_ACTION_COUNT + 1
                                         : UTILS_TOP_ACTIONS + 1;
    }
    static int utils_slot_action(int slot) {
        if (slot < UTILS_TOP_ACTIONS) return slot;
        if (slot == UTILS_TOP_ACTIONS) return -1;
        return slot - 1;
    }

    void utils_ensure_visible() {
        int rows, cols;
        getmaxyx(stdscr, rows, cols);
        (void)cols;
        int vis = rows - 6;
        if (vis < 1) vis = 1;
        int sel_row = 1 + utils_selection_;
        if (sel_row < utils_scroll_)
            utils_scroll_ = sel_row;
        else if (sel_row >= utils_scroll_ + vis)
            utils_scroll_ = sel_row - vis + 1;
    }

    void tick_level_history() {
        if (!state_.on_get_audio_level) return;
        int64_t now = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        if (last_level_ms_ != 0 && now - last_level_ms_ < LEVEL_SAMPLE_MS)
            return;
        last_level_ms_ = now;
        float db = state_.on_get_audio_level();
        int64_t sig = state_.wf_sig_ms.load();
        state_.update_level(db, state_.dcd_active.load(),
                            sig != 0 && now - sig < 450);
    }

    void tick_auto_send() {
        if ((!auto_send_enabled_ && !auto_alt_enabled_) || !state_.on_send_data)
            return;
        if (state_.transmitting.load() || state_.ptt_on.load() ||
            state_.tx_queue_size.load() > 0) return;
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration<float>(now - auto_send_last_).count() < auto_send_period())
            return;
        auto_send_last_ = now;
        unsigned seq;
        if (auto_alt_enabled_) {
            if (state_.alt_mode_mask == 0)
                return;
            do {
                alt_index_ = (alt_index_ + 1) % ALT_MODE_COUNT;
            } while (!((state_.alt_mode_mask >> alt_index_) & 1));
            apply_alt_mode(alt_index_);
            seq = alt_seqs_[alt_index_]++ % 100000;
        } else {
            seq = auto_send_seq_++ % 100000;
        }
        auto payload = lorem_payload(state_.mtu_bytes);
        char hdr[16];
        int hn = snprintf(hdr, sizeof(hdr), "SEQ:%05u:", seq);
        for (int i = 0; i < hn && i < (int)payload.size(); i++)
            payload[i] = (uint8_t)hdr[i];
        state_.on_send_data(payload);
    }


    std::string rig_cmd(const std::string& cmd) {
        if (!state_.on_rigctl_command) return "";
        return state_.on_rigctl_command(cmd);
    }

    static std::string format_freq(long long hz) {
        if (hz <= 0) return "---";
        char buf[32];
        if (hz >= 1000000) {
            snprintf(buf, sizeof(buf), "%lld.%03lld.%03lld", hz / 1000000, (hz / 1000) % 1000, hz % 1000);
        } else {
            snprintf(buf, sizeof(buf), "%lld.%03lld", hz / 1000, hz % 1000);
        }
        return buf;
    }

    void set_rig_freq(long long hz) {
        char cmd[48];
        snprintf(cmd, sizeof(cmd), "+F %lld", hz);
        if (TNCUIState::rig_ok(rig_cmd(cmd))) {
            state_.rig_freq_hz = hz;
            state_.add_log("Rig: freq set to " + format_freq(hz));
        } else {
            state_.add_log("Rig: set freq failed");
        }
        state_.rig_refresh_requested = true;
    }

    void adjust_rig_field(int delta) {
        switch (rig_field_) {
            case RIG_FIELD_FREQ: {
                long long f = state_.rig_freq_hz.load();
                if (f <= 0) {
                    state_.add_log("Rig: no frequency data yet");
                    break;
                }
                long long nf = f + delta * RIG_STEP_HZ[rig_step_index_];
                if (nf < 0) nf = 0;
                set_rig_freq(nf);
                break;
            }
            case RIG_FIELD_PRESET: {
                int n = (int)state_.freq_presets.size();
                if (n > 0)
                    rig_preset_index_ = (rig_preset_index_ + delta % n + n) % n;
                break;
            }
            case RIG_FIELD_STEP:
                rig_step_index_ = (rig_step_index_ + delta + RIG_STEP_COUNT) % RIG_STEP_COUNT;
                break;
            case RIG_FIELD_MODE: {
                std::string cur = state_.get_rig_mode();
                int n = (int)RIG_MODE_OPTIONS.size();
                int idx = -1;
                for (int i = 0; i < n; i++) {
                    if (RIG_MODE_OPTIONS[i] == cur) { idx = i; break; }
                }
                idx = idx < 0 ? (delta > 0 ? 0 : n - 1) : (idx + delta + n) % n;
                const std::string& m = RIG_MODE_OPTIONS[idx];
                if (TNCUIState::rig_ok(rig_cmd("+M " + m + " 0"))) {
                    state_.set_rig_mode_cache(m);
                    state_.add_log("Rig: mode set to " + m);
                } else {
                    state_.add_log("Rig: set mode " + m + " failed");
                }
                state_.rig_refresh_requested = true;
                break;
            }
            case RIG_FIELD_POWER: {
                float p = state_.rig_power_level.load();
                if (p < 0) {
                    state_.add_log("Rig: no power data yet");
                    break;
                }
                p = std::max(0.0f, std::min(1.0f, p + delta * 0.05f));
                char cmd[48];
                snprintf(cmd, sizeof(cmd), "+L RFPOWER %.2f", p);
                if (TNCUIState::rig_ok(rig_cmd(cmd))) {
                    state_.rig_power_level = p;
                } else {
                    state_.add_log("Rig: set power failed");
                }
                state_.rig_refresh_requested = true;
                break;
            }
            case RIG_FIELD_DRIVE: {
                float d = state_.tx_drive.load();
                d = std::max(0.05f, std::min(1.0f, d + delta * 0.05f));
                state_.tx_drive = d;
                if (state_.on_settings_changed)
                    state_.on_settings_changed(state_);
                state_.save_settings();
                break;
            }
            case RIG_FIELD_TUNER: {
                int nv = state_.rig_tuner_on.load() == 1 ? 0 : 1;
                char cmd[32];
                snprintf(cmd, sizeof(cmd), "+U TUNER %d", nv);
                if (TNCUIState::rig_ok(rig_cmd(cmd))) {
                    state_.rig_tuner_on = nv;
                    state_.add_log(std::string("Rig: tuner ") + (nv ? "ON" : "OFF"));
                } else {
                    state_.add_log("Rig: tuner toggle failed");
                }
                state_.rig_refresh_requested = true;
                break;
            }
        }
    }

    void rig_enter_action() {
        if (rig_field_ == RIG_FIELD_FREQ) {
            edit_rig_freq();
        } else if (rig_field_ == RIG_FIELD_PRESET) {
            if (state_.freq_presets.empty()) {
                state_.add_log("Rig: no frequency presets, press s to save one");
            } else if (rig_preset_index_ < (int)state_.freq_presets.size()) {
                const auto& fp = state_.freq_presets[rig_preset_index_];
                set_rig_freq(fp.hz);
                state_.add_log("Rig: " + fp.label + " " + format_freq(fp.hz));
            }
        } else if (rig_field_ == RIG_FIELD_DRIVE) {
            if (!state_.on_alc_tune) {
                state_.add_log("ALC tune: unavailable");
            } else if (!state_.alc_tune_running.exchange(true)) {
                state_.add_log("ALC tune: starting...");
                std::thread([s = &state_] {
                    float r = s->on_alc_tune();
                    if (!g_running.load())
                        return;
                    if (r > 0) {
                        s->tx_drive = r;
                        s->save_settings();
                        char b[48];
                        snprintf(b, sizeof(b), "ALC tune: drive set to %d%%",
                                 (int)lround(r * 100));
                        s->add_log(b);
                    } else {
                        s->add_log("ALC tune: failed");
                    }
                    s->alc_tune_running = false;
                }).detach();
            }
        } else if (rig_field_ == RIG_FIELD_TUNER) {
            adjust_rig_field(1);
        } else if (rig_field_ == RIG_FIELD_TUNE) {
            state_.add_log("Rig: starting tune cycle...");
            if (TNCUIState::rig_ok(rig_cmd("+G TUNE"))) {
                state_.add_log("Rig: tune cycle started");
            } else {
                state_.add_log("Rig: tune failed (not supported?)");
            }
            state_.rig_refresh_requested = true;
        }
    }

    void save_freq_preset_dialog() {
        if ((int)state_.freq_presets.size() >= TNCUIState::MAX_FREQ_PRESETS) {
            state_.add_log("Rig: maximum frequency presets reached");
            return;
        }
        long long hz = state_.rig_freq_hz.load();
        if (hz <= 0) {
            state_.add_log("Rig: no frequency read from rig yet");
            return;
        }

        int rows, cols;
        getmaxyx(stdscr, rows, cols);
        int dialog_w = 44, dialog_h = 5;
        int dx = (cols - dialog_w) / 2, dy = (rows - dialog_h) / 2;

        attron(COLOR_PAIR(4));
        for (int i = dy; i < dy + dialog_h && i < rows; i++)
            mvhline(i, dx, ' ', dialog_w);
        draw_box(dy, dx, dialog_h, dialog_w);
        attron(A_BOLD);
        mvaddstr(dy, dx + 3, " SAVE PRESET ");
        attroff(A_BOLD);
        attroff(COLOR_PAIR(4));
        attron(A_DIM);
        mvprintw(dy + 1, dx + 2, "%s  label:", format_freq(hz).c_str());
        attroff(A_DIM);

        char buf[21] = {0};
        if (!prompt_input(dy + 2, dx + 2, buf, 20))
            return;
        std::string label = buf;
        while (!label.empty() && label.back() == ' ') label.pop_back();
        if (label.empty()) {
            state_.add_log("Rig: preset not saved, label was empty");
            return;
        }

        state_.freq_presets.push_back({hz, label});
        rig_preset_index_ = (int)state_.freq_presets.size() - 1;
        state_.save_settings();
        state_.add_log("Rig: preset saved, " + label + " " + format_freq(hz));
    }

    void delete_selected_freq_preset() {
        if (state_.freq_presets.empty()) {
            state_.add_log("Rig: no frequency presets to delete");
            return;
        }
        if (rig_preset_index_ >= (int)state_.freq_presets.size())
            rig_preset_index_ = (int)state_.freq_presets.size() - 1;
        std::string label = state_.freq_presets[rig_preset_index_].label;
        state_.freq_presets.erase(state_.freq_presets.begin() + rig_preset_index_);
        if (rig_preset_index_ >= (int)state_.freq_presets.size())
            rig_preset_index_ = std::max(0, (int)state_.freq_presets.size() - 1);
        state_.save_settings();
        state_.add_log("Rig: preset deleted, " + label);
    }

    void edit_rig_freq() {
        int rows, cols;
        getmaxyx(stdscr, rows, cols);

        int dialog_w = 44;
        int dialog_h = 5;
        int dialog_x = (cols - dialog_w) / 2;
        int dialog_y = (rows - dialog_h) / 2;

        for (int y = dialog_y; y < dialog_y + dialog_h; y++) {
            move(y, dialog_x);
            for (int x = 0; x < dialog_w; x++) addch(' ');
        }

        attron(A_BOLD);
        draw_box(dialog_y, dialog_x, dialog_h, dialog_w);
        attroff(A_BOLD);
        mvaddstr(dialog_y, dialog_x + 2, " Set Frequency ");
        attron(A_DIM);
        mvaddstr(dialog_y + 3, dialog_x + 2, "e.g. 7074 (kHz) or 14.074 (MHz)");
        attroff(A_DIM);
        mvaddstr(dialog_y + 2, dialog_x + 2, "Freq: ");

        char buf[24] = {0};
        if (!prompt_input(dialog_y + 2, dialog_x + 8, buf, 15)) return;

        if (strlen(buf) == 0) return;

        long long hz;
        if (strchr(buf, '.')) {
            hz = (long long)llround(atof(buf) * 1e6);
        } else {
            long long v = atoll(buf);
            hz = v < 100000 ? v * 1000 : v;
        }
        if (hz < 10000 || hz > 470000000LL) {
            state_.add_log("Rig: invalid frequency");
            return;
        }
        set_rig_freq(hz);
    }

    void draw_rig_meter_bar(float value, float minv, float maxv, int width, int color_pair) {
        attron(A_DIM);
        addch('[');
        attroff(A_DIM);

        if (std::isnan(value)) {
            attron(A_DIM);
            for (int i = 0; i < width; i++) addch('-');
            attroff(A_DIM);
        } else {
            float frac = (value - minv) / (maxv - minv);
            frac = std::max(0.0f, std::min(1.0f, frac));
            int filled = (int)(frac * width + 0.5f);
            attron(COLOR_PAIR(color_pair) | A_BOLD);
            for (int i = 0; i < filled; i++) addch('=');
            attroff(COLOR_PAIR(color_pair) | A_BOLD);
            for (int i = filled; i < width; i++) addch(' ');
        }

        attron(A_DIM);
        addch(']');
        attroff(A_DIM);
    }

    void draw_rig(int y, int h, int cols) {
        (void)h;
        int c1 = 3;
        int c2 = 16;
        int c3 = cols / 2 + 2;
        int top = y;

        attron(COLOR_PAIR(4) | A_BOLD);
        mvaddstr(y, c1, "[ RIG CONTROL ]");
        attroff(COLOR_PAIR(4) | A_BOLD);
        attron(A_DIM);
        if (state_.ptt_type_index == 1)
            printw("  rigctld %s:%d", state_.rigctl_host.c_str(), state_.rigctl_port);
        else
            printw("  hamlib %s", hamlib_model_label(state_.hamlib_model).c_str());
        attroff(A_DIM);
        if (state_.rigctl_connected.load()) {
            attron(COLOR_PAIR(1) | A_BOLD);
            addstr("  OK");
            attroff(COLOR_PAIR(1) | A_BOLD);
        } else {
            attron(COLOR_PAIR(2) | A_BOLD);
            addstr("  --");
            attroff(COLOR_PAIR(2) | A_BOLD);
        }
        y += 2;

        attron(A_DIM);
        mvaddstr(y, c1, "FREQUENCY");
        attroff(A_DIM);
        y++;

        draw_field_ex(y, c1, c2, "Freq", rig_field_ == RIG_FIELD_FREQ,
                      format_freq(state_.rig_freq_hz.load()), true);
        y++;

        draw_selector_field_ex(y, c1, c2, "Step", rig_field_ == RIG_FIELD_STEP,
                               RIG_STEP_LABELS[rig_step_index_]);
        y++;

        {
            char pbuf[80];
            if (state_.freq_presets.empty()) {
                snprintf(pbuf, sizeof(pbuf), "(none)  s=save current");
            } else {
                if (rig_preset_index_ >= (int)state_.freq_presets.size())
                    rig_preset_index_ = 0;
                const auto& fp = state_.freq_presets[rig_preset_index_];
                snprintf(pbuf, sizeof(pbuf), "%s  %s  (%d/%d)",
                         format_freq(fp.hz).c_str(), fp.label.c_str(),
                         rig_preset_index_ + 1, (int)state_.freq_presets.size());
            }
            draw_selector_field_ex(y, c1, c2, "Preset",
                                   rig_field_ == RIG_FIELD_PRESET, pbuf);
            y++;
        }

        std::string mode = state_.get_rig_mode();
        draw_selector_field_ex(y, c1, c2, "Mode", rig_field_ == RIG_FIELD_MODE,
                               mode.empty() ? "---" : mode);
        y++;

        float pwr = state_.rig_power_level.load();
        char pwr_buf[16];
        if (pwr >= 0) {
            snprintf(pwr_buf, sizeof(pwr_buf), "%d%%", (int)lround(pwr * 100));
        } else {
            snprintf(pwr_buf, sizeof(pwr_buf), "---");
        }
        draw_selector_field_ex(y, c1, c2, "RF Power", rig_field_ == RIG_FIELD_POWER, pwr_buf);
        y++;

        char drv_buf[32];
        if (state_.alc_tune_running.load())
            snprintf(drv_buf, sizeof(drv_buf), "TUNING...");
        else
            snprintf(drv_buf, sizeof(drv_buf), "%d%%  (Enter=ALC tune)",
                     (int)lround(state_.tx_drive.load() * 100));
        draw_selector_field_ex(y, c1, c2, "TX Drive", rig_field_ == RIG_FIELD_DRIVE, drv_buf);
        y += 2;

        attron(A_DIM);
        mvaddstr(y, c1, "ANTENNA TUNER");
        attroff(A_DIM);
        if (state_.rig_tuner_supported.load() == 0) {
            attron(A_DIM);
            mvaddstr(y + 1, c1, "(not supported by rig)");
            attroff(A_DIM);
            y += 3;
        } else {
            y++;

            int tuner = state_.rig_tuner_on.load();
            draw_selector_field_ex(y, c1, c2, "Tuner", rig_field_ == RIG_FIELD_TUNER,
                                   tuner < 0 ? "---" : (tuner ? "ON" : "OFF"));
            y++;

            bool sel = (rig_field_ == RIG_FIELD_TUNE);
            if (sel) {
                attron(A_BOLD);
                mvaddch(y, c1 - 2, '>');
                mvaddstr(y, c1, "Tune");
                attroff(A_BOLD);
                attron(COLOR_PAIR(4) | A_BOLD);
                mvaddstr(y, c2, "[ START TUNE ]");
                attroff(COLOR_PAIR(4) | A_BOLD);
                attron(A_DIM);
                addstr("  [enter]");
                attroff(A_DIM);
            } else {
                attron(A_DIM);
                mvaddstr(y, c1, "Tune");
                mvaddstr(y, c2, "[ START TUNE ]");
                attroff(A_DIM);
            }
            y += 2;
        }

        attron(A_DIM);
        mvaddstr(y, c1, "Freq: Enter=type, </> step   Preset: Enter=go, s=save, x=del");
        attroff(A_DIM);

        int ry = top;
        attron(COLOR_PAIR(4) | A_BOLD);
        mvaddstr(ry, c3, "[ METERS ]");
        attroff(COLOR_PAIR(4) | A_BOLD);

        int64_t last = state_.rig_last_update_ms.load();
        if (last > 0) {
            int64_t now = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
            attron(A_DIM);
            printw("  %.0fs ago", (now - last) / 1000.0f);
            attroff(A_DIM);
        }
        ry += 2;

        int bar_w = std::min(24, cols - c3 - 22);
        if (bar_w < 10) bar_w = 10;

        for (int i = 0; i < RIG_METER_COUNT; i++) {
            float v = state_.rig_meter_values[i].load();
            bool is_temp = strcmp(RIG_METERS[i].level, "TEMP_METER") == 0;
            if (is_temp && std::isnan(v))
                continue;

            attron(A_DIM);
            mvaddstr(ry, c3, RIG_METERS[i].label);
            attroff(A_DIM);
            move(ry, c3 + 9);

            int pair = 1;
            if (strcmp(RIG_METERS[i].level, "SWR") == 0 && !std::isnan(v)) {
                pair = v < 1.5f ? 1 : v < SWR_WARN_THRESHOLD ? 3 : 2;
            } else if (is_temp) {
                pair = v < 60.0f ? 1 : v < 80.0f ? 3 : 2;
            }
            draw_rig_meter_bar(v, RIG_METERS[i].min, RIG_METERS[i].max, bar_w, pair);

            if (std::isnan(v)) {
                attron(A_DIM);
                addstr(" ---");
                attroff(A_DIM);
            } else if (strcmp(RIG_METERS[i].level, "STRENGTH") == 0) {
                // strength is db relative to s9, ~6 db per s-unit
                if (v <= 0) {
                    float s = std::max(0.0f, 9.0f + v / 6.0f);
                    printw(" S%.0f", s);
                } else {
                    printw(" S9+%.0f", v);
                }
            } else if (strcmp(RIG_METERS[i].level, "SWR") == 0) {
                if (pair != 1) attron(COLOR_PAIR(pair) | A_BOLD);
                printw(" %.1f", v);
                if (pair != 1) attroff(COLOR_PAIR(pair) | A_BOLD);
            } else if (is_temp) {
                if (pair != 1) attron(COLOR_PAIR(pair) | A_BOLD);
                printw(" %.0fC", v);
                if (pair != 1) attroff(COLOR_PAIR(pair) | A_BOLD);
            } else if (strstr(RIG_METERS[i].level, "WATTS") != nullptr) {
                printw(" %.0fW", v);
            } else {
                printw(" %.2f", v);
            }
            ry++;
        }
        ry++;

        mvaddstr(ry, c3, "PTT ");
        if (state_.ptt_on.load()) {
            attron(COLOR_PAIR(2) | A_BOLD);
            addstr("TX");
            attroff(COLOR_PAIR(2) | A_BOLD);
        } else {
            attron(COLOR_PAIR(1) | A_BOLD);
            addstr("RX");
            attroff(COLOR_PAIR(1) | A_BOLD);
        }
        ry++;

        attron(A_DIM);
        mvaddstr(ry, c3, "SWR/Power/ALC valid during TX");
        attroff(A_DIM);

        float swr_warn = state_.swr_warn_value.load();
        if (swr_warn > 0.0f) {
            ry += 2;
            attron(COLOR_PAIR(2) | A_BOLD);
            mvprintw(ry, c3, "(!) HIGH SWR %.1f on last TX - check antenna",
                     swr_warn);
            attroff(COLOR_PAIR(2) | A_BOLD);
            if (state_.rig_tuner_supported.load() == 1 &&
                state_.rig_tuner_on.load() == 0) {
                ry++;
                attron(COLOR_PAIR(3));
                mvaddstr(ry, c3, "    (tuner is off)");
                attroff(COLOR_PAIR(3));
            }
        }

        if (!state_.rig_data_valid.load()) {
            ry += 2;
            attron(COLOR_PAIR(3));
            mvaddstr(ry, c3, "No data from rigctld yet...");
            attroff(COLOR_PAIR(3));
        }
    }

    void update_calibration() {
        if (!calibrating_threshold_) return;
        
        // sample current level
        float level = state_.carrier_level_db.load();
        if (level > calibration_max_level_) {
            calibration_max_level_ = level;
        }
        
        int elapsed_frames = frame_counter_ - calibration_start_frame_;
        if (elapsed_frames >= 90) {
            calibrating_threshold_ = false;
            
            // threshold is max + 6dB margin
            float new_threshold = calibration_max_level_ + 6.0f;
            new_threshold = std::max(-80.0f, std::min(0.0f, new_threshold));
            
            state_.carrier_threshold_db = new_threshold;
            apply_settings();
            
            char msg[64];
            snprintf(msg, sizeof(msg), "Threshold set to %.0f dB (noise: %.0f dB)", 
                     new_threshold, calibration_max_level_);
            state_.add_log(msg);
        }
    }
    
    void draw_csma_help(int rows, int cols) {
        int w = 58, h = 19;
        int x0 = (cols - w) / 2, y0 = (rows - h) / 2;
        attron(COLOR_PAIR(4));
        for (int y = y0; y < y0 + h && y < rows; y++)
            mvhline(y, x0, ' ', w);
        mvhline(y0, x0, ACS_HLINE, w);
        mvhline(y0 + h - 1, x0, ACS_HLINE, w);
        mvvline(y0, x0, ACS_VLINE, h);
        mvvline(y0, x0 + w - 1, ACS_VLINE, h);
        mvaddch(y0, x0, ACS_ULCORNER);
        mvaddch(y0, x0 + w - 1, ACS_URCORNER);
        mvaddch(y0 + h - 1, x0, ACS_LLCORNER);
        mvaddch(y0 + h - 1, x0 + w - 1, ACS_LRCORNER);
        attron(A_BOLD);
        mvaddstr(y0, x0 + 3, " CSMA ");
        attroff(A_BOLD);
        attroff(COLOR_PAIR(4));
        int y = y0 + 2, lx = x0 + 2, rx = x0 + 14;
        mvaddstr(y++, lx, "Listens before transmitting so stations");
        mvaddstr(y++, lx, "do not talk over each other.");
        y++;
        auto item = [&](const char* k, const char* v) {
            attron(A_BOLD); mvaddstr(y, lx, k); attroff(A_BOLD);
            mvaddstr(y, rx, v); y++;
        };
        item("Enabled",   "turn channel checking on or off");
        item("Threshold", "level above this counts as busy");
        item("Level",     "what the channel measures right now");
        item("Band",      "HF or VHF/UHF timing for presets");
        item("Preset",    "quick setups by how busy the band is");
        item("Quiet",     "idle time required before contending");
        item("Window",    "random wait range drawn after quiet");
        item("Lead Tone", "keyup tone so others hear us sooner");
        item("Dither",    "callsign delay that separates replies");
        item("Burst",     "packets sent per channel win");
        item("FastFloor", "shorter sync waits, version 2.3>");
        y++;
        attron(A_DIM);
        mvaddstr(y, lx, "any key to close");
        attroff(A_DIM);
    }

    void draw_sync_only_help(int rows, int cols) {
        int w = 60, h = 24;
        int x0 = (cols - w) / 2, y0 = (rows - h) / 2;
        if (x0 < 0) x0 = 0;
        if (y0 < 0) y0 = 0;
        attron(COLOR_PAIR(4));
        for (int y = y0; y < y0 + h && y < rows; y++)
            mvhline(y, x0, ' ', w);
        mvhline(y0, x0, ACS_HLINE, w);
        mvhline(y0 + h - 1, x0, ACS_HLINE, w);
        mvvline(y0, x0, ACS_VLINE, h);
        mvvline(y0, x0 + w - 1, ACS_VLINE, h);
        mvaddch(y0, x0, ACS_ULCORNER);
        mvaddch(y0, x0 + w - 1, ACS_URCORNER);
        mvaddch(y0 + h - 1, x0, ACS_LLCORNER);
        mvaddch(y0 + h - 1, x0 + w - 1, ACS_LRCORNER);
        attron(A_BOLD);
        mvaddstr(y0, x0 + 3, " WHICH MODE ");
        attroff(A_BOLD);
        attroff(COLOR_PAIR(4));

        int y = y0 + 2, lx = x0 + 2;
        attron(A_BOLD);
        mvaddstr(y++, lx, "Can every station hear every other one?");
        attroff(A_BOLD);
        attron(A_DIM);
        mvaddstr(y++, lx, "local net, one NVIS region, VHF simplex, repeater");
        attroff(A_DIM);
        y++;
        attron(COLOR_PAIR(2) | A_BOLD); mvaddstr(y, lx + 1, "NO"); attroff(COLOR_PAIR(2) | A_BOLD);
        mvaddstr(y++, lx + 6, "Use THRESHOLD or SYNC.");
        attron(A_DIM);
        mvaddstr(y++, lx + 6, "RANKED makes hidden stations collide more.");
        attroff(A_DIM);
        y++;
        attron(COLOR_PAIR(1) | A_BOLD); mvaddstr(y, lx + 1, "YES"); attroff(COLOR_PAIR(1) | A_BOLD);
        attron(A_BOLD);
        mvaddstr(y++, lx + 6, "Are you on HF / Is the channel noisy?");
        attroff(A_BOLD);
        attron(A_DIM);
        mvaddstr(y++, lx + 6, "static, QRM, signals that are not modem73");
        attroff(A_DIM);
        y++;
        attron(COLOR_PAIR(2) | A_BOLD); mvaddstr(y, lx + 8, "NO"); attroff(COLOR_PAIR(2) | A_BOLD);
        mvaddstr(y++, lx + 13, "THRESHOLD. Good for quiet VHF/UHF+.");
        y++;
        attron(COLOR_PAIR(1) | A_BOLD); mvaddstr(y, lx + 8, "YES"); attroff(COLOR_PAIR(1) | A_BOLD);
        attron(A_BOLD);
        mvaddstr(y++, lx + 13, "Do all stations run 2.3 or");
        mvaddstr(y++, lx + 13, "newer with RANKED on?");
        attroff(A_BOLD);
        y++;
        attron(COLOR_PAIR(2) | A_BOLD); mvaddstr(y, lx + 15, "NO"); attroff(COLOR_PAIR(2) | A_BOLD);
        mvaddstr(y++, lx + 20, "SYNC.");
        attron(COLOR_PAIR(1) | A_BOLD); mvaddstr(y, lx + 15, "YES"); attroff(COLOR_PAIR(1) | A_BOLD);
        mvaddstr(y++, lx + 20, "RANKED. 3 to 7x faster.");
        y++;
        attron(A_DIM);
        mvaddstr(y++, lx, "RANKED turns on the Lead Tone and Presence Tone");
        mvaddstr(y++, lx, "it needs. Every station must run it.");
        mvaddstr(y, lx, "any key to close");
        attroff(A_DIM);
    }

    void draw_help(int rows, int cols) {
        int help_w = 48;
        int help_h = 19;
        int start_x = (cols - help_w) / 2;
        int start_y = (rows - help_h) / 2;
        
        attron(COLOR_PAIR(4));
        for (int y = start_y; y < start_y + help_h && y < rows; y++) {
            mvhline(y, start_x, ' ', help_w);
        }
        
        mvhline(start_y, start_x, ACS_HLINE, help_w);
        mvhline(start_y + help_h - 1, start_x, ACS_HLINE, help_w);
        mvvline(start_y, start_x, ACS_VLINE, help_h);
        mvvline(start_y, start_x + help_w - 1, ACS_VLINE, help_h);
        mvaddch(start_y, start_x, ACS_ULCORNER);
        mvaddch(start_y, start_x + help_w - 1, ACS_URCORNER);
        mvaddch(start_y + help_h - 1, start_x, ACS_LLCORNER);
        mvaddch(start_y + help_h - 1, start_x + help_w - 1, ACS_LRCORNER);
        
        attron(A_BOLD);
        mvaddstr(start_y, start_x + 3, " MODEM73 HELP ");
        attroff(A_BOLD);
        attroff(COLOR_PAIR(4));
        
        int y = start_y + 2;
        int lx = start_x + 2;
        int rx = start_x + 22;
        
        attron(A_BOLD);
        mvaddstr(y, lx, "Navigation");
        attroff(A_BOLD);
        y++;
        mvaddstr(y, lx, "Tab / Shift-Tab");
        mvaddstr(y, rx, "Switch tabs");
        y++;
        mvaddstr(y, lx, "Up / Down");
        mvaddstr(y, rx, "Navigate fields");
        y++;
        mvaddstr(y, lx, "Left / Right");
        mvaddstr(y, rx, "Adjust values");
        y++;
        mvaddstr(y, lx, "Enter");
        mvaddstr(y, rx, "Edit / activate");
        y += 2;
        
        attron(A_BOLD);
        mvaddstr(y, lx, "Config");
        attroff(A_BOLD);
        y++;
        mvaddstr(y, lx, "s");
        mvaddstr(y, rx, "Save preset");
        y++;
        mvaddstr(y, lx, "x");
        mvaddstr(y, rx, "Delete preset");
        y += 2;
        
        attron(A_BOLD);
        mvaddstr(y, lx, "General");
        attroff(A_BOLD);
        y++;
        mvaddstr(y, lx, "Esc");
        mvaddstr(y, rx, "Cancel text input");
        y++;
        mvaddstr(y, lx, "F1");
        mvaddstr(y, rx, "Toggle this help");
        y++;
        mvaddstr(y, lx, "Q");
        mvaddstr(y, rx, "Quit");
        
        attron(A_DIM);
        mvaddstr(start_y + help_h - 2, start_x + (help_w - 24) / 2, "Press any key to close");
        attroff(A_DIM);
    }
    
    TNCUIState& state_;
    bool initialized_ = false;
    std::atomic<bool> running_{false};
    int current_tab_ = 0;
    static constexpr int LEVEL_SAMPLE_MS = 100;
    int64_t last_level_ms_ = 0;
    int current_field_ = 0;
    int rx_off_dialog_field_ = -1;
    bool lan_dialog_ = false;

    bool* rx_dialog_flag() {
        switch (rx_off_dialog_field_) {
            case FIELD_RX_OFDM: return &state_.ofdm_rx_enabled;
            case FIELD_RX_ROBUST: return &state_.robust_rx_enabled;
            case FIELD_RX_MFSK: return &state_.mfsk_rx_enabled;
        }
        return nullptr;
    }

    const char* rx_dialog_name() {
        switch (rx_off_dialog_field_) {
            case FIELD_RX_OFDM: return "OFDM";
            case FIELD_RX_ROBUST: return "ROBUST";
            case FIELD_RX_MFSK: return "MFSK";
        }
        return "";
    }

    void draw_rx_forced(int dy, int c2) {
        attron(COLOR_PAIR(4));
        mvaddstr(dy, c2 + 10, "still on: current TX mode");
        attroff(COLOR_PAIR(4));
    }

    int rx_field_modem(int field) {
        switch (field) {
            case FIELD_RX_OFDM: return 0;
            case FIELD_RX_MFSK: return 1;
            case FIELD_RX_ROBUST: return 2;
        }
        return -1;
    }

    static std::string local_ip() {
        std::string ip = "unknown";
        struct ifaddrs* ifs = nullptr;
        if (getifaddrs(&ifs) != 0) return ip;
        for (struct ifaddrs* a = ifs; a; a = a->ifa_next) {
            if (!a->ifa_addr || a->ifa_addr->sa_family != AF_INET) continue;
            if (a->ifa_flags & IFF_LOOPBACK) continue;
            char buf[INET_ADDRSTRLEN];
            auto* sin = reinterpret_cast<struct sockaddr_in*>(a->ifa_addr);
            if (inet_ntop(AF_INET, &sin->sin_addr, buf, sizeof(buf))) {
                ip = buf;
                break;
            }
        }
        freeifaddrs(ifs);
        return ip;
    }

    void draw_lan_dialog() {
        int rows, cols;
        getmaxyx(stdscr, rows, cols);
        int w = 56, h = 10;
        int y = (rows - h) / 2, x = (cols - w) / 2;
        if (y < 0 || x < 0) return;
        for (int i = 0; i < h; i++) {
            move(y + i, x);
            for (int j = 0; j < w; j++) addch(' ');
        }
        draw_box(y, x, h, w);
        attron(COLOR_PAIR(3) | A_BOLD);
        mvaddstr(y + 1, x + 2, "(!) ENABLE LAN MODE?");
        attroff(COLOR_PAIR(3) | A_BOLD);
        mvaddstr(y + 3, x + 2, "KISS and control ports will listen on 0.0.0.0,");
        mvaddstr(y + 4, x + 2, "reachable by any device on your network:");
        attron(COLOR_PAIR(4) | A_BOLD);
        char line[64];
        std::string ip = local_ip();
        snprintf(line, sizeof(line), "KISS %s:%d   control %s:%d",
                 ip.c_str(), state_.port, ip.c_str(), state_.control_port);
        mvaddnstr(y + 6, x + 2, line, w - 4);
        attroff(COLOR_PAIR(4) | A_BOLD);
        attron(A_BOLD);
        mvaddstr(y + h - 2, x + 2, "[Y] Enable        [N] Cancel");
        attroff(A_BOLD);
    }

    void draw_rx_off_dialog() {
        int rows, cols;
        getmaxyx(stdscr, rows, cols);
        bool active = rx_field_modem(rx_off_dialog_field_) == state_.modem_type_index;
        int w = 56, h = active ? 11 : 8;
        int y = (rows - h) / 2, x = (cols - w) / 2;
        if (y < 0 || x < 0) return;
        for (int i = 0; i < h; i++) {
            move(y + i, x);
            for (int j = 0; j < w; j++) addch(' ');
        }
        draw_box(y, x, h, w);
        std::string name = rx_dialog_name();
        attron(COLOR_PAIR(3) | A_BOLD);
        mvaddstr(y + 1, x + 2, ("(!) DISABLE " + name + " RX DECODING?").c_str());
        attroff(COLOR_PAIR(3) | A_BOLD);
        mvaddstr(y + 3, x + 2, "This saves CPU performance, but disables ALL");
        mvaddstr(y + 4, x + 2, ("reception of " + name + " frames until re-enabled.").c_str());
        if (active) {
            attron(COLOR_PAIR(4));
            mvaddstr(y + 6, x + 2, ("NOTE: " + name + " is the current TX modem type, so").c_str());
            mvaddstr(y + 7, x + 2, "its decoder keeps running until you switch modes.");
            attroff(COLOR_PAIR(4));
        }
        attron(A_BOLD);
        mvaddstr(y + h - 2, x + 2, "[Y] Disable        [N] Cancel");
        attroff(A_BOLD);
    }

    void toggle_rx_decoder(bool& flag, const char* name) {
        if (!flag) {
            flag = true;
            apply_settings();
            state_.add_log(std::string(name) + " RX decoding enabled");
        } else {
            rx_off_dialog_field_ = current_field_;
        }
    }
    static constexpr int WF_FFT = 256;
    static constexpr int WF_BINS = WF_FFT / 2;  
    static constexpr int WF_MAX_ROWS = 64;
    static constexpr int WF_ROW_MS = 100;
    static constexpr int WF_PAIR_BASE = 20;
    int wf_pairs_ = 0;
    bool wf_sig_ok_ = false;
    int64_t wf_sig_seen_ = 0;
    struct WFRow {
        std::array<uint8_t, WF_BINS> bins;
        int64_t ms = 0;
        bool sig = false;
    };
    std::deque<WFRow> wf_rows_;
    int64_t wf_row_ms_ = 0;
    uint32_t wf_seen_ = 0;
    float wf_floor_db_ = -80.0f;
    DSP::RealToHalfComplexTransform<WF_FFT, std::complex<float>> wf_fft_;
    int config_scroll_ = 0;
    int log_scroll_ = 0;
    bool log_follow_ = true;
    int utils_selection_ = 0;
    int utils_scroll_ = 0;
    int utils_max_scroll_ = 0;
    bool auto_send_enabled_ = false;
    std::chrono::steady_clock::time_point auto_send_last_{};
    std::mt19937 auto_send_rng_{std::random_device{}()};
    unsigned auto_send_seq_ = 0;
    bool auto_alt_enabled_ = false;
    int alt_index_ = -1;
    unsigned alt_seqs_[ALT_MODE_COUNT] = {0};
    int rig_field_ = 0;
    int64_t last_ptt_seen_ms_ = 0;
    int recent_sel_ = -1;
    static constexpr int OCC_HIST_SIZE = 60;
    float occ_hist_[OCC_HIST_SIZE] = {};
    int occ_hist_n_ = 0;
    int occ_hist_pos_ = 0;
    int64_t occ_hist_ms_ = 0;
    int rig_step_index_ = 2;
    int rig_preset_index_ = 0;
    int saved_stderr_ = -1;
    int frame_counter_ = 0;  
    bool show_help_ = false;  
    bool show_csma_help_ = false;
    bool show_sync_help_ = false;
    
    bool calibrating_threshold_ = false;
    int calibration_start_frame_ = 0;
    float calibration_max_level_ = -100.0f;
};
