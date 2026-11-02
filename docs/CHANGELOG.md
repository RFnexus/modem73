Welcome to the MODEM73 changelog. This document will be sorted by ascending order first with later entries towards the bottom.

# 02-11-26 /// 2.5.0 

### A coat of new paint to start


MODEM73 now links ncurses wide (ncursesw) instead of ncurses which provides full UTF-8 encoding and Unicode glyph sets. I've touched up a few areas of the UI and added a new STYLE tab where you can chose either ASCII or Unicode glyphs to your liking.
This is a step to a much larger change that will be the UI for the integrated ARQ system. 
<img width="1198" height="754" alt="image" src="https://github.com/user-attachments/assets/26a9a292-0818-4980-a370-6925db01d0d4" />


### Networked instances
A headless instance of MODEM73 can now be attached to an already running TUI. `modem73 --attach HOST:PORT` now opens the TUI against an already running headless instance over its control port.

### And networked audio
`-d tcp:HOST:PORT` replaces the sound card with a TCP stream which enables another program to carry MODEM73's audio (Thanks @thaum-labs for the PR). You can also configure this under the new NETWORK tab in the config.

### Restart from config
A restart button lives under the config and restarts the MODEM73 instance in place

### AUTO framing
You can now select the "AUTO" type for framing instead of specifying NORMAL/SHORT/LONG when an application does not make use of MODEM73's native control port functionality. This will automatically frame incoming packets based on their size. 

### Bug fixes
- Fixed a crash on the config tab caused by preset lines in the settings file
- Number formatting no longer breaks in non-English locales in saved settings
- (even more) tweaks to the CSMA for SYNC and RANKED to improve collision management and faster turnaround times. It should be getting pretty speedy. 
