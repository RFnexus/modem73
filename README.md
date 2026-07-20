
<p align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="https://i.ibb.co/LDNR23jg/MODEM73-white.png">
    <source media="(prefers-color-scheme: light)" srcset="https://i.ibb.co/wZKznzrF/MODEM73-blk.png">
    <img alt="MODEM73" src="https://i.ibb.co/wZKznzrF/MODEM73-blk.png">
  </picture>
</p>


MODEM73 is an open source software modem that works with any HF, VHF, or UHF radio capable of 2400 Hz of bandwidth. All you need is a sound card and audio cable for your radio.  

![Screenshot](https://i.ibb.co/4ZhhvcQs/Peek-2026-01-01-10-41.gif)
<p>
<img width="546" height="423" alt="image" src="https://github.com/user-attachments/assets/7180ab80-4386-4ee1-8029-42ca5300ef13" />
<img width="276.5" height="199" alt="image" src="https://github.com/user-attachments/assets/5ac2a8bd-75a1-48a4-8264-74a851a06767" />
</p>



SSB, AM, and FM are all supported. It's plug and play compatible with any KISS application and works with rigctl, CM108 sound devices, and serial PTT out of the box.

There are three modem families each suited for covering any possible RF setup from clean line of sight FM links to poor HF band conditions. The receiver decodes all of them at the same time, so one station can hear anything another station sends without switching modes.

**OFDM**, based on the open source [COFDMTV modem](https://github.com/aicodix/modem) developed by Ahmet Inan / [aicodix GmbH](https://www.aicodix.de/). Modulations from BPSK to QAM4096 with code rates from 1/4 to 5/6 and payloads from 256 to 6144 bytes per frame. Rates run from about 790 bits per second to over 13 kilobits per second in the same 2400 Hz. 

**ROBUST**, a set of slow modes built for fading HF paths such as 40 and 80 meter NVIS.  Five modes from 1150 bps down to 149 bps in either 2400 Hz or a 600 Hz narrow variant

**MFSK**, a non-coherent mode for weak signal propagation and backup links. 

### Features

- KISS over TCP so it works with anything that speaks KISS: APRS clients, HamIRC, packet BBS software, Reticulum, custom applications
- JSON control port API for status and configuration for writing your own programs
- lightweight UI that runs straight from the terminal and headless mode for embedded use 

## Installation

Want to try out a demo of modem73 directly from your browser? Try it out here:
https://rfnexus.github.io/modem73-wasm-demo/
Hit "Start microphone" to begin decoding


### Windows?
Experimental Windows fork: https://github.com/RFnexus/modem73-win | Releases: https://github.com/RFnexus/modem73-win/releases/

It's recommended to use modem73 with [WSL](https://learn.microsoft.com/en-us/windows/wsl/install) instead 

## Building from source

### Linux

On a system with `apt`? Run the installer script:
```
./install.sh
```
--- 

1. Install dependencies

```
# Debian/Ubuntu/Pi
sudo apt install git build-essential libncurses-dev g++
```


#### Optional Addons


##### CM108 USB PTT Support

CM108-based USB audio interfaces have GPIO pins that can be used for PTT control. To enable CM108 support, install libhidapi-dev before building. The Makefile will auto-detect it and enable the feature.
```
# Debian/Ubuntu/Pi - install before building 
sudo apt install libhidapi-dev
```
----
2. Clone aiocdix DSP libraries and build.



```
# Clone modem73

git clone https://github.com/RFnexus/modem73

# Build

cd modem73
make 

# Optional: move to /usr/local/bin 
sudo make install 
```

and run `./modem73`

## Running & Operations

By default, MODEM73 will listen on port 8001

All of the modes provided by the OFDM modem require a bandwidth of 2400 Hz and work over both FM and SSB. 

There are currently five PTT options:
- NONE (speaker/mic over the air)
- Rigctl
- VOX
- Serial
- CM108


```
# Start in UI mode
./modem73

# Start in headless mode
./modem73 --headless

# See all options with:
./modem73 --help
```

### PTT options 

```
# Connect to rigctld for PTT control
./modem73 --rigctl localhost:4532
```

while running `rigctld`


```
./modem73 --ptt vox --vox-freq 1200 --vox-lead 500 --vox-tail 150
# 500ms vox lead and 150ms vox tail
```


```
./modem73 --ptt com --com-port /dev/ttyUSB0 --com-line rts
```

```
# CM108 USB audio interface PTT (GPIO3 is the default)
./modem73 --ptt cm108 --cm108-gpio 3
```

### Control port

A control port for modem73 will automatically start on port `8073` by default. View `CONTROL_PORT.md` for the full JSON spec

## Usage

### All In One Audio Cable (AIOC)
modem73 supports the [AIOC](https://github.com/skuep/AIOC) out of the box. To use the All In One Audio cable, set PTT to COM, specify your COM port, and set PTT line to `BOTH` and Invert to `INVERT RTS`. Make sure you have the correct permissions and `/dev/xxxx` specified. The AIOC on most setups will be /dev/ttyACMx (where x is 0, 1, 2). Note that it may change after a device restart, plugging it back in, etc. 

### rigctl
modem73 supports Hamlib and rigctl for any rigctl supported radio for PTT. Set rigctl to your options and run `rigctld -m (your model) -s (serial baud rate) -r /dev/XXXX)`  The `d` at the end of `rigctl` tells rigctl to run in network mode, which is what modem73 will connect to.

### Reticulum
Want to use modem73 with Reticulum? Check out the modem73interface
https://github.com/RFnexus/modem73interface

Drop it into `~/.reticulum/interfaces/` and in your Reticulum config, add something like:
```
[[MODEM73]]
type = Modem73Interface
enabled = yes
target_host = 127.0.0.1
target_port = 8001
control_host = 127.0.0.1
control_port = 8073
```




## Updating

modem73 comes included with a update utility `update.sh`

To update to the latest version:
```
./update.sh
```
