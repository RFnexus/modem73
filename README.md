<p align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="https://i.ibb.co/LDNR23jg/MODEM73-white.png">
    <source media="(prefers-color-scheme: light)" srcset="https://i.ibb.co/wZKznzrF/MODEM73-blk.png">
    <img alt="MODEM73" src="https://i.ibb.co/wZKznzrF/MODEM73-blk.png">
  </picture>
</p>

MODEM73 is a [KISS](https://en.wikipedia.org/wiki/KISS_(amateur_radio_protocol)) TNC frontend for the [aicodix](https://github.com/aicodix/modem) OFDM modem. 


![Screenshot](https://i.ibb.co/4ZhhvcQs/Peek-2026-01-01-10-41.gif)


## Building


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
# Requires DSP, code, and modem libraries 
git clone https://github.com/aicodix/dsp.git
git clone https://github.com/aicodix/code.git
git clone https://github.com/aicodix/modem.git

# Clone modem73
git clone https://github.com/RFnexus/modem73

# Your folders should look like this:
#.../
#├── dsp/           # DSP library (aicodix)
#│   └── ...
#├── code/          # Code library (aicodix)
#│   └── ...
#├── modem/         # Modem library (aicodix)
#│   └── ...
#└── modem73/       # modem73 src
#    └── ...

# Build
cd modem73
make AICODIX_DSP=../dsp AICODIX_CODE=../code MODEM_SRC=../modem
# Optional: move to /usr/local/bin 
sudo make install 
```

## Running & Operations

By default, MODEM73 will listen on port 8001

All of the modes provided by the OFDM modem require a bandwidth of 2400 Hz and work over both FM and SSB. 

There are currently four PTT options:
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
# CM108 USB audio interface PTT (GPIO3 is default)
./modem73 --ptt cm108 --cm108-gpio 3
```


## Updating

modem73 comes included with a update utility `update.sh`

To update to the latest version:
```
./update.sh
```