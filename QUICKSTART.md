# Quickstart

modem73 is a free, open source software modem. It has configs for any setup, from line of sight UHF links over FM to HF propagation. This guide gets you running in under 5 minutes.

## Step 1: Download & install

The first step is to download either the source or compiled binary for your system from [modem73.app](https://modem73.app/index.html).

If you're building from source, follow along with the instructions in the README or run `install.sh` if you are on a Debian or Arch based system.

For a precompiled build, find the release that matches your OS and architecture:

- **Raspberry Pi** (Pi 5, Pi Zero 2): Debian 12 or Debian 13 `arm64`
- **Everything else**: match your architecture. Builds are available for Ubuntu, Debian, and Mint.
- **Windows**: see the separate Windows README and find the latest releases at [github.com/RFnexus/modem73-win/releases](https://github.com/RFnexus/modem73-win/releases)

Install the package with `apt` or your system's package manager:

```
sudo apt install ./modem73_<version>_<arch>.deb
```

### Installing (Windows)

Grab the latest build from the [modem73-win releases](https://github.com/RFnexus/modem73-win/releases). The Windows build is a separate fork, ported to Win32 APIs and PDCurses with LLM assisted tooling, so treat it as experimental. If you do not need serial PTT, running the upstream build under [WSL](https://learn.microsoft.com/en-us/windows/wsl/install) is the more stable path.

To build it yourself, everything is vendored under `deps/` (aicodix DSP, PDCurses, hidapi, miniaudio, cJSON) and the exe is statically linked, so there is nothing external to install at build or run time. CM108 USB PTT is always enabled.

Native on Windows, from a MinGW64 shell after installing [MSYS2](https://www.msys2.org/):

```
pacman -S --needed make mingw-w64-x86_64-gcc
make CXX=g++ CC=gcc
```

Or cross compile from Linux:

```
# Debian/Ubuntu
sudo apt install make g++-mingw-w64-x86-64-posix
make
```

Either way you end up with `modem73.exe`.

## Step 2: Launch modem73

Once installed, you can launch modem73 in any terminal.

```
modem73
```

By default it will start in UI mode where you can configure all of the settings.

To navigate the program you can click with the mouse or use the shortcut keys:

- **Tab** to change tabs
- **Up and down arrow keys** to move options
- **Left and right arrow keys** to change values, or when on a value that can be selected from a dropdown, **Enter**

### Launching (Windows)

On Windows you will want to call the .exe instead:

```
# Start in UI mode
modem73.exe

# Start in headless mode
modem73.exe --headless

# See all options
modem73.exe --help
```

## Step 3: Config

The first tab you will want to head over to is CONFIG. Under config, you'll want to configure your:

- Sound input device
- Sound output device
- PTT

### SOUND DEVICES

**Input** is what the modem decodes. Set this to whatever your radio or sound interface shows up as on the output side.

**Output** goes the other way. Set this to your radio's input.

Most interfaces present both under the same name. An all in one audio cable shows up as `AIOC`, and similarly HF rigs like the ICOM-7300 that have internal soundcards will have their own name.

### PTT

Push to talk is what keys up the radio to transmit. There are five options:

| Option | Use case |
| --- | --- |
| `none` | Over the air (speaker/mic) or PTT handled elsewhere |
| `rigctl` | HF rigs with CAT control |
| `VOX` | Radios keyed by an audio tone |
| `COM (serial)` | Serial RTS/DTR keying, including the AIOC and Digirig |
| `CM108` | GPIO keying on CM108 based sound interfaces |

**COM** (serial) is the straightforward path for HT interfaces. Point it at the serial device your cable enumerates as and pick RTS or DTR to match the cable.

**rigctl** comes from the Hamlib library. Install it with the `libhamlib-utils` package or your distro's equivalent:

```
sudo apt install libhamlib-utils
```

modem73 connects to rigctl over TCP, so start `rigctld` with your rig's options first. For an IC-7300:

```
rigctld -m 3073 -r /dev/ttyUSB0 -s 115200
```

Run `rigctl -l` to find the model number for your radio. `rigctld` runs in the background, listens on port 4532, and gives you both PTT and control of the rig from modem73 itself, including ALC tuning under the RIG tab when rigctl PTT is selected.

### rigctl and Hamlib (Windows)

Hamlib ships the `rigctld.exe` binary. Download it from the [Hamlib 4.7.2 release](https://github.com/Hamlib/Hamlib/releases/tag/4.7.2), then add its `bin` folder to your PATH so you can start rigctld from any terminal.

An example that lets modem73 key up and change frequency on an ICOM-7300:

```
rigctld.exe -r COM3 -m 3073 -s 19200
```

Replace COM3 with whatever your radio actually enumerates as. `-m` is the model number, and `rigctl.exe -l` lists them all.

To check the connection before pointing modem73 at it, drop the `d` and run `rigctl.exe` on its own, then type `f` to read back the current frequency.

### Serial, AIOC, and CM108 (Windows)

Serial PTT takes a COM port rather than a device path. Check Device Manager for the number, and expect it to change after a reboot or a replug.

```
modem73.exe --ptt com --com-port COM4 --com-line rts
```

For the [All In One Cable](https://github.com/skuep/AIOC), set PTT to COM, pick your COM port, set PTT line to `BOTH`, and set invert to `NORMAL`. Note that this differs from Linux, where the AIOC wants invert set to INVERT RTS.

CM108 keying works the same as it does everywhere else:

```
modem73.exe --ptt cm108 --cm108-gpio 3
```

### TEST YOUR SETUP

Restart modem73 after changing audio or PTT settings.

If audio is working you will see it coming in on the waterfall. To confirm the transmit side, go to **UTILS** and hit **Send random data**, or press `2`. Your radio should key up and transmit.

## Step 4: Modem settings

modem73 has three families of data modems, and all of them decode simultaneously. Between them they cover poor HF propagation through to clean line of sight FM links.

Three things matter on this screen: the modem itself, which you change with the left and right arrow keys, its settings, and the info panel on the right, which updates to show the details of whatever mode and options you have selected.

### OFDM

Use OFDM for anything over FM, and for stable HF SSB links keep modulation under 8PSK.

**Modulation** runs from BPSK up to QAM4096. Each step up the ladder buys throughput and costs you required SNR.

**Coding rate** is the balance between data and redundancy. 1/4 carries the most redundancy, 5/6 the least. On a link with a high noise floor or fading, take the lower rate and give up the throughput.

**Frame size** is how many bytes go out in one transmission: short, normal, or long frames. It also sets how long the modem spends on air.

**Postamble** adds a second sync marker at the end of the frame. Worth turning on for high noise, busy channels, or HF fading. It costs 0.4 seconds of extra transmit time.

### ROBUST (RDM)

The RDM modes are built for daily HF use and run from RDM-1200 down to RDM-300.

| Mode | Decodes down to | Survives |
| --- | --- | --- |
| RDM-1200 | 5 dB SNR | 0.5 second of deep fading |
| RDM-600 | 1 dB SNR | 2 seconds of deep fading |

These are the modes for long distance HF DXing and NVIS.

### MFSK

MFSK is non-coherent, so it decodes without a preamble or sync. Modes run from 32R down to 8.

- MFSK-8 decodes down to -9 dB
- M32R decodes down to -7 dB

Keep these around as your weak signal backup.

### DECODERS

Every mode decodes at once, which costs CPU. On a Pi Zero 2 or anything else with limited headroom, disable the ones you aren't using under the **decoders** section.

## Step 5: CSMA

Carrier-sense multiple access (CSMA) simply means listening before we transmit. This helps prevent collisions. But CSMA is a tricky problem when using a shared resource with minimal bandwidth like the RF spectrum.

modem73 implements several CSMA strategies that can be configured in the config. They are Threshold, Sync, and Ranked.

**Threshold** means simply listening for the level of audio coming in. This is useful for quiet FM links where we know our signal will always be louder than the noise floor.

**Sync** waits and listens for a frame coming in. Instead of measuring how loud the audio is, it looks for the start of a real modem73 signal. This matters on HF, where the noise floor moves around all day. With Threshold on a noisy band you can end up waiting forever for a quiet that never comes, because the band itself is never quiet. Sync only waits for actual stations.

Sync still picks a random wait before it transmits. That wait is called the contention window, and its size is worked out from how many stations we have heard recently. With two stations on frequency the window stays short. With six or more it opens up, because more stations need more room to avoid landing on the same moment. Picking at random is simple and it always works, but it is a gamble. Two stations can draw the same moment, and a lost frame is expensive.

**Ranked** builds on Sync and takes the guessing out. Every transmission already starts with a short signature tone, and that tone carries a 16 bit station ID. Every station listening keeps a list of the IDs it has heard. Each one sorts that list the same way, so all of them work out the same order without sending anything extra. Your turn is a time slot, your position out of the stations on frequency. Whoever transmitted last moves to the back of the line, so nobody takes two turns while someone else is still waiting.

Because the order is worked out instead of drawn at random, stations that can hear each other stop colliding. Two stations passing traffic back and forth settle into clean alternation, and the reply comes back at the start of the next slot rather than after a random wait. On a channel with several users this is where most of the speed comes from.

Ranked has two requirements. Every station needs to be running it, with Lead Tone and the presence tone that goes out every 45 to 90 seconds while a station is idle, or the order will not form. Selecting Ranked turns both on.

And every station needs to hear every other one. Ranked does not solve the hidden station problem. Good fits for Ranked are an HF NVIS net in one region, VHF simplex, or everyone on a shared repeater.

## Step 6: Fragmentation and TX blanking

**Fragmentation** handles incoming packets that exceed your frame size by chunking them automatically.

**TX blanking** stops you from hearing your own packets.

### IMPORTANT ! USING EXTERNAL APPLICATIONS?

Turn **Fragmentation ON** when another external application like Winlink is sending out PACLEN frames that exceed the modems frame size or your incoming frame size is unknown. It is ideal to have fragmentation OFF, but enable it where neccessary.

### Where settings live (Windows)

Settings, presets, and the performance log are stored in `%APPDATA%\modem73\`.

If you are running [modem73interface](https://github.com/RFnexus/modem73interface) for Reticulum, the interface file goes in `%USERPROFILE%\.reticulum\interfaces\`.

## Step 7: How to connect programs

modem73 exposes two TCP ports. Applications send and receive data on the KISS port, and read or change the modem's settings on the control port.

### KISS PORT

Standard KISS framing on TCP `127.0.0.1:8001`. Point any KISS capable application at it and the bytes you write go out over the air, while decoded frames come back to every connected client.

```
modem73 --port 8001
```

### CONTROL PORT

A JSON protocol on TCP `127.0.0.1:8073`, each message prefixed with its length as a 4 byte big endian integer. Use it to read the current mode, SNR and channel state, change the modem settings, or pass a command straight through to rigctl.

```
modem73 --control-port 8073
```

Send `{"cmd": "get_config"}` to read the current settings, including `payload_size`, which tells you the largest frame the current mode can carry. `{"cmd": "set_config", "robust_mode": 6}` changes the mode on the fly, and the modem pushes an event to every connected client whenever anything changes.

Full command list in [CONTROL_PORT.md](https://github.com/RFnexus/modem73/blob/master/CONTROL_PORT.md).

### WORKING EXAMPLES

[modem73interface](https://github.com/RFnexus/modem73interface) is a Reticulum interface that reads the frame size from the control port and switches frame sizes to match the traffic.

## Step 8: What terms mean

### SNR

Signal to noise ratio. This is how far your signal sits above the noise floor in dB. A higher SNR is always better and combined with bit error rate determines link quality.

### Bit error rate

The total percentage of raw bit errors before forward error correction. modem73 works it out by re-encoding the frame once it decodes, then counting how many of the received bits disagreed. A frame can come through perfectly clean at 15% BER, because fixing those errors is the entire job of the FEC. What BER tells you is how much margin is left before frames start failing.

### Frame size

How many bytes go out in one transmission. OFDM frames run from 256 to 6144 bytes depending on modulation and code rate (MICRO, SHORT, NORMAL or LONG), ROBUST frames are 510, 170 or 30 bytes, and the info panel shows the exact number for the mode you have picked. A bigger frame wastes less time on sync and overhead but spends longer on air, and one deep fade can take the whole frame with it. Packets bigger than the frame are split up by fragmentation.

### Modulation

What carrier, number of carriers, and how many bits we send at once. Higher carriers (like QAM4096) require a better signal, or SNR. Lower modulation orders like BPSK require a lot less.

### Mode

modem73 has 3 modes: OFDM, ROBUST, or MFSK.

**OFDM** The fast family. Hundreds of carriers side by side in 2400 Hz, each carrying a PSK or QAM symbol. From about 790 bps at BPSK to over 13 kbps at QAM4096. Use it for anything over FM, and on good HF SSB paths at 8PSK or below.

**ROBUST** Built for fading HF such as 40 and 80 meter NVIS. QPSK on widely spaced carriers with a guard interval between symbols, so Doppler spread and multipath echoes do not smear one symbol into the next. RDM-1200 (about 1150 bps) decodes down to 5 dB SNR and RDM-600 near 0 dB. The RDMN modes are 600 Hz wide versions, RDMN-300 and RDMN-150, for narrow filters and crowded bands.

**MFSK** One tone at a time out of 8, 16 or 32. The receiver only has to find the loudest tone, with no phase tracking, which is why it decodes below the noise floor (MFSK-8 to about -9 dB) and why it is slow: 34 bps for MFSK-8, 99 bps for MFSK-32R. Keep it as the weak signal backup.

### How do I pick a mode?

Start with the lowest mode first. Then, go up. Don't pick something like QAM256 right out of the box. Check your SNR and BER and step up one notch at a time while frames keep decoding. BER is the number to watch: low means you have margin to go faster, climbing means you are near the edge and the next step up will start dropping frames. SNR tells you roughly where you will land. modem73 shows it green above 10 dB and yellow between 5 and 10, and the higher modulations need the green. When frames start failing, step back down one notch and stay there.

If you're on HF, start with the ROBUST modes. RDM-600 while the band is fading, RDM-1200 once it decodes cleanly with SNR over 5 dB. Only move to OFDM when the path is steady, and keep it at 8PSK or below. If nothing decodes at all, drop to MFSK.

### Every other setting

#### MODEM SETTINGS

**Code rate**

How much of the frame is data and how much is error correction. 5/6 is almost all data and needs a clean channel. 1/4 spends three quarters of the frame on correction and decodes deep in the noise. The x2 rates send the whole codeword twice so the receiver can combine both copies; use them when the path fades in and out.

**Postamble**

A second sync marker at the end of an OFDM frame. If the receiver missed the start, it can still lock on at the end and recover the frame. Costs 0.4 s of airtime. Worth it on noisy or fading channels.

**RDM mode**

Which ROBUST speed to send. Lower numbers are slower and decode at a lower SNR

**MFSK mode**

How many tones. More tones means more bits per symbol and a wider signal: MFSK-8 is 250 Hz wide, MFSK-32 is 1000 Hz. 32R keeps 32 tones with less error correction for more speed.

**RX decoders**

Which families the receiver listens for. The receiver decodes all three at once by default. Each one costs CPU, so on a Pi Zero 2 turn off the ones you are not using.

#### SIGNAL

**Level**

How loud the audio coming into the sound card is, in dB below full scale. 0 dB is the loudest the sound card can take. THRESHOLD CSMA compares this to Threshold to decide if the channel is busy.

**Threshold**

The Level above which THRESHOLD CSMA calls the channel busy. Set it a few dB above your normal noise floor.

**Constellation**

One dot per received symbol. Tight clusters mean a clean signal. Smeared or rotating dots mean noise, fading, or a frequency offset.

**Waterfall**

The audio spectrum over time. Your signal should sit in the middle of the passband with nothing else on top of it.

#### CSMA

**CSMA**

Listen before transmit, so stations do not talk over each other. With it off, modem73 keys up as soon as a packet is queued.

**Mode (CSMA)**

THRESHOLD calls the channel busy when any audio is over Threshold. SYNC only counts a real modem73 signal, so HF noise cannot hold you off forever. RANKED is SYNC plus stations taking turns in a fixed order; every station must run 2.3 or newer with RANKED on.

**Band / Preset**

Timing presets. Band picks HF or VHF/UHF numbers and Preset picks how busy the channel is. The knobs below are filled in from these; changing one by hand overrides it.

**Quiet**

How long the channel has to be idle before you contend for it.

**Window**

The random wait drawn after Quiet, so two stations that are ready at the same moment do not collide.

**Lead tone**

A short tone at keyup so other stations hear you before the data starts. RANKED needs it.

**Dither**

A small per-callsign delay so replies from several stations do not land on the same instant.

**Burst**

How many queued packets you send once you win the channel.

**FastFloor**

Shorter waits in SYNC mode. Only if every station runs 2.3 or newer.

**Beacon**

In RANKED, an idle station sends a presence tone every 45 to 90 s so the others keep it in the turn order.

#### TX AND RX

**Fragmentation**

Splits packets bigger than one frame into pieces and reassembles them at the far end. Turn **Fragmentation ON** when another external application like Winlink is sending out PACLEN frames that exceed the modems frame size or your incoming frame size is unknown. It is ideal to have fragmentation OFF, but enable it where neccessary.

**TX blanking**

Mutes the decoder while you transmit so you do not decode your own signal through the mic.

**TX delay**

Time between keying PTT and the start of audio, 250 to 2500 ms, so the radio is fully on transmit before the data starts.

**TX level**

Sound card output drive, 5 to 100 percent. Set it so the radio's ALC barely moves. Too hot distorts the signal and other stations decode less, not more.

**PTT**

How modem73 keys the radio. NONE: no keying, speaker into mic. RIGCTL: through rigctld over TCP. VOX: a tone before the data trips the radio's VOX. COM: DTR or RTS on a serial port, which is what the AIOC uses. CM108: the GPIO pin on a CM108 USB sound card. HAMLIB: direct Hamlib control without rigctld.

**VOX tone / lead / tail**

For VOX PTT: the tone frequency, how long it plays before the data so the radio keys up, and how long after so the radio does not drop early.

#### NETWORK

**Callsign**

Goes in every frame header so other stations can see who transmitted. Also drives the CSMA dither and the RANKED turn order.

**KISS port**

TCP port, 8001 by default, where applications send and receive packets.

**Control port**

TCP port, 8073 by default, for the JSON control API: read SNR and channel state, change modes, or pass commands through to rigctl.

**LAN mode**

Listen on every network interface instead of only localhost, so other machines on your LAN can use the modem.

**Config token**

A short code that packs up your modem settings. Share it so another station can paste it in and match your setup.

**Presets**

Saved sets of settings. In the config tab, s saves the current one and x deletes it.

