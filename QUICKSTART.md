# modem73 quickstart

modem73 is a free, open source software modem. It has configs for any setup, from line of sight UHF links over FM to HF propagation. This guide gets you running in under 5 minutes.

### 1. Download & Install

The first step is to download either the source or compiled binary for your system from [modem73.app](https://modem73.app).

If you're building from source, follow along with the instructions in the README or run `install.sh` if you are on a Debian or Arch based system.

For a precompiled build, find the release that matches your OS and architecture:

- **Raspberry Pi** (Pi 5, Pi Zero 2): Debian 12 or Debian 13 `arm64`
- **Everything else**: match your architecture. Builds are available for Ubuntu, Debian, and Mint.
- **Windows**: see the separate Windows README and find the latest releases here https://github.com/RFnexus/modem73-win/releases

Install the package with `apt` or your system's package manager:

```bash
sudo apt install ./modem73_<version>_<arch>.deb
```

### 2. Launch `modem73`

Once installed, you can launch modem73 in any terminal.

```bash
modem73
```

By default it will start in UI mode where you can configure all of the settings.

To navigate the program you can click with the mouse or use the shortcut keys:

- **Tab** to change tabs
- **Up and down arrow keys** to move options
- **Left and right arrow keys** to change values, or when on a value that can be selected from a dropdown, **Enter**

### 3. Config

The first tab you will want to head over to is CONFIG. Under config, you'll want to configure your:

- Sound input device
- Sound output device
- PTT

#### Sound devices

**Input** is what the modem decodes. Set this to whatever your radio or sound interface shows up as on the output side.

**Output** goes the other way. Set this to your radio's input.

Most interfaces present both under the same name. An all in one audio cable shows up as `AIOC`, and similarly HF rigs like the ICOM-7300 that have internal soundcards will have their own name. 

#### PTT

Push to talk is what keys up the radio to transmit. There are five options:

| Option | Use case |
| ---    | --- |
| `none` | Over the air (speaker/mic) or PTT handled elsewhere |
| `rigctl` | HF rigs with CAT control |
| `VOX` | Radios keyed by an audio tone |
| `COM (serial)` | Serial RTS/DTR keying, including the AIOC and Digirig |
| `CM108` | GPIO keying on CM108 based sound interfaces |

**COM** (Serial) is the straightforward path for HT interfaces. Point it at the serial device your cable enumerates as and pick RTS or DTR to match the cable.

**rigctl** comes from the Hamlib library. Install it with the `libhamlib-utils` package or your distro's equivalent:

```bash
sudo apt install libhamlib-utils
```

modem73 connects to rigctl over TCP, so start `rigctld` with your rig's options first. For an IC-7300:

```bash
rigctld -m 3073 -r /dev/ttyUSB0 -s 115200
```

Run `rigctl -l` to find the model number for your radio. `rigctld` runs in the background, listens on port 4532, and gives you both PTT and control of the rig from modem73 itself, including ALC tuning under the RIG tab when rigctl PTT is selected

#### Test your setup

Restart modem73 after changing audio or PTT settings.

If audio is working you will see it coming in on the waterfall. To confirm the transmit side, go to **UTILS** and hit **Send random data**, or press `2`. Your radio should key up and transmit.

### 4. Modem settings

modem73 has three families of data modems, and all of them decode simultaneously. Between them they cover poor HF propagation through to clean line of sight FM links.

Three things matter on this screen: the modem itself, which you change with the left and right arrow keys, its settings, and the info panel on the right, which updates to show the details of whatever mode and options you have selected.

#### OFDM

Use OFDM for anything over FM, and for stable HF SSB links keep modulation under 8PSK.

**Modulation** runs from BPSK up to QAM4096. Each step up the ladder buys throughput and costs you required SNR.

**Coding rate** is the balance between data and redundancy. 1/4 carries the most redundancy, 5/6 the least. On a link with a high noise floor or fading, take the lower rate and give up the throughput.

**Frame size** is how many bytes go out in one transmission: short, normal, or long frames. It also sets how long the modem spends on air.

**Postamble** adds a second sync marker at the end of the frame. Worth turning on for high noise, busy channels, or HF fading. It costs 0.4 seconds of extra transmit time.

#### ROBUST (RDM)

The RDM modes are built for daily HF use and run from RDM-1200 down to RDM-300.

| Mode | Decodes down to | Survives |
| --- | --- | --- |
| RDM-1200 | 5 dB SNR | 1 second of deep fading |
| RDM-600 | near 0 dB SNR | 2 seconds of deep fading |

These are the modes for long distance HF DXing and NVIS.

#### MFSK

MFSK is non-coherent, so it decodes without a preamble or sync. Modes run from 32R down to 8.

- MFSK-8 decodes down to -9 dB
- M32R decodes down to -7 dB

Keep these around as your weak signal backup.

#### Decoders

Every mode decodes at once, which costs CPU. On a Pi Zero 2 or anything else with limited headroom, disable the ones you aren't using under the **decoders** section.

### 5. CSMA

Carrier sense multiple access means listening for stations already talking before you transmit. modem73 is designed for multiple users sharing a channel, so this is worth setting.

Pick a preset by selecting your band and how busy the channel is. From there you can choose between an audio threshold and sync detection. **Enable "Sync only" when you are running modem73 over HF.**

### 6. Fragmentation and TX blanking

**Fragmentation** handles incoming packets that exceed your frame size by chunking them automatically.

**TX blanking** stops you from hearing your own packets.



