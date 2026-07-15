# Control Port API

TCP JSON protocol on port 8073 

Wire format: 4-byte big-endian length prefix + JSON payload.

## Commands

| Command | Description |
|---|---|
| `get_status` | Current modem/channel state |
| `get_config` | Current configuration |
| `set_config` | Update configuration (partial updates OK) |
| `rigctl` | Passthrough command to rigctld |
| `tx` | Transmit data via KISS |

---

## `get_status`

**Request:** `{"cmd": "get_status"}`

**Response:**

| Field | Type | Description |
|---|---|---|
| `channel_state` | string | `"idle"`, `"tx"`, or `"rx"` |
| `ptt_on` | bool | PTT currently keyed |
| `rx_frame_count` | int | Successfully decoded frames |
| `tx_frame_count` | int | Transmitted frames |
| `rx_error_count` | int | Preamble + CRC errors |
| `sync_count` | int | Preamble sync detections |
| `preamble_errors` | int | Sync found but preamble decode failed |
| `symbol_errors` | int | Symbol-level errors (OFDM only) |
| `crc_errors` | int | CRC check failures |
| `last_snr` | float | Last decoded frame SNR (dB) |
| `last_ber` | float | Last decoded frame BER (0.0-1.0, -1 if unavailable) |
| `ber_ema` | float | Exponential moving average BER |
| `client_count` | int | Connected KISS clients |
| `rigctl_connected` | bool | rigctld connection status |
| `audio_connected` | bool | Audio device health |

Stats switch between the OFDM, MFSK and RDM decoders based on active `modem_type`.

---

## `get_config`

**Request:** `{"cmd": "get_config"}`

**Response:**

| Field | Type | Description |
|---|---|---|
| `callsign` | string | Station callsign |
| `modem_type` | int | `0` = OFDM, `1` = MFSK, `2` = ROBUST (RDM) |
| `robust_mode` | int | `0` = RDM-1200 (1150 bps), `1` = RDM-600 (585 bps), `2` = RDM-300 (296 bps), `3` = RDMN-300 (296 bps), `4` = RDMN-150 (149 bps), `5` = RDM-1200S (732 bps), `6` = RDM-600S (378 bps), `7` = RDM-300S (194 bps), `8` = RDMN-300S (197 bps), `9` = RDMN-150S (99 bps), `10` = RDM-800 (780 bps), `11` = RDM-800S (510 bps). Modes 0-4 and 10 carry 512-byte frames (510 B MTU), 5-9 and 11 carry 172-byte frames (170 B MTU). RDMN modes use 600 Hz bandwidth, all others 2400 Hz. RX auto-detects the mode. |
| `mfsk_mode` | int | `0` = MFSK-8, `1` = MFSK-16, `2` = MFSK-32, `3` = MFSK-32R |
| `modulation` | string | OFDM: `"BPSK"`..`"QAM4096"`. MFSK: `"MFSK-8"`..`"MFSK-32R"`. ROBUST: `"RDM-1200"`..`"RDM-800S"` |
| `code_rate` | string | `"1/2"`, `"2/3"`, `"3/4"`, `"5/6"`, `"1/4"`, `"1/2x2"`, `"1/4x2"` (OFDM only; the `x2` rates send the codeword twice for time diversity on fading paths; every code bit airs twice, half the frame duration apart. Valid with long frames up to QAM16; not valid with QAM64+ long or QAM256+ normal frames) |
| `postamble` | bool | OFDM only. Append a trailing sync anchor (~0.4 s of extra airtime) to each transmitted frame. An aware receiver uses it to rescue frames whose preamble/meta was fade-clipped; legacy receivers reject it cleanly (invalid-callsign marker) with no interop impact. |
| `short_frame` | bool | Short frame mode (OFDM only) |
| `center_freq` | int | Center frequency in Hz |
| `payload_size` | int | Current PHY payload capacity in bytes |
| `csma_enabled` | bool | CSMA carrier sense enabled |
| `carrier_threshold_db` | float | CSMA threshold (dB) |
| `p_persistence` | int | P-persistence value (0-255) |
| `slot_time_ms` | int | CSMA slot time (ms) |
| `tx_blanking_enabled` | bool | Suppress decoder during TX |
| `fragmentation_enabled` | bool | Enable packet fragmentation/reassembly |

---

## `set_config`

**Request:** `{"cmd": "set_config", ...fields...}`

Send only the fields you want to change. All fields from `get_config` are accepted.


Example:
```json
{"cmd": "set_config", "modulation": "8PSK", "code_rate": "1/2"}
```

**Response:** `{"ok": true}` or `{"ok": false}`

---

## `rigctl`

**Request:** `{"cmd": "rigctl", "command": "F"}`

Passes the command string to rigctld and returns the response.

**Response:** `{"ok": true, "response": "145000000\n"}`

---

## `tx`

**Request:**
```json
{"cmd": "tx", "data": "<base64-encoded payload>", "oper_mode": -1}
```

| Field | Type | Description |
|---|---|---|
| `data` | string | Base64-encoded raw payload bytes |
| `oper_mode` | int | OFDM mode override (-1 = use current config) |

**Response:** `{"ok": true, "size": 123}`

---

## Events

The control port broadcasts  events to all connected clients:

| Event | When |
|---|---|
| `config_changed` | Any configuration change  |
| `rx_frame` | A frame was decoded and delivered to KISS clients |

### `rx_frame`

Sent just before the decoded frame is written to the KISS port, so stats consumers can associate the event with the next KISS frame.

| Field | Type | Description |
|---|---|---|
| `snr` | float | Frame SNR (dB) |
| `ber_pct` | float | Pre-FEC bit error rate in percent (0-100, negative if unavailable) |
| `level_db` | float | Receive audio level (dBFS) sampled at frame end |
