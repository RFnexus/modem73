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
| `send_beacon` | Queue a presence tone transmission |
| `rx_frame_history` | Data for recently decoded frames (mode, SNR, BER) |

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
| `population` | int | Stations heard on frequency counted from signature tone IDs decoded within the 300s list expiry |
| `occupancy_pct` | int | Channel occupancy 0-100 when CSMA is enabled |


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
| `code_rate` | string | `"1/2"`, `"2/3"`, `"3/4"`, `"5/6"`, `"1/4"` |
| `postamble` | bool | OFDM only. Append a trailing sync anchor (~0.4 s of extra airtime) to each transmitted frame. An aware receiver uses it to rescue frames whose preamble/meta was fade-clipped; legacy receivers reject it cleanly (invalid-callsign marker) with no interop impact. |
| `short_frame` | bool | Short frame mode (OFDM only) |
| `payload_size` | int | Current PHY payload capacity in bytes |
| `csma_enabled` | bool | CSMA carrier sense enabled |
| `csma_sync_only` | bool | Busy detection uses carrier sync (DCD) only, ignoring the audio level threshold |
| `csma_ranked` | bool | RANKED turn order on top of sync detection. Only active while `csma_sync_only` and `tx_lead_tone` are also true |
| `beacon_interval_s` | int | RANKED presence tone interval when idle, 45-90 s. Values outside the range are ignored. Leave at 45 unless every station on frequency has the 300 s list expiry |
| `csma_fast_floor` | bool | Size the sync contention window for 550 ms detection. Turn off only if a station on frequency predates 2.3 |
| `csma_band` | int | `0` = HF, `1` = VHF/UHF preset timings |
| `csma_quiet_ms` | int | Idle time required before contending (ms, `0` = auto from frame airtime) |
| `csma_cw` | int | Contention window in slots (THRESHOLD mode) |
| `csma_responder_dither` | int | Reply offset dither (ms) to spread simultaneous responders |
| `csma_burst` | int | Frames sent per channel acquisition (1-4) |
| `tx_lead_tone` | bool | Signature lead with the 16-bit station ID ahead of each transmission |
| `carrier_threshold_db` | float | CSMA threshold (dB) |
| `slot_time_ms` | int | CSMA slot time (ms) |
| `tx_drive` | float | TX audio output level, 0.05-1.0. Applies immediately, no restart. Values outside the range are ignored |
| `tx_blanking_enabled` | bool | Suppress decoder during TX. Cannot be disabled when CSMA is enabled |
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

## `send_beacon`

**Request:** `{"cmd": "send_beacon"}`

Queues one presence tone 

**Response:** `{"ok": true}`

---

## `rx_frame_history`

**Request:** `{"cmd": "rx_frame_history", "limit": 64, "since_seq": 0}`

| Field | Type | Description |
|---|---|---|
| `limit` | int | Maximum frames to return (default 64) |
| `since_seq` | int | Only return frames with `seq` greater than this (default 0 = all). Pass the last `seq` you saw to catch up after a reconnect |

Returns meta data for recently decoded frames with the of newest first.

Example:
```json
{"ok": true, "frames": [
  {"seq": 3, "age_s": 12.4, "time": 1787956059.99, "snr": 23.6, "ber_pct": 0.0, "level_db": -8.0,
   "size": 19, "reassembled": false, "modem": "robust", "mode": "RDM-1200", "robust_mode": 0,
   "callsign": "VK2ABC", "callsign_source": "payload"},
  {"seq": 2, "age_s": 18.0, "time": 1787956054.37, "snr": 38.1, "ber_pct": 0.0, "level_db": -21.0,
   "size": 14, "reassembled": false, "modem": "ofdm", "mode": "QPSK 1/2 N", "oper_mode": 17,
   "modulation": "QPSK", "code_rate": "1/2", "frame_size": "normal",
   "callsign": "VK2ABC", "callsign_source": "phy"}
]}
```

Each entry carries the same fields as the `rx_frame` event plus `age_s` which is seconds since the frame was decoded.

---

## Events

The control port broadcasts  events to all connected clients:

| Event | When |
|---|---|
| `config_changed` | Any configuration change  |
| `rx_frame` | A frame was decoded and delivered to KISS clients |

### `rx_frame`

Sent just before the decoded frame is written to the KISS port, so stats consumers can associate the event with the next KISS frame. Fragmented packets produce one event for the reassembled packet, not one per fragment.

| Field | Type | Description |
|---|---|---|
| `seq` | int | Frame sequence number, increments by one per decoded frame since startup. Matches `seq` in `rx_frame_history` |
| `time` | float | Unix time of the decode (seconds) |
| `snr` | float | Frame SNR (dB) |
| `ber_pct` | float | Pre-FEC bit error rate in percent (0-100, negative if unavailable) |
| `level_db` | float | Receive audio level (dBFS) sampled at frame end |
| `size` | int | Payload bytes delivered to KISS |
| `reassembled` | bool | Packet was reassembled from fragments |
| `modem` | string | `"ofdm"`, `"robust"` or `"mfsk"`; which decoder produced the frame |
| `mode` | string | Received mode as shown in the TUI, e.g. `"QPSK 1/2 N"`, `"RDM-600"`, `"MFSK-16"` |
| `oper_mode` | int | OFDM only. Decoded operating-mode byte from the meta symbol |
| `modulation` | string | OFDM only. `"BPSK"`..`"QAM4096"` |
| `code_rate` | string | OFDM only. `"1/2"`, `"2/3"`, `"3/4"`, `"5/6"`, `"1/4"` |
| `frame_size` | string | OFDM only. `"short"`, `"normal"`, `"long"` or `"micro"` |
| `robust_mode` | int | ROBUST only. Detected mode, same numbering as the `robust_mode` config field |
| `mfsk_mode` | int | MFSK only. Detected mode, same numbering as the `mfsk_mode` config field |
