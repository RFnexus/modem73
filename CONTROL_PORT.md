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

Stats switch between OFDM and MFSK decoder based on active `modem_type`.

---

## `get_config`

**Request:** `{"cmd": "get_config"}`

**Response:**

| Field | Type | Description |
|---|---|---|
| `callsign` | string | Station callsign |
| `modem_type` | int | `0` = OFDM, `1` = MFSK |
| `mfsk_mode` | int | `0` = MFSK-8, `1` = MFSK-16, `2` = MFSK-32, `3` = MFSK-32R |
| `modulation` | string | OFDM: `"BPSK"`..`"QAM4096"`. MFSK: `"MFSK-8"`..`"MFSK-32R"` |
| `code_rate` | string | `"1/2"`, `"2/3"`, `"3/4"`, `"5/6"`, `"1/4"` (OFDM only) |
| `short_frame` | bool | Short frame mode (OFDM only) |
| `center_freq` | int | Center frequency in Hz |
| `payload_size` | int | Current PHY payload capacity in bytes |
| `csma_enabled` | bool | CSMA carrier sense enabled |
| `carrier_threshold_db` | float | CSMA threshold (dB) |
| `p_persistence` | int | P-persistence value (0-255) |
| `slot_time_ms` | int | CSMA slot time (ms) |
| `tx_blanking_enabled` | bool | Suppress decoder during TX |

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
