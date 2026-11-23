# RFD Configurator

A small Python utility for auto-configuring an RFD900X (SiK-based) radio modem to match a remote peer. It scans a range of baud rates listening for an `RFDTEST` beacon broadcast by a remote radio, extracts the remote's `NETID` and air rate, then puts the local radio into AT command mode to set its `NETID` to match (`ATS3`) and saves the setting (`AT&W`).

## How it works

1. **Scan** — Opens the serial port at each candidate baud rate and listens for a beacon matching the pattern:
   ```
   RFDTEST,SEQ=########,NETID=###,AIR=###
   ```
2. **Discover** — Once a beacon is found, its `NETID`, `AIR` rate, and `SEQ` are parsed from the payload.
3. **Configure** — The local radio is put into AT command mode (`+++` with SiK guard timing), `ATS3=<netid>` is sent to set the NETID, `AT&W` saves it to EEPROM, and `ATZ` reboots the radio.

## Requirements

- Python 3
- [`pyserial`](https://pypi.org/project/pyserial/)

```bash
pip install pyserial
```

## Usage

```bash
python "RFD Configurator.old.py" [options]
```

### Options

| Flag | Default | Description |
|---|---|---|
| `--port` | `/dev/ttyUSB0` | Serial port for the local radio |
| `--baudrate` | `57600` | Local baud rate used for the AT configuration step |
| `--scan-duration` | `2.0` | Seconds to listen for a beacon at each candidate baud rate |
| `--baudrates` | `9600 19200 38400 57600 115200 230400` | Candidate baud rates to sweep until a valid beacon is found |
| `--netids` | `1` – `255` | Optional reserve NETID values (the discovered remote NETID is preferred) |

### Example

```bash
python "RFD Configurator.old.py" --port /dev/ttyUSB0 --scan-duration 3
```

This scans the default baud rates on `/dev/ttyUSB0`, and once a beacon is found, configures the local radio's NETID to match the remote and saves it.

## Notes

- If no beacon is found across all candidate baud rates, the script exits without making changes.
- The local radio is rebooted automatically after the NETID change (via `ATZ`), with a short delay to allow resynchronization.
