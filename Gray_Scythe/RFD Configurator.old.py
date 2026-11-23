#!/usr/bin/env python3

import argparse
import re
import time
from typing import Any, Dict, Iterable, Optional

import serial

PORT = "/dev/ttyUSB0"
LOCAL_BAUD = 57600

# A small but useful first-pass sweep. Increase this if your environment needs a wider scan.
DEFAULT_BAUDRATES = [9600, 19200, 38400, 57600, 115200, 230400]
DEFAULT_NETIDS = list(range(1, 256))

BEACON_RE = re.compile(
    rb"RFDTEST,SEQ=(\d{8}),NETID=(\d{1,3}),AIR=(\d{1,3})\r?\n"
)


def parse_beacon_payload(payload: bytes) -> Dict[str, int]:
    match = BEACON_RE.search(payload)
    if match is None:
        raise ValueError(f"No valid RFDTEST beacon found in payload: {payload!r}")

    return {
        "seq": int(match.group(1)),
        "netid": int(match.group(2)),
        "air": int(match.group(3)),
    }


def find_first_beacon(data: bytes) -> Dict[str, int]:
    match = BEACON_RE.search(data)
    if match is None:
        raise ValueError(f"No valid beacon in buffer: {data[:256]!r}")

    return parse_beacon_payload(match.group(0))


def read_for(ser, duration: float) -> bytes:
    deadline = time.monotonic() + duration
    received = bytearray()

    while time.monotonic() < deadline:
        try:
            chunk = ser.read(512)
        except serial.SerialException:
            break

        if chunk:
            received.extend(chunk)

    return bytes(received)


def enter_command_mode(ser) -> bool:
    ser.reset_input_buffer()

    # SiK requires a guard time around +++.
    time.sleep(1.2)
    ser.write(b"+++")
    ser.flush()
    time.sleep(1.2)

    response = read_for(ser, 0.8)
    return b"OK" in response


def send_command(ser, command: str, wait: float = 0.5) -> bytes:
    ser.write(command.encode("ascii") + b"\r\n")
    ser.flush()
    return read_for(ser, wait)


def configure_netid(ser, netid: int) -> None:
    if not enter_command_mode(ser):
        raise RuntimeError("Could not enter AT command mode")

    response = send_command(ser, f"ATS3={netid}")
    print(f"ATS3={netid}: {response!r}")

    response = send_command(ser, "AT&W", wait=0.8)
    print(f"AT&W: {response!r}")

    send_command(ser, "ATZ", wait=0.2)

    # Allow the radio to reboot and resynchronize after the NETID change.
    time.sleep(2.5)
    ser.reset_input_buffer()


def scan_for_beacon(port: str, baudrate: int, scan_duration: float = 2.0) -> Optional[Dict[str, Any]]:
    try:
        with serial.Serial(port, baudrate, timeout=0.1) as ser:
            raw = read_for(ser, scan_duration)
            if not raw:
                return None

            try:
                metadata = find_first_beacon(raw)
            except ValueError:
                return None

            return {"baudrate": baudrate, "metadata": metadata}
    except serial.SerialException as exc:
        print(f"[baud {baudrate}] serial open failed: {exc}")
        return None


def discover_remote_metadata(
    port: str,
    candidate_baudrates: Iterable[int] = DEFAULT_BAUDRATES,
    scan_duration: float = 2.0,
) -> Optional[Dict[str, Any]]:
    for baudrate in candidate_baudrates:
        print(f"Scanning baud {baudrate} on {port}...")
        result = scan_for_beacon(port, int(baudrate), scan_duration=scan_duration)
        if result is not None:
            metadata = result["metadata"]
            print(
                "Discovered remote: "
                f"NETID={metadata['netid']}, AIR={metadata['air']}, "
                f"SEQ={metadata['seq']}, baud={result['baudrate']}"
            )
            return result

    return None


def configure_local_radio_to_remote(port: str, netid: int, baudrate: int = LOCAL_BAUD) -> None:
    with serial.Serial(port, baudrate, timeout=0.1) as ser:
        configure_netid(ser, netid)
        print(f"Local radio set to NETID={netid} on baud {baudrate}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Discover an RFD900X peer by scanning baud rates and parsing the beacon metadata."
    )
    parser.add_argument("--port", default=PORT, help="Serial port for the local radio (default: %(default)s)")
    parser.add_argument(
        "--baudrate",
        type=int,
        default=LOCAL_BAUD,
        help="Local serial baud rate used for the AT configuration step (default: %(default)s)",
    )
    parser.add_argument(
        "--scan-duration",
        type=float,
        default=2.0,
        help="Seconds to listen for a beacon at each candidate baud rate",
    )
    parser.add_argument(
        "--baudrates",
        type=int,
        nargs="*",
        default=list(DEFAULT_BAUDRATES),
        help="Candidate baud rates to sweep until a valid beacon is found",
    )
    parser.add_argument(
        "--netids",
        type=int,
        nargs="*",
        default=DEFAULT_NETIDS,
        help="Optional NETID values to try after discovery; the discovered remote NETID is preferred",
    )
    args = parser.parse_args()

    discovered = discover_remote_metadata(
        args.port,
        candidate_baudrates=args.baudrates,
        scan_duration=args.scan_duration,
    )

    if discovered is None:
        print("No valid RFDTEST beacon was found on the candidate baud rates.")
        return

    netid = discovered["metadata"]["netid"]
    print(f"Using discovered NETID {netid} and remote baud {discovered['baudrate']}")

    if args.netids:
        print(f"Candidate NETIDs in reserve: {args.netids[:10]}...")

    configure_local_radio_to_remote(args.port, netid, baudrate=args.baudrate)


if __name__ == "__main__":
    main()