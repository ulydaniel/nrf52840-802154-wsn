#!/usr/bin/env python3
"""Capture IEEE 802.15.4 frames from Nordic's serial sniffer firmware."""

import argparse
import json
import re
import signal
import sys
import time
from datetime import datetime, timezone

try:
    import serial
except ModuleNotFoundError:
    print(
        "Missing dependency: install it with 'python3 -m pip install pyserial'",
        file=sys.stderr,
    )
    raise SystemExit(2)


FRAME_RE = re.compile(
    r"received:\s*([0-9a-fA-F ]+?)\s+power:\s*(-?\d+)\s+"
    r"lqi:\s*(\d+)\s+time:\s*(\d+)",
    re.IGNORECASE,
)


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/cu.usbmodem11301")
    parser.add_argument("--channel", type=int, default=15)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--jsonl", help="Optional JSON Lines output file")
    args = parser.parse_args()
    if not 11 <= args.channel <= 26:
        parser.error("--channel must be between 11 and 26")
    return args


def send_command(device, command):
    device.write((command + "\r\n").encode("ascii"))
    device.flush()
    time.sleep(0.15)


def main():
    args = arguments()
    output = open(args.jsonl, "a", encoding="utf-8") if args.jsonl else None
    running = True

    def stop(_signum, _frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    try:
        with serial.Serial(args.port, args.baud, timeout=0.5) as device:
            device.reset_input_buffer()
            send_command(device, f"channel {args.channel}")
            send_command(device, "receive")
            print(f"Listening on {args.port}, IEEE 802.15.4 channel {args.channel}")
            print("Press Ctrl-C to stop.\n")

            while running:
                line = device.readline().decode("utf-8", errors="replace").strip()
                if not line:
                    continue

                match = FRAME_RE.search(line)
                if not match:
                    print(f"DEVICE: {line}")
                    continue

                hex_data = "".join(match.group(1).split()).lower()
                record = {
                    "captured_at": datetime.now(timezone.utc).isoformat(),
                    "channel": args.channel,
                    "length": len(hex_data) // 2,
                    "rssi_dbm": int(match.group(2)),
                    "lqi": int(match.group(3)),
                    "device_time": int(match.group(4)),
                    "frame_hex": hex_data,
                }
                print(
                    f"{record['captured_at']}  len={record['length']:3d}  "
                    f"RSSI={record['rssi_dbm']:4d} dBm  LQI={record['lqi']:3d}  "
                    f"{record['frame_hex']}"
                )
                if output:
                    output.write(json.dumps(record) + "\n")
                    output.flush()

            send_command(device, "sleep")
            print("\nCapture stopped.")
    except serial.SerialException as error:
        print(f"Serial error: {error}", file=sys.stderr)
        return 1
    finally:
        if output:
            output.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
