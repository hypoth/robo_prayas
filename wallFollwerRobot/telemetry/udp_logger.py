"""
UDP telemetry logger for the wall-following robot.

Listens for CSV packets sent by wall_follower_udp.ino and:
  - prints them live to the console
  - appends them to a timestamped CSV file for later analysis

Usage:
    python udp_logger.py
    python udp_logger.py --port 4210 --out my_run.csv
"""

import argparse
import csv
import socket
import time

FIELDS = ["seq", "robot_millis", "side_cm", "front_cm",
          "error", "pid_output", "left_speed", "right_speed", "recv_time"]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=4210, help="UDP port to listen on")
    parser.add_argument("--out", type=str, default=None, help="Output CSV filename")
    args = parser.parse_args()

    filename = args.out or f"telemetry_{time.strftime('%Y%m%d_%H%M%S')}.csv"

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", args.port))
    print(f"Listening for UDP telemetry on port {args.port}")
    print(f"Logging to {filename}")
    print("Press Ctrl+C to stop.\n")

    last_seq = None
    dropped = 0
    received = 0

    with open(filename, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(FIELDS)

        try:
            while True:
                data, addr = sock.recvfrom(1024)
                line = data.decode("utf-8", errors="replace").strip()
                parts = line.split(",")

                recv_time = time.time()
                row = parts + [f"{recv_time:.3f}"]
                writer.writerow(row)
                f.flush()  # so data is safe on disk even if the run is interrupted

                received += 1
                try:
                    seq = int(parts[0])
                    if last_seq is not None and seq != last_seq + 1:
                        dropped += seq - last_seq - 1
                    last_seq = seq
                except (ValueError, IndexError):
                    pass

                print(f"{line}  (from {addr[0]}, total={received}, dropped~{dropped})")

        except KeyboardInterrupt:
            print(f"\nStopped. {received} packets received, ~{dropped} dropped.")
            print(f"Saved to {filename}")


if __name__ == "__main__":
    main()
