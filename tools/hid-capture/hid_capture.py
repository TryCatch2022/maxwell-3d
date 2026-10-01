#!/usr/bin/env python3
"""Passively log input reports from the Audeze Maxwell HID interfaces.

Never writes to the device. Prints each report as hex with the bytes that
changed since the previous report of the same ID marked, plus per-report-ID
rates every few seconds. Move/turn your head and watch which report reacts.

    tools/hid-capture/hid_capture.py              # all Maxwell hidraw nodes
    tools/hid-capture/hid_capture.py --only 0a    # just report 0x0A
    tools/hid-capture/hid_capture.py --log cap.txt
"""
import argparse
import glob
import os
import select
import time

VENDOR = "00003329"


def maxwell_nodes():
    nodes = {}
    for path in sorted(glob.glob("/sys/class/hidraw/hidraw*")):
        with open(os.path.join(path, "device/uevent")) as f:
            info = dict(line.strip().split("=", 1) for line in f if "=" in line)
        if info.get("HID_ID", "").split(":")[1:2] == [VENDOR]:
            nodes["/dev/" + os.path.basename(path)] = info.get("HID_NAME", "?")
    return nodes


def fmt(data, prev):
    out = []
    for i, b in enumerate(data):
        changed = prev is not None and i < len(prev) and prev[i] != b
        out.append(f"\033[1;33m{b:02x}\033[0m" if changed else f"{b:02x}")
    return " ".join(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", help="report ID in hex, e.g. 0a")
    ap.add_argument("--log", help="also append raw reports (timestamp, node, hex) to this file")
    ap.add_argument("--quiet", action="store_true", help="only print rate summaries")
    args = ap.parse_args()
    only = int(args.only, 16) if args.only else None

    nodes = maxwell_nodes()
    if not nodes:
        raise SystemExit("no Audeze hidraw node found")
    fds = {}
    for node, name in nodes.items():
        try:
            fds[os.open(node, os.O_RDONLY | os.O_NONBLOCK)] = (node, name)
            print(f"reading {node} ({name})")
        except PermissionError:
            print(f"no permission on {node} ({name}): install udev/70-audeze-maxwell.rules")
    if not fds:
        raise SystemExit(1)

    log = open(args.log, "a") if args.log else None
    prev, counts = {}, {}
    last_summary = time.monotonic()
    t0 = last_summary
    while True:
        ready, _, _ = select.select(list(fds), [], [], 1.0)
        now = time.monotonic()
        for fd in ready:
            node, _ = fds[fd]
            try:
                data = os.read(fd, 128)
            except OSError as e:
                print(f"{node}: {e}")
                del fds[fd]
                continue
            if not data or (only is not None and data[0] != only):
                continue
            key = (node, data[0])
            counts[key] = counts.get(key, 0) + 1
            if not args.quiet:
                print(f"{now - t0:9.3f} {node[5:]} id={data[0]:02x} len={len(data):2d} {fmt(data, prev.get(key))}")
            if log:
                log.write(f"{now - t0:.4f} {node} {data.hex()}\n")
            prev[key] = data
        if now - last_summary >= 5.0:
            rates = ", ".join(f"{n[5:]}/{i:02x}: {c / (now - last_summary):.1f} Hz" for (n, i), c in sorted(counts.items()))
            print(f"--- rates: {rates or 'nothing received'}")
            counts.clear()
            last_summary = now
        if not fds:
            raise SystemExit("all nodes gone")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
