#!/usr/bin/env python3
"""Stream a scene to a shadowgraph device over UDP.

Encodes laser commands in the same wire format the firmware decodes
(see components/scene_stream + components/laser_command) and streams them with
per-command sequence numbers and blind redundant re-sends: each datagram is sent
once, then the most recent few datagrams are re-sent for free, so the device can
recover dropped packets (it dedups by sequence number).

Default scene is the morphing Lissajous "ballywhoop", so a network-driven render
looks like the on-device idle demo.

Usage:
    python3 tools/stream_scene.py --host 192.168.4.1 --port 7777
    python3 tools/stream_scene.py --redundancy 3      # resend each packet 3x
"""
import argparse
import math
import random
import socket
import struct
import time

# --- wire format (must match components/scene_stream/include/scene_stream.h) --
MAGIC = 0x5347
VERSION = 1
HDR = struct.Struct("<HBBIIHH")  # magic, ver, flags, stream_id, base_seq, count, payload_len
FLAG_KEYFRAME = 0x01

# command type tags (components/laser_command)
CMD_GOTO = 0x01
CMD_LASER = 0x02
CMD_DWELL = 0x03

GALVO_CENTER = 0x8000
GALVO_AMPLITUDE = 13107          # ~20% full scale, galvo linear region
POINTS_PER_LOOP = 256
POINT_DWELL_US = 50
INTENSITY = 0.25
COLOR_EVERY = 16

MAX_PAYLOAD = 1400               # keep datagrams under the SoftAP MTU


def enc_goto(x, y):
    return struct.pack("<BHH", CMD_GOTO, x & 0xFFFF, y & 0xFFFF)


def enc_laser(r, g, b):
    return struct.pack("<BHHH", CMD_LASER, r & 0xFFFF, g & 0xFFFF, b & 0xFFFF)


def enc_dwell(dt):
    return struct.pack("<BI", CMD_DWELL, dt & 0xFFFFFFFF)


def hsv_to_rgb16(h):
    """h in degrees; S=1, V=INTENSITY -> three u16."""
    c = INTENSITY
    hp = (h % 360.0) / 60.0
    x = c * (1.0 - abs((hp % 2.0) - 1.0))
    rf, gf, bf = 0.0, 0.0, 0.0
    if hp < 1:   rf, gf = c, x
    elif hp < 2: rf, gf = x, c
    elif hp < 3: gf, bf = c, x
    elif hp < 4: gf, bf = x, c
    elif hp < 5: rf, bf = x, c
    else:        rf, bf = c, x
    return int(rf * 0xFFFF), int(gf * 0xFFFF), int(bf * 0xFFFF)


def scene_commands():
    """Infinite generator of encoded commands for the morphing Lissajous."""
    two_pi = 2.0 * math.pi
    t_step = two_pi / POINTS_PER_LOOP
    point_period_s = POINT_DWELL_US * 1e-6
    t = phase = hue = 0.0
    color_div = 0
    while True:
        x = GALVO_CENTER + int(GALVO_AMPLITUDE * math.sin(3.0 * t))
        y = GALVO_CENTER + int(GALVO_AMPLITUDE * math.sin(2.0 * t + phase))
        yield enc_goto(x, y)
        if color_div == 0:
            yield enc_laser(*hsv_to_rgb16(hue))
        color_div = (color_div + 1) % COLOR_EVERY
        yield enc_dwell(POINT_DWELL_US)

        t = (t + t_step) % two_pi
        phase = (phase + 1.0 * point_period_s) % two_pi
        hue = (hue + 50.0 * point_period_s) % 360.0


def build_packet(stream_id, base_seq, cmds, keyframe=False):
    payload = b"".join(cmds)
    flags = FLAG_KEYFRAME if keyframe else 0
    hdr = HDR.pack(MAGIC, VERSION, flags, stream_id, base_seq, len(cmds), len(payload))
    return hdr + payload


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="192.168.4.1")
    ap.add_argument("--port", type=int, default=7777)
    ap.add_argument("--redundancy", type=int, default=2,
                    help="extra copies of recent packets sent for loss recovery")
    ap.add_argument("--lead-ms", type=float, default=40.0,
                    help="how far ahead of playback to stay buffered")
    ap.add_argument("--cmds-per-packet", type=int, default=64)
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dst = (args.host, args.port)
    stream_id = random.getrandbits(32)
    print(f"streaming to {args.host}:{args.port}  stream_id=0x{stream_id:08x}  "
          f"redundancy={args.redundancy}")

    gen = scene_commands()
    seq = 0
    recent = []                         # ring of recently sent datagrams (for resends)
    first = True

    # Pace by the scene's own dwell clock: advance a virtual playback time and
    # only get `lead_ms` ahead of wall-clock, so we feed the device without
    # overflowing its reassembly window.
    start = time.monotonic()
    play_us = 0                         # virtual playback time emitted so far
    sent = 0
    try:
        while True:
            cmds = []
            pkt_us = 0
            payload_len = 0
            while len(cmds) < args.cmds_per_packet:
                c = next(gen)
                if payload_len + len(c) > MAX_PAYLOAD:
                    break
                cmds.append(c)
                payload_len += len(c)
                if c[0] == CMD_DWELL:
                    pkt_us += struct.unpack_from("<I", c, 1)[0]

            pkt = build_packet(stream_id, seq, cmds, keyframe=first)
            first = False
            sock.sendto(pkt, dst)
            sent += 1

            # Blind redundancy: re-send the most recent packets.
            for old in recent[-args.redundancy:]:
                sock.sendto(old, dst)
            recent.append(pkt)
            if len(recent) > 64:
                recent.pop(0)

            seq += len(cmds)
            play_us += pkt_us

            # Sleep so we stay ~lead_ms ahead of real playback.
            ahead_s = play_us * 1e-6 - (time.monotonic() - start)
            if ahead_s > args.lead_ms * 1e-3:
                time.sleep(ahead_s - args.lead_ms * 1e-3)

            if sent % 200 == 0:
                print(f"  sent {sent} packets, seq={seq}")
    except KeyboardInterrupt:
        print("\nstopped")


if __name__ == "__main__":
    main()
