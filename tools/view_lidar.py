#!/usr/bin/env python3
import argparse
import os
import statistics
import sys
import time

SYNC = b"\xa5\x5a"
FRAME_LEN = 8


def checksum(payload):
    a = b = 0
    for x in payload:
        a = (a + x) & 0xFF
        b = (b + a) & 0xFF
    return a, b


def encode_frame(seq, status, range_mm):
    body = bytes([seq & 0xFF, status & 0xFF, range_mm & 0xFF, (range_mm >> 8) & 0xFF])
    a, b = checksum(body)
    return SYNC + body + bytes([a, b])


class Decoder:
    def __init__(self):
        self.buf = bytearray()
        self.last_seq = None
        self.gaps = 0
        self.bad_ck = 0

    def feed(self, data):
        self.buf += data
        out = []
        while True:
            i = self.buf.find(SYNC)
            if i < 0:
                del self.buf[:-1]
                break
            del self.buf[:i]
            if len(self.buf) < FRAME_LEN:
                break
            f = bytes(self.buf[:FRAME_LEN])
            if checksum(f[2:6]) != (f[6], f[7]):
                self.bad_ck += 1
                del self.buf[:1]
                continue
            del self.buf[:FRAME_LEN]
            seq = f[2]
            if self.last_seq is not None:
                self.gaps += (seq - self.last_seq - 1) & 0xFF
            self.last_seq = seq
            out.append((seq, f[3], f[4] | (f[5] << 8)))
        return out


def open_port(port, baud):
    try:
        import serial
        s = serial.Serial(port, baud, timeout=0.05)
        return (lambda: s.read(4096)), s.close
    except ImportError:
        import termios
        fd = os.open(port, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
        a = termios.tcgetattr(fd)
        a[0] = 0
        a[1] = 0
        a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        a[3] = 0
        a[4] = a[5] = getattr(termios, "B%d" % baud, baud)
        a[6][termios.VMIN] = 0
        a[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, a)

        def rd():
            import select
            select.select([fd], [], [], 0.05)
            try:
                return os.read(fd, 4096)
            except BlockingIOError:
                return b""
        return rd, (lambda: os.close(fd))


def pct(v, p):
    v = sorted(v)
    return v[min(len(v) - 1, int(p * len(v)))]


def report(frames, times, dec, base):
    st = {}
    for _, s, _ in frames:
        st[s] = st.get(s, 0) + 1
    rg = [r for _, _, r in frames]
    dt = [(b - a) * 1000 for a, b in zip(times, times[1:])]
    line = "fps=%d gaps=%d badck=%d status=%s" % (len(frames), dec.gaps - base[0], dec.bad_ck - base[1], st)
    if rg:
        line += " range min/mean/max=%d/%d/%d" % (min(rg), statistics.mean(rg), max(rg))
    if dt:
        line += " dt_ms p50/p99=%.1f/%.1f" % (pct(dt, 0.5), pct(dt, 0.99))
    print(line, flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--seconds", type=float, default=10)
    args = ap.parse_args()
    rd, close = open_port(args.port, args.baud)
    dec = Decoder()
    end = time.monotonic() + args.seconds
    tick = time.monotonic() + 1
    frames, times, base = [], [], (0, 0)
    while time.monotonic() < end:
        data = rd()
        now = time.monotonic()
        for f in dec.feed(data):
            frames.append(f)
            times.append(now)
        if now >= tick:
            report(frames, times, dec, base)
            frames, times, base = [], [], (dec.gaps, dec.bad_ck)
            tick += 1
    close()


if __name__ == "__main__":
    sys.exit(main())
