#!/usr/bin/env python3
import argparse
import os
import statistics
import sys
import time

CMD_LASER_RANGE = 0x17
HEADER = b"db" + bytes([CMD_LASER_RANGE, 0x00, 0x04, 0x00])
FRAME_LEN = 12
STATUS_NAME = {0: "valid", 1: "sigma fail", 2: "signal fail", 3: "min range clipped", 4: "out of bounds",
               5: "hardware fail", 6: "valid, no wrap check", 7: "wrap target fail", 8: "processing fail",
               255: "no sensor (I2C error)"}


def checksum(body):
    s = sum(body) & 0xFFFF
    return s & 0xFF, s >> 8


def encode_frame(seq, status, range_mm):
    body = HEADER[2:] + bytes([seq & 0xFF, status & 0xFF, range_mm & 0xFF, (range_mm >> 8) & 0xFF])
    a, b = checksum(body)
    return b"db" + body + bytes([a, b])


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
            i = self.buf.find(HEADER)
            if i < 0:
                del self.buf[:-(len(HEADER) - 1)]
                break
            del self.buf[:i]
            if len(self.buf) < FRAME_LEN:
                break
            f = bytes(self.buf[:FRAME_LEN])
            if checksum(f[2:10]) != (f[10], f[11]):
                self.bad_ck += 1
                del self.buf[:1]
                continue
            del self.buf[:FRAME_LEN]
            seq = f[6]
            if self.last_seq is not None:
                self.gaps += (seq - self.last_seq - 1) & 0xFF
            self.last_seq = seq
            out.append((seq, f[7], f[8] | (f[9] << 8)))
        return out


def open_port(port, baud):
    try:
        import serial
    except ImportError:
        serial = None
    if serial:
        state = {"s": None}

        def rd():
            try:
                if state["s"] is None:
                    state["s"] = serial.Serial(port, baud, timeout=0.05)
                    print("# port %s open" % port, flush=True)
                return state["s"].read(max(1, state["s"].in_waiting))
            except (serial.SerialException, OSError) as e:
                if state["s"] is not None:
                    print("# port lost (%s); waiting for it to come back" % str(e).split(":")[0], flush=True)
                    state["s"].close()
                    state["s"] = None
                time.sleep(0.5)
                return b""
        return rd, (lambda: state["s"] and state["s"].close())
    else:
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
    ap.add_argument("--seconds", type=float, default=0, help="stop after this many seconds (default 0: run until Ctrl+C)")
    ap.add_argument("--each", action="store_true", help="print every reading, not a summary per second")
    args = ap.parse_args()
    rd, close = open_port(args.port, args.baud)
    dec = Decoder()
    start = time.monotonic()
    end = start + args.seconds if args.seconds > 0 else float("inf")
    tick = start + 1
    frames, times, base = [], [], (0, 0)
    try:
        while time.monotonic() < end:
            data = rd()
            now = time.monotonic()
            for f in dec.feed(data):
                if args.each:
                    seq, st, rg = f
                    print("%9.3f s  seq %3d  range %5d mm  status %3d %s" % (now - start, seq, rg, st, STATUS_NAME.get(st, "?")), flush=True)
                frames.append(f)
                times.append(now)
            if not args.each and now >= tick:
                report(frames, times, dec, base)
                frames, times, base = [], [], (dec.gaps, dec.bad_ck)
                tick += 1
    except KeyboardInterrupt:
        pass
    close()


if __name__ == "__main__":
    sys.exit(main())
