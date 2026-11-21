#!/usr/bin/env python3
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import view_lidar as v


class T(unittest.TestCase):
    def test_ec_l1_roundtrip(self):
        d = v.Decoder()
        fr = [(0, 0, 1234), (1, 4, 0), (2, 255, 65535)]
        out = d.feed(b"".join(v.encode_frame(*f) for f in fr))
        self.assertEqual(out, fr)
        self.assertEqual(len(v.encode_frame(0, 0, 0)), 12)
        self.assertEqual(v.encode_frame(1, 0, 0x0102), bytes([0x64, 0x62, 0x17, 0, 4, 0, 1, 0, 2, 1, 0x1F, 0]))

    def test_ec_l2_bad_checksum(self):
        d = v.Decoder()
        bad = bytearray(v.encode_frame(5, 0, 100))
        bad[10] ^= 1
        out = d.feed(bytes(bad) + v.encode_frame(6, 0, 101))
        self.assertEqual(out, [(6, 0, 101)])
        self.assertEqual(d.bad_ck, 1)

    def test_ec_l3_resync(self):
        d = v.Decoder()
        data = b"\x00db\x17boot garbage db\x03\x01\x01\x00\x09\x0d\x00" + v.encode_frame(1, 0, 10) + b"d\xffdb\x17\x00" + v.encode_frame(2, 0, 20)
        self.assertEqual([f[0] for f in d.feed(data)], [1, 2])

    def test_ec_l4_seq_wrap(self):
        d = v.Decoder()
        d.feed(v.encode_frame(255, 0, 1) + v.encode_frame(0, 0, 1))
        self.assertEqual(d.gaps, 0)

    def test_ec_l5_gap(self):
        d = v.Decoder()
        d.feed(v.encode_frame(10, 0, 1) + v.encode_frame(13, 0, 1))
        self.assertEqual(d.gaps, 2)

    def test_ec_l6_split_feed(self):
        d = v.Decoder()
        f = v.encode_frame(9, 0, 300)
        self.assertEqual(d.feed(f[:3]), [])
        self.assertEqual(d.feed(f[3:]), [(9, 0, 300)])


if __name__ == "__main__":
    unittest.main()
