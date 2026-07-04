import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

import struct
import unittest
from nexcore_ground_station import MAVLink


class TestMAVLink(unittest.TestCase):

    def test_parse_frame_invalid_short(self):
        self.assertIsNone(MAVLink.parse_frame(b""))
        self.assertIsNone(MAVLink.parse_frame(b"\xFE\x01"))

    def test_parse_frame_invalid_header(self):
        self.assertIsNone(MAVLink.parse_frame(b"\x00\x01\x02\x03\x04\x05\x06"))

    def test_parse_frame_heartbeat(self):
        payload = struct.pack("<BBBI", 1, 2, 3, 4) + b"\x05\x06"
        length = len(payload)
        calc = 0
        for b in payload:
            calc ^= b
        frame = b"\xFE" + bytes([length, 0, 1, 2, 3]) + payload + bytes([calc])
        result = MAVLink.parse_frame(frame)
        self.assertIsNotNone(result)
        self.assertEqual(result["msgid"], 3)
        self.assertEqual(result["sysid"], 1)
        self.assertEqual(result["compid"], 2)
        self.assertEqual(result["payload"], payload)

    def test_parse_frame_bad_checksum(self):
        payload = b"\x01\x02\x03"
        frame = b"\xFE\x03\x00\x01\x02\x00" + payload + b"\xFF"
        self.assertIsNone(MAVLink.parse_frame(frame))

    def test_decode_heartbeat(self):
        p = struct.pack("<BBBI", 1, 2, 3, 4) + b"\x05\x06"
        hb = MAVLink.decode_heartbeat(p)
        self.assertEqual(hb["type"], 1)
        self.assertEqual(hb["autopilot"], 2)
        self.assertEqual(hb["base_mode"], 3)
        self.assertEqual(hb["custom_mode"], 4)
        self.assertEqual(hb["system_status"], 5)
        self.assertEqual(hb["mavlink_version"], 6)

    def test_decode_heartbeat_short(self):
        self.assertEqual(MAVLink.decode_heartbeat(b"\x01"), {})

    def test_decode_sys_status(self):
        p = struct.pack("<hhBB", 5000, 300, 80, 1) + b"\x00" * 10 + struct.pack("<HB", 45, 0)
        ss = MAVLink.decode_sys_status(p)
        self.assertAlmostEqual(ss["voltage"], 5.0, places=3)
        self.assertAlmostEqual(ss["current"], 3.0, places=3)
        self.assertEqual(ss["remaining"], 80)
        self.assertEqual(ss["armed"], 1)
        self.assertEqual(ss["load"], 45)
        self.assertEqual(ss["failsafe"], 0)

    def test_decode_sys_status_short(self):
        self.assertEqual(MAVLink.decode_sys_status(b"\x01\x02"), {})

    def test_known_messages(self):
        self.assertEqual(MAVLink.KNOWN_MSGS[0], "HEARTBEAT")
        self.assertEqual(MAVLink.KNOWN_MSGS[30], "ATTITUDE")
        self.assertEqual(MAVLink.KNOWN_MSGS[36], "SERVO_OUTPUT_RAW")


if __name__ == "__main__":
    unittest.main()
