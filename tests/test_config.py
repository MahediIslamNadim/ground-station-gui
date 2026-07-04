import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

import json
import tempfile
import unittest
from nexcore_ground_station.config import load_config, save_config, DEFAULT_CONFIG


class TestConfig(unittest.TestCase):

    def test_default_config(self):
        config = load_config("nonexistent.json")
        self.assertEqual(config["serial"]["baud"], 115200)
        self.assertEqual(config["display"]["theme"], "dark")

    def test_save_and_load(self):
        with tempfile.NamedTemporaryFile(mode="w", suffix=".json", delete=False) as f:
            path = f.name
        try:
            custom = {"serial": {"baud": 57600}}
            save_config(custom, path)
            loaded = load_config(path)
            self.assertEqual(loaded["serial"]["baud"], 57600)
        finally:
            os.unlink(path)

    def test_deep_merge(self):
        with tempfile.NamedTemporaryFile(mode="w", suffix=".json", delete=False) as f:
            json.dump({"display": {"update_interval_ms": 100}}, f)
            path = f.name
        try:
            config = load_config(path)
            self.assertEqual(config["display"]["update_interval_ms"], 100)
            self.assertEqual(config["serial"]["baud"], 115200)
        finally:
            os.unlink(path)


if __name__ == "__main__":
    unittest.main()
