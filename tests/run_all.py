#!/usr/bin/env python3
"""Run all tests."""

import unittest
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

loader = unittest.TestLoader()
suite = unittest.TestSuite()

# Discover all tests
suite.addTests(loader.discover(os.path.dirname(__file__), pattern="test_*.py"))

runner = unittest.TextTestRunner(verbosity=2)
result = runner.run(suite)
sys.exit(0 if result.wasSuccessful() else 1)
