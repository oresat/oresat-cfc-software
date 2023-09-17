"""Unit tests for the Pirt1280 driver."""

import unittest

from oresat_cfc.drivers.pirt1280 import Pirt1280, pirt1280_raw_to_numpy


class TestPirt1280(unittest.TestCase):
    """Tests the Pirt1280 driver."""

    def test_general(self):
        """Test general functionality."""

        cam = Pirt1280(1, 1, 20, 1, True)
        cam.READ_BACK_WAIT = 0

        cam.enable()
        self.assertTrue(cam.is_enabled)

        cam.disable()
        self.assertFalse(cam.is_enabled)

        cam.enable()

        raw = cam.capture()

        pirt1280_raw_to_numpy(raw)
