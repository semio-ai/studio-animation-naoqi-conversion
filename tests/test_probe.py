import argparse
import contextlib
import io
import sys
import time
import unittest
from pathlib import Path


PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))
import probe_naoqi_leds as probe
from test_conversion import FakeMotionService
from test_leds import FakeLedService


class ScaledTime:
    """Stands in for the time module so the probe's waits pass quickly."""

    def __init__(self, factor):
        self.factor = factor
        self.origin = time.time()

    def time(self):
        return self.origin + (time.time() - self.origin) * self.factor

    def sleep(self, seconds):
        time.sleep(seconds / self.factor)


class FakeFuture:
    def __init__(self):
        self.cancelled = False

    def cancel(self):
        self.cancelled = True


class SimulatedLeds(FakeLedService):
    """A fake ALLeds whose commands set the intensities they aim for."""

    def __init__(self):
        FakeLedService.__init__(self)
        self.intensity = dict((name, 0.0) for name in self.devices)

    def devices_of(self, name):
        if name in self.intensity:
            return [name]
        if name == "AllLeds":
            return list(self.devices)
        return self.listed[name]

    def getIntensity(self, name):
        return self.intensity[name]

    def setIntensity(self, name, value):
        for device in self.devices_of(name):
            self.intensity[device] = value

    def fade(self, name, intensity, duration, _async=False):
        FakeLedService.fade(self, name, intensity, duration)
        self.setIntensity(name, intensity)
        return FakeFuture() if _async else None

    def fadeRGB(self, name, color, duration):
        self.fadeListRGB(name, [color], [duration])

    def fadeListRGB(self, name, colors, times):
        FakeLedService.fadeListRGB(self, name, colors, times)
        for device in self.devices_of(name):
            # Single-colour devices follow the blue byte in this simulation.
            shift = 16 if "/Red/" in device else 8 if "/Green/" in device else 0
            self.intensity[device] = ((colors[-1] >> shift) & 0xFF) / 255.0

    def reset(self, name):
        self.setIntensity(name, 0.0)


class AwakeMotionService(FakeMotionService):
    def robotIsWakeUp(self):
        return True

    def getAngles(self, names, use_sensors):
        return [0.0]


class ProbeTests(unittest.TestCase):
    def test_every_check_runs_against_a_simulated_robot(self):
        args = argparse.Namespace(checks=[], allow_motion=True)
        output = io.StringIO()
        real_time = probe.time
        probe.time = ScaledTime(40)
        try:
            with contextlib.redirect_stdout(output):
                probe.run(SimulatedLeds(), AwakeMotionService(), args)
        finally:
            probe.time = real_time
        text = output.getvalue()
        for number in range(1, 8):
            self.assertIn("\n{}. ".format(number), text)
        self.assertIn("listGroup('AllLeds') returns 89 devices; the converter knows 89.", text)
        self.assertEqual(text.count("): ok"), 19)
        self.assertIn("ALMotion.angleInterpolationBezier", text)
        self.assertRegex(text, r"Ear intensity first moved \d+\.\d+ s after the first call")
        self.assertIn("HeadYaw did not move", text)


if __name__ == "__main__":
    unittest.main()
