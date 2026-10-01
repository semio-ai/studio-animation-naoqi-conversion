import bisect
import json
import random
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path


PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))
from semio_naoqi_motion import (
    LED_DEVICES, LED_MIN_INTERVAL_SECONDS, RGB_LED_POSITIONS, MotionFormatError,
    convert_motion, play_motion, prepare_motion, start_motion,
)
from test_conversion import FakeMotionService, cubic, sample_input


CHEST = tuple("ChestBoard/Led/{}/Actuator/Value".format(color)
              for color in ("Red", "Green", "Blue"))
EAR = "Ears/Led/Right/0Deg/Actuator/Value"


def documented_v6_leds():
    # The device names in Aldebaran's 2.8 actuator list for NAO V6.
    names = set()
    for color in ("Red", "Green", "Blue"):
        for side in ("Left", "Right"):
            for angle in (0, 45, 90, 135, 180, 225, 270, 315):
                names.add("Face/Led/{}/{}/{}Deg/Actuator/Value".format(color, side, angle))
        names.add("ChestBoard/Led/{}/Actuator/Value".format(color))
        names.add("LFoot/Led/{}/Actuator/Value".format(color))
        names.add("RFoot/Led/{}/Actuator/Value".format(color))
    for side in ("Left", "Right"):
        for angle in (0, 36, 72, 108, 144, 180, 216, 252, 288, 324):
            names.add("Ears/Led/{}/{}Deg/Actuator/Value".format(side, angle))
        for place in ("Front/{}/0", "Front/{}/1", "Middle/{}/0",
                      "Rear/{}/0", "Rear/{}/1", "Rear/{}/2"):
            names.add("Head/Led/{}/Actuator/Value".format(place.format(side)))
    return names


def linear(points):
    """Keys for a straight line through ``(timeMs, value)`` points."""
    keys = []
    for index, (stamp, value) in enumerate(points):
        key = {"timeMs": stamp, "value": value}
        if index > 0:
            previous_stamp, previous_value = points[index - 1]
            key["in"] = {"deltaTimeMs": (previous_stamp - stamp) / 3.0,
                         "deltaValue": (previous_value - value) / 3.0}
        if index < len(points) - 1:
            next_stamp, next_value = points[index + 1]
            key["out"] = {"deltaTimeMs": (next_stamp - stamp) / 3.0,
                          "deltaValue": (next_value - value) / 3.0}
        keys.append(key)
    return keys


def eased(points):
    """Keys with flat handles a third of the way along each segment."""
    keys = linear(points)
    for key in keys:
        for side in ("in", "out"):
            if side in key:
                key[side]["deltaValue"] = 0.0
    return keys


def export(channels):
    data = sample_input()
    data["channels"] = channels
    return data


def led(output, keys):
    return {"output": output, "units": "%", "keys": keys}


def head_yaw(points):
    return {"output": "HeadYaw", "units": "rad", "keys": eased(points)}


def streams_by_name(result):
    return dict((stream["name"], stream) for stream in result["leds"])


class FakeLedService:
    """Answers ALLeds' listing calls as the 2.8 docs describe a NAO V6.

    ``fade`` and ``fadeListRGB`` are recorded with the time they were issued.
    With ``blocking``, they return when their last target is reached.
    """

    GROUPS = ("ChestLeds", "LeftFootLeds", "RightFootLeds")

    def __init__(self, devices=None, listed=None, blocking=False, failing=None):
        self.devices = sorted(documented_v6_leds()) if devices is None else list(devices)
        self.listed = dict((name, list(channels)) for name, _, channels in RGB_LED_POSITIONS)
        self.listed.update(listed or {})
        self.blocking = blocking
        self.failing = failing
        self.calls = []
        self.started = time.time()
        self.lock = threading.Lock()

    def listGroup(self, name):
        if name == "AllLeds":
            return list(self.devices)
        assert name in self.GROUPS, name
        return self.listed[name]

    def listLED(self, name):
        assert name not in self.GROUPS, name
        return self.listed[name]

    def _record(self, *call):
        with self.lock:
            self.calls.append((time.time() - self.started,) + call)
        if call[0] == self.failing:
            raise RuntimeError("ALLeds refused {}".format(call[0]))

    def fade(self, name, intensity, duration):
        self._record("fade", name, intensity, duration)
        if self.blocking:
            time.sleep(duration)

    def fadeListRGB(self, name, colors, times):
        self._record("fadeListRGB", name, colors, times)
        if self.blocking:
            time.sleep(times[-1])

    def named(self, method, name):
        return [call for call in self.calls if call[1] == method and call[2] == name]


class BlockingMotionService(FakeMotionService):
    """Blocks in angleInterpolationBezier until its last key or a kill."""

    def __init__(self, awake=True):
        FakeMotionService.__init__(self)
        self.awake = awake
        self.killed = threading.Event()
        self.kills = []

    def angleInterpolationBezier(self, names, times, keys):
        FakeMotionService.angleInterpolationBezier(self, names, times, keys)
        self.killed.wait(max(channel[-1] for channel in times))
        return "killed" if self.killed.is_set() else "finished"

    def killTasksUsingResources(self, names):
        self.kills.append(list(names))
        self.killed.set()

    def robotIsWakeUp(self):
        return self.awake


class LedConversionTests(unittest.TestCase):
    def test_led_table_is_the_documented_v6_device_set(self):
        self.assertEqual(set(LED_DEVICES), documented_v6_leds())
        self.assertEqual(len(LED_DEVICES), 89)
        positions = dict((name, (lookup, channels))
                         for name, lookup, channels in RGB_LED_POSITIONS)
        self.assertEqual(len(positions), 19)
        # Short names count both eyes from 0Deg; the FaceLed groups mirror them.
        self.assertEqual(positions["RightFaceLed1"][1][0],
                         "Face/Led/Red/Right/0Deg/Actuator/Value")
        self.assertEqual(positions["RightFaceLed8"][1][2],
                         "Face/Led/Blue/Right/315Deg/Actuator/Value")
        self.assertEqual(positions["LeftFaceLed3"][1][1],
                         "Face/Led/Green/Left/90Deg/Actuator/Value")
        self.assertEqual(positions["ChestLeds"], ("listGroup", CHEST))
        self.assertEqual(positions["LeftFootLeds"][0], "listGroup")

    def test_joint_only_output_is_unchanged(self):
        self.assertEqual(sorted(convert_motion(sample_input())), ["keys", "names", "times"])

    def test_joints_and_leds_share_one_time_zero(self):
        result = convert_motion(export([
            head_yaw([(400, 0.0), (1000, 0.3)]),
            led(EAR, linear([(0, 0.0), (1000, 1.0)])),
        ]))
        self.assertEqual(result["names"], ["HeadYaw"])
        self.assertAlmostEqual(result["times"][0][0], 0.6)
        ear = streams_by_name(result)[EAR]
        self.assertAlmostEqual(ear["timeList"][0], 0.2)
        self.assertAlmostEqual(ear["timeList"][-1], 1.2)

    def test_complete_rgb_position_becomes_one_fade_list(self):
        result = convert_motion(export([
            led(CHEST[0], linear([(0, 0.0), (1000, 1.0)])),
            led(CHEST[1], linear([(0, 0.5), (1000, 0.5)])),
            led(CHEST[2], linear([(0, 0.0), (250, 0.0), (1000, 1.0)])),
        ]))
        self.assertEqual(result["names"], [])
        self.assertEqual(len(result["leds"]), 1)
        chest = result["leds"][0]
        self.assertEqual(chest["method"], "fadeListRGB")
        self.assertEqual(chest["name"], "ChestLeds")
        self.assertEqual(chest["devices"], list(CHEST))
        # Sampled at the union of the channels' breakpoints, packed 0x00RRGGBB.
        for actual, expected in zip(chest["timeList"], [0.2, 0.45, 1.2]):
            self.assertAlmostEqual(actual, expected)
        self.assertEqual(chest["rgbList"], [0x008000, 0x408000, 0xFF80FF])

    def test_partial_rgb_position_fades_only_its_animated_channels(self):
        result = convert_motion(export([led(CHEST[0], linear([(0, 0.2), (800, 0.9)]))]))
        self.assertEqual(result["leds"], [{
            "method": "fade",
            "name": CHEST[0],
            "intensityList": [0.2, 0.9],
            "timeList": [0.2, 1.0],
        }])

    def test_single_colour_led_becomes_a_fade_chain(self):
        result = convert_motion(export([
            led(EAR, linear([(0, 0.0), (300, 1.0), (900, 0.25)])),
        ]))
        ear = streams_by_name(result)[EAR]
        self.assertEqual(ear["method"], "fade")
        self.assertEqual(ear["intensityList"], [0.0, 1.0, 0.25])
        for actual, expected in zip(ear["timeList"], [0.2, 0.5, 1.1]):
            self.assertAlmostEqual(actual, expected)

    def test_flattened_curves_follow_the_bezier_within_tolerance(self):
        generator = random.Random(7)
        for trial in range(60):
            stamps = [0]
            for _ in range(generator.randint(1, 4)):
                stamps.append(stamps[-1] + generator.randint(200, 3000))
            keys = []
            for index, stamp in enumerate(stamps):
                key = {"timeMs": stamp, "value": generator.random()}
                if index > 0:
                    span = stamp - stamps[index - 1]
                    key["in"] = {"deltaTimeMs": -generator.uniform(0, span),
                                 "deltaValue": generator.uniform(-0.6, 0.6)}
                if index < len(stamps) - 1:
                    span = stamps[index + 1] - stamp
                    key["out"] = {"deltaTimeMs": generator.uniform(0, span),
                                  "deltaValue": generator.uniform(-0.6, 0.6)}
                keys.append(key)
            device = generator.choice(sorted(LED_DEVICES))
            # A whole RGB position animates as one fadeListRGB, a lone channel as fades.
            position = [channels for _, _, channels in RGB_LED_POSITIONS if device in channels]
            outputs = position[0] if position and trial % 2 else (device,)
            with self.subTest(trial=trial, device=device):
                result = convert_motion(export([led(output, keys) for output in outputs]),
                                        lead_in_seconds=1.0)
                self.assertEqual(len(result["leds"]), 1)
                self.assert_follows(result["leds"][0], device, keys)

    def assert_follows(self, stream, device, keys):
        times = stream["timeList"]
        if stream["method"] == "fadeListRGB":
            shift = 16 - 8 * stream["devices"].index(device)
            values = [((color >> shift) & 0xFF) / 255.0 for color in stream["rgbList"]]
            tolerance = LED_DEVICES[device] + 0.5 / 255
        else:
            self.assertEqual(stream["name"], device)
            values = stream["intensityList"]
            tolerance = LED_DEVICES[device]
        key_times = [1.0 + key["timeMs"] / 1000.0 for key in keys]

        def is_key(stamp):
            return any(abs(stamp - key_time) < 1e-9 for key_time in key_times)

        self.assertTrue(all(is_key(stamp) for stamp in key_times[:1] + key_times[-1:]))
        for start, end in zip(times, times[1:]):
            if not (is_key(start) and is_key(end)):
                self.assertGreaterEqual(end - start, LED_MIN_INTERVAL_SECONDS - 1e-6)
        for index in range(len(keys) - 1):
            start, end = keys[index], keys[index + 1]
            xs = [key_times[index], key_times[index] + start["out"]["deltaTimeMs"] / 1000.0,
                  key_times[index + 1] + end["in"]["deltaTimeMs"] / 1000.0, key_times[index + 1]]
            ys = [start["value"], start["value"] + start["out"]["deltaValue"],
                  end["value"] + end["in"]["deltaValue"], end["value"]]
            for step in range(201):
                fraction = step / 200.0
                stamp = cubic(*(xs + [fraction]))
                expected = min(1.0, max(0.0, cubic(*(ys + [fraction]))))
                after = min(max(bisect.bisect_right(times, stamp), 1), len(times) - 1)
                if times[after] - times[after - 1] < 2 * LED_MIN_INTERVAL_SECONDS:
                    continue  # Too short to split: the spacing wins over tolerance.
                share = (stamp - times[after - 1]) / (times[after] - times[after - 1])
                actual = values[after - 1] + (values[after] - values[after - 1]) * share
                # Probes between breakpoints can miss a clamped corner by a little.
                self.assertLessEqual(abs(actual - expected), 1.5 * tolerance)

    def test_overshooting_curves_are_clamped_and_holds_collapse(self):
        keys = [
            {"timeMs": 0, "value": 0.8, "out": {"deltaTimeMs": 300, "deltaValue": 0.9}},
            {"timeMs": 1000, "value": 1.0, "in": {"deltaTimeMs": -300, "deltaValue": 0.0},
             "out": {"deltaTimeMs": 300, "deltaValue": 0.0}},
            {"timeMs": 2000, "value": 1.0, "in": {"deltaTimeMs": -300, "deltaValue": 0.0},
             "out": {"deltaTimeMs": 300, "deltaValue": 0.0}},
            {"timeMs": 3000, "value": 1.0, "in": {"deltaTimeMs": -300, "deltaValue": 0.0}},
        ]
        ear = streams_by_name(convert_motion(export([led(EAR, keys)])))[EAR]
        self.assertTrue(all(0.0 <= value <= 1.0 for value in ear["intensityList"]))
        # The flat stretch from the clamp to the last key needs only its ends.
        self.assertEqual(ear["intensityList"][-2:], [1.0, 1.0])
        self.assertAlmostEqual(ear["timeList"][-1], 3.2)
        self.assertLess(ear["timeList"][-2], 1.2)

    def test_rejects_invalid_led_channels(self):
        def overshoot_out(data):
            data["channels"][0]["keys"][0]["out"]["deltaTimeMs"] = 1200

        def overshoot_in(data):
            data["channels"][0]["keys"][1]["in"]["deltaTimeMs"] = -1001

        changes = [
            (lambda data: data["channels"][0].update(units="rad"), "must be '%' for LED"),
            (lambda data: data["channels"][0].update(units="dimensionless"), "must be '%' for LED"),
            (lambda data: data["channels"][0]["keys"][1].update(value=1.5), "LED intensity"),
            (lambda data: data["channels"][0]["keys"][1].update(value=65), "LED intensity"),
            (lambda data: data["channels"][0]["keys"][0].update(value=-0.01), "LED intensity"),
            (overshoot_out, "within their segment"),
            (overshoot_in, "within their segment"),
        ]
        for change, message in changes:
            with self.subTest(message=message):
                data = export([led(EAR, linear([(0, 0.0), (1000, 1.0)]))])
                change(data)
                with self.assertRaisesRegex(MotionFormatError, message):
                    convert_motion(data)

    def test_leds_false_leaves_leds_out_without_moving_the_joints(self):
        data = export([
            head_yaw([(400, 0.0), (1000, 0.3)]),
            led(EAR, linear([(0, 0.0), (1000, 1.0)])),
        ])
        with_leds = convert_motion(data)
        without = convert_motion(data, leds=False)
        self.assertNotIn("leds", without)
        self.assertEqual(without["times"], with_leds["times"])
        self.assertEqual(without["keys"], with_leds["keys"])
        data["channels"][1]["units"] = "rad"
        with self.assertRaisesRegex(MotionFormatError, "must be '%' for LED"):
            convert_motion(data, leds=False)

    def test_leds_false_refuses_an_led_only_export(self):
        with self.assertRaisesRegex(MotionFormatError, "only LED channels"):
            convert_motion(export([led(EAR, linear([(0, 0.0), (1000, 1.0)]))]), leds=False)

    def test_cli_writes_led_streams_unless_told_not_to(self):
        data = export([
            head_yaw([(0, 0.0), (1000, 0.3)]),
            led(EAR, linear([(0, 0.0), (1000, 1.0)])),
        ])
        with tempfile.TemporaryDirectory() as directory:
            input_path = Path(directory) / "input.json"
            input_path.write_text(json.dumps(data), encoding="utf-8")
            command = [sys.executable,
                       str(PROJECT / "convert_semio_effective_motion_to_naoqi_bezier.py"),
                       str(input_path)]
            full = json.loads(subprocess.run(
                command, capture_output=True, text=True, check=True).stdout)
            joints = json.loads(subprocess.run(
                command + ["--no-leds"], capture_output=True, text=True, check=True).stdout)
        self.assertEqual([stream["name"] for stream in full["leds"]], [EAR])
        self.assertEqual(sorted(joints), ["keys", "names", "times"])
        self.assertEqual(joints["times"], full["times"])


class LedPlaybackTests(unittest.TestCase):
    def mixed_motion(self):
        return prepare_motion(export([
            head_yaw([(0, 0.0), (200, 0.2)]),
            led(CHEST[0], linear([(0, 0.0), (200, 1.0)])),
            led(CHEST[1], linear([(0, 0.0), (200, 0.0)])),
            led(CHEST[2], linear([(0, 1.0), (200, 0.0)])),
            led(EAR, linear([(0, 0.0), (60, 1.0), (120, 0.0), (200, 1.0)])),
        ]), lead_in_seconds=0.1)

    def test_motion_with_leds_needs_an_led_service(self):
        motion = FakeMotionService()
        with self.assertRaisesRegex(MotionFormatError, "leds=False"):
            play_motion(motion, self.mixed_motion())
        self.assertEqual(motion.calls, [])

    def test_starts_joints_and_leds_from_one_time_zero(self):
        prepared = self.mixed_motion()
        motion = FakeMotionService()
        leds = FakeLedService()
        started = time.time()
        self.assertEqual(play_motion(motion, prepared, leds), "finished")
        # Returns after the last key even though the fake calls return at once.
        self.assertGreaterEqual(time.time() - started, 0.3 - 0.01)

        names, times, keys = motion.calls[0]
        self.assertEqual(names, ["HeadYaw"])
        self.assertEqual(keys, prepared["keys"])
        lag = prepared["times"][0][0] - times[0][0]
        self.assertTrue(0 <= lag < 0.05, lag)
        self.assertAlmostEqual(times[0][1], prepared["times"][0][1] - lag)

        (issued, _, name, colors, color_times), = leds.named("fadeListRGB", "ChestLeds")
        self.assertEqual(colors, [0x0000FF, 0xFF0000])
        chest = streams_by_name(prepared)["ChestLeds"]
        lag = chest["timeList"][0] - color_times[0]
        self.assertTrue(0 <= lag < 0.05, lag)

        # One fade per breakpoint, each issued at its segment's start and timed
        # to end at the breakpoint's absolute deadline.
        fades = leds.named("fade", EAR)
        ear = streams_by_name(prepared)[EAR]
        self.assertEqual([call[3] for call in fades], ear["intensityList"])
        begin = 0.0
        for (issued, _, _, _, duration), end in zip(fades, ear["timeList"]):
            self.assertGreaterEqual(issued, begin - 0.005)
            self.assertAlmostEqual(issued + duration, end, delta=0.02)
            begin = end

    def test_blocking_fades_keep_their_deadlines(self):
        prepared = self.mixed_motion()
        leds = FakeLedService(blocking=True)
        play_motion(FakeMotionService(), prepared, leds)
        fades = leds.named("fade", EAR)
        ear = streams_by_name(prepared)[EAR]
        for (issued, _, _, _, duration), end in zip(fades, ear["timeList"]):
            self.assertAlmostEqual(issued + duration, end, delta=0.02)

    def test_led_only_motion_makes_no_motion_calls(self):
        prepared = prepare_motion(export([led(EAR, linear([(0, 0.0), (100, 1.0)]))]),
                                  lead_in_seconds=0.05)
        motion = FakeMotionService()
        leds = FakeLedService()
        self.assertIsNone(play_motion(motion, prepared, leds))
        self.assertEqual(motion.calls, [])
        self.assertEqual(len(leds.named("fade", EAR)), 2)

    def test_checks_leds_against_the_robot_before_playing(self):
        motion = FakeMotionService()
        missing = FakeLedService(devices=sorted(documented_v6_leds() - {EAR}))
        with self.assertRaisesRegex(MotionFormatError, "does not provide LEDs: " + EAR):
            play_motion(motion, self.mixed_motion(), missing)
        remapped = FakeLedService(listed={"ChestLeds": list(CHEST[:2])})
        with self.assertRaisesRegex(MotionFormatError, "ALLeds maps 'ChestLeds'"):
            play_motion(motion, self.mixed_motion(), remapped)
        self.assertEqual(motion.calls, [])
        self.assertEqual(missing.calls + remapped.calls, [])

    def test_checks_joints_against_the_robot_before_playing(self):
        motion = FakeMotionService(joints=("HeadPitch",))
        leds = FakeLedService()
        with self.assertRaisesRegex(MotionFormatError, "HeadYaw"):
            play_motion(motion, self.mixed_motion(), leds)
        self.assertEqual(leds.calls, [])

    def test_a_failed_led_call_is_raised_after_the_others_finish(self):
        motion = FakeMotionService()
        leds = FakeLedService(failing="fadeListRGB")
        with self.assertRaisesRegex(RuntimeError, "refused fadeListRGB"):
            play_motion(motion, self.mixed_motion(), leds)
        self.assertEqual(len(motion.calls), 1)
        self.assertEqual(len(leds.named("fade", EAR)), 4)



class StartMotionTests(unittest.TestCase):
    def long_motion(self):
        return prepare_motion(export([
            head_yaw([(0, 0.0), (600, 0.2)]),
            led(EAR, linear([(0, 0.0), (100, 1.0), (200, 0.0), (300, 1.0), (400, 0.0),
                             (500, 1.0), (600, 0.0)])),
        ]), lead_in_seconds=0.05)

    def test_start_returns_at_once_and_wait_returns_the_result(self):
        motion = BlockingMotionService()
        started = time.time()
        playback = start_motion(motion, prepare_motion(sample_input(), lead_in_seconds=0.05))
        self.assertLess(time.time() - started, 0.05)
        self.assertEqual(playback.wait(), "finished")
        self.assertGreaterEqual(time.time() - started, 1.65 - 0.01)
        self.assertEqual(motion.kills, [])

    def test_stop_ends_the_joints_and_the_fade_chains(self):
        motion = BlockingMotionService()
        leds = FakeLedService()
        playback = start_motion(motion, self.long_motion(), leds)
        time.sleep(0.2)
        stopped = time.time() - leds.started
        playback.stop()
        returned = time.time()
        self.assertIsNone(playback.wait())
        self.assertLess(time.time() - returned, 0.1)
        self.assertEqual(motion.kills, [["HeadYaw"]])
        fades = leds.named("fade", EAR)
        self.assertTrue(0 < len(fades) < 7, len(fades))
        self.assertTrue(all(call[0] <= stopped + 0.01 for call in fades))

    def test_a_stop_that_reaches_motion_first_is_repeated(self):
        motion = BlockingMotionService()
        missed = []
        kill = motion.killTasksUsingResources
        # The first kill arrives before ALMotion has the task, and does nothing.
        motion.killTasksUsingResources = lambda names: missed.append(names) if not missed \
            else kill(names)
        playback = start_motion(motion, prepare_motion(sample_input(), lead_in_seconds=0.05))
        time.sleep(0.05)
        returned = time.time()
        playback.stop()
        self.assertIsNone(playback.wait())
        self.assertLess(time.time() - returned, 0.3)
        self.assertEqual(missed, [["HeadYaw", "HeadPitch"]])
        self.assertEqual(motion.kills, [["HeadYaw", "HeadPitch"]])

    def test_start_runs_the_same_checks_as_play(self):
        motion = BlockingMotionService()
        with self.assertRaisesRegex(MotionFormatError, "leds=False"):
            start_motion(motion, self.long_motion())
        with self.assertRaisesRegex(MotionFormatError, "HeadYaw"):
            start_motion(FakeMotionService(joints=("HeadPitch",)), self.long_motion(),
                         FakeLedService())
        self.assertEqual(motion.calls, [])


if __name__ == "__main__":
    unittest.main()
