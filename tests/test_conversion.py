import json
import math
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))
from semio_naoqi_motion import (
    MotionFormatError, convert_motion, play_effective_motion, play_motion, prepare_motion,
)


def sample_input():
    return {
        "format": "semio-effective-motion",
        "version": 1,
        "animationName": "Look around",
        "timeBasis": "animation-local",
        "timeUnit": "ms",
        "channels": [
            {
                "output": "HeadYaw",
                "units": "rad",
                "keys": [
                    {"timeMs": 0, "value": 0.0,
                     "out": {"deltaTimeMs": 250, "deltaValue": 0.1}},
                    {"timeMs": 1000, "value": 0.5,
                     "in": {"deltaTimeMs": -300, "deltaValue": -0.2},
                     "out": {"deltaTimeMs": 100, "deltaValue": 0.3}},
                    {"timeMs": 1600, "value": 0.0,
                     "in": {"deltaTimeMs": -200, "deltaValue": 0.1}},
                ],
            },
            {
                "output": "HeadPitch",
                "units": "rad",
                "keys": [
                    {"timeMs": 400, "value": 0.1,
                     "out": {"deltaTimeMs": 100, "deltaValue": 0}},
                    {"timeMs": 1000, "value": 0.3,
                     "in": {"deltaTimeMs": -100, "deltaValue": 0}},
                ],
            },
        ],
    }


def hand_channel(output, units):
    return {
        "output": output,
        "units": units,
        "keys": [
            {"timeMs": 0, "value": 0.3,
             "out": {"deltaTimeMs": 200, "deltaValue": 0.1}},
            {"timeMs": 600, "value": 0.7,
             "in": {"deltaTimeMs": -200, "deltaValue": -0.1}},
        ],
    }


def cubic(a, b, c, d, t):
    remaining = 1 - t
    return (remaining ** 3 * a + 3 * remaining ** 2 * t * b
            + 3 * remaining * t ** 2 * c + t ** 3 * d)


class FakeMotionService:
    def __init__(self, joints=("HeadYaw", "HeadPitch")):
        self.joints = joints
        self.calls = []

    def getBodyNames(self, group):
        assert group == "Body"
        return list(self.joints)

    def angleInterpolationBezier(self, names, times, keys):
        self.calls.append((names, times, keys))
        return "finished"


class ConversionTests(unittest.TestCase):
    def test_preserves_mixed_handles_and_channel_timing(self):
        result = convert_motion(sample_input())
        self.assertEqual(result["names"], ["HeadYaw", "HeadPitch"])
        for actual, expected in zip(result["times"], [[0.2, 1.2, 1.8], [0.6, 1.2]]):
            for actual_time, expected_time in zip(actual, expected):
                self.assertAlmostEqual(actual_time, expected_time)
        self.assertEqual(result["keys"][0][1],
                         [0.5, [3, -0.3, -0.2], [3, 0.1, 0.3]])
        self.assertEqual(result["keys"][0][0][1], [3, 0.0, 0.0])
        self.assertEqual(result["keys"][0][-1][2], [3, 0.0, 0.0])

    def test_segment_geometry_is_unchanged_except_for_time_translation(self):
        source = sample_input()["channels"][0]["keys"]
        result = convert_motion(sample_input(), lead_in_seconds=0.5)
        times = result["times"][0]
        keys = result["keys"][0]
        for index in range(len(source) - 1):
            start, end = source[index:index + 2]
            source_x = [
                start["timeMs"] / 1000,
                (start["timeMs"] + start["out"]["deltaTimeMs"]) / 1000,
                (end["timeMs"] + end["in"]["deltaTimeMs"]) / 1000,
                end["timeMs"] / 1000,
            ]
            source_y = [
                start["value"],
                start["value"] + start["out"]["deltaValue"],
                end["value"] + end["in"]["deltaValue"],
                end["value"],
            ]
            naoqi_x = [
                times[index],
                times[index] + keys[index][2][1],
                times[index + 1] + keys[index + 1][1][1],
                times[index + 1],
            ]
            naoqi_y = [
                keys[index][0],
                keys[index][0] + keys[index][2][2],
                keys[index + 1][0] + keys[index + 1][1][2],
                keys[index + 1][0],
            ]
            for fraction in (0, 0.1, 0.33, 0.7, 1):
                self.assertAlmostEqual(
                    cubic(*naoqi_x, fraction) - cubic(*source_x, fraction), 0.5,
                )
                self.assertAlmostEqual(cubic(*naoqi_y, fraction), cubic(*source_y, fraction))

    def test_hand_opening_channels_keep_fraction_values(self):
        # Studio writes a hand marked as a percentage with the same stored
        # fraction NAOqi takes, so both spellings convert to identical keys.
        for units in ("%", "dimensionless"):
            with self.subTest(units=units):
                data = sample_input()
                data["channels"] = [hand_channel("LHand", units)]
                result = convert_motion(data)
                self.assertEqual(result["names"], ["LHand"])
                self.assertEqual(result["keys"][0][0][0], 0.3)
                self.assertEqual(result["keys"][0][0][2], [3, 0.2, 0.1])
                self.assertEqual(result["keys"][0][1][1], [3, -0.2, -0.1])

    def test_hand_opening_accepts_both_limits(self):
        data = sample_input()
        hand = hand_channel("RHand", "%")
        hand["keys"][0]["value"] = 0.0
        hand["keys"][1]["value"] = 1.0
        data["channels"] = [hand]
        result = convert_motion(data)
        self.assertEqual([key[0] for key in result["keys"][0]], [0.0, 1.0])

    def test_rejects_hand_values_off_the_fraction_scale(self):
        # A percentage written as 0..100 would otherwise reach NAOqi, which
        # clamps it to fully open instead of refusing it.
        for value in (35, 1.01, -0.01):
            with self.subTest(value=value):
                data = sample_input()
                hand = hand_channel("LHand", "%")
                hand["keys"][1]["value"] = value
                data["channels"] = [hand]
                with self.assertRaisesRegex(MotionFormatError, "fraction from 0 to 1"):
                    convert_motion(data)

    def test_rejects_incompatible_input(self):
        changes = [
            (lambda data: data.update(version=2), "version"),
            (lambda data: data.update(version=True), "version"),
            (lambda data: data.update(timeBasis="scene"), "timeBasis"),
            (lambda data: data["channels"][0].update(units="m"), "units"),
            (lambda data: data["channels"][0].update(units="dimensionless"), "units"),
            (lambda data: data["channels"][0].update(output="RHand"), "units"),
            (lambda data: data["channels"][0].update(units="%"),
             "only LHand and RHand take percentages"),
            (lambda data: data["channels"][0].update(
                output="ChestBoard/Led/Red/Actuator/Value", units="%"),
             "LEDs are not converted"),
            (lambda data: data["channels"][0].update(output="ChestBoard/Led/Red/Actuator/Value"),
             "LEDs are not converted"),
            (lambda data: data["channels"][1].update(output="HeadYaw"), "appears more than once"),
            (lambda data: data["channels"][1].update(output=" HeadPitch "), "nonempty joint name"),
            (lambda data: data["channels"][0]["keys"][1].pop("in"), "in is required"),
            (lambda data: data["channels"][0]["keys"][1].update(timeMs=0), "increase strictly"),
            (lambda data: data["channels"][0]["keys"][0].update(value=math.inf), "finite number"),
        ]
        for change, message in changes:
            with self.subTest(message=message):
                data = sample_input()
                change(data)
                with self.assertRaisesRegex(MotionFormatError, message):
                    convert_motion(data)

    def test_cli_writes_valid_json(self):
        with tempfile.TemporaryDirectory() as directory:
            input_path = Path(directory) / "input.json"
            output_path = Path(directory) / "naoqi.json"
            input_path.write_text(json.dumps(sample_input()), encoding="utf-8")
            command = [
                sys.executable,
                str(PROJECT / "convert_semio_effective_motion_to_naoqi_bezier.py"),
                str(input_path),
                "--lead-in-seconds", "0.4",
                "--output", str(output_path),
            ]
            completed = subprocess.run(command, capture_output=True, text=True, check=True)
            self.assertEqual(completed.stdout, "")
            result = json.loads(output_path.read_text(encoding="utf-8"))
            self.assertEqual(result["times"][0][0], 0.4)
            self.assertEqual(result["names"], ["HeadYaw", "HeadPitch"])

    def test_prepared_motion_can_be_triggered_repeatedly_from_an_existing_proxy(self):
        motion = FakeMotionService()
        prepared = prepare_motion(sample_input())
        self.assertEqual(play_motion(motion, prepared), "finished")
        self.assertEqual(play_motion(motion, prepared), "finished")
        self.assertEqual(motion.calls, [
            (prepared["names"], prepared["times"], prepared["keys"]),
            (prepared["names"], prepared["times"], prepared["keys"]),
        ])

    def test_play_effective_motion_loads_the_studio_export(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "animation.effective-motion.json"
            path.write_text(json.dumps(sample_input()), encoding="utf-8")
            motion = FakeMotionService()
            self.assertEqual(play_effective_motion(motion, str(path)), "finished")
            self.assertEqual(motion.calls[0][0], ["HeadYaw", "HeadPitch"])

    def test_missing_robot_joint_rejects_before_motion(self):
        motion = FakeMotionService(joints=("HeadYaw",))
        with self.assertRaisesRegex(MotionFormatError, "HeadPitch"):
            play_motion(motion, prepare_motion(sample_input()))
        self.assertEqual(motion.calls, [])


if __name__ == "__main__":
    unittest.main()
