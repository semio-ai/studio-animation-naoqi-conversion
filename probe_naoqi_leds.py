#!/usr/bin/env python
"""Measure how ALLeds behaves on a NAO, for the assumptions LED playback makes.

Each check prints what the robot did; none of them decides pass or fail. LED
values are read back with ALLeds.getIntensity, which reports ALLeds' own view;
confirm what the LEDs show with a camera or a photodiode. Only the skew check
with --allow-motion moves the robot.
"""

from __future__ import print_function

import argparse
import sys
import threading
import time

from semio_naoqi_motion import LED_DEVICES, RGB_LED_POSITIONS, play_motion, prepare_motion


CHEST = tuple("ChestBoard/Led/{}/Actuator/Value".format(color)
              for color in ("Red", "Green", "Blue"))
EAR = "Ears/Led/Right/0Deg/Actuator/Value"
HEAD = "Head/Led/Front/Right/0/Actuator/Value"
FACE_RED = "Face/Led/Red/Right/0Deg/Actuator/Value"
SINGLE_COLOUR = tuple(sorted(name for name in LED_DEVICES
                             if name.startswith(("Ears/", "Head/"))))


def timed(function, *arguments):
    start = time.time()
    try:
        function(*arguments)
    except Exception as error:
        return time.time() - start, error
    return time.time() - start, None


def outcome(elapsed, error):
    if error is not None:
        return "raised {}: {}".format(type(error).__name__, error)
    return "returned after {:.3f} s".format(elapsed)


def in_background(function, *arguments):
    thread = threading.Thread(target=function, args=arguments)
    thread.daemon = True
    thread.start()
    return thread


def run_and_sample(leds, devices, seconds, schedule, period=0.04):
    """Issue ``(at, label, function, arguments)`` commands while sampling LEDs."""
    start = time.time()
    results = {}

    def issue(at, label, function, arguments):
        delay = start + at - time.time()
        if delay > 0:
            time.sleep(delay)
        results[label] = timed(function, *arguments)

    for at, label, function, arguments in schedule:
        in_background(issue, at, label, function, arguments)
    rows = []
    while time.time() - start < seconds:
        rows.append((time.time() - start, [leds.getIntensity(name) for name in devices]))
        time.sleep(period)
    for at, label, _, _ in schedule:
        result = outcome(*results[label]) if label in results else "still running"
        print("  at {:.1f} s: {}: {}".format(at, label, result))
    print("     time  " + "  ".join(name.split("/")[-3][:6].rjust(6) for name in devices))
    for stamp, values in rows:
        print("  {:7.3f}  ".format(stamp) + "  ".join("{:6.3f}".format(value) for value in values))


def check_blocking(leds, motion, args):
    print("1. Do fade, fadeRGB and fadeListRGB block until their last target?")
    for label, function, arguments in [
        ("fade({!r}, 1.0, 1.0)".format(EAR), leds.fade, (EAR, 1.0, 1.0)),
        ("fadeRGB('ChestLeds', 0xFF0000, 1.0)", leds.fadeRGB, ("ChestLeds", 0xFF0000, 1.0)),
        ("fadeListRGB('ChestLeds', [0x00FF00, 0x0000FF], [0.5, 1.0])", leds.fadeListRGB,
         ("ChestLeds", [0x00FF00, 0x0000FF], [0.5, 1.0])),
    ]:
        print("  {}: last target at 1.0 s; {}".format(label, outcome(*timed(function, *arguments))))


def check_times(leds, motion, args):
    print("2. fadeListRGB timing. Red, then green, then blue ramp up, with times [0.5, 1.0, 1.5].")
    print("   Absolute times finish at 1.5 s, per-segment durations at 3.0 s.")
    for name in CHEST:
        leds.setIntensity(name, 0.0)
    run_and_sample(leds, CHEST, 3.5, [
        (0.0, "fadeListRGB('ChestLeds', [0xFF0000, 0xFFFF00, 0xFFFFFF], [0.5, 1.0, 1.5])",
         leds.fadeListRGB, ("ChestLeds", [0xFF0000, 0xFFFF00, 0xFFFFFF], [0.5, 1.0, 1.5])),
    ])
    for times in ([0.0, 0.5], [-0.1, 0.5], [0.5, 0.5], [0.5, 0.3]):
        result = timed(leds.fadeListRGB, "ChestLeds", [0xFF0000, 0x00FF00], times)
        print("  timeList {}: {}".format(times, outcome(*result)))


def check_single(leds, motion, args):
    print("3. fadeListRGB on single-colour devices: which byte of 0x00RRGGBB drives them?")
    for name in (EAR, HEAD, FACE_RED):
        for color in (0x0000FF, 0x00FF00, 0xFF0000, 0xFFFFFF):
            leds.setIntensity(name, 0.0)
            result = timed(leds.fadeListRGB, name, [color], [0.3])
            time.sleep(max(0.0, 0.4 - result[0]))
            reading = "intensity {:.3f}".format(leds.getIntensity(name))
            print("  fadeListRGB({!r}, [0x{:06X}], [0.3]): {}; {}".format(
                name, color, outcome(*result), reading))


def check_overlap(leds, motion, args):
    print("4. A new command reaches an LED mid-fade: does it cancel, queue or blend?")
    leds.setIntensity(EAR, 0.0)
    run_and_sample(leds, [EAR], 3.0, [
        (0.0, "fade(ear, 1.0, 2.0)", leds.fade, (EAR, 1.0, 2.0)),
        (0.5, "fade(ear, 0.0, 0.5)", leds.fade, (EAR, 0.0, 0.5)),
    ])
    for name in CHEST:
        leds.setIntensity(name, 0.0)
    run_and_sample(leds, CHEST, 3.0, [
        (0.0, "fadeListRGB('ChestLeds', [0xFFFFFF], [2.0])",
         leds.fadeListRGB, ("ChestLeds", [0xFFFFFF], [2.0])),
        (0.5, "fadeRGB('ChestLeds', 0x000000, 0.5)", leds.fadeRGB, ("ChestLeds", 0x000000, 0.5)),
    ])
    print("  Cancelling a qi future of fade(ear, 1.0, 2.0) after 0.5 s:")
    leds.setIntensity(EAR, 0.0)
    future = leds.fade(EAR, 1.0, 2.0, _async=True)
    run_and_sample(leds, [EAR], 2.5, [(0.5, "future.cancel()", future.cancel, ())])


def check_limits(leds, motion, args):
    print("5. List length, concurrent fades and short durations")
    for count in (100, 1000, 5000):
        colors = [0xFF0000 if index % 2 else 0x0000FF for index in range(count)]
        times = [2.0 * (index + 1) / count for index in range(count)]
        print("  fadeListRGB with {} colours over 2.0 s: {}".format(
            count, outcome(*timed(leds.fadeListRGB, "ChestLeds", colors, times))))
    start = time.time()
    issued = {}

    def fade(name):
        issued[name] = (time.time() - start,) + timed(leds.fade, name, 1.0, 0.5)

    threads = [in_background(fade, name) for name in SINGLE_COLOUR]
    for thread in threads:
        thread.join()
    errors = [result[2] for result in issued.values() if result[2] is not None]
    print("  {} fades of 0.5 s in parallel threads: issued within {:.3f} s, "
          "returned {:.3f} to {:.3f} s after the first issue{}".format(
              len(issued), max(r[0] for r in issued.values()),
              min(r[0] + r[1] for r in issued.values()),
              max(r[0] + r[1] for r in issued.values()),
              "; errors: {}".format(errors) if errors else ""))
    for duration in (0.0, 0.006, 0.012, 0.024):
        leds.setIntensity(EAR, 0.0)
        result = timed(leds.fade, EAR, 1.0, duration)
        print("  fade(ear, 1.0, {}): {}; intensity then {:.3f}".format(
            duration, outcome(*result), leds.getIntensity(EAR)))


class Timestamped(object):
    """Pass calls through to a service and note when each was issued."""

    def __init__(self, service, label, start, log):
        self._service, self._label, self._start, self._log = service, label, start, log

    def __getattr__(self, name):
        method = getattr(self._service, name)

        def call(*arguments):
            self._log.append((time.time() - self._start, self._label, name, arguments[:1]))
            return method(*arguments)
        return call


def check_skew(leds, motion, args):
    print("6. How far apart do play_motion's calls start?")
    channels = [
        {"output": EAR, "units": "%", "keys": step_keys(0.0, 1.0)},
        {"output": CHEST[0], "units": "%", "keys": step_keys(0.0, 1.0)},
    ]
    if args.allow_motion:
        if not motion.robotIsWakeUp():
            print("  The robot is not awake; skipping the head movement.")
            args.allow_motion = False
        else:
            base = motion.getAngles("HeadYaw", True)[0]
            channels.append({"output": "HeadYaw", "units": "rad",
                             "keys": step_keys(base, base + 0.15)})
    prepared = prepare_motion({
        "format": "semio-effective-motion", "version": 1, "animationName": "skew probe",
        "timeBasis": "animation-local", "timeUnit": "ms", "channels": channels,
    }, lead_in_seconds=0.5)
    log = []
    start = time.time()
    playback = in_background(
        play_motion, Timestamped(motion, "ALMotion", start, log), prepared,
        Timestamped(leds, "ALLeds", start, log))
    rows = []
    while playback.is_alive():
        angle = motion.getAngles("HeadYaw", True)[0] if args.allow_motion else 0.0
        rows.append((time.time() - start, angle, leds.getIntensity(EAR)))
        time.sleep(0.01)
    calls = [entry for entry in log
             if entry[2] in ("angleInterpolationBezier", "fadeListRGB", "fade")]
    if not calls:
        print("  play_motion made no playback calls")
        return
    origin = calls[0][0]
    print("  Calls, in seconds after the first one:")
    for stamp, label, name, first in calls:
        print("  {:+.4f} s: {}.{}({})".format(stamp - origin, label, name, first[0]))
    print("  Each stream rises from 0.5 s after play_motion's start (about the first call).")
    print_departure("Ear intensity", rows, 2, origin, 0.1)
    if args.allow_motion:
        print_departure("HeadYaw", rows, 1, origin, 0.01)


def step_keys(low, high):
    return [
        {"timeMs": 0, "value": low, "out": {"deltaTimeMs": 30, "deltaValue": 0.0}},
        {"timeMs": 100, "value": high, "in": {"deltaTimeMs": -30, "deltaValue": 0.0},
         "out": {"deltaTimeMs": 100, "deltaValue": 0.0}},
        {"timeMs": 400, "value": low, "in": {"deltaTimeMs": -100, "deltaValue": 0.0}},
    ]


def print_departure(label, rows, column, origin, threshold):
    # Compare with the value sampled at the first key, when the rise begins.
    reference = None
    for row in rows:
        stamp, value = row[0] - origin, row[column]
        if reference is None and stamp >= 0.5:
            reference = value
        elif reference is not None and abs(value - reference) > threshold:
            print("  {} first moved {:.3f} s after the first call (planned: 0.5 s)".format(
                label, stamp))
            return
    print("  {} did not move while sampled".format(label))


def check_names(leds, motion, args):
    print("7. Does the robot report the converter's 89 device names and 19 RGB positions?")
    listed = set(leds.listGroup("AllLeds"))
    expected = set(LED_DEVICES)
    print("  listGroup('AllLeds') returns {} devices; the converter knows {}.".format(
        len(listed), len(expected)))
    for label, names in (("Missing", expected - listed), ("Not in the converter", listed - expected)):
        for name in sorted(names):
            print("  {}: {}".format(label, name))
    for name, lookup, channels in RGB_LED_POSITIONS:
        devices = getattr(leds, lookup)(name)
        status = "ok" if set(devices) == set(channels) else "differs: {}".format(devices)
        print("  {}({!r}): {}".format(lookup, name, status))


CHECKS = [
    ("blocking", check_blocking),
    ("times", check_times),
    ("single", check_single),
    ("overlap", check_overlap),
    ("limits", check_limits),
    ("skew", check_skew),
    ("names", check_names),
]


def run(leds, motion, args):
    for name, check in CHECKS:
        if args.checks and name not in args.checks:
            continue
        print()
        try:
            check(leds, motion, args)
        finally:
            leds.reset("AllLeds")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ip", default="127.0.0.1", help="robot address (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=9559, help="NAOqi port (default: 9559)")
    parser.add_argument("--allow-motion", action="store_true",
                        help="let the skew check turn the head by 0.15 rad and back")
    parser.add_argument("checks", nargs="*", metavar="check",
                        help="checks to run: {} (default: all)".format(
                            ", ".join(name for name, _ in CHECKS)))
    args = parser.parse_args(argv)
    unknown = sorted(set(args.checks) - set(name for name, _ in CHECKS))
    if unknown:
        parser.error("unknown checks: {}".format(", ".join(unknown)))

    import qi
    session = qi.Session()
    try:
        session.connect("tcp://{}:{}".format(args.ip, args.port))
    except RuntimeError as error:
        parser.exit(2, "error: cannot connect to NAOqi at {}:{}: {}\n".format(
            args.ip, args.port, error))
    run(session.service("ALLeds"), session.service("ALMotion"), args)


if __name__ == "__main__":
    main()
