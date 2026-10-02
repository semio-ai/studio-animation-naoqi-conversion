"""Prepare Studio effective motion and play it through an ALMotion proxy."""

import io
import json
import math


try:
    string_types = (basestring,)
    number_types = (int, long, float)
except NameError:
    string_types = (str,)
    number_types = (int, float)


BEZIER_HANDLE_MODE = 3

# NAOqi takes hand opening as a fraction from 0 to 1. Studio
# writes a percentage the same way, as the stored fraction, so both spellings
# carry identical values; "dimensionless" is what exports from models that do
# not mark the hands as percentages still say.
HAND_JOINTS = ("LHand", "RHand")
HAND_UNITS = ("%", "dimensionless")


class MotionFormatError(ValueError):
    """The input cannot be represented as NAOqi joint Bézier arguments."""


def _object(value, path):
    if not isinstance(value, dict):
        raise MotionFormatError("{} must be an object".format(path))
    return value


def _list(value, path):
    if not isinstance(value, list) or not value:
        raise MotionFormatError("{} must be a nonempty array".format(path))
    return value


def _number(value, path):
    if isinstance(value, bool) or not isinstance(value, number_types):
        raise MotionFormatError("{} must be a finite number".format(path))
    try:
        result = float(value)
    except (OverflowError, ValueError):
        raise MotionFormatError("{} must be a finite number".format(path))
    if math.isnan(result) or math.isinf(result):
        raise MotionFormatError("{} must be a finite number".format(path))
    return result


def _handle(key, side, required, path):
    value = key.get(side)
    if value is None:
        if required:
            raise MotionFormatError("{}.{} is required".format(path, side))
        # This side has no neighboring segment, so its tangent has no effect.
        return [BEZIER_HANDLE_MODE, 0.0, 0.0]

    handle = _object(value, "{}.{}".format(path, side))
    delta_time = _number(handle.get("deltaTimeMs"), "{}.{}.deltaTimeMs".format(path, side))
    delta_value = _number(handle.get("deltaValue"), "{}.{}.deltaValue".format(path, side))
    if side == "in" and delta_time > 0:
        raise MotionFormatError("{}.in must point backward in time".format(path))
    if side == "out" and delta_time < 0:
        raise MotionFormatError("{}.out must point forward in time".format(path))
    return [BEZIER_HANDLE_MODE, delta_time / 1000.0, delta_value]


def convert_motion(source, lead_in_seconds=0.2):
    """Return ``names``, ``times``, ``keys`` for ``ALMotion.angleInterpolationBezier``.

    All keys receive the same time translation, preserving the exported curves.
    No robot SDK is imported or called.
    """
    motion = _object(source, "input")
    expected = {
        "format": "semio-effective-motion",
        "version": 1,
        "timeBasis": "animation-local",
        "timeUnit": "ms",
    }
    for field, value in expected.items():
        if motion.get(field) != value or (field == "version" and isinstance(motion.get(field), bool)):
            raise MotionFormatError("{} must be {!r}".format(field, value))
    if not isinstance(motion.get("animationName"), string_types):
        raise MotionFormatError("animationName must be a string")

    lead_in = _number(lead_in_seconds, "lead_in_seconds")
    if lead_in <= 0:
        raise MotionFormatError("lead_in_seconds must be positive")

    channels = _list(motion.get("channels"), "channels")
    validated = []
    names_seen = set()
    first_times = []

    for channel_index, raw_channel in enumerate(channels):
        path = "channels[{}]".format(channel_index)
        channel = _object(raw_channel, path)
        name = channel.get("output")
        if not isinstance(name, string_types) or not name.strip() or name != name.strip():
            raise MotionFormatError("{}.output must be a nonempty joint name".format(path))
        if name in names_seen:
            raise MotionFormatError("joint output {!r} appears more than once".format(name))
        names_seen.add(name)
        # NAOqi joint names have no "/"; device names such as LEDs' do.
        if "/" in name:
            raise MotionFormatError(
                "{}.output {!r} is not a joint: non-joint outputs such as LEDs are not "
                "converted".format(path, name))
        units = channel.get("units")
        is_hand = name in HAND_JOINTS
        if is_hand and units not in HAND_UNITS:
            raise MotionFormatError("{}.units must be '%' or 'dimensionless' for {!r}".format(
                path, name))
        if not is_hand and units == "%":
            raise MotionFormatError(
                "{}.units is '%' for {!r}: only LHand and RHand take percentages, and "
                "non-joint outputs such as LEDs are not converted".format(path, name))
        if not is_hand and units != "rad":
            raise MotionFormatError("{}.units must be 'rad' for {!r}".format(path, name))

        points = _list(channel.get("keys"), "{}.keys".format(path))
        converted_points = []
        previous_time = None
        for key_index, raw_key in enumerate(points):
            key_path = "{}.keys[{}]".format(path, key_index)
            key = _object(raw_key, key_path)
            stamp = _number(key.get("timeMs"), "{}.timeMs".format(key_path))
            angle = _number(key.get("value"), "{}.value".format(key_path))
            # A fraction outside 0..1 means the export is not on that scale (a
            # percentage written as 0..100, say); NAOqi would clamp it to a limit
            # rather than refuse, so the mismatch would play as a slammed hand.
            if is_hand and not 0.0 <= angle <= 1.0:
                raise MotionFormatError(
                    "{}.value must be a hand opening fraction from 0 to 1".format(key_path))
            if previous_time is not None and stamp <= previous_time:
                raise MotionFormatError("{}.timeMs must increase strictly".format(key_path))
            incoming = _handle(key, "in", key_index > 0, key_path)
            outgoing = _handle(key, "out", key_index < len(points) - 1, key_path)
            converted_points.append((stamp, [angle, incoming, outgoing]))
            previous_time = stamp

        first_times.append(converted_points[0][0])
        validated.append((name, converted_points))

    # NAOqi times are seconds after the call and must be positive. A single
    # translation keeps every key spacing and handle offset from Studio intact.
    shift_seconds = lead_in - min(first_times) / 1000.0
    names = []
    times = []
    keys = []
    for name, points in validated:
        joint_times = [stamp / 1000.0 + shift_seconds for stamp, _ in points]
        if any(time <= 0 or math.isnan(time) or math.isinf(time) for time in joint_times):
            raise MotionFormatError("converted times for {!r} must be positive".format(name))
        names.append(name)
        times.append(joint_times)
        keys.append([value for _, value in points])

    return {"names": names, "times": times, "keys": keys}


def prepare_motion(source, lead_in_seconds=0.2):
    """Load a Studio export path or dictionary and return reusable NAOqi arguments."""
    if isinstance(source, string_types):
        with io.open(source, "r", encoding="utf-8") as stream:
            source = json.load(stream)
    return convert_motion(source, lead_in_seconds)


def play_motion(motion_service, prepared_motion):
    """Run prepared arguments on an existing ALMotion proxy or qi service.

    The caller owns connection, stiffness, posture, and scheduling. This call
    blocks until ALMotion finishes the trajectory.
    """
    names = prepared_motion["names"]
    available = set(motion_service.getBodyNames("Body"))
    missing = sorted(set(names) - available)
    if missing:
        raise MotionFormatError("NAOqi does not provide joints: {}".format(", ".join(missing)))
    return motion_service.angleInterpolationBezier(
        names, prepared_motion["times"], prepared_motion["keys"]
    )


def play_effective_motion(motion_service, source, lead_in_seconds=0.2):
    """Load, convert, and play one Studio export through an existing ALMotion service."""
    return play_motion(motion_service, prepare_motion(source, lead_in_seconds))
