"""Prepare Studio effective motion and play it through ALMotion and ALLeds."""

import bisect
import io
import json
import math
import threading
import time


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

# Breakpoints added while flattening an LED curve stay at least this far apart.
# LoLA exchanges data with the hardware every 12 ms and ALMotion processes
# every second cycle, so closer targets cannot be told apart on the robot.
LED_MIN_INTERVAL_SECONDS = 0.024


def _led_table():
    """Return the NAO V6 LED devices and the RGB positions ALLeds can address.

    Device names are Aldebaran's 2.8 actuator names, which ALLeds accepts as
    they are. Each maps to a flattening tolerance of half a brightness step:
    the eyes have 64 levels, the chest and feet 256, and the ears and head 16.
    Each RGB position is ``(name, lookup, (red, green, blue))``, where
    ``lookup`` is the ALLeds method that lists the devices behind ``name``.
    """
    devices = {}
    positions = []

    def rgb(name, lookup, pattern, tolerance):
        channels = tuple(pattern.format(color) for color in ("Red", "Green", "Blue"))
        for device in channels:
            devices[device] = tolerance
        positions.append((name, lookup, channels))

    for side in ("Right", "Left"):
        for number, angle in enumerate((0, 45, 90, 135, 180, 225, 270, 315), 1):
            # Short names number both eyes alike; the FaceLed groups mirror them.
            rgb("{}FaceLed{}".format(side, number), "listLED",
                "Face/Led/{{}}/{}/{}Deg/Actuator/Value".format(side, angle), 1.0 / 128)
    rgb("ChestLeds", "listGroup", "ChestBoard/Led/{}/Actuator/Value", 1.0 / 512)
    rgb("LeftFootLeds", "listGroup", "LFoot/Led/{}/Actuator/Value", 1.0 / 512)
    rgb("RightFootLeds", "listGroup", "RFoot/Led/{}/Actuator/Value", 1.0 / 512)
    for side in ("Right", "Left"):
        for angle in range(0, 360, 36):
            devices["Ears/Led/{}/{}Deg/Actuator/Value".format(side, angle)] = 1.0 / 32
        for place in ("Front/{}/0", "Front/{}/1", "Middle/{}/0",
                      "Rear/{}/0", "Rear/{}/1", "Rear/{}/2"):
            devices["Head/Led/{}/Actuator/Value".format(place.format(side))] = 1.0 / 32
    return devices, tuple(positions)


LED_DEVICES, RGB_LED_POSITIONS = _led_table()


class MotionFormatError(ValueError):
    """The input cannot be represented as NAOqi joint or LED arguments."""


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


def _check_led_handles(points, path):
    # ALLeds only interpolates between targets, so an LED curve is sampled as a
    # function of time. Handles that stay within their segment guarantee that.
    for index in range(len(points) - 1):
        start_ms, start_key = points[index]
        end_ms, end_key = points[index + 1]
        span = (end_ms - start_ms) / 1000.0 + 1e-9
        if start_key[2][1] > span or -end_key[1][1] > span:
            raise MotionFormatError(
                "{}.keys[{}] to keys[{}]: LED handles must stay within their segment".format(
                    path, index, index + 1))


def _cubic(points, fraction):
    a, b, c, d = points
    rest = 1.0 - fraction
    return (rest * rest * rest * a + 3.0 * rest * rest * fraction * b
            + 3.0 * rest * fraction * fraction * c + fraction * fraction * fraction * d)


def _unit(value):
    return min(1.0, max(0.0, value))


def _flatten_led(times, keys, tolerance):
    """Return ``(time, value)`` breakpoints for one LED channel.

    Linear interpolation between the breakpoints stays within ``tolerance`` of
    the Bézier curve, clamped to 0..1, wherever the breakpoints are at least
    twice LED_MIN_INTERVAL_SECONDS apart. Every Studio key remains a breakpoint.
    """
    breakpoints = [(times[0], keys[0][0])]
    for index in range(len(times) - 1):
        start, end = keys[index], keys[index + 1]
        xs = (times[index], times[index] + start[2][1],
              times[index + 1] + end[1][1], times[index + 1])
        ys = (start[0], start[0] + start[2][2], end[0] + end[1][2], end[0])
        _subdivide(xs, ys, tolerance, breakpoints)
    return _without_holds(breakpoints)


def _subdivide(xs, ys, tolerance, breakpoints):
    # Halve the piece in time while its error bound exceeds the tolerance and
    # both halves stay at least LED_MIN_INTERVAL_SECONDS long; where the curve
    # changes faster than that, the spacing wins over the tolerance.
    if xs[3] - xs[0] >= 2 * LED_MIN_INTERVAL_SECONDS and _error_bound(xs, ys) > tolerance:
        fraction = _parameter_at(xs, (xs[0] + xs[3]) / 2.0)
        for half_xs, half_ys in zip(_split(xs, fraction), _split(ys, fraction)):
            _subdivide(half_xs, half_ys, tolerance, breakpoints)
    else:
        breakpoints.append((xs[3], _unit(ys[3])))


def _error_bound(xs, ys):
    """Bound how far the clamped curve strays from the chord between its clamped ends.

    The curve lies within the hull of its control points, so its vertical
    distance from a line is at most theirs. Clamping to 0..1 only moves a
    value closer to a chord that lies within 0..1.
    """
    if max(ys) <= 0.0 or min(ys) >= 1.0:
        return 0.0  # The whole piece clamps to 0, or to 1.
    first, last = _unit(ys[0]), _unit(ys[3])
    slope = (last - first) / (xs[3] - xs[0])
    return max(abs(y - first - slope * (x - xs[0])) for x, y in zip(xs, ys))


def _split(points, fraction):
    # de Casteljau's construction: the control points of the two halves.
    a, b, c, d = points
    ab, bc, cd = a + (b - a) * fraction, b + (c - b) * fraction, c + (d - c) * fraction
    abc, bcd = ab + (bc - ab) * fraction, bc + (cd - bc) * fraction
    middle = abc + (bcd - abc) * fraction
    return (a, ab, abc, middle), (middle, bcd, cd, d)


def _parameter_at(xs, stamp):
    # Time never decreases along the curve, so bisection finds where it is stamp.
    low, high = 0.0, 1.0
    for _ in range(40):
        middle = (low + high) / 2.0
        if _cubic(xs, middle) < stamp:
            low = middle
        else:
            high = middle
    return (low + high) / 2.0


def _without_holds(points):
    # A point whose neighbors hold its value adds nothing to the interpolation.
    kept = points[:1]
    for index in range(1, len(points) - 1):
        if not points[index - 1][1] == points[index][1] == points[index + 1][1]:
            kept.append(points[index])
    return kept + points[1:][-1:]


def _value_at(times, values, stamp):
    # Before its first key and after its last, a channel holds the key's value.
    index = bisect.bisect_right(times, stamp)
    if index == 0:
        return values[0]
    if index == len(times):
        return values[-1]
    start, end = times[index - 1], times[index]
    return values[index - 1] + (values[index] - values[index - 1]) * (stamp - start) / (end - start)


def _byte(value):
    return int(math.floor(value * 255.0 + 0.5))


def _led_streams(lines):
    """Group flattened LED channels into ALLeds calls.

    A complete red, green and blue triple becomes one ``fadeListRGB`` on its
    position, sampled at the union of the three channels' breakpoints. Any
    other channel becomes a chain of ``fade`` calls on its own device, so the
    channels an export does not animate are left alone.
    """
    lines = dict(lines)
    streams = []
    for name, _, channels in RGB_LED_POSITIONS:
        if not all(channel in lines for channel in channels):
            continue
        columns = [lines.pop(channel) for channel in channels]
        timeline = sorted(set(stamp for column in columns for stamp, _ in column))
        samples = []
        for column in columns:
            times = [stamp for stamp, _ in column]
            values = [value for _, value in column]
            samples.append([_value_at(times, values, stamp) for stamp in timeline])
        colors = [(_byte(red) << 16) | (_byte(green) << 8) | _byte(blue)
                  for red, green, blue in zip(*samples)]
        points = _without_holds(list(zip(timeline, colors)))
        streams.append({
            "method": "fadeListRGB",
            "name": name,
            "devices": list(channels),
            "rgbList": [color for _, color in points],
            "timeList": [stamp for stamp, _ in points],
        })
    for device in sorted(lines):
        streams.append({
            "method": "fade",
            "name": device,
            "intensityList": [value for _, value in lines[device]],
            "timeList": [stamp for stamp, _ in lines[device]],
        })
    return streams


def convert_motion(source, lead_in_seconds=0.2, leds=True):
    """Return NAOqi arguments for the joint and LED channels of a Studio export.

    ``names``, ``times`` and ``keys`` are the arguments of
    ``ALMotion.angleInterpolationBezier``. When the export has LED channels,
    ``leds`` lists the ALLeds calls that play them; pass ``leds=False`` to
    validate the LED channels but leave them out. All channels, joints and
    LEDs, receive the same time translation, preserving the exported timing.
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
            raise MotionFormatError("{}.output must be a nonempty output name".format(path))
        if name in names_seen:
            raise MotionFormatError("output {!r} appears more than once".format(name))
        names_seen.add(name)
        is_hand = name in HAND_JOINTS
        is_led = name in LED_DEVICES
        # NAOqi joint names have no "/"; device names such as the LEDs' do.
        if "/" in name and not is_led:
            raise MotionFormatError(
                "{}.output {!r} is neither a joint nor a NAO V6 LED".format(path, name))
        units = channel.get("units")
        if is_hand and units not in HAND_UNITS:
            raise MotionFormatError("{}.units must be '%' or 'dimensionless' for {!r}".format(
                path, name))
        if is_led and units != "%":
            raise MotionFormatError("{}.units must be '%' for LED {!r}".format(path, name))
        if not (is_hand or is_led) and units == "%":
            raise MotionFormatError(
                "{}.units is '%' for {!r}: only LHand, RHand and NAO V6 LED outputs "
                "take percentages".format(path, name))
        if not (is_hand or is_led) and units != "rad":
            raise MotionFormatError("{}.units must be 'rad' for {!r}".format(path, name))

        points = _list(channel.get("keys"), "{}.keys".format(path))
        converted_points = []
        previous_time = None
        for key_index, raw_key in enumerate(points):
            key_path = "{}.keys[{}]".format(path, key_index)
            key = _object(raw_key, key_path)
            stamp = _number(key.get("timeMs"), "{}.timeMs".format(key_path))
            value = _number(key.get("value"), "{}.value".format(key_path))
            # A fraction outside 0..1 means the export is not on that scale (a
            # percentage written as 0..100, say); NAOqi would clamp it to a limit
            # rather than refuse, so the mismatch would play as a slammed hand.
            if is_hand and not 0.0 <= value <= 1.0:
                raise MotionFormatError(
                    "{}.value must be a hand opening fraction from 0 to 1".format(key_path))
            if is_led and not 0.0 <= value <= 1.0:
                raise MotionFormatError(
                    "{}.value must be an LED intensity fraction from 0 to 1".format(key_path))
            if previous_time is not None and stamp <= previous_time:
                raise MotionFormatError("{}.timeMs must increase strictly".format(key_path))
            incoming = _handle(key, "in", key_index > 0, key_path)
            outgoing = _handle(key, "out", key_index < len(points) - 1, key_path)
            converted_points.append((stamp, [value, incoming, outgoing]))
            previous_time = stamp

        if is_led:
            _check_led_handles(converted_points, path)
        first_times.append(converted_points[0][0])
        validated.append((name, is_led, converted_points))

    # NAOqi times are seconds after the call and must be positive. A single
    # translation keeps every key spacing and handle offset from Studio intact,
    # and gives joints and LEDs the same time zero.
    shift_seconds = lead_in - min(first_times) / 1000.0
    names = []
    times = []
    keys = []
    led_lines = {}
    for name, is_led, points in validated:
        channel_times = [stamp / 1000.0 + shift_seconds for stamp, _ in points]
        if any(stamp <= 0 or math.isnan(stamp) or math.isinf(stamp) for stamp in channel_times):
            raise MotionFormatError("converted times for {!r} must be positive".format(name))
        channel_keys = [value for _, value in points]
        if not is_led:
            names.append(name)
            times.append(channel_times)
            keys.append(channel_keys)
        elif leds:
            led_lines[name] = _flatten_led(channel_times, channel_keys, LED_DEVICES[name])

    if not names and not led_lines:
        raise MotionFormatError("input has only LED channels, and leds is false")
    result = {"names": names, "times": times, "keys": keys}
    if led_lines:
        result["leds"] = _led_streams(led_lines)
    return result


def prepare_motion(source, lead_in_seconds=0.2, leds=True):
    """Load a Studio export path or dictionary and return reusable NAOqi arguments."""
    if isinstance(source, string_types):
        with io.open(source, "r", encoding="utf-8") as stream:
            source = json.load(stream)
    return convert_motion(source, lead_in_seconds, leds)


_clock = getattr(time, "monotonic", time.time)


def _sleep_until(deadline):
    remaining = deadline - _clock()
    if remaining > 0:
        time.sleep(remaining)


def _check_joints(motion_service, names):
    available = set(motion_service.getBodyNames("Body"))
    missing = sorted(set(names) - available)
    if missing:
        raise MotionFormatError("NAOqi does not provide joints: {}".format(", ".join(missing)))


def _check_leds(led_service, streams):
    available = set(led_service.listGroup("AllLeds"))
    devices = set()
    for stream in streams:
        if stream["method"] == "fadeListRGB":
            devices.update(stream["devices"])
        else:
            devices.add(stream["name"])
    missing = sorted(devices - available)
    if missing:
        raise MotionFormatError("NAOqi does not provide LEDs: {}".format(", ".join(missing)))
    lookups = dict((name, lookup) for name, lookup, _ in RGB_LED_POSITIONS)
    for stream in streams:
        if stream["method"] != "fadeListRGB":
            continue
        name = stream["name"]
        if name not in lookups:
            raise MotionFormatError("{!r} is not a NAO V6 RGB LED position".format(name))
        listed = getattr(led_service, lookups[name])(name)
        if set(listed) != set(stream["devices"]):
            raise MotionFormatError("ALLeds maps {!r} to {}, not {}".format(
                name, ", ".join(sorted(listed)), ", ".join(stream["devices"])))


def _play_together(motion_service, led_service, prepared_motion, streams):
    """Start the joint call and every LED stream together, then wait for all.

    NAOqi times count from the start of each call, so each worker subtracts
    the time that passed between the shared start and its own call. A chain of
    fades aims every fade at an absolute deadline, so a late return does not
    delay the keys after it, whether or not ALLeds blocks during a fade.
    """
    start = _clock()

    def lateness(first_time):
        late = _clock() - start
        if late >= first_time:
            raise MotionFormatError(
                "starting the NAOqi calls took {:.3f} s, longer than the lead-in".format(late))
        return late

    def joints():
        joint_times = prepared_motion["times"]
        late = lateness(min(channel[0] for channel in joint_times))
        return motion_service.angleInterpolationBezier(
            prepared_motion["names"],
            [[stamp - late for stamp in channel] for channel in joint_times],
            prepared_motion["keys"])

    def colors(stream):
        late = lateness(stream["timeList"][0])
        led_service.fadeListRGB(
            stream["name"], stream["rgbList"], [stamp - late for stamp in stream["timeList"]])

    def fades(stream):
        lateness(stream["timeList"][0])
        begin = 0.0
        for intensity, end in zip(stream["intensityList"], stream["timeList"]):
            _sleep_until(start + begin)
            led_service.fade(stream["name"], intensity, max(0.0, start + end - _clock()))
            begin = end

    tasks = []
    if prepared_motion["names"]:
        tasks.append((joints, ()))
    for stream in streams:
        tasks.append((colors if stream["method"] == "fadeListRGB" else fades, (stream,)))
    results = [None] * len(tasks)
    errors = [None] * len(tasks)

    def run(index):
        task, arguments = tasks[index]
        try:
            results[index] = task(*arguments)
        except Exception as error:
            errors[index] = error

    threads = [threading.Thread(target=run, args=(index,)) for index in range(len(tasks))]
    for thread in threads:
        thread.daemon = True
        thread.start()
    for thread in threads:
        thread.join()
    for error in errors:
        if error is not None:
            raise error

    end_times = [channel[-1] for channel in prepared_motion["times"]]
    end_times.extend(stream["timeList"][-1] for stream in streams)
    _sleep_until(start + max(end_times))
    return results[0] if prepared_motion["names"] else None


def play_motion(motion_service, prepared_motion, led_service=None):
    """Run prepared arguments on existing ALMotion and ALLeds proxies or qi services.

    The caller owns connection, stiffness, posture, and scheduling. This call
    blocks until the last key has played and returns the result of
    ``angleInterpolationBezier``. A motion with LED channels needs
    ``led_service``; prepare it with ``leds=False`` to play its joints alone.
    """
    names = prepared_motion["names"]
    streams = prepared_motion.get("leds") or []
    if streams and led_service is None:
        raise MotionFormatError(
            "the motion has LED channels but no ALLeds service was given; "
            "prepare it with leds=False to play the joints alone")
    if names or not streams:
        _check_joints(motion_service, names)
    if not streams:
        return motion_service.angleInterpolationBezier(
            names, prepared_motion["times"], prepared_motion["keys"]
        )
    _check_leds(led_service, streams)
    return _play_together(motion_service, led_service, prepared_motion, streams)


def play_effective_motion(motion_service, source, lead_in_seconds=0.2, led_service=None):
    """Load, convert, and play one Studio export through existing NAOqi services."""
    return play_motion(motion_service, prepare_motion(source, lead_in_seconds), led_service)
