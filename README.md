# Studio effective motion to NAOqi Bézier

`semio_naoqi_motion.py` is a single-file module that an existing NAOqi Python
program can include to trigger animations exported from Studio. Copy it beside
that program or add this repository to its Python path. It converts **Export
Effective Motion** JSON into the three nested arrays accepted by
`ALMotion.angleInterpolationBezier(names, times, keys)`. It uses only the Python
standard library; pass it the `ALMotion` proxy or service your program already
uses. `convert_semio_effective_motion_to_naoqi_bezier.py` remains a separate
conversion command.

## Trigger animations from Python

For code using the classic NAOqi `ALProxy` API:

```python
from naoqi import ALProxy
from semio_naoqi_motion import (
    play_effective_motion, play_motion, prepare_motion,
)

motion = ALProxy("ALMotion", "<robot-ip>", 9559)

# One-off trigger. This call blocks until the animation finishes.
play_effective_motion(motion, "animations/greeting.effective-motion.json")

# Prepare once if GPT can request the same animation repeatedly.
ANIMATIONS = {
    "greeting": prepare_motion("animations/greeting.effective-motion.json"),
    "look_around": prepare_motion("animations/look_around.effective-motion.json"),
}

def trigger_animation(name):
    return play_motion(motion, ANIMATIONS[name])
```

The same functions accept `session.service("ALMotion")` from the `qi.Session`
API. They check every output against `ALMotion.getBodyNames("Body")` before
starting motion, then pass the converted arrays directly to
`angleInterpolationBezier`. Invalid or unknown joint names raise
`MotionFormatError`. `prepare_motion` accepts a path or a parsed Studio export
dictionary, so a program can load animations from its own storage.

Use the Python interpreter supported by your installed NAOqi SDK. This module
is written to be importable from Python 2.7 or Python 3; it has only been run
under Python 3 here. The downloaded Choregraphe 2.8.8 bundle includes an
x86_64 Python 2.7 runtime, but it cannot start on the arm64 development Mac.
The caller remains responsible for connecting, preparing the robot's posture
and stiffness, and deciding when motion is allowed. The
functions do not call `wakeUp`, set stiffness, or change posture. NAOqi's
`angleInterpolationBezier` call is blocking; schedule it outside any GPT event
loop that must continue responding during playback.

## Convert a file

The conversion CLI also works without a NAOqi SDK:

```sh
python3 convert_semio_effective_motion_to_naoqi_bezier.py \
  examples/look_around.effective-motion.json \
  --output look_around.naoqi.json
```

Omit `--output` to print JSON to standard output. Run `--help` to see all options.
The result has this shape:

```json
{
  "names": ["HeadYaw"],
  "times": [[0.2, 1.1]],
  "keys": [[
    [0.0, [3, 0.0, 0.0], [3, 0.3, 0.0]],
    [0.4, [3, -0.3, 0.0], [3, 0.0, 0.0]]
  ]]
}
```

The script preserves Studio's effective Bézier geometry. It places a key's
`out` handle in the following handle slot and the next key's `in` handle in
the preceding slot. A middle key can have different incoming and outgoing
handles. Unused first-in and last-out slots are `[3, 0, 0]`.

Studio exports animation-local times in milliseconds. NAOqi takes times in
seconds after the call starts. The script moves **every** key by the same
amount so the earliest key occurs at 0.2 seconds by default. Change that with
`--lead-in-seconds 0.5`; the shared time translation leaves durations and
handle offsets unchanged. The output does not bake a Studio scene instance's
offset or playback speed.

## Input contract

The input must have `format: "semio-effective-motion"`, `version: 1`,
`timeBasis: "animation-local"`, and `timeUnit: "ms"`. Each channel requires a
unique, resolved joint `output` and at least one key. Rotating joints require
`units: "rad"`. `LHand` and `RHand` carry hand opening, which NAOqi takes as a
fraction from 0 to 1; they accept `units: "%"`, which is how
Studio marks a percentage and writes as that same stored fraction, or
`units: "dimensionless"`. Their key values must lie within 0 to 1, so a
percentage written on a 0 to 100 scale is refused rather than clamped to a
limit on the robot. Every key needs finite `timeMs` and `value`. Each segment
requires the first key's `out`
and the next key's `in` handles. Their `deltaTimeMs` and `deltaValue` fields
are relative to their own keys. The example file shows the complete structure.

The converter rejects missing handles, repeated names, unordered timestamps,
incorrect channel units, non-finite values, and unsupported format versions.
It converts joint channels only: a non-joint output such as an LED channel is
refused, and the whole file with it.
Studio already resolves named easing, inferred handles, and playback clamps before
writing this file; this script does not interpret Studio authoring directives.

## NAOqi compatibility and execution

The intended robot is NAO V6, for which
[Aldebaran lists NAOqi 2.8](https://doc.aldebaran.com/).
The [Aldebaran 2.8 joint-control reference](https://docs.nextinsight.eu/doc.aldebaran.com/2-8/naoqi/motion/control-joint-api.html)
describes each Bézier key as an angle plus a preceding and following handle,
with each handle ordered `[InterpolationType, dTime, dAngle]`. This script emits
mode `3`, the Bézier handle form used in Choregraphe motion exports. Older
NAOqi references describe the last two handle fields in the opposite order;
check the installed SDK before execution. Choregraphe 2.8.8's bundled NAO
`idle.qianim` contains 26 curves, including degree-based joint angles and
dimensionless hand channels. All 26 curves (130 keys) passed a static
handle-mapping comparison after converting frames to seconds and degrees to
radians. Curve behavior on a physical robot has not been verified here.

The Python trigger checks joint names but cannot establish that every sampled
pose is achievable on a particular robot. Check joint limits, startup pose,
timing, and the target SDK before executing a new animation on hardware. In
particular, the handle mode and field order still need a runtime check against
the target NAO V6's NAOqi installation; the conversion tests alone cannot prove
curve equivalence on the robot.

## Tests

```sh
python3 -m unittest discover -s tests -v
```

The tests cover mixed key-side handles, shared time translation, curve geometry,
hand opening units and range, input validation, command-line output, and triggering through
a fake `ALMotion` service. They do not require a NAOqi SDK or robot and do not
verify physical playback.
