# Studio effective motion to NAOqi

`semio_naoqi_motion.py` is a single-file module that an existing NAOqi Python
program can include to trigger animations exported from Studio. Copy it beside
that program or add this repository to its Python path. It converts **Export
Effective Motion** JSON into the three nested arrays accepted by
`ALMotion.angleInterpolationBezier(names, times, keys)` for joint channels, and
into `ALLeds` calls for the LED channels of a NAO V6. It uses only the Python
standard library; pass it the `ALMotion` and `ALLeds` proxies or services your
program already uses. `convert_semio_effective_motion_to_naoqi_bezier.py`
remains a separate conversion command, and `probe_naoqi_leds.py` measures the
ALLeds behavior that LED playback relies on
([Checking LED playback on a robot](#checking-led-playback-on-a-robot)).

## Trigger animations from Python

For code using the classic NAOqi `ALProxy` API:

```python
from naoqi import ALProxy
from semio_naoqi_motion import (
    play_effective_motion, play_motion, prepare_motion,
)

motion = ALProxy("ALMotion", "<robot-ip>", 9559)
leds = ALProxy("ALLeds", "<robot-ip>", 9559)

# One-off trigger. This call blocks until the animation finishes.
play_effective_motion(motion, "animations/greeting.effective-motion.json",
                      led_service=leds)

# Prepare once if GPT can request the same animation repeatedly.
ANIMATIONS = {
    "greeting": prepare_motion("animations/greeting.effective-motion.json"),
    "look_around": prepare_motion("animations/look_around.effective-motion.json"),
}

def trigger_animation(name):
    return play_motion(motion, ANIMATIONS[name], leds)
```

The same functions accept `session.service("ALMotion")` and
`session.service("ALLeds")` from the `qi.Session` API. Before anything moves,
they check every joint against `ALMotion.getBodyNames("Body")` and every LED
against `ALLeds.listGroup("AllLeds")`; unknown names raise `MotionFormatError`.
An animation without LED channels is passed directly to
`angleInterpolationBezier`, and needs no ALLeds service. An animation with LED
channels does: without one, `play_motion` raises rather than play the joints
alone. To play only the joints of such an animation, prepare it with
`prepare_motion(path, leds=False)`. `prepare_motion` accepts a path or a parsed
Studio export dictionary, so a program can load animations from its own storage.

Use the Python interpreter supported by your installed NAOqi SDK. This module
is written to be importable from Python 2.7 or Python 3; it has only been run
under Python 3 here. The downloaded Choregraphe 2.8.8 bundle includes an
x86_64 Python 2.7 runtime, but it cannot start on the arm64 development Mac.
The caller remains responsible for connecting, preparing the robot's posture
and stiffness, and deciding when motion is allowed. The
functions do not call `wakeUp`, set stiffness, or change posture. LEDs keep the
values of their last keys; call `ALLeds.reset` to return them to their
defaults. `play_motion` blocks until the last key has played; schedule it
outside any GPT event loop that must continue responding during playback.

## LED channels

A NAO V6 has 89 LED devices. Studio names its LED outputs after them, and the
converter plays them through ALLeds:

| LEDs | Studio outputs | ALLeds calls |
|---|---|---|
| Eyes, 16 RGB positions | `Face/Led/{Red,Green,Blue}/{Left,Right}/{0,45,...,315}Deg/Actuator/Value` | one `fadeListRGB` per position, on `RightFaceLed1`-`8` and `LeftFaceLed1`-`8` |
| Chest, RGB | `ChestBoard/Led/{Red,Green,Blue}/Actuator/Value` | one `fadeListRGB` on `ChestLeds` |
| Feet, RGB | `{LFoot,RFoot}/Led/{Red,Green,Blue}/Actuator/Value` | one `fadeListRGB` each, on `LeftFootLeds` and `RightFootLeds` |
| Ears, 20 blue | `Ears/Led/{Left,Right}/{0,36,...,324}Deg/Actuator/Value` | a chain of `fade` calls per LED |
| Head, 12 white | `Head/Led/{Front,Middle,Rear}/{Left,Right}/<n>/Actuator/Value` | a chain of `fade` calls per LED |

- **Flattening.** ALLeds moves between targets rather than along curves, so
  each LED curve becomes straight segments. A segment is halved while the
  Bézier curve could stray from it by more than half a brightness step (the
  eyes have 64 levels, the chest and feet 256, the ears and head 16), as long
  as both halves stay at least 24 ms long, two LoLA cycles. The curve lies
  within the hull of its control points, so their distance from the segment
  bounds the curve's. Where a curve changes faster than the spacing allows,
  the spacing wins. Every Studio key stays a breakpoint, and values are
  clamped to 0..1.
- **RGB positions.** When an export animates all three channels of an eye
  position, the chest or a foot, they play as one `fadeListRGB`. It is sampled
  at the union of the three channels' breakpoints and packed as `0x00RRGGBB`.
  When an export animates only some of a position's channels, each animated
  channel plays as its own chain of fades and the others are left as they are.
- **Timing.** Joints and LEDs share one time zero: the earliest key of any
  channel occurs at the lead-in. As joints do, each LED moves from its current
  state to its first key's value by that key's time, follows the curve, and
  holds its last value. Within one RGB position, a channel whose keys start
  later than the others' holds its first value until then, as in Studio.
- **Playback.** `play_motion` starts the joint call and every LED stream from
  its own thread. NAOqi times count from the start of each call, so each call
  subtracts the time that passed since the shared start. A chain of fades aims
  each fade at an absolute deadline, so a late return does not delay the keys
  after it. `play_motion` returns once the last key's time has passed, whether
  or not ALLeds calls block. If any call raises, it re-raises the first error
  after the others finish.
- **Name check.** Each play first calls `listGroup("AllLeds")`, and then lists
  each RGB position it plays with `listLED` or `listGroup` to confirm that the
  position names the expected three devices.

## Convert a file

The conversion CLI also works without a NAOqi SDK:

```sh
python3 convert_semio_effective_motion_to_naoqi_bezier.py \
  examples/look_around.effective-motion.json \
  --output look_around.naoqi.json
```

Omit `--output` to print JSON to standard output. Run `--help` to see all options.
For a joint-only export, the result has this shape:

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

When the export has LED channels, the output also lists the ALLeds calls that
`play_motion` makes, so it doubles as a dry run. Times are as planned, before
each call subtracts its issue delay. For
`examples/look_and_glow.effective-motion.json` it begins:

```json
"leds": [
  {
    "method": "fadeListRGB",
    "name": "ChestLeds",
    "devices": ["ChestBoard/Led/Red/Actuator/Value", "...Green...", "...Blue..."],
    "rgbList": [19967, 85502, 216572, "..."],
    "timeList": [0.2, 0.228, 0.256, "..."]
  },
  {
    "method": "fade",
    "name": "Ears/Led/Right/0Deg/Actuator/Value",
    "intensityList": [0.0, 0.688, 0.914, "..."],
    "timeList": [0.2, 0.425, 0.538, "..."]
  }
]
```

`rgbList` holds `0x00RRGGBB` integers for `fadeListRGB(name, rgbList,
timeList)`. A `fade` stream plays as one `fade(name, intensity, duration)` per
entry, timed to reach each intensity at its time. `--no-leds` checks the LED
channels but leaves them out, without moving the joints' times.

## Input contract

The input must have `format: "semio-effective-motion"`, `version: 1`,
`timeBasis: "animation-local"`, and `timeUnit: "ms"`. Each channel requires a
unique, resolved joint or LED `output` and at least one key. Rotating joints require
`units: "rad"`. `LHand` and `RHand` carry hand opening, which NAOqi takes as a
fraction from 0 to 1; they accept `units: "%"`, which is how
Studio marks a percentage and writes as that same stored fraction, or
`units: "dimensionless"`. Their key values must lie within 0 to 1, so a
percentage written on a 0 to 100 scale is refused rather than clamped to a
limit on the robot. The [LED outputs](#led-channels) require `units: "%"`
and key values within 0 to 1, Studio's stored intensity fraction. Their
handles must stay within their segment, so that each LED curve is a function
of time. No other output takes `"%"`, and no other output whose name
contains `/` is converted, since NAOqi joint names have none. Every key needs finite `timeMs` and
`value`. Each segment requires the first key's `out`
and the next key's `in` handles. Their `deltaTimeMs` and `deltaValue` fields
are relative to their own keys. The example files show the complete structure.

The converter rejects missing handles, repeated names, unordered timestamps,
incorrect channel units, non-finite values, LED handles that leave their
segment, and unsupported format versions.
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

LED playback uses ALLeds only. On NAOqi 2.8, LoLA replaces the DCM and
[removes direct actuator control](https://docs.nextinsight.eu/doc.aldebaran.com/2-8/naoqi/lola/lola.html),
so ALLeds is the documented way to drive LEDs. The
[ALLeds 2.8 API](https://docs.nextinsight.eu/doc.aldebaran.com/2-8/naoqi/sensors/alleds-api.html)
lists `fade` and `fadeListRGB` but says little about their timing. Playback
rests on these assumptions, none of which has been checked on a robot:

1. `fadeListRGB` times are seconds from the start of the call, like
   `angleInterpolationBezier`, not durations of each step.
2. ALLeds interpolates linearly between targets.
3. A fade that reaches an LED as an earlier one ends replaces it. Chained fades
   are timed not to overlap, so this matters only when a fade returns late.
4. ALLeds accepts up to 70 concurrent calls and colour lists of a few hundred
   entries. 70 is the most `play_motion` makes at once: one chain of fades for
   each of two channels in all 19 RGB positions, and for all 32 ear and head
   LEDs.
5. `listGroup("AllLeds")` lists all 89 devices, and the short names and groups
   in the table name the documented devices. `play_motion` checks this on
   every play and refuses to start otherwise.

The tests check the conversion and the call scheduling against fake services.
They cannot show what the LEDs do.

## Checking LED playback on a robot

`probe_naoqi_leds.py` connects to a robot with the NAOqi Python SDK and runs
checks that answer the questions above. Run it from this directory with the
SDK's interpreter:

```sh
python probe_naoqi_leds.py --ip <robot-ip>
```

Name checks to run only those, for example `python probe_naoqi_leds.py --ip
<robot-ip> times overlap`. Each check prints what the robot did; none decides
pass or fail.

| Check | What it shows |
|---|---|
| `blocking` | Whether `fade`, `fadeRGB` and `fadeListRGB` return when their last target is reached. Playback works either way. |
| `times` | Whether `fadeListRGB` times are absolute or per step, the ramp's shape, and which time lists are accepted (assumptions 1 and 2). |
| `single` | Whether `fadeListRGB` drives a single-colour ear, head or face device, and from which byte. If it does, the ears and head could play as lists instead of fade chains. |
| `overlap` | Whether a command that reaches an LED mid-fade cancels it, queues behind it or blends with it, and whether cancelling a qi future stops a fade (assumption 3). |
| `limits` | Long colour lists, a fade on all 89 LEDs at once, and the shortest effective fade (assumption 4). |
| `skew` | How far apart `play_motion`'s calls start, and when an LED changes relative to the plan. With `--allow-motion`, it also turns the head by 0.15 rad and back, and samples `HeadYaw`. |
| `names` | Whether the robot lists the 89 devices and maps the 19 RGB positions as the converter expects (assumption 5). |

The probe reads LEDs back with `ALLeds.getIntensity`, which reports ALLeds' own
view; a camera or photodiode shows what the LEDs actually do. It resets the
LEDs after each check. Only `skew --allow-motion` moves the robot, and only if
it is awake. LEDs cannot be tested on a simulated robot.

## Tests

```sh
python3 -m unittest discover -s tests -v
```

The tests cover mixed key-side handles, shared time translation, curve geometry,
hand opening units and range, input validation, and command-line output. For
LEDs, they cover the device table against the documented names, flattening
against densely sampled Bézier curves, RGB merging, and playback through fake
`ALMotion` and `ALLeds` services: call timing and deadlines, robot name checks,
and error propagation. A smoke test runs every probe check against a simulated
ALLeds. The tests do not require a NAOqi SDK or robot and do not verify
physical playback.
