#!/usr/bin/env python
"""Build a NAOqi package that plays Studio effective-motion exports as behaviors.

Each export becomes one behavior, which ALAnimatedSpeech (^start, ^run, ^wait,
^stop) and ALAnimationPlayer.run name as <package>/<behavior>.
"""

import argparse
import io
import os
import re
import sys
import tempfile
import zipfile
from xml.sax.saxutils import escape, quoteattr

from semio_naoqi_motion import MotionFormatError, prepare_motion


HERE = os.path.dirname(os.path.abspath(__file__))
MODULE = "semio_naoqi_motion.py"
MOTION = "motion.effective-motion.json"
SEGMENT = re.compile(r"^[A-Za-z0-9_-]+$")
VERSION = re.compile(r"^[0-9]+(\.[0-9]+)*$")

# The per-joint resource names that NAOqi's stock NAO animations lock. A
# behavior that locks the joints it animates makes ALSpeakingMovement yield
# them; any other joint falls back to locking every motor.
JOINT_RESOURCES = frozenset([
    "HeadYaw", "HeadPitch",
    "LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw", "LHand",
    "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw", "RHand",
    "LHipYawPitch", "LHipRoll", "LHipPitch", "LKneePitch", "LAnklePitch", "LAnkleRoll",
    "RHipRoll", "RHipPitch", "RKneePitch", "RAnklePitch", "RAnkleRoll",
])

BOX_SCRIPT = '''import os


def load_semio_motion(path):
    # A module name of this package's own, so other copies do not collide.
    name = {module_name!r}
    try:
        import importlib.util
    except ImportError:
        import imp
        return imp.load_source(name, path)
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class MyClass(GeneratedClass):
    def __init__(self):
        GeneratedClass.__init__(self)
        self.playback = None

    def onLoad(self):
        self.unloading = False
        here = self.behaviorAbsolutePath()
        self.semio = load_semio_motion(os.path.join(here, {root!r}, {module!r}))
        self.prepared = self.semio.prepare_motion(os.path.join(here, {motion!r}), {lead_in!r})
        self.motion = self.session().service("ALMotion")
        self.leds = None
        if self.prepared.get("leds"):
            self.leds = self.session().service("ALLeds")

    def onUnload(self):
        self.unloading = True
        playback = self.playback
        if playback is not None:
            playback.stop()

    def onInput_onStart(self):
        if not self.motion.robotIsWakeUp():
            self.log({not_awake!r})
            self.onStopped()
            return
        self.playback = self.semio.start_motion(self.motion, self.prepared, self.leds)
        try:
            # The behavior may have been stopped while the motion was starting.
            if self.unloading:
                self.playback.stop()
            self.playback.wait()
        finally:
            self.playback = None
            self.onStopped()

    def onInput_onStop(self):
        self.onUnload()
'''

PORTS = '''{indent}<Input name="onLoad" type="1" type_size="1" nature="0" inner="1" tooltip="Signal sent when diagram is loaded." id="1" />
{indent}<Input name="onStart" type="1" type_size="1" nature="2" inner="0" tooltip="Box behavior starts when a signal is received on this input." id="2" />
{indent}<Input name="onStop" type="1" type_size="1" nature="3" inner="0" tooltip="Box behavior stops when a signal is received on this input." id="3" />
{indent}<Output name="onStopped" type="1" type_size="1" nature="1" inner="0" tooltip="Signal sent when box behavior is finished." id="4" />'''

BEHAVIOR = '''<?xml version="1.0" encoding="UTF-8" ?>
<ChoregrapheProject xmlns="http://www.aldebaran-robotics.com/schema/choregraphe/project.xsd" xar_version="3">
    <Box name="root" id="-1" localization="8" tooltip={tooltip} x="0" y="0">
        <bitmap>media/images/box/root.png</bitmap>
        <script language="4">
            <content>
                <![CDATA[]]>
</content>
        </script>
{root_ports}
        <Timeline enable="0">
            <BehaviorLayer name="behavior_layer1">
                <BehaviorKeyframe name="keyframe1" index="1">
                    <Diagram>
                        <Box name="Play Studio motion" id="1" localization="8" tooltip={tooltip} x="200" y="40">
                            <bitmap>media/images/box/movement/move.png</bitmap>
                            <script language="4">
                                <content>
                                    <![CDATA[{script}]]>
</content>
                            </script>
{box_ports}
                        </Box>
                        <Link inputowner="1" indexofinput="2" outputowner="0" indexofoutput="2" />
                        <Link inputowner="0" indexofinput="4" outputowner="1" indexofoutput="4" />
                    </Diagram>
                </BehaviorKeyframe>
            </BehaviorLayer>
        </Timeline>
{resources}
    </Box>
</ChoregrapheProject>
'''


class PackageError(ValueError):
    """The animations cannot be packaged as given."""


def behavior_path(text):
    segments = text.split("/")
    if not all(SEGMENT.match(segment) for segment in segments):
        raise PackageError(
            "behavior name {!r} must be letters, digits, '_' or '-', with '/' between "
            "folders".format(text))
    return text


def parse_animation(argument):
    """Return ``(behavior, path)`` for ``NAME=PATH`` or a bare export path."""
    if "=" in argument:
        name, path = argument.split("=", 1)
        return behavior_path(name), path
    name = os.path.basename(argument)
    for suffix in (".effective-motion.json", ".json"):
        if name.endswith(suffix):
            name = name[:-len(suffix)]
            break
    try:
        return behavior_path(name), argument
    except PackageError:
        raise PackageError(
            "cannot name a behavior after {!r}; give it as NAME={}".format(argument, argument))


def lock_resources(prepared):
    """Return the resources a behavior locks: its joints, and its eyes when it lights them."""
    resources = []
    for joint in prepared["names"]:
        resources.append(joint if joint in JOINT_RESOURCES else "All motors")
    devices = []
    for stream in prepared.get("leds", []):
        devices.extend(stream["devices"] if stream["method"] == "fadeListRGB" else [stream["name"]])
    for side in ("Left", "Right"):
        if any(device.startswith("Face/") and "/{}/".format(side) in device for device in devices):
            resources.append("{} eye leds".format(side))
    return sorted(set(resources), key=resources.index)


def behavior_xar(package, behavior, prepared, lead_in_seconds):
    script = BOX_SCRIPT.format(
        module_name="semio_naoqi_motion_" + package.replace("-", "_"),
        root="/".join([".."] * len(behavior.split("/"))),
        module=MODULE,
        motion=MOTION,
        lead_in=lead_in_seconds,
        not_awake="{}/{}: the robot is not awake, so the animation does not play".format(
            package, behavior),
    )
    resources = "\n".join(
        '        <Resource name={} type="Lock" timeout="1" />'.format(quoteattr(name))
        for name in lock_resources(prepared))
    return BEHAVIOR.format(
        tooltip=quoteattr("Plays the Studio animation {}.".format(behavior)),
        root_ports=PORTS.format(indent=" " * 8),
        box_ports=PORTS.format(indent=" " * 28),
        script=script,
        resources=resources,
    )


def manifest_xml(package, name, version, behaviors, tags):
    lines = [
        "<?xml version='1.0' encoding='UTF-8'?>",
        '<package version={} typeVersion="1.0" uuid={}>'.format(
            quoteattr(version), quoteattr(package)),
        " <names>",
        '  <name lang="en_US">{}</name>'.format(escape(name)),
        " </names>",
        " <descriptions>",
        '  <description lang="en_US">Studio animations for ALAnimatedSpeech and '
        'ALAnimationPlayer.</description>',
        " </descriptions>",
        " <descriptionLanguages>",
        "  <language>en_US</language>",
        " </descriptionLanguages>",
        " <contents>",
    ]
    for behavior in behaviors:
        lines.append("  <behaviorContent path={}>".format(quoteattr(behavior)))
        lines.append("   <nature></nature>")
        if tags.get(behavior):
            lines.append("   <tags>")
            lines.extend('    <tag lang="en_US">{}</tag>'.format(escape(tag))
                         for tag in tags[behavior])
            lines.append("   </tags>")
        lines.append("   <permissions/>")
        lines.append("  </behaviorContent>")
    lines.extend([
        " </contents>",
        " <requirements>",
        '  <naoqiRequirement minVersion="2.8"/>',
        " </requirements>",
        "</package>",
        "",
    ])
    return "\n".join(lines)


def project_pml(package, behaviors):
    lines = [
        '<?xml version="1.0" encoding="UTF-8" ?>',
        '<Package name={} format_version="4">'.format(quoteattr(package)),
        '    <Manifest src="manifest.xml" />',
        "    <BehaviorDescriptions>",
    ]
    lines.extend('        <BehaviorDescription name="behavior" src={} xar="behavior.xar" />'.format(
        quoteattr(behavior)) for behavior in behaviors)
    lines.extend([
        "    </BehaviorDescriptions>",
        "    <Dialogs />",
        "    <Resources>",
        '        <File name="semio_naoqi_motion" src="{}" />'.format(MODULE),
    ])
    lines.extend('        <File name="motion.effective-motion" src={} />'.format(
        quoteattr("{}/{}".format(behavior, MOTION))) for behavior in behaviors)
    lines.extend([
        "    </Resources>",
        "    <Topics />",
        "    <IgnoredPaths />",
        "</Package>",
        "",
    ])
    return "\n".join(lines)


def build_package(animations, package, output, name=None, version="1.0.0", tags=None,
                  lead_in_seconds=0.2):
    """Write a ``.pkg`` with one behavior per ``(behavior, export path)`` pair.

    Every export is converted first, so an export the robot would refuse stops
    the build instead. Returns the behaviors' full names, as ALAnimatedSpeech
    and ALAnimationPlayer take them.
    """
    if not SEGMENT.match(package):
        raise PackageError("package id {!r} must be letters, digits, '_' or '-'".format(package))
    if not VERSION.match(version):
        raise PackageError("version {!r} must be numbers separated by dots".format(version))
    if not animations:
        raise PackageError("give at least one Studio export")
    tags = dict(tags or {})
    behaviors = [behavior_path(behavior) for behavior, _ in animations]
    for behavior in behaviors:
        if behaviors.count(behavior) > 1:
            raise PackageError("behavior {!r} is given more than once".format(behavior))
        if any(other.startswith(behavior + "/") for other in behaviors):
            raise PackageError("behavior {!r} is also a folder of another".format(behavior))
    unknown = sorted(set(tags) - set(behaviors))
    if unknown:
        raise PackageError("tags name unknown behaviors: {}".format(", ".join(unknown)))

    entries = []
    for behavior, path in animations:
        with io.open(path, "rb") as stream:
            export = stream.read()
        try:
            prepared = prepare_motion(path, lead_in_seconds)
        except MotionFormatError as error:
            raise PackageError("{}: {}".format(path, error))
        xar = behavior_xar(package, behavior, prepared, lead_in_seconds)
        if xar.count("]]>") != 2:
            raise PackageError("the box script for {!r} cannot sit in CDATA".format(behavior))
        entries.append(("{}/behavior.xar".format(behavior), xar.encode("utf-8")))
        entries.append(("{}/{}".format(behavior, MOTION), export))
    with io.open(os.path.join(HERE, MODULE), "rb") as stream:
        module = stream.read()
    entries[:0] = [
        ("manifest.xml", manifest_xml(package, name or package, version, behaviors,
                                      tags).encode("utf-8")),
        ("{}.pml".format(package), project_pml(package, behaviors).encode("utf-8")),
        (MODULE, module),
    ]

    # Write beside the output and rename, so a failed build leaves no package.
    directory = os.path.dirname(os.path.abspath(output))
    handle, temporary = tempfile.mkstemp(suffix=".pkg", dir=directory)
    os.close(handle)
    try:
        with zipfile.ZipFile(temporary, "w", zipfile.ZIP_DEFLATED) as archive:
            for entry, data in entries:
                info = zipfile.ZipInfo(entry, date_time=(1980, 1, 1, 0, 0, 0))
                info.compress_type = zipfile.ZIP_DEFLATED
                info.external_attr = 0o644 << 16
                archive.writestr(info, data)
        getattr(os, "replace", os.rename)(temporary, output)
    except Exception:
        os.remove(temporary)
        raise
    return ["{}/{}".format(package, behavior) for behavior in behaviors]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "animations", nargs="+", metavar="[NAME=]EXPORT",
        help="a Studio .effective-motion.json file; the behavior is named after the file "
             "unless NAME= is given, and NAME may contain '/' folders",
    )
    parser.add_argument("--package", required=True,
                        help="package id, the first part of every behavior's name")
    parser.add_argument("-o", "--output", help="package file (default: <package>.pkg)")
    parser.add_argument("--name", help="display name (default: the package id)")
    parser.add_argument("--version", default="1.0.0", help="package version (default: 1.0.0)")
    parser.add_argument(
        "--tag", action="append", default=[], metavar="NAME=TAG",
        help="tag a behavior for ^startTag and ALAnimationPlayer.runTag; repeatable",
    )
    parser.add_argument(
        "--lead-in-seconds", type=float, default=0.2,
        help="time of the earliest key after the behavior starts (default: 0.2)",
    )
    args = parser.parse_args(argv)

    try:
        animations = [parse_animation(argument) for argument in args.animations]
        tags = {}
        for argument in args.tag:
            behavior, separator, tag = argument.partition("=")
            if not separator or not tag.strip():
                raise PackageError("--tag {!r} must be NAME=TAG".format(argument))
            tags.setdefault(behavior, []).append(tag.strip())
        output = args.output or "{}.pkg".format(args.package)
        names = build_package(animations, args.package, output, args.name, args.version,
                              tags, args.lead_in_seconds)
    except (IOError, OSError, ValueError) as error:
        parser.exit(2, "error: {}\n".format(error))
    sys.stdout.write("Wrote {} with {} behavior{}:\n".format(
        output, len(names), "" if len(names) == 1 else "s"))
    for name in names:
        sys.stdout.write("  ^start({0}) ... ^wait({0})\n".format(name))


if __name__ == "__main__":
    main()
