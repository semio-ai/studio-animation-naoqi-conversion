import json
import os
import re
import subprocess
import sys
import tempfile
import threading
import time
import unittest
import zipfile
from pathlib import Path
from xml.etree import ElementTree


PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))
from make_naoqi_animation_package import PackageError, build_package, parse_animation
from test_conversion import sample_input
from test_leds import (
    EAR, BlockingMotionService, FakeLedService, eased, head_yaw, led, linear,
)


XAR_NAMESPACE = "{http://www.aldebaran-robotics.com/schema/choregraphe/project.xsd}"
LEFT_EYE = "Face/Led/{}/Left/0Deg/Actuator/Value"


def short_export():
    """Head, left eye and one ear over 0.3 s."""
    data = sample_input()
    data["channels"] = [head_yaw([(0, 0.0), (300, 0.2)])]
    data["channels"].extend(led(LEFT_EYE.format(color), linear([(0, 0.0), (300, value)]))
                            for color, value in (("Red", 1.0), ("Green", 0.5), ("Blue", 0.0)))
    data["channels"].append(led(EAR, eased([(0, 0.0), (150, 1.0), (300, 0.0)])))
    return data


class FakeSession:
    def __init__(self, services):
        self.services = services

    def service(self, name):
        return self.services[name]


class FakeBehaviorManager:
    """Installs a .pkg and runs its behaviors' boxes as NAOqi would.

    Each behavior's Python box is read from the CDATA of its behavior.xar and
    run against a stand-in for Choregraphe's GeneratedClass.
    """

    def __init__(self, services):
        self.session = FakeSession(services)
        self.installed = {}
        self.running = {}
        self.logs = []
        self.finished = []

    def install(self, package, directory):
        with zipfile.ZipFile(package) as archive:
            archive.extractall(directory)
        manifest = ElementTree.parse(os.path.join(directory, "manifest.xml")).getroot()
        for content in manifest.iter("behaviorContent"):
            name = "{}/{}".format(manifest.get("uuid"), content.get("path"))
            self.installed[name] = os.path.join(directory, content.get("path"))

    def isBehaviorInstalled(self, name):
        return name in self.installed

    def load(self, name):
        directory = self.installed[name]
        with open(os.path.join(directory, "behavior.xar"), encoding="utf-8") as stream:
            scripts = [script for script in
                       re.findall(r"<!\[CDATA\[(.*?)\]\]>", stream.read(), re.DOTALL) if script]
        manager = self

        class GeneratedClass:
            def __init__(self, *arguments):
                pass

            def behaviorAbsolutePath(self):
                return directory

            def session(self):
                return manager.session

            def log(self, message):
                manager.logs.append(message)

            def onStopped(self):
                manager.finished.append(name)

        namespace = {"GeneratedClass": GeneratedClass}
        exec(compile(scripts[0], name, "exec"), namespace)
        box = namespace["MyClass"]()
        box.onLoad()
        return box

    def runBehavior(self, name):
        box = self.load(name)
        try:
            box.onInput_onStart()
        finally:
            box.onUnload()

    def startBehavior(self, name):
        box = self.load(name)
        thread = threading.Thread(target=box.onInput_onStart)
        thread.start()
        self.running[name] = (box, thread)

    def waitBehavior(self, name):
        self.running.pop(name)[1].join()

    def stopBehavior(self, name):
        box, thread = self.running.pop(name)
        box.onUnload()
        thread.join()


class FakeAnimatedSpeech:
    """Plays annotated text as the ALAnimatedSpeech 2.8 docs describe.

    ^run plays a behavior and waits; ^start starts one, ^wait waits for it and
    ^stop stops it; a behavior still running when the text ends is stopped.
    """

    INSTRUCTION = re.compile(r"\^(start|run|wait|stop)\(([^)]+)\)")

    def __init__(self, behavior_manager):
        self.behaviors = behavior_manager
        self.said = []

    def say(self, text, configuration=None):
        self.said.append((self.INSTRUCTION.sub("", text).strip(), configuration))
        started = []
        for kind, name in self.INSTRUCTION.findall(text):
            name = name.strip()
            if kind == "run":
                self.behaviors.runBehavior(name)
            elif kind == "start":
                self.behaviors.startBehavior(name)
                started.append(name)
            elif kind == "wait":
                self.behaviors.waitBehavior(name)
                started.remove(name)
            else:
                self.behaviors.stopBehavior(name)
                started.remove(name)
        for name in started:
            self.behaviors.stopBehavior(name)


class PackageTestCase(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.root = Path(self.directory.name)
        self.exports = {}
        for name, data in (("short", short_export()), ("long", sample_input())):
            path = self.root / "{}.effective-motion.json".format(name)
            path.write_text(json.dumps(data), encoding="utf-8")
            self.exports[name] = str(path)

    def tearDown(self):
        self.directory.cleanup()

    def build(self, **options):
        package = str(self.root / "semio.pkg")
        options.setdefault("lead_in_seconds", 0.05)
        names = build_package(
            [("short", self.exports["short"]), ("Stand/Long", self.exports["long"])],
            "semio", package, **options)
        return package, names

    def install(self, motion=None, leds=None):
        package, _ = self.build()
        self.motion = motion or BlockingMotionService()
        self.leds = leds or FakeLedService()
        manager = FakeBehaviorManager({"ALMotion": self.motion, "ALLeds": self.leds})
        manager.install(package, str(self.root / "installed"))
        return manager


class PackageLayoutTests(PackageTestCase):
    def test_package_holds_one_behavior_per_export(self):
        package, names = self.build()
        self.assertEqual(names, ["semio/short", "semio/Stand/Long"])
        with zipfile.ZipFile(package) as archive:
            self.assertEqual(archive.namelist(), [
                "manifest.xml", "semio.pml", "semio_naoqi_motion.py",
                "short/behavior.xar", "short/motion.effective-motion.json",
                "Stand/Long/behavior.xar", "Stand/Long/motion.effective-motion.json",
            ])
            self.assertEqual(archive.read("semio_naoqi_motion.py"),
                             (PROJECT / "semio_naoqi_motion.py").read_bytes())
            self.assertEqual(archive.read("Stand/Long/motion.effective-motion.json"),
                             Path(self.exports["long"]).read_bytes())
            manifest = ElementTree.fromstring(archive.read("manifest.xml"))
            project = ElementTree.fromstring(archive.read("semio.pml"))
        self.assertEqual((manifest.get("uuid"), manifest.get("version")), ("semio", "1.0.0"))
        self.assertEqual([content.get("path") for content in manifest.iter("behaviorContent")],
                         ["short", "Stand/Long"])
        self.assertEqual(manifest.find("requirements/naoqiRequirement").get("minVersion"), "2.8")
        self.assertEqual([entry.get("src") for entry in project.iter("BehaviorDescription")],
                         ["short", "Stand/Long"])

    def test_tags_and_names_reach_the_manifest(self):
        package, _ = self.build(name="Semio & friends", version="2.1",
                                tags={"short": ["glow", "hello there"]})
        with zipfile.ZipFile(package) as archive:
            manifest = ElementTree.fromstring(archive.read("manifest.xml"))
        self.assertEqual(manifest.find("names/name").text, "Semio & friends")
        self.assertEqual(manifest.get("version"), "2.1")
        contents = dict((content.get("path"), content)
                        for content in manifest.iter("behaviorContent"))
        self.assertEqual([tag.text for tag in contents["short"].iter("tag")],
                         ["glow", "hello there"])
        self.assertEqual(list(contents["Stand/Long"].iter("tag")), [])

    def test_behaviors_lock_what_they_animate(self):
        package, _ = self.build()
        with zipfile.ZipFile(package) as archive:
            short = ElementTree.fromstring(archive.read("short/behavior.xar"))
            long = ElementTree.fromstring(archive.read("Stand/Long/behavior.xar"))
        root = short.find(XAR_NAMESPACE + "Box")
        self.assertEqual(root.get("name"), "root")
        resources = root.findall(XAR_NAMESPACE + "Resource")
        # Wait 1 s at box startup, lock during execution, as stock animations do.
        self.assertEqual([(r.get("name"), r.get("type"), r.get("timeout")) for r in resources],
                         [("HeadYaw", "Lock", "1"), ("Left eye leds", "Lock", "1")])
        self.assertEqual([r.get("name") for r in long.iter(XAR_NAMESPACE + "Resource")],
                         ["HeadYaw", "HeadPitch"])
        links = [(link.get("outputowner"), link.get("indexofoutput"),
                  link.get("inputowner"), link.get("indexofinput"))
                 for link in root.iter(XAR_NAMESPACE + "Link")]
        # The root's onStart starts the box, and the box's onStopped ends the root.
        self.assertEqual(links, [("0", "2", "1", "2"), ("1", "4", "0", "4")])

    def test_rejects_what_the_robot_could_not_play(self):
        broken = sample_input()
        broken["channels"][0]["units"] = "deg"
        broken_path = self.root / "broken.json"
        broken_path.write_text(json.dumps(broken), encoding="utf-8")
        output = self.root / "never.pkg"
        cases = [
            ([("broken", str(broken_path))], {}, "broken.json: channels\\[0\\].units"),
            ([("a", self.exports["short"]), ("a", self.exports["long"])], {},
             "more than once"),
            ([("a", self.exports["short"]), ("a/b", self.exports["long"])], {},
             "also a folder"),
            ([("../a", self.exports["short"])], {}, "letters, digits"),
            ([("a b", self.exports["short"])], {}, "letters, digits"),
            ([("a", self.exports["short"])], {"tags": {"b": ["x"]}}, "unknown behaviors: b"),
            ([("a", self.exports["short"])], {"version": "one"}, "version"),
            ([], {}, "at least one"),
        ]
        for animations, options, message in cases:
            with self.subTest(message=message):
                with self.assertRaisesRegex(PackageError, message):
                    build_package(animations, "semio", str(output), **options)
                self.assertFalse(output.exists())
        with self.assertRaisesRegex(PackageError, "package id"):
            build_package([("a", self.exports["short"])], "semio animations", str(output))
        self.assertEqual(sorted(path.name for path in self.root.iterdir()),
                         ["broken.json", "long.effective-motion.json",
                          "short.effective-motion.json"])

    def test_behavior_names_come_from_the_file_or_name_equals(self):
        self.assertEqual(parse_animation("exports/wave_hello.effective-motion.json"),
                         ("wave_hello", "exports/wave_hello.effective-motion.json"))
        self.assertEqual(parse_animation("Stand/Wave=exports/a b.json"),
                         ("Stand/Wave", "exports/a b.json"))
        with self.assertRaisesRegex(PackageError, "give it as NAME="):
            parse_animation("exports/a b.json")

    def test_cli_writes_the_package_and_prints_the_instructions(self):
        output = self.root / "cli.pkg"
        completed = subprocess.run([
            sys.executable, str(PROJECT / "make_naoqi_animation_package.py"),
            "--package", "semio", "--output", str(output), "--tag", "short=glow",
            self.exports["short"], "Stand/Long=" + self.exports["long"],
        ], capture_output=True, text=True, check=True)
        self.assertIn("^start(semio/short) ... ^wait(semio/short)", completed.stdout)
        self.assertIn("^start(semio/Stand/Long)", completed.stdout)
        with zipfile.ZipFile(str(output)) as archive:
            self.assertIn("Stand/Long/behavior.xar", archive.namelist())
        failed = subprocess.run([
            sys.executable, str(PROJECT / "make_naoqi_animation_package.py"),
            "--package", "semio", "--tag", "nothing", self.exports["short"],
        ], capture_output=True, text=True)
        self.assertEqual(failed.returncode, 2)
        self.assertIn("must be NAME=TAG", failed.stderr)


class BehaviorTests(PackageTestCase):
    def test_box_plays_the_bundled_export_through_the_bundled_module(self):
        manager = self.install()
        self.assertTrue(manager.isBehaviorInstalled("semio/Stand/Long"))
        started = time.time()
        manager.runBehavior("semio/short")
        self.assertGreaterEqual(time.time() - started, 0.35 - 0.01)
        self.assertEqual(manager.finished, ["semio/short"])
        (names, times, _), = self.motion.calls
        self.assertEqual(names, ["HeadYaw"])
        self.assertAlmostEqual(times[0][-1], 0.35, delta=0.05)
        self.assertEqual(len(self.leds.named("fadeListRGB", "LeftFaceLed1")), 1)
        self.assertTrue(self.leds.named("fade", EAR))
        self.assertEqual(self.motion.kills, [])

    def test_box_does_not_move_a_robot_that_is_not_awake(self):
        manager = self.install(motion=BlockingMotionService(awake=False))
        manager.runBehavior("semio/short")
        self.assertEqual(manager.finished, ["semio/short"])
        self.assertEqual(self.motion.calls, [])
        self.assertEqual(self.leds.calls, [])
        self.assertEqual(manager.logs, [
            "semio/short: the robot is not awake, so the animation does not play"])

    def test_stopping_the_behavior_stops_the_animation(self):
        manager = self.install()
        manager.startBehavior("semio/Stand/Long")
        time.sleep(0.1)
        stopped = time.time()
        manager.stopBehavior("semio/Stand/Long")
        self.assertLess(time.time() - stopped, 0.2)
        self.assertEqual(self.motion.kills, [["HeadYaw", "HeadPitch"]])
        self.assertEqual(manager.finished, ["semio/Stand/Long"])


class AnimatedSpeechTests(PackageTestCase):
    def test_start_and_wait_play_the_whole_animation(self):
        manager = self.install()
        speech = FakeAnimatedSpeech(manager)
        speech.say("^start(semio/short) Look at my eyes! ^wait(semio/short)",
                   {"bodyLanguageMode": "disabled"})
        self.assertEqual(speech.said, [("Look at my eyes!", {"bodyLanguageMode": "disabled"})])
        self.assertEqual(manager.finished, ["semio/short"])
        self.assertEqual(self.motion.kills, [])
        self.assertEqual(len(self.motion.calls), 1)

    def test_run_plays_before_the_rest_of_the_text(self):
        manager = self.install()
        FakeAnimatedSpeech(manager).say("^run(semio/short) Done.")
        self.assertEqual(manager.finished, ["semio/short"])
        self.assertEqual(self.motion.kills, [])

    def test_an_animation_still_running_when_the_text_ends_is_stopped(self):
        manager = self.install()
        FakeAnimatedSpeech(manager).say("^start(semio/Stand/Long) Hi!")
        self.assertEqual(self.motion.kills, [["HeadYaw", "HeadPitch"]])
        self.assertEqual(manager.finished, ["semio/Stand/Long"])


if __name__ == "__main__":
    unittest.main()
