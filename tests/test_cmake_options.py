#!/usr/bin/env python3
"""Check internal-state defaults, cache migration, and option compatibility."""

from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


CMAKE = sys.argv.pop(1)
OPTIONS_MODULE = Path(__file__).resolve().parents[1] / "cmake" / "IVPInternalState.cmake"


class InternalStateOptionsTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        root = Path(self.temp.name)
        self.source = root / "source"
        self.source.mkdir()
        self.build = root / "build"
        (self.source / "CMakeLists.txt").write_text(
            "cmake_minimum_required(VERSION 3.20)\n"
            "project(option_test NONE)\n"
            'include("${OPTIONS_MODULE}")\n',
            encoding="utf-8",
        )

    def configure(self, *options, success=True):
        result = subprocess.run(
            [CMAKE, "-S", str(self.source), "-B", str(self.build),
             f"-DOPTIONS_MODULE={OPTIONS_MODULE}", *options],
            text=True, capture_output=True, check=False,
        )
        if success:
            self.assertEqual(0, result.returncode, result.stdout + result.stderr)
        else:
            self.assertNotEqual(0, result.returncode)
        return result

    def cache(self):
        entries = {}
        for line in (self.build / "CMakeCache.txt").read_text(encoding="utf-8").splitlines():
            if line and not line.startswith(("//", "#")) and "=" in line:
                key, value = line.split("=", 1)
                entries[key.split(":", 1)[0]] = value
        return entries

    def test_fresh_default_and_explicit_stable_mode(self):
        self.configure()
        self.assertEqual("OFF", self.cache()["IVP_STABLE_INTERNAL_STATE"])
        self.assertEqual("OFF", self.cache()["IVP_EDIT_LOCALITY"])
        self.configure("-DIVP_STABLE_INTERNAL_STATE=ON")
        self.assertEqual("ON", self.cache()["IVP_STABLE_INTERNAL_STATE"])

    def test_legacy_values_migrate_and_survive_reconfigure(self):
        for old, new in (("ON", "OFF"), ("OFF", "ON")):
            with self.subTest(old=old):
                # A fresh build directory models an existing legacy cache.
                self.build = Path(self.temp.name) / f"legacy-{old}"
                result = self.configure(f"-DIVP_REFERENCE_INTERNAL_STATE={old}")
                self.assertIn("IVP_REFERENCE_INTERNAL_STATE", result.stderr)
                self.assertEqual(new, self.cache()["IVP_STABLE_INTERNAL_STATE"])
                self.assertNotIn("IVP_REFERENCE_INTERNAL_STATE", self.cache())
                self.configure()
                self.assertEqual(new, self.cache()["IVP_STABLE_INTERNAL_STATE"])

    def test_explicit_new_option_takes_precedence(self):
        self.configure("-DIVP_REFERENCE_INTERNAL_STATE=OFF",
                       "-DIVP_STABLE_INTERNAL_STATE=OFF")
        self.assertEqual("OFF", self.cache()["IVP_STABLE_INTERNAL_STATE"])
        self.assertNotIn("IVP_REFERENCE_INTERNAL_STATE", self.cache())

    def test_locality_requires_stable_internals(self):
        result = self.configure("-DIVP_EDIT_LOCALITY=ON", success=False)
        self.assertIn("IVP_EDIT_LOCALITY requires -DIVP_STABLE_INTERNAL_STATE=ON",
                      result.stderr)
        self.configure("-DIVP_STABLE_INTERNAL_STATE=ON")
        self.assertEqual("ON", self.cache()["IVP_EDIT_LOCALITY"])


if __name__ == "__main__":
    unittest.main()
