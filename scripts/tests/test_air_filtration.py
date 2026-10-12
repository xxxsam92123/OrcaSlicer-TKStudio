#!/usr/bin/env python3
"""Tests for the BBL air-filtration guard: check-air-filtration, fix-air-filtration
and the version bump the repair implies (stdlib unittest, no external deps).

Upstream re-exports resources/profiles/BBL/ wholesale from Bambu Studio's own
folder every so often, and the copy that lands carries support_air_filtration
where Bambu's folder does -- on the base (which disables it) and on one machine
-- but not on the nine machines this fork declares it on, so a sync strips those
nine and nothing else in the tree fails. The guard has to notice that, and the
repair has to put one line back into each file without reserialising it --
exactly once.

Run from the repo root:  python -m unittest discover -s scripts/tests -v
"""

import contextlib
import io
import json
import os
import shutil
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import orca_profile_tool as apt  # noqa: E402

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
REAL_PROFILES = os.path.join(REPO_ROOT, "resources", "profiles")
VENDOR = apt.AIR_FILTRATION_VENDOR
KEY = apt.AIR_FILTRATION_KEY
H2D = "Bambu Lab H2D 0.4 nozzle.json"  # tab-indented, carries both filter anchors
P1S = "Bambu Lab P1S 0.4 nozzle.json"  # four-space indented, carries neither

# How each guarded machine looks in the shipped tree: its indentation, the settings
# block it carries after the identifying keys, and where the key sits inside that
# block. The names and their order are the real ones; the values are this fixture's
# own. All three matter, because they are what the repair has to preserve and what
# it picks the insert point from -- guessed from the machine's name they would hide
# a regression in exactly that.
SHAPES = {
    H2D: ("\t", ("support_cooling_filter", "support_chamber_temp_control"), 1),
    "Bambu Lab H2D Pro 0.4 nozzle.json":
        ("\t", ("support_cooling_filter", "support_chamber_temp_control"), 1),
    "Bambu Lab H2S 0.4 nozzle.json":
        ("\t", ("support_chamber_temp_control", "support_cooling_filter"), 0),
    "Bambu Lab H2C 0.4 nozzle.json":
        ("\t", ("support_chamber_temp_control", "support_cooling_filter"), 0),
    "Bambu Lab P1S 0.4 nozzle.json":
        ("    ", ("retraction_distances_when_cut", "wipe_distance"), 1),
    "Bambu Lab P2S 0.4 nozzle.json":
        ("\t", ("printable_height", "printer_extruder_id"), 1),
    "Bambu Lab X1 0.4 nozzle.json": ("    ", ("scan_first_layer", "wipe_distance"), 1),
    "Bambu Lab X1 Carbon 0.4 nozzle.json":
        ("    ", ("scan_first_layer", "wipe_distance"), 1),
    "Bambu Lab X2D 0.4 nozzle.json": ("    ", ("support_chamber_temp_control",), 1),
}


class GuardTree:
    """A throwaway resources/profiles tree holding just the guarded BBL machines.

    Shaped like the shipped files (see SHAPES): the same indentation, the same
    settings block, the same place in it for the key, and the caller picks the line
    ending. A repair has to keep all of them, and has to work out where the line
    goes from the anchors alone.
    """

    OTHER = "Other"  # an empty bundle beside BBL, so --vendor has something to name

    def __init__(self, nl="\r\n"):
        self.dir = tempfile.mkdtemp(prefix="air_filtration_test_")
        self.profiles = os.path.join(self.dir, "profiles")
        self.nl = nl
        self.version = "02.08.00.13"
        os.makedirs(os.path.join(self.profiles, self.OTHER))
        os.makedirs(self.machine_dir)
        self.write_index()
        for name in apt.AIR_FILTRATION_MACHINES:
            self.write_machine(name)

    @property
    def machine_dir(self):
        return os.path.join(self.profiles, VENDOR, "machine")

    @property
    def index_path(self):
        return os.path.join(self.profiles, VENDOR + ".json")

    def cleanup(self):
        shutil.rmtree(self.dir, ignore_errors=True)

    def indent(self, name):
        """The shipped indentation of this machine's file."""
        return SHAPES[name][0]

    def machine_path(self, name):
        return os.path.join(self.machine_dir, name)

    def write_index(self):
        index = json.dumps({"name": "Bambulab", "version": self.version},
                           indent="\t", ensure_ascii=False)
        with open(self.index_path, "w", encoding="utf-8", newline="") as f:
            f.write(index.replace("\n", self.nl) + self.nl)

    def set_index_version(self, version):
        """Rename the version the bump is measured against."""
        self.version = version
        self.write_index()

    def members(self, name, key):
        """The preset's members; key=None leaves the key out, as a sync does."""
        _ind, settings, at = SHAPES[name]
        head = [("type", "machine"), ("name", name),
                ("inherits", "fdm_bbl_3dp_002_common"), ("from", "system"),
                ("setting_id", "GM033"), ("instantiation", "true")]
        body = [(member, "1") for member in settings]
        body.append(("support_object_skip_flush", "0"))
        if key is not None:
            body.insert(at, (KEY, key))
        return head + body

    def render(self, name, key):
        ind = self.indent(name)
        members = self.members(name, key)
        lines = ["{"]
        for i, (member, value) in enumerate(members):
            lines.append(f"{ind}{json.dumps(member)}: {json.dumps(value)}")
            lines[-1] += "," if i < len(members) - 1 else ""
        lines.append("}")
        return self.nl.join(lines) + self.nl

    def write_machine(self, name, key="1"):
        """Write the machine in its shipped shape; key=None drops the key."""
        with open(self.machine_path(name), "w", encoding="utf-8", newline="") as f:
            f.write(self.render(name, key))

    def write_raw_machine(self, name, raw):
        """Write the machine's bytes verbatim, for a file no parse can reach."""
        with open(self.machine_path(name), "wb") as f:
            f.write(raw)

    def text(self, name):
        with open(self.machine_path(name), "rb") as f:
            return f.read().decode("utf-8")

    def raw(self, name):
        with open(self.machine_path(name), "rb") as f:
            return f.read()

    def lines(self, name):
        return self.text(name).split(self.nl)

    def line_for(self, name, key):
        """The file's own line for `key`, indentation and comma included."""
        prefix = f'{self.indent(name)}"{key}":'
        return next(line for line in self.lines(name) if line.startswith(prefix))

    def version_now(self):
        with open(self.index_path, encoding="utf-8") as f:
            return json.load(f)["version"]

    def bytes_map(self):
        """Every file in the tree -> its bytes, for "nothing was written" asserts."""
        out = {}
        for root, dirs, files in os.walk(self.profiles):
            dirs.sort()
            for name in sorted(files):
                path = os.path.join(root, name)
                with open(path, "rb") as f:
                    out[os.path.relpath(path, self.profiles)] = f.read()
        return out


class GuardCase(unittest.TestCase):
    def setUp(self):
        self.t = GuardTree()
        self.addCleanup(self.t.cleanup)

    def run_command(self, *argv):
        """main() against this tree, capturing stdout."""
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            rc = apt.main([*argv, "--profiles", self.t.profiles])
        return rc, buf.getvalue()


# ---------------------------------------------------------------------------
# check-air-filtration
# ---------------------------------------------------------------------------

class TestCheckAirFiltration(GuardCase):
    def test_a_tree_with_every_key_passes(self):
        rc, out = self.run_command("check-air-filtration")
        self.assertEqual(rc, 0, out)
        self.assertIn(f'all {len(apt.AIR_FILTRATION_MACHINES)} {VENDOR} machines '
                      f'declare {KEY}: "1"', out)

    def test_a_stripped_key_fails_and_names_the_file(self):
        self.t.write_machine(H2D, key=None)
        rc, out = self.run_command("check-air-filtration")
        self.assertEqual(rc, 1, out)
        self.assertIn(f"{VENDOR}/machine/{H2D}: missing {KEY}", out)
        self.assertIn("fix-air-filtration", out)  # the remedy is spelled out

    def test_only_the_stripped_machines_are_reported(self):
        self.t.write_machine(H2D, key=None)
        self.t.write_machine(P1S, key=None)
        rc, out = self.run_command("check-air-filtration")
        self.assertEqual(rc, 1, out)
        self.assertEqual(out.count(f"missing {KEY}"), 2, out)

    def test_a_key_that_is_not_1_fails(self):
        self.t.write_machine(H2D, key="0")
        rc, out = self.run_command("check-air-filtration")
        self.assertEqual(rc, 1, out)
        self.assertIn(f'{VENDOR}/machine/{H2D}: {KEY} is "0", not "1"', out)

    def test_a_missing_machine_fails(self):
        os.remove(self.t.machine_path(H2D))
        rc, out = self.run_command("check-air-filtration")
        self.assertEqual(rc, 1, out)
        self.assertIn(f"{VENDOR}/machine/{H2D}: no such file", out)

    def test_a_top_level_that_is_not_an_object_fails_cleanly(self):
        # Every other check in the tool guards this; a traceback here would be the
        # gate failing for a reason its output does not name.
        self.t.write_raw_machine(H2D, b'["not", "an", "object"]')
        rc, out = self.run_command("check-air-filtration")
        self.assertEqual(rc, 1, out)
        self.assertIn(f"{VENDOR}/machine/{H2D}: not a JSON object", out)

    def test_check_writes_nothing(self):
        self.t.write_machine(H2D, key=None)
        before = self.t.bytes_map()
        self.assertEqual(self.run_command("check-air-filtration")[0], 1)
        self.assertEqual(self.t.bytes_map(), before)

    def test_a_scope_without_bbl_has_nothing_to_check(self):
        self.t.write_machine(H2D, key=None)
        rc, out = self.run_command("check-air-filtration", "--vendor", GuardTree.OTHER)
        self.assertEqual(rc, 0, out)
        self.assertIn("out of scope", out)


# ---------------------------------------------------------------------------
# fix-air-filtration
# ---------------------------------------------------------------------------

class TestFixAirFiltration(GuardCase):
    def test_the_key_goes_between_the_filter_anchors(self):
        self.t.write_machine(H2D, key=None)
        rc, out = self.run_command("fix-air-filtration")
        self.assertEqual(rc, 0, out)
        lines = self.t.lines(H2D)
        at = lines.index(f'\t"{KEY}": "1",')
        self.assertEqual(lines[at - 1], '\t"support_cooling_filter": "1",')
        self.assertEqual(lines[at + 1], '\t"support_chamber_temp_control": "1",')

    def test_a_file_without_an_anchor_gets_the_key_after_instantiation(self):
        # inherits would sit inside the identifying block, which check_normalized
        # rejects: instantiation closes that block, so the line lands among the
        # settings where normalize would leave it.
        self.t.write_machine(P1S, key=None)
        rc, out = self.run_command("fix-air-filtration")
        self.assertEqual(rc, 0, out)
        lines = self.t.lines(P1S)
        at = lines.index(f'    "{KEY}": "1",')
        self.assertEqual(lines[at - 1], '    "instantiation": "true",')
        self.assertEqual(lines[at + 1], self.t.line_for(P1S, "retraction_distances_when_cut"))

    def test_the_line_lands_on_the_anchor_every_machine_has(self):
        # Per machine, because with only H2D and P1S asserted the chamber anchor
        # could be dropped from AIR_FILTRATION_BEFORE and nothing would notice:
        # for H2D "just after support_cooling_filter" and "just before
        # support_chamber_temp_control" are the same slot.
        expected = {
            H2D: ("support_cooling_filter", "support_chamber_temp_control"),
            "Bambu Lab H2D Pro 0.4 nozzle.json":
                ("support_cooling_filter", "support_chamber_temp_control"),
            "Bambu Lab H2S 0.4 nozzle.json":
                ("instantiation", "support_chamber_temp_control"),
            "Bambu Lab H2C 0.4 nozzle.json":
                ("instantiation", "support_chamber_temp_control"),
            "Bambu Lab P1S 0.4 nozzle.json":
                ("instantiation", "retraction_distances_when_cut"),
            "Bambu Lab P2S 0.4 nozzle.json": ("instantiation", "printable_height"),
            "Bambu Lab X1 0.4 nozzle.json": ("instantiation", "scan_first_layer"),
            "Bambu Lab X1 Carbon 0.4 nozzle.json": ("instantiation", "scan_first_layer"),
            "Bambu Lab X2D 0.4 nozzle.json":
                ("instantiation", "support_chamber_temp_control"),
        }
        self.assertEqual(set(expected), set(apt.AIR_FILTRATION_MACHINES))
        for name, (before, after) in expected.items():
            with self.subTest(machine=name):
                t = GuardTree()
                self.addCleanup(t.cleanup)
                t.write_machine(name, key=None)
                buf = io.StringIO()
                with contextlib.redirect_stdout(buf):
                    rc = apt.main(["fix-air-filtration", "--profiles", t.profiles])
                self.assertEqual(rc, 0, buf.getvalue())
                lines = t.lines(name)
                at = lines.index(f'{t.indent(name)}"{KEY}": "1",')
                self.assertEqual(lines[at - 1], t.line_for(name, before))
                self.assertEqual(lines[at + 1], t.line_for(name, after))

    def test_exactly_one_line_goes_in_and_the_rest_keeps_its_bytes(self):
        for name in apt.AIR_FILTRATION_MACHINES:
            self.t.write_machine(name, key=None)
        before = {name: self.t.text(name) for name in apt.AIR_FILTRATION_MACHINES}
        rc, out = self.run_command("fix-air-filtration")
        self.assertEqual(rc, 0, out)
        for name in apt.AIR_FILTRATION_MACHINES:
            with self.subTest(machine=name):
                new, old = self.t.text(name), before[name]
                self.assertEqual(new.count(self.t.nl) - old.count(self.t.nl), 1)
                self.assertNotIn("\n", new.replace(self.t.nl, ""))
                self.assertNotIn("\r", new.replace(self.t.nl, ""))
                self.assertTrue(new.endswith(self.t.nl))
                added = [l for l in new.split(self.t.nl) if l not in old.split(self.t.nl)]
                self.assertEqual(added, [f'{self.t.indent(name)}"{KEY}": "1",'])

    def test_check_passes_after_a_repair(self):
        self.t.write_machine(H2D, key=None)
        self.t.write_machine(P1S, key=None)
        self.assertEqual(self.run_command("fix-air-filtration")[0], 0)
        rc, out = self.run_command("check-air-filtration")
        self.assertEqual(rc, 0, out)

    def test_a_second_run_changes_nothing(self):
        self.t.write_machine(H2D, key=None)
        self.assertEqual(self.run_command("fix-air-filtration")[0], 0)
        settled = self.t.bytes_map()
        rc, out = self.run_command("fix-air-filtration")
        self.assertEqual(rc, 0, out)
        self.assertEqual(self.t.bytes_map(), settled)
        self.assertIn("nothing to write", out)

    def test_a_correct_tree_is_not_written_at_all(self):
        before = self.t.bytes_map()
        rc, out = self.run_command("fix-air-filtration")
        self.assertEqual(rc, 0, out)
        self.assertEqual(self.t.bytes_map(), before)
        self.assertEqual(self.t.version_now(), "02.08.00.13")

    def test_a_version_that_cannot_be_bumped_writes_nothing(self):
        # Content changed under an unchanged version is the one state the .opc cache
        # cannot survive, so an index whose version cannot grow has to stop the run
        # before the first write rather than after it.
        for version in ("", "02.08.00.beta"):
            with self.subTest(version=version):
                t = GuardTree()
                self.addCleanup(t.cleanup)
                t.set_index_version(version)
                t.write_machine(H2D, key=None)
                before = t.bytes_map()
                buf = io.StringIO()
                with contextlib.redirect_stdout(buf):
                    rc = apt.main(["fix-air-filtration", "--profiles", t.profiles])
                self.assertEqual(rc, 1, buf.getvalue())
                self.assertIn("nothing written", buf.getvalue())
                self.assertEqual(t.bytes_map(), before)

    def test_a_partial_repair_still_bumps_the_version(self):
        # One file that could not be repaired must not stop the version from
        # covering the ones that were: those bytes did change on disk.
        self.t.write_machine(H2D, key=None)
        os.remove(self.t.machine_path(P1S))
        rc, out = self.run_command("fix-air-filtration")
        self.assertEqual(rc, 1, out)
        self.assertIn(f"{VENDOR}/machine/{P1S}: no such file", out)
        self.assertIn(f'add "{KEY}": "1" to {VENDOR}/machine/{H2D}', out)
        self.assertEqual(self.t.version_now(), "02.08.00.14")

    def test_fix_reports_a_non_object_file_instead_of_crashing(self):
        self.t.write_raw_machine(H2D, b'["not", "an", "object"]')
        before = self.t.bytes_map()
        rc, out = self.run_command("fix-air-filtration")
        self.assertEqual(rc, 1, out)
        self.assertIn(f"{VENDOR}/machine/{H2D}: not a JSON object", out)
        self.assertEqual(self.t.bytes_map(), before)

    def test_a_null_value_is_reported_instead_of_inserted_over(self):
        # data.get() cannot tell "absent" from "null", and inserting over a null
        # leaves two copies of the key with the gate still green, because json
        # keeps the last one.
        self.t.write_raw_machine(
            H2D, self.t.text(H2D).replace(f'"{KEY}": "1"', f'"{KEY}": null').encode("utf-8"))
        before = self.t.bytes_map()
        for command in ("check-air-filtration", "fix-air-filtration"):
            with self.subTest(command=command):
                rc, out = self.run_command(command)
                self.assertEqual(rc, 1, out)
                self.assertIn(f'{VENDOR}/machine/{H2D}: {KEY} is null, not "1"', out)
        self.assertEqual(self.t.bytes_map(), before)

    def test_a_duplicate_key_is_reported_not_counted_as_missing(self):
        # A file carrying the key twice is broken on its own terms; writing a third
        # copy would be worse than saying so.
        self.t.write_raw_machine(H2D, self.t.text(H2D).replace(
            f'\t"{KEY}": "1",',
            f'\t"{KEY}": "0",{self.t.nl}\t"{KEY}": "1",').encode("utf-8"))
        before = self.t.bytes_map()
        rc, out = self.run_command("check-air-filtration")
        self.assertEqual(rc, 1, out)
        self.assertIn("Duplicate key detected", out)
        rc, out = self.run_command("fix-air-filtration")
        self.assertEqual(rc, 1, out)
        self.assertEqual(self.t.bytes_map(), before)

    def test_a_version_that_cannot_be_written_leaves_the_machines_alone(self):
        # The version is raised before the machines precisely so this cannot happen:
        # content changed under the old version keeps being served from the .opc
        # cache while the gate stays green.
        self.t.write_machine(H2D, key=None)
        before = self.t.bytes_map()
        real = apt._write_air_filtration_version
        self.addCleanup(setattr, apt, "_write_air_filtration_version", real)
        apt._write_air_filtration_version = lambda *args, **kwargs: 1
        rc, out = self.run_command("fix-air-filtration")
        self.assertEqual(rc, 1, out)
        self.assertEqual(self.t.bytes_map(), before)

    def test_the_version_is_raised_before_the_machines_are_written(self):
        self.t.write_machine(H2D, key=None)
        seen = {}
        real = apt._write_air_filtration_version
        self.addCleanup(setattr, apt, "_write_air_filtration_version", real)

        def spy(profiles_dir, bump, dry_run):
            seen["version"], seen["h2d"] = self.t.version_now(), self.t.text(H2D)
            return real(profiles_dir, bump, dry_run)

        apt._write_air_filtration_version = spy
        rc, out = self.run_command("fix-air-filtration")
        self.assertEqual(rc, 0, out)
        self.assertEqual(seen["version"], "02.08.00.13")  # raised by this very call
        self.assertNotIn(KEY, seen["h2d"])                # machines still untouched
        self.assertIn(KEY, self.t.text(H2D))
        self.assertEqual(self.t.version_now(), "02.08.00.14")

    def test_a_machine_that_cannot_be_written_names_the_file(self):
        # Same shape as every other message in the tool: the file it is about,
        # then the reason.
        self.t.write_machine(H2D, key=None)
        real = apt._edit_profile
        self.addCleanup(setattr, apt, "_edit_profile", real)

        def refuse_machine(path, edit, dry_run=False, what="edit"):
            if path.endswith(H2D):
                raise PermissionError("simulated write failure")
            return real(path, edit, dry_run, what)

        apt._edit_profile = refuse_machine
        rc, out = self.run_command("fix-air-filtration")
        self.assertEqual(rc, 1, out)
        self.assertIn(f"{VENDOR}/machine/{H2D}: simulated write failure", out)

    def test_an_index_that_is_not_a_json_object_is_reported(self):
        self.t.write_machine(H2D, key=None)
        with open(self.t.index_path, "wb") as f:
            f.write(b'["not", "an", "object"]')
        before = self.t.bytes_map()
        rc, out = self.run_command("fix-air-filtration")
        self.assertEqual(rc, 1, out)
        self.assertIn(f"{VENDOR}.json: not a JSON object", out)
        self.assertEqual(self.t.bytes_map(), before)

    def test_a_repair_leaves_the_tree_in_normalizes_shape(self):
        # The anchors are what make the inserted line land where normalize would put
        # it. A repair that produced a file check calls unnormalized would ship a
        # tree the next CI run rejects -- moving the key to just after `inherits`
        # would do exactly that, and nothing else here would notice.
        for name in apt.AIR_FILTRATION_MACHINES:
            self.t.write_machine(name, key=None)
        self.assertEqual(self.run_command("fix-air-filtration")[0], 0)
        rc, out = self.run_command("normalize", "--dry-run", "--vendor", VENDOR)
        self.assertEqual(rc, 0, out)
        self.assertIn("0 profile(s) would be normalized", out)

    def test_the_version_is_bumped_once_for_the_whole_run(self):
        for name in apt.AIR_FILTRATION_MACHINES:
            self.t.write_machine(name, key=None)
        rc, out = self.run_command("fix-air-filtration")
        self.assertEqual(rc, 0, out)
        self.assertEqual(self.t.version_now(), "02.08.00.14")
        self.assertEqual(out.count(f"bump {VENDOR}.json"), 1, out)
        self.assertIn(f"{VENDOR}.json version 02.08.00.13 -> 02.08.00.14", out)

    def test_the_bump_changes_the_version_line_and_nothing_else(self):
        self.t.write_machine(H2D, key=None)
        with open(self.t.index_path, "rb") as f:
            before = f.read()
        self.assertEqual(self.run_command("fix-air-filtration")[0], 0)
        with open(self.t.index_path, "rb") as f:
            after = f.read()
        self.assertEqual(after.replace(b"02.08.00.14", b"02.08.00.13"), before)

    def test_a_dry_run_reports_the_write_and_the_bump_but_writes_nothing(self):
        self.t.write_machine(H2D, key=None)
        before = self.t.bytes_map()
        rc, out = self.run_command("fix-air-filtration", "--dry-run")
        self.assertEqual(rc, 0, out)
        self.assertIn(f'would add "{KEY}": "1" to {VENDOR}/machine/{H2D}', out)
        self.assertIn(f"would bump {VENDOR}.json version 02.08.00.13 -> 02.08.00.14", out)
        self.assertEqual(self.t.bytes_map(), before)

    def test_a_present_but_wrong_value_is_an_error_not_a_duplicate(self):
        self.t.write_machine(H2D, key="0")
        before = self.t.bytes_map()
        rc, out = self.run_command("fix-air-filtration")
        self.assertEqual(rc, 1, out)
        self.assertIn(f'{VENDOR}/machine/{H2D}: {KEY} is "0", not "1"', out)
        self.assertEqual(self.t.bytes_map(), before)

    def test_a_repair_survives_a_lf_tree(self):
        t = GuardTree(nl="\n")
        self.addCleanup(t.cleanup)
        t.write_machine(H2D, key=None)
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            rc = apt.main(["fix-air-filtration", "--profiles", t.profiles])
        self.assertEqual(rc, 0, buf.getvalue())
        self.assertNotIn("\r", t.text(H2D))
        self.assertEqual(t.version_now(), "02.08.00.14")


# ---------------------------------------------------------------------------
# the version bump rule
# ---------------------------------------------------------------------------

class TestVersionBump(unittest.TestCase):
    def test_the_last_segment_goes_up_by_one(self):
        self.assertEqual(apt._bumped_version("02.08.00.13"), "02.08.00.14")

    def test_the_result_is_strictly_greater_by_segment(self):
        # The cache gate compares the version it finds against the installed one,
        # so a bump that is not strictly greater would leave the repair invisible.
        for version in ("01.00.00.00", "02.08.00.13", "02.08.00.99", "02.08.99.99"):
            with self.subTest(version=version):
                bumped = apt._bumped_version(version)
                self.assertGreater([int(p) for p in bumped.split(".")],
                                   [int(p) for p in version.split(".")])

    def test_a_segment_carries_instead_of_growing_a_digit(self):
        self.assertEqual(apt._bumped_version("02.08.00.99"), "02.08.01.00")
        self.assertEqual(apt._bumped_version("02.08.99.99"), "02.09.00.00")

    def test_a_version_with_no_trailing_number_is_refused(self):
        with self.assertRaises(RuntimeError):
            apt._bumped_version("02.08.00.beta")


# ---------------------------------------------------------------------------
# the option the guard defends
# ---------------------------------------------------------------------------

class TestTheKeyIsTheOneTheAppReads(unittest.TestCase):
    """The name matters: dropping it from a machine preset is what takes the
    printer's air-filtration control, and the chamber fan's M106 P3, away."""

    def source(self, *parts):
        with open(os.path.join(REPO_ROOT, *parts), encoding="utf-8") as f:
            return f.read()

    def test_the_key_is_a_print_config_option(self):
        self.assertIn('this->add("support_air_filtration"',
                      self.source("src", "libslic3r", "PrintConfig.cpp"))

    def test_the_printer_page_reads_it_from_the_preset(self):
        self.assertIn('printer_cfg.opt_bool("support_air_filtration")',
                      self.source("src", "slic3r", "GUI", "Tab.cpp"))

    def test_the_g_code_writer_reads_it(self):
        self.assertIn("m_config.support_air_filtration.value",
                      self.source("src", "libslic3r", "GCode.cpp"))


# ---------------------------------------------------------------------------
# the CLI dispatch
# ---------------------------------------------------------------------------

class TestDispatch(GuardCase):
    def test_the_new_commands_take_their_own_options_only(self):
        for argv in (["check-air-filtration", "--dry-run"],
                     ["fix-air-filtration", "--strict"]):
            with self.subTest(argv=argv):
                with self.assertRaises(SystemExit) as cm, \
                        contextlib.redirect_stdout(io.StringIO()), \
                        contextlib.redirect_stderr(io.StringIO()):
                    apt.main([*argv, "--profiles", self.t.profiles])
                self.assertEqual(cm.exception.code, 2)


# ---------------------------------------------------------------------------
# the real tree
# ---------------------------------------------------------------------------

@unittest.skipUnless(os.path.isdir(REAL_PROFILES), "resources/profiles not present")
class TestRealTree(unittest.TestCase):
    def run_command(self, *argv):
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            rc = apt.main(argv)
        return rc, buf.getvalue()

    def test_the_shipped_tree_passes_the_guard(self):
        rc, out = self.run_command("check-air-filtration")
        self.assertEqual(rc, 0, out)

    def test_the_shipped_tree_needs_no_repair(self):
        rc, out = self.run_command("fix-air-filtration", "--dry-run")
        self.assertEqual(rc, 0, out)
        self.assertIn("nothing to write", out)

    def test_every_guarded_name_is_a_machine_the_shipped_tree_has(self):
        machine_dir = os.path.join(REAL_PROFILES, VENDOR, "machine")
        for name in apt.AIR_FILTRATION_MACHINES:
            self.assertTrue(os.path.isfile(os.path.join(machine_dir, name)), name)


if __name__ == "__main__":
    unittest.main()
