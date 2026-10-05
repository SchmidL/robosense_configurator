"""Tests for rs_webcfg. Run from the tool folder:  python3 -m unittest discover -s tests -v"""

from __future__ import annotations

import contextlib
import io
import json
import os
import sys
import tempfile
import unittest
import urllib.parse

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)

import rs_webcfg as rs  # noqa: E402
from fake_lidar import FIXTURES, FakeServer  # noqa: E402


def fixture(name: str) -> str:
    with open(os.path.join(FIXTURES, name), encoding="utf-8") as f:
        return f.read()


AIRY = json.loads(fixture("setting_data.json"))
CAPS = rs.caps_of(AIRY)


def quiet(fn, *a, **kw):
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf), contextlib.redirect_stderr(buf):
        rc = fn(*a, **kw)
    return rc, buf.getvalue()


def fast(**kw) -> rs.UploadOptions:
    base = dict(reboot="soft", yes=True, settle_s=0.05, poll_s=0.05, between_s=0.0,
                restart_timeout_s=10, backup_dir=None)
    base.update(kw)
    return rs.UploadOptions(**base)


class CatalogAndForm(unittest.TestCase):
    def test_catalog_covers_every_key_of_the_airy(self):
        self.assertEqual(sorted(set(AIRY) - set(rs.BY_KEY)), [])
        self.assertEqual(sorted(set(rs.BY_KEY) - set(AIRY)), [])

    def test_captured_form_matches_catalog(self):
        self.assertEqual(rs.ui_differences(rs.parse_form(fixture("Parameter_Setting.html"))), [])

    def test_builtin_layout_equals_captured_form(self):
        live = [(c.kind, c.name, c.section, c.value) for c in rs.parse_form(fixture("Parameter_Setting.html"))]
        builtin = [(c.kind, c.name, c.section, c.value) for c in rs._builtin_layout()]
        self.assertEqual(live, builtin)

    def test_body_of_general_save_is_the_browser_body(self):
        # Hand-written from the page: document order, the open tab radio, the
        # clicked Save button only, hidden <tbody> fields included, the disabled
        # pulse fields left out (GroupSelect 0), Mode2 checked (TriggerMode 0).
        expected = (
            "tabs-two=on&SrcIp=192.168.13.200&SrcMask=255.255.255.0&SrcGateWay=192.168.13.1"
            "&DstIp=192.168.13.102&MPort=6699&MPort1=9966&DPort=7788&ReMode=0&FovS=0&FovE=360&SynSrc=1"
            "&PDomNum=0&RTPdelay=0&NoLeap=0&SyTOut=5&UToL=1&LToU=20&OpM=1&PhaLockSet=0&GPSBRate=3"
            "&MotorR=0&ImuCtrl=1&IPort=6688&ImuOutR=3&AccRg=1&GyroRg=1&Lpf=0&ResDefault=0&save_param=Save"
            "&RefEn=0&TFilLvl=4&BlkRnDis=0&Rn=0&Blk=0&FSAgl=0&PoorMask=0&DZ10cm=0&C81858993=0&GapFill=0"
            "&RanM=0&RnFgSwt=0&L48=0&mode2=Mode2")
        layout = rs.parse_form(fixture("Parameter_Setting.html"))
        self.assertEqual(rs.encode_form(rs.form_pairs(layout, AIRY, "general")).decode(), expected)

    def test_body_of_pulse_save_with_group_on(self):
        s = dict(AIRY, TriggerMode="1", GroupSelect="1", PulseStart="90.0")
        body = rs.encode_form(rs.form_pairs(rs._builtin_layout(), s, "pulse")).decode()
        pairs = urllib.parse.parse_qsl(body)
        self.assertIn(("mode1", "Mode1(+25%)"), pairs)
        self.assertNotIn("mode2", dict(pairs))
        self.assertIn(("first_group_switch", "ON"), pairs)
        self.assertIn(("first_start_angle", "90.0"), pairs)
        self.assertTrue(body.endswith("first_pulse_step=360.0&save_param_pulse=Save"))
        self.assertIn("mode1=Mode1%28%2B25%25%29", body)
        self.assertEqual([n for n, _ in pairs].count("tabs-two"), 1)
        self.assertNotIn("save_param", dict(pairs))


class Values(unittest.TestCase):
    def test_labels_and_codes(self):
        p = rs.BY_KEY
        self.assertEqual(rs.parse_value(p["SynSrc"], "PTP-E2E"), "1")
        self.assertEqual(rs.parse_value(p["SynSrc"], "ptp e2e"), "1")
        self.assertEqual(rs.parse_value(p["SynSrc"], 3), "3")
        self.assertEqual(rs.parse_value(p["GPSBRate"], "115200"), "10")
        self.assertEqual(rs.parse_value(p["L48"], "48"), "1")
        self.assertEqual(rs.parse_value(p["DZ10cm"], "Off"), "1")   # inverted coding
        self.assertEqual(rs.parse_value(p["MPort"], 6699), "6699")
        self.assertEqual(rs.parse_value(p["SrcIp"], " 192.168.13.200 "), "192.168.13.200")
        for key, bad in (("SynSrc", "PTP"), ("MPort", "66.5"), ("SrcIp", "192.168.1"), ("FovS", True)):
            with self.assertRaises(ValueError):
                rs.parse_value(p[key], bad)

    def test_find_by_web_label(self):
        self.assertEqual([p.key for p in rs.find_params("Time Synchronization Source")], ["SynSrc"])
        self.assertEqual(rs._resolve_assignments(["Return Mode=Dual"]), {"ReMode": "3"})
        self.assertEqual(rs._resolve_assignments(["first_pulse_width=100"]), {"PulseWidth": "100"})
        with self.assertRaises(rs.LidarError):
            rs._resolve_assignments(["port=7000"])   # several ports match

    def test_profile(self):
        self.assertEqual(rs.profile_patch(13), {
            "SrcIp": "192.168.13.200", "SrcMask": "255.255.255.0", "SrcGateWay": "192.168.13.1",
            "DstIp": "192.168.13.102", "MPort": "6699", "DPort": "7788", "IPort": "6688"})
        with self.assertRaises(ValueError):
            rs.profile_patch(256)


class Checks(unittest.TestCase):
    def errors(self, **change):
        s = dict(AIRY, **change)
        return {(i.level, i.key) for i in rs.check_settings(s, CAPS, AIRY)}

    def test_live_airy_is_clean(self):
        self.assertEqual(self.errors(), set())

    def test_rules(self):
        self.assertIn(("error", "DPort"), self.errors(DPort="6699"))
        self.assertIn(("error", "MPort"), self.errors(MPort="80"))
        self.assertIn(("error", "ResDefault"), self.errors(ResDefault="1"))
        self.assertIn(("error", "ImuOutR"), self.errors(ImuOutR="4"))           # Cyc 5
        self.assertIn(("error", "VarT"), self.errors(VarT="1"))                 # read only
        self.assertIn(("error", "SrcIp"), self.errors(SrcIp="192.168.13.255"))
        self.assertIn(("error", "SrcMask"), self.errors(SrcMask="255.0.255.0"))
        self.assertIn(("error", "PhaLockSet"), self.errors(PhaLockSet="360"))
        self.assertIn(("error", "PulseWidth"), self.errors(PulseWidth="15"))
        self.assertIn(("warn", "DstIp"), self.errors(DstIp="192.168.13.1"))      # = gateway
        self.assertIn(("warn", "Rn"), self.errors(Rn="2"))                       # hidden for VarT 0
        self.assertNotIn(("error", "ResDefault"),
                         {(i.level, i.key) for i in rs.check_settings(dict(AIRY, ResDefault="1"), CAPS, AIRY, True)})


class EndToEnd(unittest.TestCase):
    def test_download_and_diff(self):
        with FakeServer() as srv, tempfile.TemporaryDirectory() as tmp:
            out = os.path.join(tmp, "a.json")
            rc, _ = quiet(rs.main, ["download", "--host", srv.host, "-o", out])
            self.assertEqual(rc, 0)
            self.assertEqual(rs.load_file(out), AIRY)
            rc, text = quiet(rs.main, ["diff", "--host", srv.host, out])
            self.assertIn("No difference", text)

    def test_upload_two_sections_soft_restart_pass(self):
        with FakeServer() as srv:
            rc, text = quiet(rs.upload, rs.Lidar(srv.host), {"ReMode": "1", "RefEn": "2"}, fast())
            self.assertEqual(rc, rs.EXIT_OK, text)
            self.assertIn("PASS", text)
            saves = [dict(p) for path, p in srv.fake.posts if path == "/Parameter_Setting.html"]
            self.assertEqual([("save_param_performance" in s, "save_param" in s) for s in saves],
                             [(True, False), (False, True)])          # performance first, general last
            self.assertEqual((srv.fake.stored["ReMode"], srv.fake.stored["RefEn"]), ("1", "2"))

    def test_pulse_checkboxes_reach_the_lidar(self):
        with FakeServer() as srv:
            patch = {"TriggerMode": "1", "GroupSelect": "1", "PulseStart": "45.0", "PulseWidth": "1000"}
            rc, text = quiet(rs.upload, rs.Lidar(srv.host), patch, fast())
            self.assertEqual(rc, rs.EXIT_OK, text)
            self.assertEqual({k: srv.fake.stored[k] for k in patch}, patch)

    def test_volatile_save_fails_verify(self):
        with FakeServer("volatile") as srv:
            rc, text = quiet(rs.upload, rs.Lidar(srv.host), {"SynSrc": "PTP-GPTP"}, fast())
            self.assertEqual(rc, rs.EXIT_VERIFY)
            self.assertIn("1 of 1 changes are visible", text)
            self.assertIn("FAIL", text)

    def test_refused_save_is_detected_at_once(self):
        with FakeServer("refuse") as srv:
            rc, text = quiet(rs.upload, rs.Lidar(srv.host), {"DstIp": "192.168.13.1"}, fast())
            self.assertEqual(rc, rs.EXIT_VERIFY)
            self.assertIn("destination = gateway", text)
            self.assertIn("probably refused", text)

    def test_json_upload(self):
        with FakeServer() as srv:
            rc, text = quiet(rs.upload, rs.Lidar(srv.host), {"OpM": "Standby"}, fast(via="json"))
            self.assertEqual(rc, rs.EXIT_OK, text)
            self.assertEqual(srv.fake.posts[0][0], "/upload-json")
            self.assertEqual(len(srv.fake.posts[0][1]), len(AIRY))

    def test_errors_send_nothing(self):
        with FakeServer() as srv:
            for patch, opt in (({"RotaSpe": "3"}, fast()),            # not on the web form
                               ({"ResDefault": "1"}, fast()),         # danger
                               ({"MPort": "7788"}, fast())):          # port clash
                rc, _ = quiet(rs.upload, rs.Lidar(srv.host), patch, opt)
                self.assertEqual(rc, rs.EXIT_CHECK)
            rc, text = quiet(rs.upload, rs.Lidar(srv.host), {"ReMode": "2"}, fast(dry_run=True))
            self.assertEqual(rc, rs.EXIT_OK)
            self.assertIn("save_param=Save", text)
            self.assertEqual(srv.fake.posts, [])

    def test_address_change_follows_the_lidar(self):
        with FakeServer() as srv:
            patch = rs.profile_patch(14)
            rc, text = quiet(rs.upload, rs.Lidar(srv.host), patch, fast(new_host=srv.host))
            self.assertEqual(rc, rs.EXIT_OK, text)
            self.assertEqual(srv.fake.stored["SrcIp"], "192.168.14.200")
            self.assertEqual(srv.fake.stored["DstIp"], "192.168.14.102")

    def test_verify_requires_restart_with_since(self):
        with FakeServer() as srv, tempfile.TemporaryDirectory() as tmp:
            f = os.path.join(tmp, "want.json")
            rs.save_file(f, {"SynSrc": "PTP-E2E"})
            n = int(srv.fake.diag["startup_times"])
            rc, _ = quiet(rs.main, ["verify", "--host", srv.host, f])
            self.assertEqual(rc, rs.EXIT_OK)
            rc, _ = quiet(rs.main, ["verify", "--host", srv.host, f, "--since", str(n)])
            self.assertEqual(rc, rs.EXIT_VERIFY)

    def test_offline_set_on_a_file(self):
        with tempfile.TemporaryDirectory() as tmp:
            f = os.path.join(tmp, "a.json")
            rs.save_file(f, AIRY)
            rc, _ = quiet(rs.main, ["set", "--file", f, "Time Synchronization Source=PTP-GPTP", "TFilLvl=7"])
            self.assertEqual(rc, 0)
            data = rs.load_file(f)
            self.assertEqual((data["SynSrc"], data["TFilLvl"]), ("3", "7"))
            rc, _ = quiet(rs.main, ["network", "15", "--file", f])
            self.assertEqual((rc, rs.load_file(f)["SrcIp"]), (0, "192.168.15.200"))
            rc, _ = quiet(rs.main, ["check", f])
            self.assertEqual(rc, 0)
            rc, _ = quiet(rs.main, ["set", "--file", f, "MPort=7788"])
            self.assertEqual(rc, rs.EXIT_CHECK)

    def test_ui_check_live(self):
        with FakeServer() as srv:
            rc, text = quiet(rs.main, ["ui-check", "--host", srv.host])
            self.assertEqual(rc, 0, text)


if __name__ == "__main__":
    unittest.main()
