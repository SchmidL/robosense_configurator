#!/usr/bin/env python3
"""rs_webcfg: read, change, write, and verify the settings of a RoboSense lidar
through the small web server inside the lidar.

Tested against the RoboSense Airy (web page of software 26042112). The Fairy uses
the same page family; run `rs_webcfg.py ui-check` on a new model or firmware first.

Why this tool exists:

* The labels on the web page are not the JSON keys ("Time Synchronization Source"
  is `SynSrc`), some values are coded the other way round (`DZ10cm` 0 = On), and
  the page hides some fields for some lidar variants. The catalog below holds
  every field once: label <-> key <-> allowed values <-> meaning.
* The page has three "Save" buttons. Each one posts the whole form, plus its own
  name. This tool sends exactly that request, field for field, in page order.
* `/setting_data.json` shows the values that run now, not the values in flash.
  Only a read after a restart proves a save. `upload` does that read, and it uses
  the start-up counter in `/diag_data.json` to prove that the restart happened.

Only the Python standard library is used (Python >= 3.8), so the file runs as is
on a laptop, on a headless box, or through `ssh host python3 - <args> < rs_webcfg.py`.

Exit codes: 0 good, 1 usage or connection error, 2 the settings fail the check,
3 the read-back after the save does not agree.
"""

from __future__ import annotations

import argparse
import datetime as _dt
import ipaddress
import json
import os
import re
import socket
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
from dataclasses import dataclass
from html.parser import HTMLParser
from typing import Callable, Dict, List, Optional, Sequence, Tuple

__version__ = "1.0.0"

# ---------------------------------------------------------------------------
# Address profile
# ---------------------------------------------------------------------------
# The RoboSense factory pattern is lidar 192.168.1.200, gateway 192.168.1.1,
# destination (the computer) 192.168.1.102. We keep that pattern and change only
# the third octet X, so each sensor port gets its own subnet 192.168.X.0/24.
# This pattern saved and survived a restart on the Airy and the Fairy on
# 2026-10-02 (zuriverse4d DEC-024). The pattern "lidar .10, destination .1 =
# gateway" did NOT survive a restart on the Airy on 2026-10-01.
PROFILE_ADDRESS = {
    "SrcIp": "192.168.{x}.200",       # the lidar
    "SrcMask": "255.255.255.0",
    "SrcGateWay": "192.168.{x}.1",    # no function on a point-to-point cable; keep it in the subnet
    "DstIp": "192.168.{x}.102",       # the computer port that receives the data
    "MPort": "6699",                  # MSOP: point data (RoboSense default)
    "DPort": "7788",                  # DIFOP: device status, once a second (RoboSense default)
    "IPort": "6688",                  # IMU data (RoboSense default)
}
FACTORY_HOST = "192.168.1.200"
HOST_OCTET = 102                      # the computer side of the pattern

PATH_SETTINGS = "/setting_data.json"
PATH_DEVICE = "/device_data.json"
PATH_DIAG = "/diag_data.json"
PATH_FORM = "/Parameter_Setting.html"  # the form has no action: it posts to its own URL
PATH_UPLOAD_JSON = "/upload-json"      # "JSON Configuration Upload" on the System page
PATH_SOFT_RESET = "/soft_reset"        # the System page posts here after a firmware update

EXIT_OK, EXIT_ERROR, EXIT_CHECK, EXIT_VERIFY = 0, 1, 2, 3

SECTIONS = ("general", "performance", "pulse")
SECTION_TITLE = {
    "general": "General Setting",
    "performance": "Performance Setting",
    "pulse": "Angle Pulse Setting",
    "hidden": "Not on the web page",
    "capability": "Read only (what this lidar can do)",
}
SUBMIT_NAME = {
    "general": "save_param",
    "performance": "save_param_performance",
    "pulse": "save_param_pulse",
}
# Order of the saves when more than one section changes. "general" is last,
# because it holds the address.
SAVE_ORDER = ("performance", "pulse", "general")


# ---------------------------------------------------------------------------
# Catalog
# ---------------------------------------------------------------------------
Caps = Dict[str, str]  # capability keys read from the lidar: VarT, CWCSts, Cyc, DRE

# The web page hides these fields for a lidar variant (key VarT), see
# fetchAndUpdateData() in Parameter_Setting.html. A hidden field is still posted.
HIDDEN_BY_VART = {
    "0": {"PoorMask", "MPort1", "BlkRnDis", "Rn", "Blk"},
    "1": {"DZ10cm", "C81858993", "RanM", "RnFgSwt", "L48", "MPort1"},
    "2": {"PoorMask", "L48"},
}


@dataclass(frozen=True)
class Param:
    key: str                    # key in setting_data.json
    label: str                  # text on the web page
    section: str                # general | performance | pulse | hidden | capability
    kind: str                   # ip | mask | port | int | number | choice
    help: str = ""
    choices: Tuple[Tuple[str, str], ...] = ()   # (code, label on the web page)
    lo: Optional[float] = None
    hi: Optional[float] = None
    hi_open: bool = False       # True: hi itself is not allowed
    unit: str = ""
    form: Optional[str] = None  # name of the form field, if it is not the key
    danger: str = ""            # shown before a change; a change needs --allow-danger

    @property
    def form_name(self) -> str:
        return self.form or self.key

    @property
    def numeric(self) -> bool:
        return self.kind in ("port", "int", "number", "choice")

    def choice_label(self, code: str) -> Optional[str]:
        for c, lab in self.choices:
            if _num_eq(c, code):
                return lab
        return None

    def available_choices(self, caps: Caps) -> Tuple[Tuple[str, str], ...]:
        return tuple(c for c in self.choices if _option_shown(self.key, c[0], caps))

    def shown(self, caps: Caps) -> bool:
        """True if the web page shows this field for this lidar."""
        if self.section in ("hidden", "capability"):
            return False
        if self.key in HIDDEN_BY_VART.get(caps.get("VarT", ""), set()):
            return False
        if self.key == "MotorR" and caps.get("CWCSts") == "0":
            return False
        return True

    def range_text(self) -> str:
        if self.kind == "choice":
            return ", ".join(f"{c}={lab}" for c, lab in self.choices)
        if self.kind in ("ip", "mask"):
            return "IPv4 address" if self.kind == "ip" else "IPv4 netmask"
        if self.lo is not None and self.hi is not None:
            close = ")" if self.hi_open else "]"
            return f"[{_fmt_num(self.lo)}, {_fmt_num(self.hi)}{close}" + (f" {self.unit}" if self.unit else "")
        return self.unit


def _option_shown(key: str, code: str, caps: Caps) -> bool:
    # Parameter_Setting.html removes these options with JavaScript.
    if key == "ImuOutR" and code == "4" and caps.get("Cyc") == "5":
        return False
    if key == "ReMode" and code == "3" and caps.get("VarT") in ("0", "2") and caps.get("DRE") == "0":
        return False
    return True


ON_OFF = (("0", "OFF"), ("1", "ON"))
HML = (("0", "high"), ("1", "medium"), ("2", "low"))

CATALOG: Tuple[Param, ...] = (
    # ---- General Setting -------------------------------------------------
    Param("SrcIp", "Device IP Address", "general", "ip",
          "Address of the lidar. Used after the next restart."),
    Param("SrcMask", "Device IP Mask", "general", "mask", "Netmask of the lidar."),
    Param("SrcGateWay", "Device IP Gateway", "general", "ip",
          "Gateway of the lidar. No function on a direct cable, but keep it inside the subnet."),
    Param("DstIp", "Destination IP Address", "general", "ip",
          "Address that receives MSOP, DIFOP and IMU data: the computer port. "
          "255.255.255.255 sends broadcast."),
    Param("MPort", "MSOP Port Number", "general", "port",
          "UDP destination port of the point data (MSOP).", lo=1025, hi=65535),
    Param("MPort1", "MSOP1 Port Number", "general", "port",
          "Second MSOP port. The web page shows it only for lidar variant VarT=2.", lo=1025, hi=65535),
    Param("DPort", "DIFOP Port Number", "general", "port",
          "UDP destination port of the device status packets (DIFOP), one each second.", lo=1025, hi=65535),
    Param("ReMode", "Return Mode", "general", "choice",
          "Which echo the lidar reports. Dual sends two echoes and doubles the data rate.",
          choices=(("0", "Strongest"), ("1", "First"), ("2", "Last"), ("3", "Dual"))),
    Param("FovS", "FOV Setting (start)", "general", "number",
          "Start azimuth of the horizontal field of view that the lidar sends.", lo=0, hi=360, unit="deg"),
    Param("FovE", "FOV Setting (end)", "general", "number",
          "End azimuth of the horizontal field of view that the lidar sends.", lo=0, hi=360, unit="deg"),
    Param("SynSrc", "Time Synchronization Source", "general", "choice",
          "Clock source. GPS = PPS + GPRMC on the cable. PTP-E2E = IEEE 1588 end-to-end over UDP "
          "(diagnostic page: E2E-L4). PTP-E2E-L2 = the same over Ethernet frames. "
          "PTP-P2P = peer-to-peer delay. PTP-GPTP = IEEE 802.1AS. "
          "A plain ptp4l (UDPv4, E2E) needs PTP-E2E.",
          choices=(("0", "GPS"), ("1", "PTP-E2E"), ("2", "PTP-P2P"), ("3", "PTP-GPTP"), ("4", "PTP-E2E-L2"))),
    Param("PDomNum", "PTP Domain Number", "general", "int",
          "PTP domain. Must match domainNumber of the grandmaster.", lo=0, hi=127),
    Param("RTPdelay", "Respond To PeerDelayRequest", "general", "choice",
          "Answer Pdelay_Req messages of a peer (P2P and gPTP).", choices=ON_OFF),
    Param("NoLeap", "No Leap Second", "general", "choice",
          "Leap second handling of the time stamps. With OFF, the Airy sends UTC "
          "(zuriverse4d EXP-006). The effect of ON is not tested.", choices=ON_OFF),
    Param("SyTOut", "Sync Timeout", "general", "int",
          "Time without sync messages before the lidar reports a sync loss.", lo=1, hi=255, unit="s"),
    Param("UToL", "Unlock To Lock Threshold", "general", "int",
          "Time offset limit to change from unlocked to locked.", lo=1, hi=255, unit="ms"),
    Param("LToU", "Lock To Unlock Threshold", "general", "int",
          "Time offset limit to change from locked to unlocked.", lo=1, hi=255, unit="ms"),
    Param("OpM", "Operation Mode", "general", "choice",
          "Standby stops the measurement. High-Performance is normal operation.",
          choices=(("0", "Standby"), ("1", "High-Performance"))),
    Param("PhaLockSet", "Phase Lock Setting", "general", "number",
          "Azimuth of the rotor at the start of each second, when the time is synchronized.",
          lo=0, hi=360, hi_open=True, unit="deg"),
    Param("GPSBRate", "GPS Baud Rate", "general", "choice",
          "Baud rate of the GPRMC serial input (sync source GPS only).",
          choices=(("3", "9600"), ("4", "14400"), ("5", "19200"), ("6", "38400"), ("7", "43200"),
                   ("8", "57600"), ("9", "76800"), ("10", "115200"))),
    Param("MotorR", "Motor Reversal", "general", "choice",
          "Direction of rotation. The page shows it only if the lidar can reverse (CWCSts=1).",
          choices=(("0", "FORWARD"), ("1", "REVERSE"))),
    Param("ImuCtrl", "Imu Ctrl", "general", "choice", "Send the data of the internal IMU.", choices=ON_OFF),
    Param("IPort", "Imu Port Number", "general", "port",
          "UDP destination port of the IMU data.", lo=1025, hi=65535),
    Param("ImuOutR", "Imu Output Rate", "general", "choice",
          "IMU packet rate. 500Hz is removed from the page when Cyc=5.",
          choices=(("0", "25Hz"), ("1", "50Hz"), ("2", "100Hz"), ("3", "200Hz"), ("4", "500Hz"))),
    Param("AccRg", "Accel Range", "general", "choice", "Accelerometer full scale.",
          choices=(("0", "[-2g, 2g]"), ("1", "[-4g, 4g]"), ("2", "[-8g, 8g]"), ("3", "[-16g, 16g]"))),
    Param("GyroRg", "Gyro Range", "general", "choice", "Gyroscope full scale.",
          choices=(("0", "[-250, 250]dps"), ("1", "[-500, 500]dps"), ("2", "[-1000, 1000]dps"),
                   ("3", "[-2000, 2000]dps"))),
    Param("Lpf", "Imu LPF Config", "general", "int", "IMU low-pass filter setting.", lo=0, hi=255),
    Param("ResDefault", "Restore Default", "general", "choice",
          "ON restores the factory settings at the save.", choices=ON_OFF,
          danger="Restore Default ON erases every setting. The lidar returns to 192.168.1.200."),
    # ---- Performance Setting --------------------------------------------
    Param("RefEn", "Reflectivity Enhancement", "performance", "choice",
          "Reflectivity enhancement level.", choices=(("0", "OFF"), ("1", "ON1"), ("2", "ON2"), ("3", "ON3"))),
    Param("TFilLvl", "Trail Filter Level", "performance", "choice",
          "Strength of the filter against trail points at object edges. 1 = weak, 7 = strong.",
          choices=tuple((str(i), str(i)) for i in range(1, 8))),
    Param("BlkRnDis", "Rain/Blockage Detection Distance", "performance", "choice",
          "Range of the rain and blockage detection.", choices=(("0", "30cm"), ("1", "20cm"), ("2", "10cm"))),
    Param("Rn", "Rain Detection Sensitivity", "performance", "choice", "Rain detection sensitivity.", choices=HML),
    Param("Blk", "Blockage Detection Sensitivity", "performance", "choice",
          "Blockage detection sensitivity.", choices=HML),
    Param("FSAgl", "Frame Start Angle", "performance", "number",
          "Azimuth where each frame (one rotation) starts.", lo=0, hi=360, hi_open=True, unit="deg"),
    Param("PoorMask", "Poor Performance Channel Mask", "performance", "choice",
          "Mask the channels with poor performance. INVERTED: 0 = On.", choices=(("0", "On"), ("1", "Off"))),
    Param("DZ10cm", "Dead Zone 10cm Enable", "performance", "choice",
          "10 cm dead zone near the window. INVERTED: 0 = On.", choices=(("0", "On"), ("1", "Off"))),
    Param("C81858993", "Channel 81 85 89 93 Enable", "performance", "choice",
          "Send the channels 81, 85, 89 and 93.", choices=(("0", "Off"), ("1", "On"))),
    Param("GapFill", "Gap Filling Enable", "performance", "choice", "Gap filling.", choices=(("0", "Off"), ("1", "On"))),
    Param("RanM", "Ranging Mode", "performance", "choice", "Ranging mode.",
          choices=(("0", "Precision"), ("1", "RainFog"), ("2", "Balance"))),
    Param("RnFgSwt", "Rain Fog Switch", "performance", "choice", "Rain and fog filter.", choices=(("0", "Off"), ("1", "On"))),
    Param("L48", "Number Of Lines", "performance", "choice", "Number of laser channels sent.",
          choices=(("0", "96"), ("1", "48"))),
    # ---- Angle Pulse Setting --------------------------------------------
    Param("TriggerMode", "Trigger Mode", "pulse", "choice",
          "Mode of the angle pulse output. The page shows two check boxes: "
          "Mode1(+25%) = 1, Mode2 = 0.", choices=(("0", "Mode2"), ("1", "Mode1(+25%)")), form=""),
    Param("GroupSelect", "Pulse Trigger Switch (First Group)", "pulse", "choice",
          "Angle pulse output of the first group. Off disables the three pulse fields on the page; "
          "they are then not posted.", choices=(("0", "OFF"), ("1", "ON")), form=""),
    Param("PulseStart", "Pulse Start Angle", "pulse", "number",
          "Azimuth of the first pulse.", lo=0, hi=360, hi_open=True, unit="deg", form="first_start_angle"),
    Param("PulseWidth", "Pulse Width", "pulse", "int",
          "Pulse width. Must be a multiple of 10 ns.", lo=10, hi=1_000_000_000, unit="ns", form="first_pulse_width"),
    Param("PulseStep", "Pulse Step", "pulse", "number",
          "Azimuth step between two pulses.", lo=0, hi=360, unit="deg", form="first_pulse_step"),
    # ---- In the JSON, but not on the web page ---------------------------
    Param("TFilNrEn", "Trail Filter Near Enhancement", "hidden", "choice",
          "Commented out on the web page.", choices=ON_OFF),
    Param("TSpeM", "Trail Special Mode", "hidden", "choice", "Commented out on the web page.", choices=ON_OFF),
    Param("MSideEn", "Side Installation Mode", "hidden", "choice", "Commented out on the web page.", choices=ON_OFF),
    Param("RotaSpe", "Rotation Speed", "hidden", "choice", "Commented out on the web page.",
          choices=(("1", "300 rpm"), ("2", "600 rpm"), ("3", "1200 rpm"))),
    Param("P1K", "P1K (unknown)", "hidden", "int", "Meaning unknown. Not on the web page."),
    Param("PMaster", "PMaster (unknown)", "hidden", "int", "Meaning unknown. Not on the web page."),
    Param("MVEn", "MSOP Vlan enable", "hidden", "int", "Commented out on the web page."),
    Param("MVId", "MSOP Vlan ID", "hidden", "int", "Commented out on the web page."),
    Param("MVPri", "MSOP Vlan Priority", "hidden", "int", "Commented out on the web page."),
    Param("DVEn", "DIFOP Vlan enable", "hidden", "int", "Commented out on the web page."),
    Param("DVId", "DIFOP Vlan ID", "hidden", "int", "Commented out on the web page."),
    Param("DVPri", "DIFOP Vlan Priority", "hidden", "int", "Commented out on the web page."),
    Param("PVEn", "PTP Vlan enable", "hidden", "int", "Commented out on the web page."),
    Param("PVId", "PTP Vlan ID", "hidden", "int", "Commented out on the web page."),
    Param("PVPri", "PTP Vlan Priority", "hidden", "int", "Commented out on the web page."),
    # ---- Read only: they tell the web page what this lidar can do -------
    Param("VarT", "Variant type", "capability", "int", "Selects the fields that the web page shows."),
    Param("CWCSts", "Motor reversal support", "capability", "int", "0: the page hides Motor Reversal."),
    Param("Cyc", "Cyc", "capability", "int", "5: the page removes the IMU rate 500Hz."),
    Param("DRE", "Dual return support", "capability", "int", "0 (with VarT 0 or 2): the page removes Dual."),
)

BY_KEY: Dict[str, Param] = {p.key: p for p in CATALOG}
BY_FORM: Dict[str, Param] = {p.form_name: p for p in CATALOG if p.form != ""}
CAP_KEYS = tuple(p.key for p in CATALOG if p.section == "capability")
ADDRESS_KEYS = ("SrcIp", "SrcMask", "SrcGateWay")
PULSE_FIELDS = ("first_start_angle", "first_pulse_width", "first_pulse_step")


# ---------------------------------------------------------------------------
# The form of Parameter_Setting.html
# ---------------------------------------------------------------------------
@dataclass
class Control:
    kind: str                   # field | select | radio | checkbox | submit
    name: str
    section: str
    value: str = ""             # value attribute (checkbox, radio, submit)
    options: Tuple[Tuple[str, str], ...] = ()


def _builtin_layout() -> List[Control]:
    """The form as captured from the Airy, software 26042112. Used when the
    live page cannot be read or parsed."""
    gen = ("SrcIp SrcMask SrcGateWay DstIp MPort MPort1 DPort ReMode FovS FovE SynSrc PDomNum RTPdelay "
           "NoLeap SyTOut UToL LToU OpM PhaLockSet GPSBRate MotorR ImuCtrl IPort ImuOutR AccRg GyroRg Lpf "
           "ResDefault").split()
    perf = "RefEn TFilLvl BlkRnDis Rn Blk FSAgl PoorMask DZ10cm C81858993 GapFill RanM RnFgSwt L48".split()
    out = [Control("radio", "tabs-two", "general", "on")]
    out += [Control("select" if BY_KEY[k].kind == "choice" else "field", k, "general") for k in gen]
    out += [Control("submit", "save_param", "general", "Save"), Control("radio", "tabs-two", "performance", "on")]
    out += [Control("select" if BY_KEY[k].kind == "choice" else "field", k, "performance") for k in perf]
    out += [Control("submit", "save_param_performance", "performance", "Save"),
            Control("radio", "tabs-two", "pulse", "on"),
            Control("checkbox", "mode1", "pulse", "Mode1(+25%)"),
            Control("checkbox", "mode2", "pulse", "Mode2"),
            Control("checkbox", "first_group_switch", "pulse", "ON")]
    out += [Control("field", n, "pulse") for n in PULSE_FIELDS]
    out.append(Control("submit", "save_param_pulse", "pulse", "Save"))
    return out


class _FormParser(HTMLParser):
    """Collects the controls of the POST form in document order. Controls in
    HTML comments are skipped by HTMLParser itself."""

    def __init__(self) -> None:
        super().__init__(convert_charrefs=True)
        self.controls: List[Control] = []
        self._in_form = False
        self._radios = 0
        self._select: Optional[Control] = None
        self._opt_value: Optional[str] = None
        self._opt_text: List[str] = []
        self._opts: List[Tuple[str, str]] = []

    def _section(self) -> str:
        return SECTIONS[min(max(self._radios - 1, 0), len(SECTIONS) - 1)]

    def handle_starttag(self, tag: str, attrs_list) -> None:
        a = {k: (v if v is not None else "") for k, v in attrs_list}
        if tag == "form":
            self._in_form = a.get("method", "").lower() == "post"
            return
        if not self._in_form:
            return
        if tag == "input":
            typ = a.get("type", "text").lower()
            name = a.get("name", "")
            if not name:
                return
            if typ == "radio":
                self._radios += 1
                self.controls.append(Control("radio", name, self._section(), a.get("value", "on") or "on"))
            elif typ == "checkbox":
                self.controls.append(Control("checkbox", name, self._section(), a.get("value", "on") or "on"))
            elif typ == "submit":
                self.controls.append(Control("submit", name, self._section(), a.get("value", "")))
            elif typ in ("text", "hidden", "number"):
                self.controls.append(Control("field", name, self._section()))
        elif tag == "select":
            self._select = Control("select", a.get("name", ""), self._section())
            self._opts = []
        elif tag == "option" and self._select is not None:
            self._flush_option()
            self._opt_value = a.get("value", "")
            self._opt_text = []

    def handle_data(self, data: str) -> None:
        if self._opt_value is not None:
            self._opt_text.append(data)

    def _flush_option(self) -> None:
        if self._opt_value is not None:
            text = " ".join("".join(self._opt_text).split())
            self._opts.append((self._opt_value, text or self._opt_value))
        self._opt_value = None

    def handle_endtag(self, tag: str) -> None:
        if tag == "option":
            self._flush_option()
        elif tag == "select" and self._select is not None:
            self._flush_option()
            self._select.options = tuple(self._opts)
            if self._select.name:
                self.controls.append(self._select)
            self._select = None
        elif tag == "form":
            self._in_form = False


def parse_form(html: str) -> List[Control]:
    p = _FormParser()
    p.feed(html)
    p.close()
    if not any(c.kind == "submit" for c in p.controls):
        raise ValueError("no POST form with a submit button found")
    return p.controls


def form_pairs(layout: Sequence[Control], settings: Dict[str, str], section: str) -> List[Tuple[str, str]]:
    """The name=value pairs that a browser posts when the user clicks the Save
    button of `section`, with the form filled from `settings`."""
    trig_on = not _num_eq(settings.get("TriggerMode", "0"), "0")
    group_on = not _num_eq(settings.get("GroupSelect", "0"), "0")
    pairs: List[Tuple[str, str]] = []
    for c in layout:
        if c.kind == "radio":
            # The tab radios: only the tab that is open (the one of the clicked Save) is checked.
            if c.section == section:
                pairs.append((c.name, c.value or "on"))
        elif c.kind == "submit":
            if c.section == section:
                pairs.append((c.name, c.value or "Save"))
        elif c.kind == "checkbox":
            states = {"mode1": trig_on, "mode2": not trig_on, "first_group_switch": group_on}
            if c.name not in states:
                raise ValueError(f"unknown check box '{c.name}' on the web page; run 'ui-check'")
            if states[c.name]:
                pairs.append((c.name, c.value))
        else:
            if c.name in PULSE_FIELDS and not group_on:
                continue  # disabled on the page, so the browser does not post it
            p = BY_FORM.get(c.name)
            key = p.key if p else c.name
            if key not in settings:
                raise ValueError(f"form field '{c.name}' has no value in the settings; run 'ui-check'")
            pairs.append((c.name, str(settings[key])))
    return pairs


def encode_form(pairs: Sequence[Tuple[str, str]]) -> bytes:
    # application/x-www-form-urlencoded, as a browser encodes it (space -> '+').
    return urllib.parse.urlencode(list(pairs), quote_via=urllib.parse.quote_plus).encode("ascii")


# ---------------------------------------------------------------------------
# Values
# ---------------------------------------------------------------------------
def _fmt_num(x: float) -> str:
    return str(int(x)) if float(x).is_integer() else str(x)


def _num_eq(a: object, b: object) -> bool:
    try:
        return float(str(a)) == float(str(b))
    except ValueError:
        return str(a).strip() == str(b).strip()


def values_equal(key: str, a: object, b: object) -> bool:
    p = BY_KEY.get(key)
    if p is not None and p.numeric:
        return _num_eq(a, b)
    return str(a).strip() == str(b).strip()


def _norm(text: str) -> str:
    return re.sub(r"[^a-z0-9]", "", text.lower())


def display(key: str, value: object) -> str:
    p = BY_KEY.get(key)
    v = str(value)
    if p is not None and p.kind == "choice":
        lab = p.choice_label(v)
        return f"{v} ({lab})" if lab is not None and lab != v else v
    if p is not None and p.unit and v:
        return f"{v} {p.unit}"
    return v


def parse_value(p: Param, raw: object) -> str:
    """Convert a value from a file or from the keyboard to the string that the
    lidar stores. Accept the code ("1") or the web page label ("PTP-E2E")."""
    if isinstance(raw, bool) or raw is None or isinstance(raw, (list, dict)):
        raise ValueError(f"{p.key}: a value must be a string or a number, not {json.dumps(raw)}")
    text = str(raw).strip()
    if p.kind == "choice":
        for code, _ in p.choices:
            if _num_eq(text, code) and re.fullmatch(r"-?\d+(\.0*)?", text):
                return code
        hits = [code for code, lab in p.choices if _norm(lab) == _norm(text)]
        if len(hits) == 1:
            return hits[0]
        raise ValueError(f"{p.key}: '{text}' is not a value of '{p.label}'. Use one of: {p.range_text()}")
    if p.kind in ("ip", "mask"):
        try:
            return str(ipaddress.IPv4Address(text))
        except ValueError:
            raise ValueError(f"{p.key}: '{text}' is not an IPv4 address") from None
    if p.kind in ("port", "int"):
        try:
            f = float(text)
        except ValueError:
            raise ValueError(f"{p.key}: '{text}' is not a number") from None
        if not f.is_integer():
            raise ValueError(f"{p.key}: '{text}' must be a whole number")
        return str(int(f))
    if p.kind == "number":
        try:
            float(text)
        except ValueError:
            raise ValueError(f"{p.key}: '{text}' is not a number") from None
        return text
    return text


def normalize_settings(raw: Dict[str, object]) -> Dict[str, str]:
    """Values of a file: catalog keys are parsed (labels allowed), other keys are
    kept as strings."""
    out: Dict[str, str] = {}
    errors = []
    for k, v in raw.items():
        p = BY_KEY.get(k)
        try:
            out[k] = parse_value(p, v) if p is not None else str(v)
        except ValueError as e:
            errors.append(str(e))
    if errors:
        raise ValueError("\n".join(errors))
    return out


def find_params(query: str, in_help: bool = False) -> List[Param]:
    """Find parameters by key, form name, or web page label. An exact hit wins;
    otherwise every label or key that contains the query (and, with in_help,
    every help text that contains it)."""
    q = _norm(query)
    if not q:
        return list(CATALOG)
    exact = [p for p in CATALOG if q in (_norm(p.key), _norm(p.label), _norm(p.form_name))]
    if exact:
        return exact
    return [p for p in CATALOG
            if q in _norm(p.label) or q in _norm(p.key) or q in _norm(p.form_name)
            or (in_help and (q in _norm(p.help) or any(q in _norm(lab) for _, lab in p.choices)))]


# ---------------------------------------------------------------------------
# Check
# ---------------------------------------------------------------------------
@dataclass
class Issue:
    level: str   # error | warn
    key: str
    text: str


def caps_of(settings: Dict[str, str]) -> Caps:
    return {k: str(settings[k]) for k in CAP_KEYS if k in settings}


def _ip_ok_device(ip: str) -> bool:
    o = [int(x) for x in ip.split(".")]
    # The same rules as isInvalidIp() of the web page.
    return not (all(x == 0 for x in o) or all(x == 255 for x in o) or o[3] == 255 or o[0] in (0, 255))


def check_settings(s: Dict[str, str], caps: Caps, live: Optional[Dict[str, str]] = None,
                   allow_danger: bool = False) -> List[Issue]:
    """The checks of the web page (validateForm), the lidar capabilities, and the
    lessons from the zuriverse4d bring-up."""
    issues: List[Issue] = []

    def err(k: str, t: str) -> None:
        issues.append(Issue("error", k, t))

    def warn(k: str, t: str) -> None:
        issues.append(Issue("warn", k, t))

    for k, v in s.items():
        p = BY_KEY.get(k)
        if p is None:
            if live is not None and k not in live:
                warn(k, "unknown key: the lidar does not report it")
            continue
        try:
            v = parse_value(p, v)
        except ValueError as e:
            err(k, str(e))
            continue
        if p.kind == "choice":
            if caps and not _option_shown(k, v, caps):
                err(k, f"'{display(k, v)}' is not offered by the web page of this lidar")
        elif p.kind in ("port", "int", "number") and p.lo is not None and p.hi is not None:
            f = float(v)
            if f < p.lo or f > p.hi or (p.hi_open and f >= p.hi):
                err(k, f"{v} is out of range {p.range_text()}")
        if k == "PulseWidth" and int(v) % 10 != 0:
            err(k, "pulse width must be a multiple of 10 ns")
        if p.danger and v != "0" and not allow_danger:
            err(k, p.danger + " Add --allow-danger to do this.")
        if caps and live is not None and k in live and not values_equal(k, live[k], v):
            if p.section == "capability":
                err(k, "read-only value: it describes the lidar and cannot be changed")
            elif not p.shown(caps) and p.section != "hidden":
                warn(k, "the web page hides this field on this lidar; the lidar can ignore it")

    def ip(k: str) -> Optional[ipaddress.IPv4Address]:
        try:
            return ipaddress.IPv4Address(s[k]) if k in s else None
        except ValueError:
            return None

    src, mask, gw, dst = ip("SrcIp"), ip("SrcMask"), ip("SrcGateWay"), ip("DstIp")
    if src is not None and not _ip_ok_device(str(src)):
        err("SrcIp", f"{src} is not a valid device address (web page rule)")
    if gw is not None and not _ip_ok_device(str(gw)):
        err("SrcGateWay", f"{gw} is not a valid gateway (web page rule)")
    if dst is not None and (int(dst) == 0 or str(dst).startswith("0.")):
        err("DstIp", f"{dst} is not a valid destination (web page rule)")
    net = None
    if mask is not None:
        if str(mask) == "255.255.255.255":
            err("SrcMask", "255.255.255.255 is not allowed (web page rule)")
        else:
            try:
                net = ipaddress.IPv4Network(f"0.0.0.0/{mask}").prefixlen
            except ValueError:
                err("SrcMask", f"{mask} is not a contiguous netmask")
    if src is not None and dst is not None and src == dst:
        err("DstIp", "the destination is the lidar itself")
    if src is not None and net is not None:
        subnet = ipaddress.IPv4Network(f"{src}/{net}", strict=False)
        if gw is not None and gw not in subnet:
            warn("SrcGateWay", f"{gw} is outside {subnet}")
        if dst is not None and dst not in subnet and str(dst) != "255.255.255.255":
            warn("DstIp", f"{dst} is outside {subnet}; the data must then pass a router")
    if gw is not None and dst is not None and gw == dst:
        warn("DstIp", "destination = gateway. The Airy did not keep such a save on 2026-10-01. "
                      "Use the pattern of 'network X' (.200 / .1 / .102).")

    ports = {k: s[k] for k in ("MPort", "DPort", "IPort") if k in s}
    if "MPort1" in s and caps.get("VarT") == "2":
        ports["MPort1"] = s["MPort1"]
    seen: Dict[str, str] = {}
    for k, v in ports.items():
        if v in seen:
            err(k, f"UDP port {v} is also used by {seen[v]} (web page rule)")
        seen[v] = k
    return issues


def print_issues(issues: Sequence[Issue]) -> None:
    for i in issues:
        p = BY_KEY.get(i.key)
        name = f"{p.label} [{i.key}]" if p else i.key
        print(f"  {style('ERROR' if i.level == 'error' else 'warn ', 'red' if i.level == 'error' else 'yellow')}"
              f"  {name}: {i.text}")


# ---------------------------------------------------------------------------
# HTTP
# ---------------------------------------------------------------------------
class LidarError(Exception):
    pass


def split_host(text: str) -> Tuple[str, int]:
    text = text.strip()
    if text.startswith("http://"):
        text = text[7:]
    text = text.rstrip("/")
    if text.count(":") == 1:
        h, p = text.split(":")
        return h, int(p)
    return text, 80


class Lidar:
    """A thin client for the lwIP web server inside the lidar (HTTP/1.0)."""

    def __init__(self, host: str, timeout: float = 5.0, retries: int = 2) -> None:
        self.host, self.port = split_host(host)
        self.timeout = timeout
        self.retries = retries

    @property
    def base(self) -> str:
        return f"http://{self.host}" + ("" if self.port == 80 else f":{self.port}")

    def __str__(self) -> str:
        return self.base

    def _open(self, req: urllib.request.Request) -> bytes:
        with urllib.request.urlopen(req, timeout=self.timeout) as r:
            return r.read()

    def get(self, path: str) -> bytes:
        last: Optional[Exception] = None
        for attempt in range(self.retries + 1):
            try:
                req = urllib.request.Request(self.base + path, headers=self._headers())
                return self._open(req)
            except urllib.error.HTTPError as e:
                raise LidarError(f"{self.base}{path}: HTTP {e.code}") from None
            except (urllib.error.URLError, OSError) as e:
                last = e
                if attempt < self.retries:
                    time.sleep(0.5)
        raise LidarError(f"cannot reach {self.base}{path}: {_reason(last)}")

    def get_json(self, path: str) -> Dict[str, str]:
        body = self.get(path)
        try:
            data = json.loads(body.decode("utf-8", "replace"))
        except ValueError:
            raise LidarError(f"{self.base}{path} did not return JSON") from None
        if not isinstance(data, dict):
            raise LidarError(f"{self.base}{path} did not return a JSON object")
        return {k: str(v) for k, v in data.items()}

    def settings(self) -> Dict[str, str]:
        return self.get_json(PATH_SETTINGS)

    def device(self) -> Dict[str, str]:
        return self.get_json(PATH_DEVICE)

    def diag(self) -> Dict[str, str]:
        return self.get_json(PATH_DIAG)

    def startup_count(self) -> Optional[int]:
        try:
            return int(self.diag().get("startup_times", "").strip())
        except (LidarError, ValueError):
            return None

    def layout(self) -> Tuple[List[Control], str]:
        """The live form, or the built-in copy if the page cannot be parsed."""
        try:
            return parse_form(self.get(PATH_FORM).decode("utf-8", "replace")), "live web page"
        except (LidarError, ValueError) as e:
            return _builtin_layout(), f"built-in copy ({e})"

    def _headers(self, referer: Optional[str] = None) -> Dict[str, str]:
        h = {"User-Agent": f"rs_webcfg/{__version__}", "Connection": "close"}
        if referer:
            h["Origin"] = self.base
            h["Referer"] = self.base + referer
        return h

    def post(self, path: str, body: bytes, content_type: str, referer: str) -> Optional[int]:
        """The HTTP status, or None if the lidar closed the connection without an
        answer. The Airy (software 26042112) resets the connection after a save
        of the form, and it applies the save all the same (seen 2026-10-05).
        So a reset is no error here: the read-back after the save decides."""
        headers = self._headers(referer)
        headers["Content-Type"] = content_type
        req = urllib.request.Request(self.base + path, data=body, headers=headers, method="POST")
        try:
            with urllib.request.urlopen(req, timeout=max(self.timeout, 10)) as r:
                r.read()
                return r.status
        except urllib.error.HTTPError as e:
            raise LidarError(f"POST {self.base}{path}: HTTP {e.code}") from None
        except (urllib.error.URLError, OSError) as e:
            cause = e.reason if isinstance(e, urllib.error.URLError) else e
            if isinstance(cause, ConnectionResetError):   # includes http.client.RemoteDisconnected
                return None
            raise LidarError(f"POST {self.base}{path}: {_reason(e)}") from None

    def post_form(self, pairs: Sequence[Tuple[str, str]]) -> Optional[int]:
        return self.post(PATH_FORM, encode_form(pairs), "application/x-www-form-urlencoded", PATH_FORM)

    def post_json_upload(self, settings: Dict[str, str]) -> Optional[int]:
        # The page builds a hidden form from the JSON and posts URLSearchParams.
        return self.post(PATH_UPLOAD_JSON, encode_form(list(settings.items())),
                         "application/x-www-form-urlencoded;charset=UTF-8", "/Firmware_Update.html")

    def soft_reset(self) -> Optional[int]:
        return self.post(PATH_SOFT_RESET, b"", "application/x-www-form-urlencoded;charset=UTF-8",
                         "/Firmware_Update.html")


def _answer(status: Optional[int]) -> str:
    return f"HTTP {status}" if status is not None else "sent (the lidar closed the connection without an answer)"


def _reason(e: Optional[BaseException]) -> str:
    if isinstance(e, urllib.error.URLError):
        e = e.reason if isinstance(e.reason, BaseException) else e
    if isinstance(e, socket.timeout):
        return "no answer (timeout)"
    return str(e) if e else "unknown error"


def route_hint(target_ip: str, prefix: int = 24) -> Optional[str]:
    """None if this computer has an address in the subnet of target_ip.
    Otherwise a text that tells how to add one."""
    try:
        tgt = ipaddress.IPv4Address(target_ip)
    except ValueError:
        return None  # a host name or a tunnel
    if tgt.is_loopback:
        return None
    subnet = ipaddress.IPv4Network(f"{tgt}/{prefix}", strict=False)
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect((str(tgt), 80))   # UDP connect sends nothing; it only selects a route
            src = ipaddress.IPv4Address(s.getsockname()[0])
    except OSError:
        src = None
    if src is not None and src in subnet:
        return None
    host = ipaddress.IPv4Address(int(subnet.network_address) + HOST_OCTET) if prefix <= 24 else None
    want = f"{host}/{prefix}" if host else f"an address in {subnet}"
    return (f"This computer has no address in {subnet}"
            + (f" (it would use {src})" if src else "") + ".\n"
            f"    Linux:   sudo ip addr add {want} dev <port>\n"
            f"    Windows: give the Ethernet adapter the address {want.split('/')[0]}, mask "
            f"{ipaddress.IPv4Network(f'0.0.0.0/{prefix}').netmask}")


# ---------------------------------------------------------------------------
# Output helpers
# ---------------------------------------------------------------------------
_COLOR = sys.stdout.isatty() and os.environ.get("NO_COLOR") is None and os.environ.get("TERM") != "dumb"
if _COLOR and os.name == "nt":
    os.system("")  # switches on ANSI escape codes in the Windows console
_CODES = {"red": "31", "green": "32", "yellow": "33", "bold": "1", "dim": "2", "cyan": "36"}


def style(text: str, color: str) -> str:
    return f"\033[{_CODES[color]}m{text}\033[0m" if _COLOR else text


def table(rows: Sequence[Sequence[str]], header: Optional[Sequence[str]] = None, indent: int = 2) -> None:
    allrows = ([list(header)] if header else []) + [list(r) for r in rows]
    if not allrows:
        return
    widths = [max(len(_strip_ansi(r[i])) for r in allrows) for i in range(len(allrows[0]))]
    for n, r in enumerate(allrows):
        cells = [c + " " * (widths[i] - len(_strip_ansi(c))) for i, c in enumerate(r)]
        line = " " * indent + "  ".join(cells).rstrip()
        print(style(line, "bold") if header and n == 0 else line)


def _strip_ansi(s: str) -> str:
    return re.sub(r"\033\[[0-9;]*m", "", s)


def diff_rows(old: Dict[str, str], new: Dict[str, str]) -> List[List[str]]:
    rows = []
    for k, v in new.items():
        if k in old and values_equal(k, old[k], v):
            continue
        p = BY_KEY.get(k)
        rows.append([p.label if p else "(unknown)", k, display(k, old[k]) if k in old else "-", display(k, v)])
    return rows


def changed_keys(old: Dict[str, str], new: Dict[str, str]) -> List[str]:
    return [k for k, v in new.items() if not (k in old and values_equal(k, old[k], v))]


def confirm(question: str, assume_yes: bool) -> bool:
    if assume_yes:
        return True
    if not sys.stdin.isatty():
        print("No terminal for the question. Add --yes to write without a question.")
        return False
    try:
        return input(f"{question} [y/N] ").strip().lower() in ("y", "yes")
    except EOFError:
        return False


# ---------------------------------------------------------------------------
# Files
# ---------------------------------------------------------------------------
def load_file(path: str) -> Dict[str, str]:
    try:
        with open(path, "r", encoding="utf-8-sig") as f:
            data = json.load(f)
    except OSError as e:
        raise LidarError(f"cannot read {path}: {e.strerror}") from None
    except ValueError as e:
        raise LidarError(f"{path} is not valid JSON: {e}") from None
    if not isinstance(data, dict):
        raise LidarError(f"{path}: the file must hold one JSON object")
    data.pop("_comment", None)
    try:
        return normalize_settings(data)
    except ValueError as e:
        raise LidarError(f"{path}:\n{e}") from None


def save_file(path: str, settings: Dict[str, str]) -> None:
    # The same flat form as /setting_data.json, so the web page can upload it too.
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8", newline="\n") as f:
        json.dump(settings, f, indent=2)
        f.write("\n")
    os.replace(tmp, path)


def backup_live(live: Dict[str, str], device: Dict[str, str], folder: str) -> str:
    stamp = _dt.datetime.now().strftime("%Y%m%dT%H%M%S")
    sn = re.sub(r"[^A-Za-z0-9]", "", device.get("sn", "") or "unknown")
    os.makedirs(folder, exist_ok=True)
    path = os.path.join(folder, f"{device.get('lidar_model', 'lidar')}_{sn}_{stamp}.json")
    save_file(path, live)
    return path


# ---------------------------------------------------------------------------
# Upload: save, read back, restart, verify
# ---------------------------------------------------------------------------
@dataclass
class UploadOptions:
    via: str = "form"           # form | json
    reboot: str = "manual"      # manual | soft | none
    yes: bool = False
    dry_run: bool = False
    allow_danger: bool = False
    allow_hidden: bool = False
    backup_dir: Optional[str] = "rs_webcfg_backups"
    settle_s: float = 3.0
    restart_timeout_s: float = 240.0
    new_host: Optional[str] = None   # where to look after an address change (for a tunnel)
    poll_s: float = 2.0
    between_s: float = 1.0           # pause between two saves


def compare(want: Dict[str, str], got: Dict[str, str]) -> List[Tuple[str, str, str, bool]]:
    rows = []
    for k, v in want.items():
        if k == "ResDefault":
            continue  # a one-shot action, not a stored value
        g = got.get(k)
        rows.append((k, v, "-" if g is None else g, g is not None and values_equal(k, g, v)))
    return rows


def print_compare(rows: Sequence[Tuple[str, str, str, bool]], only: Optional[Sequence[str]] = None) -> int:
    bad = 0
    out = []
    for k, w, g, ok in rows:
        if only is not None and k not in only and ok:
            continue
        bad += 0 if ok else 1
        p = BY_KEY.get(k)
        out.append([style("ok  ", "green") if ok else style("FAIL", "red"), p.label if p else k, k,
                    display(k, w), display(k, g)])
    if out:
        table(out, ["", "Setting", "Key", "Wanted", "Lidar"])
    return bad


def upload(lidar: Lidar, wanted_patch: Dict[str, str], opt: UploadOptions) -> int:
    live = lidar.settings()
    device = _try(lidar.device) or {}
    caps = caps_of(live)
    wanted = dict(live)
    wanted.update(wanted_patch)

    print(f"Lidar: {device.get('lidar_model', '?')} S/N {device.get('sn', '?')} at {lidar}")
    changes = changed_keys(live, wanted)
    issues = check_settings(wanted, caps, live, opt.allow_danger)
    for k in changes:
        p = BY_KEY.get(k)
        if p is not None and p.section == "hidden" and not (opt.via == "json" and opt.allow_hidden):
            issues.append(Issue("error", k, "not on the web page. Only '--via json --allow-hidden' sends it; "
                                            "the effect is not known."))
        if p is None and opt.via == "form":
            issues.append(Issue("error", k, "unknown key; the web form cannot send it"))
    if issues:
        print("Check:")
        print_issues(issues)
    if any(i.level == "error" for i in issues):
        print(style("Nothing was sent. Correct the errors first.", "red"))
        return EXIT_CHECK
    if not changes:
        print(style("The lidar already has these values. Nothing to send.", "green"))
        return EXIT_OK

    print("Changes:")
    table(diff_rows(live, wanted), ["Setting", "Key", "Now", "New"])

    addr_change = any(k in ADDRESS_KEYS for k in changes)
    old_ip, new_ip = live.get("SrcIp", ""), wanted.get("SrcIp", "")
    after = Lidar(opt.new_host, lidar.timeout) if opt.new_host else (
        Lidar(f"{new_ip}:{lidar.port}" if lidar.port != 80 else new_ip, lidar.timeout)
        if new_ip != old_ip and _is_ip(lidar.host) else lidar)
    if addr_change:
        print(style(f"Address change: after the restart the lidar is at {new_ip}.", "yellow"))
        if new_ip != old_ip and not _is_ip(lidar.host) and not opt.new_host:
            print("  You use a host name or a tunnel. After the restart, point it to the new address,\n"
                  f"  then run: rs_webcfg.py verify <file> --host <new address>")
        hint = route_hint(new_ip, _prefix(wanted.get("SrcMask", "255.255.255.0"))) if after.host == new_ip else None
        if hint:
            print("  " + hint)

    sections = [s for s in SAVE_ORDER if any(BY_KEY[k].section == s for k in changes if k in BY_KEY)]
    if opt.via == "form":
        layout, source = lidar.layout()
        try:
            bodies = [(s, form_pairs(layout, wanted, s)) for s in sections]
        except ValueError as e:
            print(style(f"Cannot build the form: {e}", "red"))
            return EXIT_ERROR
        if source != "live web page":
            print(style(f"Warning: the form comes from the {source}.", "yellow"))
    else:
        bodies = []

    if opt.dry_run:
        print("Plan:")
        for s in sections if opt.via == "form" else []:
            print(f"  POST {PATH_FORM} with the '{SECTION_TITLE[s]}' Save button")
        if opt.via == "json":
            print(f"  POST {PATH_UPLOAD_JSON} with all {len(wanted)} keys")
        restart_text = {"manual": "you disconnect and connect the power", "soft": f"POST {PATH_SOFT_RESET}",
                        "none": "no restart: the result stays unproven"}[opt.reboot]
        print(f"  then read back, restart ({restart_text}), read back again and compare")
        for s, pairs in bodies:
            print(f"\n--- body for '{s}' ---\n{encode_form(pairs).decode()}")
        if opt.via == "json":
            print(f"\n--- body ---\n{encode_form(list(wanted.items())).decode()}")
        print(style("\nDry run: nothing was sent.", "yellow"))
        return EXIT_OK

    if not confirm("Write these values to the lidar?", opt.yes):
        print("Nothing was sent.")
        return EXIT_ERROR

    if opt.backup_dir:
        print(f"Backup of the old values: {backup_live(live, device, opt.backup_dir)}")
    before = lidar.startup_count()

    # 1. Save
    try:
        if opt.via == "form":
            for s, pairs in bodies:
                status = lidar.post_form(pairs)
                print(f"Saved '{SECTION_TITLE[s]}': {_answer(status)}")
                time.sleep(opt.between_s)
        else:
            print(f"Uploaded JSON: {_answer(lidar.post_json_upload(wanted))}")
    except LidarError as e:
        print(style(f"Save failed: {e}", "red"))
        return EXIT_ERROR
    time.sleep(opt.settle_s)

    # 2. Read back the running values (these do not prove the flash contents)
    running = _try(lidar.settings)
    if running is None and after is not lidar:
        running = _try(after.settings)
    if running is None:
        print("Read-back now: no answer. The lidar can use a new address at once.")
    else:
        rows = compare({k: wanted[k] for k in changes}, running)
        n_ok = sum(1 for r in rows if r[3])
        print(f"Read-back now (running values): {n_ok} of {len(rows)} changes are visible.")
        if n_ok == 0:
            print(style("  The lidar shows none of the changes. The save was probably refused.", "yellow"))

    # 3. Restart
    if opt.reboot == "none":
        print(style("No restart. The values are NOT proven to be stored. After the next power cycle, run:\n"
                    "  rs_webcfg.py verify <file>", "yellow"))
        return EXIT_OK if running is not None and all(r[3] for r in compare(wanted, running)) else EXIT_VERIFY
    if opt.reboot == "soft":
        try:
            print(f"Soft reset: {_answer(lidar.soft_reset())}")
        except LidarError as e:
            print(style(f"Soft reset failed: {e}. Disconnect and connect the power instead.", "yellow"))
    else:
        print(style("Now disconnect the power of the lidar. Wait 5 s. Connect the power again.", "bold"))
    restarted = wait_restart([after, lidar] if after is not lidar else [lidar], before, opt)
    if restarted is None:
        print(style("The lidar did not come back with a new start-up count in time.", "red"))
        print(f"  Look for it at {after}. Then run: rs_webcfg.py verify <file> --host <address>")
        return EXIT_VERIFY
    where, count = restarted
    print(f"Restart seen at {where}: start-up count {before} -> {count}.")
    stored = _try(where.settings)
    if stored is None:
        print(style("Cannot read the settings after the restart.", "red"))
        return EXIT_VERIFY

    # 4. Verify
    print("Values after the restart:")
    bad = print_compare(compare(wanted, stored), only=changes)
    if bad:
        print(style(f"FAIL: {bad} value(s) did not survive the restart.", "red"))
        return EXIT_VERIFY
    print(style(f"PASS: all {len(changes)} change(s) are stored, and all other values are unchanged.", "green"))
    return EXIT_OK


def wait_restart(candidates: Sequence[Lidar], before: Optional[int], opt: UploadOptions
                 ) -> Optional[Tuple[Lidar, Optional[int]]]:
    """Wait until a candidate answers with a start-up count above `before`.
    Without a count before the save, wait until the lidar was gone and is back."""
    start = time.monotonic()
    deadline = start + opt.restart_timeout_s
    quick = [Lidar(f"{c.host}:{c.port}", timeout=1.5, retries=0) for c in candidates]
    seen_down = False
    hinted = False
    print(f"Waiting for the restart (max {int(opt.restart_timeout_s)} s, Ctrl+C stops) ", end="", flush=True)
    try:
        while time.monotonic() < deadline:
            answers = [(c, q.startup_count()) for c, q in zip(candidates, quick)]
            for c, n in answers:
                if n is None:
                    continue
                if (before is not None and n > before) or (before is None and seen_down):
                    print()
                    return c, n
            if all(n is None for _, n in answers):
                seen_down = True
            if opt.reboot == "soft" and not hinted and not seen_down and time.monotonic() - start > 45:
                print("\n  No restart yet. If nothing happens, disconnect and connect the power. ", end="")
                hinted = True
            print(".", end="", flush=True)
            time.sleep(opt.poll_s)
    except KeyboardInterrupt:
        print("\nStopped.")
        return None
    print()
    return None


def _try(fn: Callable[[], Dict[str, str]]) -> Optional[Dict[str, str]]:
    try:
        return fn()
    except LidarError:
        return None


def _is_ip(text: str) -> bool:
    try:
        ipaddress.IPv4Address(text)
        return True
    except ValueError:
        return False


def _prefix(mask: str) -> int:
    try:
        return ipaddress.IPv4Network(f"0.0.0.0/{mask}").prefixlen
    except ValueError:
        return 24


def profile_patch(x: int) -> Dict[str, str]:
    """The network settings of the address profile for subnet 192.168.X.0/24.
    Only addresses and ports: the time sync settings are not touched."""
    if not 0 <= x <= 255:
        raise ValueError("the subnet number X must be in 0..255")
    return {k: v.format(x=x) for k, v in PROFILE_ADDRESS.items()}


# ---------------------------------------------------------------------------
# Commands
# ---------------------------------------------------------------------------
def _lidar(args) -> Lidar:
    return Lidar(args.host, timeout=args.timeout)


def _upload_options(args) -> UploadOptions:
    return UploadOptions(via=args.via, reboot=args.reboot, yes=args.yes, dry_run=args.dry_run,
                       allow_danger=args.allow_danger, allow_hidden=args.allow_hidden,
                       backup_dir=None if args.no_backup else args.backup_dir,
                       restart_timeout_s=args.restart_timeout, new_host=args.new_host)


def cmd_info(args) -> int:
    lidar = _lidar(args)
    dev = lidar.device()
    diag = _try(lidar.diag) or {}
    s = _try(lidar.settings) or {}
    print(f"{dev.get('lidar_model', '?')}  S/N {dev.get('sn', '?')}  MAC {dev.get('mac_addr', '?')}  at {lidar}")
    print(f"  Firmware  top {dev.get('top_v', '?')}  bottom {dev.get('bot_v', '?')}  "
          f"software {dev.get('bot_soft_v', '?')}  motor {dev.get('motor_v', '?')}  config {dev.get('cfg_v', '?')}")
    if s:
        print(f"  Address   {s.get('SrcIp')}/{_prefix(s.get('SrcMask', ''))}  gateway {s.get('SrcGateWay')}  "
              f"-> {s.get('DstIp')}  MSOP {s.get('MPort')}  DIFOP {s.get('DPort')}  IMU {s.get('IPort')}")
        print(f"  Sync      {display('SynSrc', s.get('SynSrc', '?'))}  domain {s.get('PDomNum')}  "
              f"now: {diag.get('sync_sts', '?')} ({diag.get('sync_mode', '?')})")
        print(f"  Variant   VarT {s.get('VarT')}  CWCSts {s.get('CWCSts')}  Cyc {s.get('Cyc')}  DRE {s.get('DRE')}")
    if diag:
        print(f"  Health    {diag.get('temp', '?')} C  {diag.get('rpm', '?')} rpm  {diag.get('volt', '?')} V  "
              f"{diag.get('power', '?')} W  laser {diag.get('laser_sts', '?')}  start-ups {diag.get('startup_times', '?')}  "
              f"run time {diag.get('total', '?')}")
    if args.json:
        print(json.dumps({"device": dev, "diag": diag, "settings": s}, indent=2))
    return EXIT_OK


def cmd_download(args) -> int:
    lidar = _lidar(args)
    s = lidar.settings()
    dev = _try(lidar.device) or {}
    out = args.output or f"{dev.get('lidar_model', 'lidar')}_{dev.get('sn', 'unknown')}.json"
    if out == "-":
        print(json.dumps(s, indent=2))
        return EXIT_OK
    if os.path.exists(out) and not args.force:
        print(f"{out} exists. Add --force to overwrite it.")
        return EXIT_ERROR
    save_file(out, s)
    print(f"Wrote {len(s)} settings of {dev.get('lidar_model', 'lidar')} S/N {dev.get('sn', '?')} to {out}")
    return EXIT_OK


def cmd_upload(args) -> int:
    patch = load_file(args.file)
    return upload(_lidar(args), patch, _upload_options(args))


def cmd_diff(args) -> int:
    lidar = _lidar(args)
    live = lidar.settings()
    want = load_file(args.file)
    rows = diff_rows(live, want)
    if not rows:
        print("No difference.")
        return EXIT_OK
    table(rows, ["Setting", "Key", "Lidar", "File"])
    return EXIT_OK


def cmd_check(args) -> int:
    want = load_file(args.file)
    live = None
    if args.against_lidar:
        live = _lidar(args).settings()
        merged = dict(live)
        merged.update(want)
        want = merged
    caps = caps_of(live or want)
    if not caps:
        print("No capability keys (VarT, Cyc, DRE, CWCSts) in the file: the variant checks are skipped.")
    issues = check_settings(want, caps, live, args.allow_danger)
    unknown = [k for k in want if k not in BY_KEY]
    for k in unknown:
        issues.append(Issue("warn", k, "not in the catalog of this tool"))
    if not issues:
        print(style(f"{args.file}: no problems found ({len(want)} keys).", "green"))
        return EXIT_OK
    print_issues(issues)
    return EXIT_CHECK if any(i.level == "error" for i in issues) else EXIT_OK


def cmd_verify(args) -> int:
    lidar = _lidar(args)
    want = load_file(args.file)
    live = lidar.settings()
    n = lidar.startup_count()
    print(f"Lidar at {lidar}: start-up count {n}")
    if args.since is not None and (n is None or n <= args.since):
        print(style(f"FAIL: no restart since start-up count {args.since}. The read-back proves nothing yet.", "red"))
        return EXIT_VERIFY
    bad = print_compare(compare(want, live), only=None if args.all else [])
    if bad:
        print(style(f"FAIL: {bad} value(s) differ.", "red"))
        return EXIT_VERIFY
    print(style(f"PASS: all {len(want)} value(s) agree.", "green"))
    if args.since is None:
        print("Note: without a restart this shows the running values, not the stored ones. "
              "Use --since <count before the restart>.")
    return EXIT_OK


def _resolve_assignments(items: Sequence[str]) -> Dict[str, str]:
    patch: Dict[str, str] = {}
    for item in items:
        if "=" not in item:
            raise LidarError(f"'{item}': use KEY=VALUE, for example SynSrc=PTP-E2E")
        k, v = item.split("=", 1)
        hits = find_params(k)
        exact = [p for p in hits if _norm(k) in (_norm(p.key), _norm(p.label), _norm(p.form_name))]
        hits = exact or hits
        if len(hits) != 1:
            names = ", ".join(f"{p.key} ({p.label})" for p in hits[:8]) or "nothing"
            raise LidarError(f"'{k}' matches {names}. Be more exact.")
        p = hits[0]
        try:
            patch[p.key] = parse_value(p, v)
        except ValueError as e:
            raise LidarError(str(e)) from None
    return patch


def _apply_patch(args, patch: Dict[str, str]) -> int:
    if args.file:
        base = load_file(args.file) if os.path.exists(args.file) else {}
        before = dict(base)
        base.update(patch)
        rows = diff_rows(before, base)
        caps = caps_of(base)
        issues = check_settings(base, caps, None, args.allow_danger)
        print_issues(issues)
        if any(i.level == "error" for i in issues):
            print(style(f"{args.file} not changed.", "red"))
            return EXIT_CHECK
        if rows:
            table(rows, ["Setting", "Key", "Before", "After"])
        else:
            print("No change.")
        save_file(args.file, base)
        print(f"Wrote {args.file}. Send it with: rs_webcfg.py upload {args.file} --host <lidar>")
        return EXIT_OK
    return upload(_lidar(args), patch, _upload_options(args))


def cmd_set(args) -> int:
    return _apply_patch(args, _resolve_assignments(args.assignments))


def cmd_network(args) -> int:
    try:
        patch = profile_patch(args.x)
    except ValueError as e:
        raise LidarError(str(e)) from None
    if args.no_ports:
        for k in ("MPort", "DPort", "IPort"):
            patch.pop(k)
    print(f"Profile for X = {args.x}: lidar {patch['SrcIp']}, destination {patch['DstIp']}, "
          f"gateway {patch['SrcGateWay']}" + ("" if args.no_ports else ", ports 6699/7788/6688"))
    return _apply_patch(args, patch)


def cmd_reboot(args) -> int:
    lidar = _lidar(args)
    before = lidar.startup_count()
    if not confirm(f"Restart the lidar at {lidar} with {PATH_SOFT_RESET}?", args.yes):
        return EXIT_ERROR
    print(f"Soft reset: {_answer(lidar.soft_reset())}")
    opt = UploadOptions(reboot="soft", restart_timeout_s=args.restart_timeout)
    res = wait_restart([lidar], before, opt)
    if res is None:
        print(style("No restart seen. This firmware may not restart on this request. "
                    "Disconnect and connect the power.", "yellow"))
        return EXIT_VERIFY
    print(style(f"Restarted: start-up count {before} -> {res[1]}.", "green"))
    return EXIT_OK


def cmd_params(args) -> int:
    hits = find_params(args.query or "", in_help=True)
    if not args.all and not args.query:
        hits = [p for p in hits if p.section in SECTIONS]
    if not hits:
        print(f"No setting matches '{args.query}'.")
        return EXIT_ERROR
    if args.query and len(hits) <= 6:
        for p in hits:
            print(style(f"{p.label}", "bold") + f"   key {p.key}   tab {SECTION_TITLE[p.section]}"
                  + (f"   form field {p.form_name}" if p.form_name != p.key and p.form != "" else ""))
            print(f"  Values: {p.range_text()}")
            print(f"  {p.help}")
            if p.danger:
                print(style(f"  DANGER: {p.danger}", "red"))
            print()
        return EXIT_OK
    rows = []
    for p in hits:
        rows.append([SECTION_TITLE[p.section].split()[0], p.label, p.key, _short(p.range_text(), 60)])
    table(rows, ["Tab", "Label on the web page", "JSON key", "Values"])
    print("\nDetails: rs_webcfg.py params <name>.  Hidden and read-only keys: --all.")
    return EXIT_OK


def _short(s: str, n: int) -> str:
    return s if len(s) <= n else s[: n - 3] + "..."


def cmd_ui_check(args) -> int:
    if args.html:
        with open(args.html, encoding="utf-8", errors="replace") as f:
            html = f.read()
        where = args.html
    else:
        lidar = _lidar(args)
        html = lidar.get(PATH_FORM).decode("utf-8", "replace")
        where = str(lidar)
    controls = parse_form(html)
    problems = ui_differences(controls)
    if not problems:
        print(style(f"The web form at {where} matches the catalog ({len(controls)} controls).", "green"))
        return EXIT_OK
    print(f"The web form at {where} differs from the catalog of this tool:")
    for t in problems:
        print("  - " + t)
    print("Update CATALOG and _builtin_layout() in rs_webcfg.py before you upload to this lidar.")
    return EXIT_CHECK


def ui_differences(controls: Sequence[Control]) -> List[str]:
    out = []
    known_forms = set(BY_FORM) | {"mode1", "mode2", "first_group_switch", "tabs-two"} | set(SUBMIT_NAME.values())
    for c in controls:
        if c.name not in known_forms:
            out.append(f"new control '{c.name}' ({c.kind}) in the {c.section} tab")
            continue
        p = BY_FORM.get(c.name)
        if p is None:
            continue
        if p.section != c.section:
            out.append(f"'{c.name}' is in the {c.section} tab, catalog says {p.section}")
        if c.kind == "select":
            if p.kind != "choice":
                out.append(f"'{c.name}' is a list on the page, catalog says {p.kind}")
            elif set(c.options) != set(p.choices):
                out.append(f"'{c.name}' options on the page {list(c.options)} != catalog {list(p.choices)}")
        elif c.kind == "field" and p.kind == "choice":
            out.append(f"'{c.name}' is a text field on the page, catalog says list")
    names = {c.name for c in controls}
    for p in CATALOG:
        if p.section in SECTIONS and p.form != "" and p.form_name not in names:
            out.append(f"catalog field '{p.form_name}' ({p.label}) is not on the page")
    return out


# ---------------------------------------------------------------------------
# Interactive configurator
# ---------------------------------------------------------------------------
class Configurator:
    """A menu in the terminal. It works over SSH and needs no extra package."""

    def __init__(self, base: Dict[str, str], source: str, lidar: Optional[Lidar], args) -> None:
        self.base = base
        self.pending: Dict[str, str] = {}
        self.source = source
        self.lidar = lidar
        self.args = args
        self.caps = caps_of(base)

    def value(self, k: str) -> str:
        return self.pending.get(k, self.base.get(k, ""))

    def run(self) -> int:
        print(style(f"rs_webcfg configurator - {self.source}", "bold"))
        print("Enter a number or a letter. '?' shows help. Ctrl+C or 'q' leaves.\n")
        while True:
            n = len(changed_keys(self.base, self.merged()))
            print(style("Main menu", "bold"))
            for i, s in enumerate(SECTIONS, 1):
                cnt = sum(1 for p in CATALOG if p.section == s)
                print(f"  {i}  {SECTION_TITLE[s]} ({cnt} settings)")
            print("  4  Not on the web page / read only")
            print("  n  Address profile: give X, get 192.168.X.200 -> 192.168.X.102")
            print("  /  Find a setting by its web page label, its key, or a word")
            print(f"  r  Review the changes ({n})")
            print("  u  Undo all changes")
            if self.lidar is not None:
                print("  s  Save to the lidar, restart, and verify")
            print("  w  Write the settings to a file")
            print("  q  Quit")
            c = self.ask("> ").lower()
            if c in ("1", "2", "3"):
                self.section_menu([p for p in CATALOG if p.section == SECTIONS[int(c) - 1]],
                                  SECTION_TITLE[SECTIONS[int(c) - 1]])
            elif c == "4":
                self.section_menu([p for p in CATALOG if p.section in ("hidden", "capability")],
                                  "Not on the web page / read only")
            elif c == "n":
                self.profile()
            elif c.startswith("/"):
                self.find(c[1:] or self.ask("Find: "))
            elif c == "r":
                self.review()
            elif c == "u":
                self.pending.clear()
                print("All changes are removed.\n")
            elif c == "s" and self.lidar is not None:
                rc = self.save()
                if rc is not None:
                    return rc
            elif c == "w":
                self.write()
            elif c in ("q", "quit", "exit"):
                if self.pending and changed_keys(self.base, self.merged()):
                    if not self.yes_no("Changes are not saved. Quit?"):
                        continue
                return EXIT_OK
            elif c in ("?", "h", "help"):
                print(__doc__)

    def merged(self) -> Dict[str, str]:
        m = dict(self.base)
        m.update(self.pending)
        return m

    @staticmethod
    def ask(prompt: str) -> str:
        try:
            return input(prompt).strip()
        except EOFError:
            raise KeyboardInterrupt from None

    def yes_no(self, q: str) -> bool:
        return self.ask(f"{q} [y/N] ").lower() in ("y", "yes")

    def section_menu(self, params: List[Param], title: str) -> None:
        while True:
            print(style(f"\n{title}", "bold"))
            rows = []
            for i, p in enumerate(params, 1):
                if p.key not in self.base and p.key not in self.pending:
                    continue
                cur = display(p.key, self.value(p.key))
                mark = style(" *", "yellow") if p.key in self.pending and not values_equal(
                    p.key, self.pending[p.key], self.base.get(p.key, "")) else ""
                note = "" if p.shown(self.caps) or p.section not in SECTIONS else style(" (hidden on this lidar)", "dim")
                rows.append([str(i), p.label + note, p.key, cur + mark])
            table(rows, ["#", "Label on the web page", "Key", "Value"])
            c = self.ask("Number to change, Enter = back: ")
            if not c:
                print()
                return
            if c.isdigit() and 1 <= int(c) <= len(params):
                self.edit(params[int(c) - 1])

    def edit(self, p: Param) -> None:
        print(style(f"\n{p.label}", "bold") + f"  [{p.key}]  in '{SECTION_TITLE[p.section]}'")
        print(f"  {p.help}")
        if p.danger:
            print(style(f"  DANGER: {p.danger}", "red"))
        if p.section == "capability":
            print("  Read only.")
            return
        if p.section in SECTIONS and not p.shown(self.caps):
            print(style("  The web page hides this field on this lidar. The lidar can ignore a change.", "yellow"))
        if p.section == "hidden":
            print(style("  Not on the web page. Only 'upload --via json --allow-hidden' sends it.", "yellow"))
        print(f"  Now: {display(p.key, self.value(p.key))}")
        opts = p.available_choices(self.caps) if p.kind == "choice" else ()
        if opts:
            for i, (code, lab) in enumerate(opts, 1):
                print(f"    {i}) {lab}" + (f"  (JSON value {code})" if lab != code else ""))
            prompt = f"  Choose 1-{len(opts)}, or type the label; Enter = keep: "
        else:
            prompt = f"  New value {p.range_text()}; Enter = keep: "
        while True:
            raw = self.ask(prompt)
            if not raw:
                return
            if opts and raw.isdigit() and 1 <= int(raw) <= len(opts):
                raw = opts[int(raw) - 1][0]   # a number is always the menu entry
            try:
                v = parse_value(p, raw)
            except ValueError as e:
                print(style(f"  {e}", "red"))
                continue
            trial = self.merged()
            trial[p.key] = v
            old = {(i.key, i.text) for i in check_settings(self.merged(), self.caps, self.base,
                                                           self.args.allow_danger)}
            errs = [i for i in check_settings(trial, self.caps, self.base, self.args.allow_danger)
                    if (i.key, i.text) not in old or i.key == p.key]
            print_issues(errs)
            if any(i.level == "error" for i in errs):
                continue
            if values_equal(p.key, self.base.get(p.key, ""), v):
                self.pending.pop(p.key, None)
            else:
                self.pending[p.key] = v
            print(f"  -> {display(p.key, v)}\n")
            return

    def find(self, query: str) -> None:
        hits = [p for p in find_params(query, in_help=True) if p.key in self.base or p.section in SECTIONS]
        if not hits:
            print(f"No setting matches '{query}'.\n")
            return
        self.section_menu(hits, f"Settings that match '{query}'")

    def profile(self) -> None:
        raw = self.ask("Subnet number X (lidar 192.168.X.200), Enter = cancel: ")
        if not raw:
            return
        try:
            patch = profile_patch(int(raw))
        except ValueError as e:
            print(style(f"  {e}", "red"))
            return
        for k, v in patch.items():
            if values_equal(k, self.base.get(k, ""), v):
                self.pending.pop(k, None)
            else:
                self.pending[k] = v
        self.review()

    def review(self) -> None:
        m = self.merged()
        rows = diff_rows(self.base, m)
        if not rows:
            print("No changes.\n")
            return
        print(style("\nChanges", "bold"))
        table(rows, ["Setting", "Key", "Now", "New"])
        issues = check_settings(m, self.caps, self.base, self.args.allow_danger)
        if issues:
            print("Check:")
            print_issues(issues)
        print()

    def write(self) -> None:
        default = getattr(self.args, "out", None) or "lidar_settings.json"
        path = self.ask(f"File name [{default}]: ") or default
        if os.path.exists(path) and not self.yes_no(f"{path} exists. Overwrite?"):
            return
        full = self.yes_no("Write all keys? (No = only the changes, as a patch file)")
        save_file(path, self.merged() if full else {k: self.pending[k] for k in changed_keys(self.base, self.merged())})
        print(f"Wrote {path}.\n")

    def save(self) -> Optional[int]:
        patch = {k: v for k, v in self.merged().items() if k in changed_keys(self.base, self.merged())}
        if not patch:
            print("No changes.\n")
            return None
        opt = _upload_options(self.args)
        modes = {"1": "manual", "2": "soft", "3": "none"}
        default = next(n for n, m in modes.items() if m == opt.reboot)
        print("Restart after the save: 1) power cycle by hand (sure)  2) soft reset (not tested)  3) none")
        opt.reboot = modes.get(self.ask(f"Choose [{default}]: ") or default, opt.reboot)
        assert self.lidar is not None
        rc = upload(self.lidar, patch, opt)
        if rc == EXIT_OK:
            return rc
        print("Back in the configurator. Your changes are kept.\n")
        return None


def cmd_configure(args) -> int:
    lidar = None
    if args.file:
        base = load_file(args.file)
        source = f"file {args.file} (offline)"
    else:
        lidar = _lidar(args)
        base = lidar.settings()
        dev = _try(lidar.device) or {}
        source = f"{dev.get('lidar_model', 'lidar')} S/N {dev.get('sn', '?')} at {lidar}"
    if not args.out and args.file:
        args.out = args.file
    try:
        return Configurator(base, source, lidar, args).run()
    except KeyboardInterrupt:
        print("\nLeft without a save.")
        return EXIT_ERROR


# ---------------------------------------------------------------------------
# Command line
# ---------------------------------------------------------------------------
EPILOG = f"""\
examples:
  %(prog)s info --subnet 13                      device, address, sync, health
  %(prog)s download --subnet 13 -o airy.json     save all settings to a file
  %(prog)s params sync                           find a setting by its web page label
  %(prog)s set --subnet 13 "Return Mode=Dual"    change one value, restart, verify
  %(prog)s upload --subnet 13 airy.json          write a file, restart, verify
  %(prog)s network 13 --host {FACTORY_HOST}    new lidar: give it 192.168.13.200
  %(prog)s configure --subnet 13                 interactive menu
  %(prog)s configure --file airy.json            interactive, offline on a file

Address profile (network X): lidar 192.168.X.200, mask 255.255.255.0, gateway
192.168.X.1, destination 192.168.X.102, MSOP 6699, DIFOP 7788, IMU 6688. It does
not touch the time sync settings. The default lidar address is the factory one,
{FACTORY_HOST}.
"""


def build_parser() -> argparse.ArgumentParser:
    common = argparse.ArgumentParser(add_help=False)
    g = common.add_argument_group("lidar address")
    g.add_argument("--host", default=None, help=f"lidar address, optional :port (default {FACTORY_HOST})")
    g.add_argument("--subnet", type=int, metavar="X", help="short form of --host 192.168.X.200")
    g.add_argument("--timeout", type=float, default=5.0, help="HTTP timeout in s (default 5)")

    write = argparse.ArgumentParser(add_help=False)
    w = write.add_argument_group("write to the lidar")
    w.add_argument("--via", choices=("form", "json"), default="form",
                   help="form: the Save buttons of the Setting page (default). json: the JSON upload of the System page")
    w.add_argument("--reboot", choices=("manual", "soft", "none"), default="manual",
                   help="restart before the verify: manual = you cycle the power (default), "
                        "soft = POST /soft_reset (not tested), none = no proof of storage")
    w.add_argument("-y", "--yes", action="store_true", help="no question before the write")
    w.add_argument("--dry-run", action="store_true", help="show the plan and the request bodies, send nothing")
    w.add_argument("--allow-danger", action="store_true", help="allow Restore Default = ON")
    w.add_argument("--allow-hidden", action="store_true", help="with --via json: allow keys that are not on the web page")
    w.add_argument("--backup-dir", default="rs_webcfg_backups", help="folder for the backup of the old values")
    w.add_argument("--no-backup", action="store_true", help="do not write a backup")
    w.add_argument("--restart-timeout", type=float, default=240.0, help="max wait for the restart in s (default 240)")
    w.add_argument("--new-host", help="where the lidar answers after an address change, if not at its new SrcIp "
                                      "(for example a new SSH tunnel: localhost:8081)")

    ap = argparse.ArgumentParser(
        prog="rs_webcfg.py", formatter_class=argparse.RawDescriptionHelpFormatter, epilog=EPILOG,
        description="Read, change, write and verify the settings of a RoboSense lidar (Airy, Fairy) "
                    "through its web server.")
    ap.add_argument("--version", action="version", version=f"%(prog)s {__version__}")
    sub = ap.add_subparsers(dest="cmd", metavar="command")
    sub.required = True

    p = sub.add_parser("info", parents=[common], help="model, serial, firmware, address, sync, health")
    p.add_argument("--json", action="store_true", help="also print the raw JSON")
    p.set_defaults(fn=cmd_info)

    p = sub.add_parser("download", parents=[common], help="save the settings of the lidar to a JSON file")
    p.add_argument("-o", "--output", help="file name ('-' = print). Default: <model>_<serial>.json")
    p.add_argument("-f", "--force", action="store_true", help="overwrite an existing file")
    p.set_defaults(fn=cmd_download)

    p = sub.add_parser("upload", parents=[common, write],
                       help="write a JSON file (all keys, or only some) to the lidar, restart, verify")
    p.add_argument("file")
    p.set_defaults(fn=cmd_upload)

    p = sub.add_parser("diff", parents=[common], help="show the differences between the lidar and a file")
    p.add_argument("file")
    p.set_defaults(fn=cmd_diff)

    p = sub.add_parser("check", parents=[common], help="check a file offline: ranges, lists, ports, addresses")
    p.add_argument("file")
    p.add_argument("--against-lidar", action="store_true", help="also compare with the lidar and its variant")
    p.add_argument("--allow-danger", action="store_true", help=argparse.SUPPRESS)
    p.set_defaults(fn=cmd_check)

    p = sub.add_parser("verify", parents=[common], help="compare the lidar with a file")
    p.add_argument("file")
    p.add_argument("--since", type=int, metavar="COUNT",
                   help="require a restart: the start-up count must be above COUNT")
    p.add_argument("--all", action="store_true", help="list also the values that agree")
    p.set_defaults(fn=cmd_verify)

    for name, fn, hlp in (("set", cmd_set, "change values: KEY=VALUE (key, label, or a unique part of it)"),
                          ("network", cmd_network, "apply the address profile 192.168.X.200 -> .102")):
        p = sub.add_parser(name, parents=[common, write], help=hlp)
        if name == "set":
            p.add_argument("assignments", nargs="+", metavar="KEY=VALUE")
        else:
            p.add_argument("x", type=int, metavar="X", help="the third octet of the new subnet")
            p.add_argument("--no-ports", action="store_true", help="do not set the MSOP, DIFOP and IMU ports")
        p.add_argument("--file", help="change this JSON file instead of the lidar")
        p.set_defaults(fn=fn)

    p = sub.add_parser("reboot", parents=[common], help="restart with POST /soft_reset (not tested) and wait")
    p.add_argument("-y", "--yes", action="store_true")
    p.add_argument("--restart-timeout", type=float, default=240.0)
    p.set_defaults(fn=cmd_reboot)

    p = sub.add_parser("params", help="list or find settings: web page label, key, values, meaning")
    p.add_argument("query", nargs="?")
    p.add_argument("--all", action="store_true", help="include the keys that are not on the web page")
    p.set_defaults(fn=cmd_params)

    p = sub.add_parser("ui-check", parents=[common], help="compare the live web form with the catalog of this tool")
    p.add_argument("--html", help="check a saved Parameter_Setting.html instead")
    p.set_defaults(fn=cmd_ui_check)

    p = sub.add_parser("configure", parents=[common, write], help="interactive menu (lidar or --file)")
    p.add_argument("--file", help="edit this file offline instead of the lidar")
    p.add_argument("--out", help="default file name for 'w'")
    p.set_defaults(fn=cmd_configure)
    return ap


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    if hasattr(args, "host"):
        if args.subnet is not None:
            if args.host:
                print("Give --host or --subnet, not both.")
                return EXIT_ERROR
            args.host = f"192.168.{args.subnet}.200"
        args.host = args.host or FACTORY_HOST
    try:
        return args.fn(args)
    except LidarError as e:
        print(style(f"Error: {e}", "red"), file=sys.stderr)
        host = getattr(args, "host", None)
        if host and "cannot reach" in str(e):
            hint = route_hint(split_host(host)[0])
            if hint:
                print("  " + hint, file=sys.stderr)
        return EXIT_ERROR
    except KeyboardInterrupt:
        print("\nStopped.", file=sys.stderr)
        return EXIT_ERROR


if __name__ == "__main__":
    sys.exit(main())
