#!/usr/bin/env python3
"""A fake RoboSense Airy web server, for the tests and for a dry practice run.

It serves the captured pages of a real Airy (fixtures/airy_26042112) and keeps two
copies of the settings, like the real lidar: the running values (shown by
/setting_data.json) and the stored values (used after a restart).

It has its own map of form fields to keys, on purpose: it must not share the
catalog of rs_webcfg.py, or a wrong catalog would pass its own test.

Modes:
  normal    a save changes the running and the stored values
  volatile  a save changes only the running values (lost at a restart)
  refuse    a save with destination = gateway is ignored (seen on 2026-10-01)
  reset     a save works, the answer is a TCP reset, and the lidar restarts by
            itself (the real Airy, 2026-10-05)

Practice run:  python3 fake_lidar.py --port 8080
               python3 ../rs_webcfg.py info --host 127.0.0.1:8080
"""

from __future__ import annotations

import argparse
import copy
import json
import os
import socket
import struct
import threading
import time
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Dict, List, Optional, Tuple

FIXTURE_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "fixtures")
MODELS = ("airy_26042112", "fairy_26031303")   # folders in fixtures/
FIXTURES = os.path.join(FIXTURE_ROOT, MODELS[0])

SECTION_FIELDS = {
    "save_param": ("SrcIp SrcMask SrcGateWay DstIp MPort MPort1 DPort ReMode FovS FovE SynSrc PDomNum "
                   "RTPdelay NoLeap SyTOut UToL LToU OpM PhaLockSet GPSBRate MotorR ImuCtrl IPort ImuOutR "
                   "AccRg GyroRg Lpf ResDefault").split(),
    "save_param_performance": "RefEn TFilLvl BlkRnDis Rn Blk FSAgl PoorMask DZ10cm C81858993 GapFill RanM "
                              "RnFgSwt L48".split(),
    "save_param_pulse": ["first_start_angle", "first_pulse_width", "first_pulse_step"],
}
PULSE_MAP = {"first_start_angle": "PulseStart", "first_pulse_width": "PulseWidth", "first_pulse_step": "PulseStep"}


def _read(name: str, model: str = MODELS[0]) -> bytes:
    with open(os.path.join(FIXTURE_ROOT, model, name), "rb") as f:
        return f.read()


class FakeAiry:
    def __init__(self, mode: str = "normal", downtime_s: float = 0.3, model: str = MODELS[0]) -> None:
        self.mode = mode
        self.model = model
        self.downtime_s = downtime_s
        self.running: Dict[str, str] = json.loads(_read("setting_data.json", model))
        self.stored = copy.deepcopy(self.running)
        self.device = json.loads(_read("device_data.json", model))
        self.diag = json.loads(_read("diag_data.json", model))
        self.form_html = _read("Parameter_Setting.html", model)
        self.posts: List[Tuple[str, List[Tuple[str, str]]]] = []
        self.down_until = 0.0
        self.lock = threading.Lock()

    def restart(self) -> None:
        with self.lock:
            self.running = copy.deepcopy(self.stored)
            self.diag["startup_times"] = str(int(self.diag["startup_times"]) + 1)
            self.down_until = time.monotonic() + self.downtime_s

    def apply_form(self, pairs: List[Tuple[str, str]]) -> None:
        names = [n for n, _ in pairs]
        submit = next((n for n in names if n in SECTION_FIELDS), None)
        if submit is None:
            return
        d = dict(pairs)
        new = copy.deepcopy(self.running)
        for f in SECTION_FIELDS[submit]:
            if f in d:
                new[PULSE_MAP.get(f, f)] = d[f]
        if submit == "save_param_pulse":
            new["TriggerMode"] = "1" if "mode1" in d else "0"
            new["GroupSelect"] = "1" if "first_group_switch" in d else "0"
        self._commit(new)

    def apply_json(self, pairs: List[Tuple[str, str]]) -> None:
        new = copy.deepcopy(self.running)
        for k, v in pairs:
            if k in new:
                new[k] = v
        self._commit(new)

    def _commit(self, new: Dict[str, str]) -> None:
        if self.mode == "refuse" and new.get("DstIp") == new.get("SrcGateWay"):
            return
        with self.lock:
            self.running = new
            if self.mode != "volatile":
                self.stored = copy.deepcopy(new)


def make_handler(fake: FakeAiry):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.0"   # like lwIP

        def log_message(self, *a) -> None:  # quiet
            pass

        def _send(self, code: int, body: bytes, ctype: str) -> None:
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _down(self) -> bool:
            if time.monotonic() < fake.down_until:
                self._send(503, b"booting", "text/plain")
                return True
            return False

        def do_GET(self) -> None:
            if self._down():
                return
            path = self.path.split("?")[0]
            if path == "/setting_data.json":
                self._send(200, json.dumps(fake.running).encode(), "application/json")
            elif path == "/device_data.json":
                self._send(200, json.dumps(fake.device).encode(), "application/json")
            elif path == "/diag_data.json":
                self._send(200, json.dumps(fake.diag).encode(), "application/json")
            elif path == "/Parameter_Setting.html":
                self._send(200, fake.form_html, "text/html")
            else:
                self._send(404, b"404 - Page not found", "text/html")

        def do_POST(self) -> None:
            if self._down():
                return
            n = int(self.headers.get("Content-Length", "0") or 0)
            body = self.rfile.read(n).decode("ascii", "replace")
            pairs = urllib.parse.parse_qsl(body, keep_blank_values=True)
            fake.posts.append((self.path, pairs))
            if self.path == "/Parameter_Setting.html":
                fake.apply_form(pairs)
                if fake.mode == "reset":
                    fake.restart()
                    # Close with SO_LINGER 0: the client gets a RST and no answer.
                    self.connection.setsockopt(socket.SOL_SOCKET, socket.SO_LINGER, struct.pack("ii", 1, 0))
                    self.connection.close()
                    self.close_connection = True
                    return
                self._send(200, fake.form_html, "text/html")
            elif self.path == "/upload-json":
                fake.apply_json(pairs)
                self._send(200, b"", "text/html")
            elif self.path == "/soft_reset":
                self._send(200, b"", "text/html")
                threading.Timer(0.05, fake.restart).start()
            else:
                self._send(404, b"404 - Page not found", "text/html")

    return Handler


class FakeServer:
    """Start with `with FakeServer() as srv:`; the address is srv.host."""

    def __init__(self, mode: str = "normal", port: int = 0, model: str = MODELS[0]) -> None:
        self.fake = FakeAiry(mode, model=model)
        self.httpd = ThreadingHTTPServer(("127.0.0.1", port), make_handler(self.fake))
        self.thread: Optional[threading.Thread] = None

    @property
    def host(self) -> str:
        return f"127.0.0.1:{self.httpd.server_address[1]}"

    def __enter__(self) -> "FakeServer":
        self.thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        self.thread.start()
        return self

    def __exit__(self, *exc) -> None:
        self.httpd.shutdown()
        self.httpd.server_close()


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--mode", choices=("normal", "volatile", "refuse", "reset"), default="normal")
    ap.add_argument("--model", choices=MODELS, default=MODELS[0], help="captured pages to serve")
    a = ap.parse_args()
    srv = FakeServer(a.mode, a.port, a.model)
    print(f"Fake {a.model} ({a.mode}) at http://{srv.host}. Ctrl+C stops. "
          "A soft reset (POST /soft_reset) is the power cycle.")
    try:
        srv.httpd.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
