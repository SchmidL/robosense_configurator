#!/usr/bin/env python3
"""Make a test fixture from the pages of a real lidar, without the vendor content.

The fixture keeps only what the tests need:

* Parameter_Setting.html: only the controls of the settings form, in page order
  (names, types, values, list options). No vendor script, style, or text.
* setting_data.json and diag_data.json: as captured.
* device_data.json: the serial number and the MAC address are replaced by fake values.

Usage, with a folder that holds the four files as the lidar served them:

    python3 tests/make_fixture.py <capture folder> tests/fixtures/<model>_<software>
"""

from __future__ import annotations

import html
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import rs_webcfg as rs  # noqa: E402

FAKE_IDENTITY = {"sn": "000000000000", "mac_addr": "02:00:00:00:00:00"}  # locally administered MAC


def reduced_form(page_html: str, model: str, software: str) -> str:
    controls = rs.parse_form(page_html)
    out = ["<!DOCTYPE html>",
           f"<!-- Reduced copy of Parameter_Setting.html of a RoboSense {model}, software {software}.",
           "     Only the controls of the settings form, in page order. Made by tests/make_fixture.py. -->",
           '<html><body><form method="POST">']
    a = html.escape
    for c in controls:
        if c.kind == "radio":
            out.append(f'<input type="radio" name="{a(c.name)}">')
        elif c.kind in ("checkbox", "submit"):
            out.append(f'<input type="{c.kind}" name="{a(c.name)}" value="{a(c.value)}">')
        elif c.kind == "select":
            opts = "".join(f'<option value="{a(v)}">{a(lab)}</option>' for v, lab in c.options)
            out.append(f'<select name="{a(c.name)}">{opts}</select>')
        else:
            out.append(f'<input type="text" name="{a(c.name)}">')
    out.append("</form></body></html>")
    return "\n".join(out) + "\n"


def main(src: str, dst: str) -> None:
    def read(name: str) -> str:
        with open(os.path.join(src, name), encoding="utf-8") as f:
            return f.read()

    device = json.loads(read("device_data.json"))
    device.update({k: v for k, v in FAKE_IDENTITY.items() if k in device})
    model = device.get("lidar_model", "lidar")
    software = device.get("bot_soft_v", "unknown")
    os.makedirs(dst, exist_ok=True)
    files = {
        "Parameter_Setting.html": reduced_form(read("Parameter_Setting.html"), model, software),
        "device_data.json": json.dumps(device, separators=(",", ":")) + "\n",
        "setting_data.json": read("setting_data.json"),
        "diag_data.json": read("diag_data.json"),
    }
    for name, text in files.items():
        with open(os.path.join(dst, name), "w", encoding="utf-8", newline="\n") as f:
            f.write(text)
    print(f"Wrote {len(files)} files to {dst}")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    main(sys.argv[1], sys.argv[2])
