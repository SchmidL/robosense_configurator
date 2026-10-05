# rs_webcfg

Read, change, write, and verify the settings of a RoboSense lidar through the web server
inside the lidar. No browser is necessary. It works on a laptop and on a headless box.

- One file: `rs_webcfg.py`. Python 3.8 or newer. Standard library only.
- Tested on the RoboSense Airy, software `26042112` (web page captured in
  `tests/fixtures/airy_26042112/`).
- The Fairy uses the same page family. Run `ui-check` before you write to a new model or
  a new firmware.
- The E1R has no web page. Use LidarAssistant for the E1R.

## Quick start

```bash
python3 rs_webcfg.py info  --subnet 13                 # lidar at 192.168.13.200
python3 rs_webcfg.py download --subnet 13 -o airy.json # all settings to a file
python3 rs_webcfg.py params "return"                   # find a setting by its label
python3 rs_webcfg.py set   --subnet 13 "Return Mode=Dual"
python3 rs_webcfg.py upload --subnet 13 airy.json      # write a file
python3 rs_webcfg.py configure --subnet 13             # menu in the terminal
```

Without `--host` or `--subnet`, the tool uses the factory address `192.168.1.200`.
`--host` also accepts `host:port`, for example an SSH tunnel `localhost:8080`.

On a box without a copy of the file:

```bash
ssh munin@<box> "python3 - info --subnet 13" < rs_webcfg.py
```

## A new lidar: one command for the address

All RoboSense lidars leave the factory at `192.168.1.200`. Connect one lidar at a time.
Give the computer port the address `192.168.1.102/24`. Then:

```bash
python3 rs_webcfg.py network 13      # X = 13: the third octet of the new subnet
```

`network X` writes the address profile. The profile is the RoboSense factory pattern with
a new third octet. It saved and survived a restart on the Airy and the Fairy. It sets only
the network values. It does not change the time sync settings:

| Field on the web page | Key | Value |
|---|---|---|
| Device IP Address | `SrcIp` | `192.168.X.200` |
| Device IP Mask | `SrcMask` | `255.255.255.0` |
| Device IP Gateway | `SrcGateWay` | `192.168.X.1` |
| Destination IP Address | `DstIp` | `192.168.X.102`, the computer port |
| MSOP / DIFOP / Imu Port Number | `MPort` / `DPort` / `IPort` | `6699` / `7788` / `6688` |

`--no-ports` keeps the ports. The values are in `PROFILE_ADDRESS` at the top of
`rs_webcfg.py`.

Set the time sync separately. For a `ptp4l` grandmaster with UDPv4 and E2E, domain 0:

```bash
python3 rs_webcfg.py set --subnet 13 SynSrc=PTP-E2E PDomNum=0
```

After the restart the lidar is at `192.168.X.200`. The computer needs an address in the
new subnet to read it back. The tool tells you the command, for example:

```bash
sudo ip addr add 192.168.13.102/24 dev <port>
```

Add the address in a second terminal while the tool waits for the restart.

## How a write works

`upload`, `set`, `network`, and the menu use the same steps:

1. **Check.** The rules of the web page (ranges, address rules, no port used twice), the
   options of this lidar variant, and the lessons below. An error stops everything.
2. **Show.** A table with each change: label on the web page, key, old value, new value.
3. **Ask.** Type `y`. `--yes` skips the question. Without a terminal and without `--yes`,
   nothing is sent.
4. **Backup.** The old values go to `rs_webcfg_backups/<model>_<serial>_<time>.json`.
5. **Save.** The same request as the browser: one POST to `/Parameter_Setting.html` for
   each tab with a change, with its own Save button. The tab with the address is last.
6. **Read back now.** This shows the running values. If none of the changes is visible,
   the lidar refused the save.
7. **Restart.** Default: you disconnect and connect the power. The tool waits until the
   start-up counter (`startup_times` in `/diag_data.json`) increases. That proves the
   restart. It looks at the old and at the new address.
8. **Verify.** It reads all values after the restart. `PASS`: each change is stored and
   no other value changed. `FAIL`: a table shows each value that is wrong. Exit code 3.

`--dry-run` stops after step 2. It prints the request bodies and sends nothing.

| Option | Effect |
|---|---|
| `--reboot manual` | Default. You cycle the power. The only tested method. |
| `--reboot soft` | POST `/soft_reset`. The web page uses it after a firmware update. Not tested for a restart without an update. If no restart comes, cycle the power. |
| `--reboot none` | No restart. The values are not proven. Use `verify --since <count>` later. |
| `--via json` | Use the "JSON Configuration Upload" of the System page (`/upload-json`) instead of the form. |
| `--allow-hidden` | With `--via json`: also send keys that are not on the web page. Their effect is not known. |
| `--allow-danger` | Allow `Restore Default` = ON. That erases all settings. |
| `--new-host H` | After an address change, look for the lidar at `H`, for example a new tunnel. |

## Files

`download` writes the same flat JSON as `/setting_data.json`. The web page can also upload it.

A file for `upload` can have all keys or only some. Missing keys keep the value of the
lidar. A value can be the code or the label on the web page:

```json
{ "SynSrc": "PTP-E2E", "ReMode": "Dual", "TFilLvl": 5 }
```

| Command | Use |
|---|---|
| `check FILE` | Check a file offline. `--against-lidar` also uses the lidar variant. |
| `diff FILE` | Differences between the lidar and the file. |
| `verify FILE --since N` | Compare. `--since N`: the start-up counter must be above `N`, so the values come from flash. |
| `set --file FILE K=V ...` | Change a file, not the lidar. `network X --file FILE` does the same. |
| `configure --file FILE` | Menu on a file, offline. |

## The web page is not the JSON

The labels and the keys differ, and some values are coded in a way that you do not expect.
Use `params` to find a setting, and `params <name>` for its values and meaning. `params --all`
also lists the keys that are not on the web page.

| On the web page | In the JSON | Trap |
|---|---|---|
| Dead Zone 10cm Enable = On | `DZ10cm` = `0` | Inverted |
| Poor Performance Channel Mask = On | `PoorMask` = `0` | Inverted |
| Number Of Lines = 48 | `L48` = `1` | The label is not the value |
| GPS Baud Rate = 9600 | `GPSBRate` = `3` | Codes 0 to 2 are removed from the page |
| Trigger Mode: check box Mode1(+25%) | `TriggerMode` = `1` | Two check boxes for one key. Mode2 = `0` |
| First Group: check box ON | `GroupSelect` = `1` | Off disables the three pulse fields; the browser then does not send them |
| Pulse Start Angle / Width / Step | `PulseStart` / `PulseWidth` / `PulseStep` | Other names in the form (`first_start_angle`, ...) |
| FOV Setting: two boxes | `FovS`, `FovE` | |
| Fields that a lidar variant hides | still in the form | The browser sends them. The lidar can ignore them |
| Rotation Speed, VLAN, Side Installation | `RotaSpe`, `MV*`, `DV*`, `PV*`, `MSideEn` | Commented out on the page. Only `--via json --allow-hidden` |
| Variant, capabilities | `VarT`, `CWCSts`, `Cyc`, `DRE` | Read only. They select what the page shows |

On this Airy (`VarT` = 0), the page hides `MSOP1 Port Number`, `Rain/Blockage Detection
Distance`, `Rain Detection Sensitivity`, `Blockage Detection Sensitivity`, and
`Poor Performance Channel Mask`. `Cyc` = 5 removes the IMU rate 500 Hz.

## Lessons from the bring-up

- **A save can return HTTP 200 and still not reach the flash.** On 2026-10-01 the Airy
  did not keep three saves of the pattern lidar `.10`, destination `.1` = gateway. The
  check gives a warning for destination = gateway. Step 6 and step 8 find such a failure.
- **`/setting_data.json` shows the running values.** Only a read after a restart proves a
  save. The start-up counter proves the restart.
- **The Airy sends UTC with `No Leap Second` = OFF.** The effect of ON is not tested.

## If the web page changes

`ui-check` reads `Parameter_Setting.html` from the lidar and compares each field, each
list, and each option with the catalog. Do this on each new model and firmware. If it
reports a difference, update `CATALOG` and `_builtin_layout()` in `rs_webcfg.py`, and add
the new page to `tests/fixtures/`.

The tool reads the form from the lidar at each write. It uses the built-in copy only if it
cannot read the page.

## Tests

```bash
cd misc/robosense_webcfg
python3 -m unittest discover -s tests -v
```

`tests/fake_lidar.py` is a fake Airy web server. It serves the captured pages and keeps
running and stored values apart, like the real lidar. Its modes `volatile` and `refuse`
simulate a save that is lost at the restart and a save that the lidar refuses. To practise
without a lidar:

```bash
python3 tests/fake_lidar.py --port 8080 &
python3 rs_webcfg.py configure --host 127.0.0.1:8080 --reboot soft
```

## Exit codes

| Code | Meaning |
|---|---|
| 0 | Done. For a write: `PASS` |
| 1 | Usage error, no connection, or you said no |
| 2 | The check found an error. Nothing was sent |
| 3 | The read-back does not agree, or no restart was seen |
