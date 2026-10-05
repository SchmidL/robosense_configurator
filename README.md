# rs_webcfg

Read, change, write, and verify the settings of a RoboSense lidar through the web server
inside the lidar. No browser is necessary. It works on a laptop and on a headless box.

- One file: `rs_webcfg.py`. Python 3.8 or newer. Standard library only.
- The E1R has no web page. Use LidarAssistant for the E1R.

## Tested models

| Model | Software (`bot_soft_v`) | Write test | Web page |
|---|---|---|---|
| Airy | `26042112` | PASS, 2026-10-05 | All fields of the catalog |
| Fairy | `26031303` | PASS, 2026-10-05 | 12 fields fewer, no IMU rate 500Hz. See `ui-check` |

Reduced copies of their pages are in `tests/fixtures/`. Run `ui-check` before you write to a new
model or a new firmware.

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
   The Airy and the Fairy answer a save with a TCP reset and no HTTP answer, but they
   apply the save. So the tool does not stop at a reset. Step 6 decides.
6. **Read back now.** This shows the running values. If none of the changes is visible,
   the lidar refused the save.
7. **Restart.** The Airy and the Fairy restart by themselves approximately 3 s after a
   save. The start-up counter (`startup_times` in `/diag_data.json`) proves the restart.
   If the counter does not increase, the tool asks you to disconnect and connect the
   power, and it waits. It looks at the old and at the new address.
8. **Verify.** It reads all values after the restart. `PASS`: each change is stored and
   no other value changed. `FAIL`: a table shows each value that is wrong. Exit code 3.

`--dry-run` stops after step 2. It prints the request bodies and sends nothing.

| Option | Effect |
|---|---|
| `--reboot manual` | Default. If the lidar does not restart by itself, you cycle the power. |
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
| `check FILE` | Check a file offline. `--against-lidar` also uses the fields and lists of the lidar. |
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

The Fairy page has no `VarT`. These fields are not in its form: `MSOP1 Port Number`,
`Motor Reversal`, the three rain and blockage fields, `Poor Performance Channel Mask`,
`Dead Zone 10cm Enable`, `Channel 81 85 89 93 Enable`, `Gap Filling Enable`,
`Ranging Mode`, `Rain Fog Switch`, and `Number Of Lines`. The JSON of the Fairy still has
most of these keys. The tool does not send them, and the check stops a change to them.

## Lessons from the bring-up

- **A save can return HTTP 200 and still not reach the flash.** On 2026-10-01 the Airy
  did not keep three saves of the pattern lidar `.10`, destination `.1` = gateway. The
  check gives a warning for destination = gateway. Step 6 and step 8 find such a failure.
- **Only a read after a restart proves a save.** The start-up counter proves the restart.
- **The Airy sends UTC with `No Leap Second` = OFF.** The effect of ON is not tested.

## If the web page changes

The catalog in `rs_webcfg.py` holds every field that the tool knows. Each model has a
part of it. The tool reads the form of the lidar before each write, and it sends only the
fields of that form. Without the form, it sends nothing.

`ui-check` compares the form of the lidar with the catalog:

- A field or an option of the catalog that the page does not have: a note. That is
  normal for a smaller model. Uploads are possible.
- A field, an option, or a label on the page that the catalog does not know: an error,
  exit code 2. Add it to `CATALOG` in `rs_webcfg.py`, and add the page to
  `tests/fixtures/` with `tests/make_fixture.py` (see below).

## Tests

```bash
python3 -m unittest discover -s tests -v
```

`tests/fixtures/<model>_<software>/` holds the test data of each tested lidar. The pages
are not copies of the RoboSense pages. `tests/make_fixture.py` keeps only the controls of
the settings form (names, lists, options, in page order), and it replaces the serial
number and the MAC address with fake values. For a new model, save
`Parameter_Setting.html`, `setting_data.json`, `device_data.json`, and `diag_data.json` of
the lidar in a folder, then:

```bash
python3 tests/make_fixture.py <folder> tests/fixtures/<model>_<software>
```

Do not commit the original pages: RoboSense holds their copyright.

`tests/fake_lidar.py` is a fake lidar web server (`--model` selects Airy or Fairy). It
serves the test data and keeps running and stored values apart, like the real lidar. Its mode `reset` acts like the real lidars:
TCP reset after a save, then a restart. The modes `volatile` and `refuse` simulate a save
that is lost at the restart and a save that the lidar refuses. To practise
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

## License

MIT, see `LICENSE`. RoboSense, Airy, and Fairy are names of RoboSense. This project has
no connection with RoboSense.
