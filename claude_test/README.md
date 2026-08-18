# claude_test

Debug and hardware verification scripts. These are one-off diagnostics,
not CI tests -- production-quality tests belong in `tests/`.

Run them from the repository root with the package importable:

```bash
PYTHONPATH=. .venv/bin/python claude_test/verify_connection.py 192.168.1.238
```

| File | What it does | What it verified |
|---|---|---|
| `verify_connection.py` | V1. Probes the seven service ports, connects `URRobotController`, prints the connection report and the robot state. Commands no motion. | Dashboard 29999, Primary 30001, Secondary 30002 and RTDE 30004 accept connections on `192.168.1.238`; 4242, 50002 and 63352 time out. Controller is PolyScope 5.25.2, so `polyscope_x=False` and `use_ext_urcap=False` are the correct settings for this cell. |
| `verify_joint1_move.py` | V2. Moves joint 1 by +20 deg at 0.3 rad/s and returns, reporting arrival error against the 0.001 rad limit of specification T02. **Moves the robot.** | **PASS** on 2026-08-13 with the operator present. Joint 1 travelled +20.0069 deg, outbound error 0.000121 rad, return error 0.000019 rad, 1.72 s each way. Mode moved IDLE -> MOTION -> IDLE. Requires Remote Control; in local control it raises `ModeConflictError` (E600). |
| `verify_gripper_travel.py` | V3. Activates the Hand-E and drives it 0 % to 100 % to 0 %, reporting position error and OBJ status. **Moves the fingers.** | **PASS** on 2026-08-18 with the operator present, after the firewall was opened. Travelled 0 % (POS 3) to 100 % (POS 249) to 0 %, OBJ 3 at every stop, about 1.05 s per stroke, error within 6 counts. POS 3 and 249 rather than 0 and 255 are the Hand-E's calibrated end stops. Two real bugs surfaced here: `GTO` was being set before activation finished (answered FLT 7), and `OBJ` was polled before the gripper accepted the new target, so a fresh command read the previous move's completion and returned instantly. |
| `debug_urcap_daemons.py` | Probes `127.0.0.1:29999`, `:63352` and `:4242` from the controller itself via URScript, reporting through RTDE registers 12 to 14 with a run id written last into register 15. Port 29999 is the control. Issues no move command. | Both URCap ports are **OPEN from inside the controller**, and so is the 29999 control. Combined with an external TIMEOUT, that means the **controller firewall drops external traffic** to 63352 and 4242 -- the URCaps are installed and running. Two traps found: `ur_rtde` only exposes output int registers 12 to 19 (register 0 raises `ValueError`), and polling the result registers directly reads leftover values from an earlier run, which is what produced this file's first, wrong verdict. |
| `debug_urcap_script_api.py` | Probes which URCap `rq_*` URScript functions the controller defines, by referencing each candidate inside `if False` so the program must compile but nothing executes. Issues no move command. | **None of the 14 candidates are defined** in a script sent to port 30002. URCap script functions are injected by the installation node into programs built on the pendant, so they do not exist in an externally supplied script. This rules out driving the gripper by `rq_*` calls from outside. |
| `debug_port_policy.py` | Scans the controller's inbound TCP policy from outside, with port 12345 as a control that nothing listens on. Read-only. | The open set is **exactly UR's native service ports** (502, 29999, 30001 to 30004, 30020). Every URCap port (4242, 50001, 50002, 63352) and every system port (22, 80, 443) is dropped, and so is the closed control port 12345. That is a **default-deny allow-list** built from UR's own port list, not a rule aimed at the URCaps. It also explains why 50002 looked absent: the External Control URCap may well be installed. |
| `debug_loopback_bridge.py` | Runs a URScript on the controller that talks to `127.0.0.1:63352` and `127.0.0.1:4242` and returns the answers in RTDE registers. Validates both protocols while eth0 is still firewalled. Issues no move command. | Gripper answered `STA 0` in 6 bytes, so the Robotiq daemon and its ASCII protocol are confirmed working (STA 0 = not yet activated after the reboot). The camera answered **HTTP 200 OK** on `/current.jpg?type=color`, which confirms the URL that specification section 7 item 2 flagged as unverified. Two URScript traps recorded in the file header: string literals do not interpret `\n` escapes, so terminators must come from `socket_send_line`; and `socket_read_string` cannot distinguish 'nothing arrived yet' from an empty reply, so `socket_read_byte_list` is the reliable read. |
| `verify_camera_frame.py` | V4. Fetches all four wrist camera image types over the Vision URCap HTTP endpoint and saves them under `claude_test/output/`. Commands no motion. | **PASS** on 2026-08-18 after the firewall was opened. All four types decoded at 640x480 BGR from `/current.jpg?type=<type>`. Mean intensity differs per type (magnitude 3.72, annotations 126.33), which confirms the `type` parameter is honoured rather than ignored. |

## Environment notes

- The robot must be in **Remote Control** for V2. In local control the
  controller refuses externally supplied scripts and `move_j` raises
  `ModeConflictError` (E600).
- V3 needs port 63352 and V4 needs port 4242 reachable **from the
  external PC**. From PolyScope 5.14 onward UR closes every eth0 port
  not tied to an enabled service, while leaving loopback untouched.
  Open them on the pendant under **Settings -> Security -> General**
  (Admin password); the field lists ports to BLOCK, so
  `1-4241,4243-63351,63353-65535` leaves 4242 and 63352 reachable.
  Done on 2026-08-18, after which V3 and V4 both pass.
- Port 50002 answers with a connection refused rather than a timeout
  once the firewall is open, which proves the External Control URCap is
  genuinely not installed. `use_ext_urcap=False` stays correct.
- Diagnosing a dropped port needs both directions. Externally a
  firewall DROP looks like a timeout while a dead daemon gives an
  immediate connection refused; probing from inside the controller
  separates the two conclusively.
