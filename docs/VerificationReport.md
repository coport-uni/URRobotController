# Verification Report — URRobotController

Conformance of the implementation against
`docs/URWorkCell_개발사양서.md` v1.0, as measured on the real cell.

| Item | Value |
|---|---|
| Robot | UR7e at `192.168.1.238` |
| Controller | PolyScope 5.25.2.130406 (Apr 22 2026) |
| Gripper | Robotiq Hand-E, URCap socket on 63352 |
| Camera | Robotiq Wrist Camera, URCap HTTP on 4242 |
| Verified | 2026-08-13 and 2026-08-18, operator present |
| Client | Python 3.12, `ur_rtde` 1.6.5, OpenCV 5.0.0 |

---

## 1. Deviations from the specification

The cell differs from the specification's assumptions in three ways.
All are handled by constructor arguments, not by forks in the code.

| Item | Spec assumed | Measured | Handling |
|---|---|---|---|
| PolyScope generation | X, programs without extension | 5.25.2, programs named `*.urp` | `polyscope_x=False` |
| External Control URCap | in use, port 50002 | not installed — the port answers with a connection refused once the firewall is open | `use_ext_urcap=False`, so `ur_rtde` uploads the control script itself |
| RTDE frequency | 500 Hz, spec section 3.2 | the controller closes the stream after about 1.5 s at 500 Hz; stable at 125 Hz | `rtde_frequency=125.0` |

The wrist camera URL that specification section 7 item 2 flags as an
unverified, unofficial path was confirmed: `/current.jpg?type=<type>`
answers HTTP 200 and honours the `type` parameter.

---

## 2. Test items from specification section 8

| # | Item | Result | Evidence |
|---|---|---|---|
| T01 | Connect and release four interfaces | **PASS** | All four report OK; `verify_connection.py` |
| T02 | `move_j` and `move_l` alone | **PASS** | Joint error 0.000121 rad against the 0.001 rad limit; TCP error 0.15 mm against the 0.5 mm limit |
| T03 | `gripper` with `trigger="start"` | **PASS** | Gripper command issued 6.6 ms after the move was dispatched, limit 50 ms |
| T04 | `gripper` with a remaining-distance trigger | **PASS** | Fired at 0.09972 rad remaining against a 0.10000 rad threshold, error 0.00028 rad within the 0.003 rad that a 10 ms watch period at 0.3 rad/s allows |
| T05 | `gripper` with `trigger="end"` | **PASS** | Fired 45 ms after the motion completed, limit 50 ms |
| T06 | `wait_object` grasp detection | **PARTIAL** | `wait_object` reports the settled position and OBJ correctly (POS 249, OBJ 3 after a close). OBJ 1 and 2 need an object physically between the fingers and were **not** exercised |
| T07 | Stored program execution and mode switch | **NOT VERIFIED** | Needs a stored program whose motion is known to be safe. The program on the controller is unknown, so running it was not attempted |
| T08 | Four camera image types | **PASS** | All four decode at 640x480 BGR; mean intensity differs per type, confirming `type` is honoured |
| T09 | Tolerance of a gripper communication failure | **PASS** | Motion completed and arrived while `gripper_error` was recorded and `gripper_pos` stayed `None` |
| T10 | Protective stop handling | **NOT VERIFIED** | Needs a deliberate protective stop on the bench |

Beyond the section 8 list, the non-blocking path of section 4.4 item 3
was verified: `blocking=False` returned in 18 ms and
`wait_motion_done()` then reported the finished motion.

### Not verified, and why

- **T06 object detection**, **T07 stored programs** and **T10
  protective stop** have not run. T06 and T10 need a physical setup
  step; T07 needs a program whose behaviour is known. The code paths
  exist but must be treated as unproven until they run on the bench.

---

## 3. Measured results

### T01 — interfaces

```
  motion     OK        robot mode : Robotmode: RUNNING
  gripper    OK        safety     : Safetystatus: NORMAL
  camera     OK        mode       : IDLE
  dashboard  OK
```

### T02 — motion alone

```
--- move joint 1 by +20.0 deg ---
  outbound error : 0.000121 rad (limit 0.001) PASS
  joint 1 travel : +20.0069 deg
  return   error : 0.000019 rad (limit 0.001) PASS

=== TCP Z shift, plus and minus 20 mm ===
  up      : offset +19.85 mm   0.61 s
  down    : offset -19.93 mm   0.62 s
  returned to start within 0.0061 mm
```

### T03, T04, T05, T09 — motion with a gripper action

```
=== T03  trigger="start" ===
  fire latency     : 6.6 ms (limit 50 ms)                       PASS
=== T05  trigger="end" ===
  gripper_pos      : 249    gripper_obj : 3                     PASS
  post-fire slack  : 45 ms after motion completion
=== T04  trigger="remaining_joint", threshold 0.1 rad ===
  remaining at fire: 0.09972 rad, error 0.00028 rad             PASS
=== 4.4-3  blocking=False ===
  call returned in : 18.3 ms, final motion_ok True              PASS
=== T09  gripper unavailable ===
  motion_ok True, arrived True, gripper_error recorded          PASS
```

### T08 — camera

```
--- color ---       shape (480, 640, 3)  mean intensity  97.92
--- edges ---       shape (480, 640, 3)  mean intensity  98.32
--- magnitude ---   shape (480, 640, 3)  mean intensity   3.72
--- annotations --- shape (480, 640, 3)  mean intensity 126.33
```

---

## 4. Bugs found by running on hardware

Every one of these was invisible to a reading of the code, and four of
the five were only reachable after the firewall was opened.

| # | Symptom | Cause | Fix |
|---|---|---|---|
| 1 | `gripper_activate()` raised `FLT 7` | `GTO 1` requests motion and was sent before activation reported `STA 3` | Wait for `STA 3` first; treat FLT 5 and 7 as expected during activation |
| 2 | A 0 to 255 gripper move "completed" in 0.02 s without moving | `OBJ` was polled before the gripper accepted the new target, so it read the previous move's completion | Wait for the `PRE` echo of the accepted target before polling `OBJ` |
| 3 | `wait_object` reported the gripper open right after a close | The same race, in a second place: `_collect_gripper_status` had its own loop with no echo wait | Share one `_wait_gripper_settled()` between both callers |
| 4 | Camera recordings aborted at random | The URCap occasionally stalls past a 3 s timeout under back-to-back requests | One retry per fetch, a 5 s timeout, and a dropped frame no longer kills a recording |
| 5 | The first `move_j` delayed its `"start"` trigger by 212 ms | The one-off RTDE control script upload was charged to the first motion | Added `acquire_motion()` so the cost can be paid during setup |

---

## 5. Controller firewall

From PolyScope 5.14 onward UR closes every eth0 port not tied to an
enabled service, while leaving loopback untouched. That is what made
the two URCaps look absent.

Diagnosis, all read-only:

1. `debug_port_policy.py` — the open set was exactly UR's native
   service ports, and a port that nothing listens on timed out instead
   of refusing, which identifies a default-deny allow-list rather than
   a rule aimed at the URCaps.
2. `debug_urcap_daemons.py` — both URCap ports were reachable from
   inside the controller, so the URCaps were installed and running.
3. `debug_loopback_bridge.py` — both protocols answered correctly over
   loopback before the firewall was touched.

Fix, on the pendant under **Settings → Security → General** with the
Admin password. The field lists ports to **block**:

```
1-4241,4243-63351,63353-65535
```

Pair it with "Restrict inbound network access to a specific subnet".

> Related: CVE-2026-8153, an unauthenticated command injection in the
> PolyScope 5 Dashboard Server (CVSS 9.8), is fixed in 5.25.1. This
> controller runs 5.25.2 and is patched. Worth recording because this
> class drives the Dashboard server on port 29999.

---

## 6. Sources

- [UR client libraries for external monitoring and control](https://www.universal-robots.com/developer/client-libraries-for-external-monitoring-and-control/)
- [RTDE Python Client Library](https://github.com/UniversalRobots/RTDE_Python_Client_Library)
- [Securely enable external access to services on UR cobots](https://www.universal-robots.com/articles/ur/cybersecurity/securely-enable-external-access-to-services-on-ur-cobots/)
- [CVE-2026-8153](https://www.universal-robots.com/articles/ur/cybersecurity/cve-2026-8153-command-injection-in-the-polyscope-5-dashboard-server/)
- `docs/UR_e-Series_Software_Handbook_PolyScope5_SW5.19_en.pdf`
- `docs/UR_Connectivity_Kit_PolyScope5X_723-162-00_en.pdf`

UR ships no official Python library beyond RTDE; the library that spans
RTDE, Dashboard and Interpreter is C++. Implementing the Dashboard,
gripper and camera protocols directly in this class is therefore the
only option available, not a shortcut taken around one.
