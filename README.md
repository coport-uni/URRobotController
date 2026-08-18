# URRobotController

Integrated control for a UR7e work cell — RTDE motion, a Robotiq Hand-E
gripper, a Robotiq wrist camera, and stored controller programs — in one
class, `URRobotController`.

Its defining feature is that a motion and a gripper action are a single
call. `move_l(pose, gripper=GripperAction.close(trigger="remaining_dist",
threshold=0.05))` starts the motion and closes the fingers once the TCP
is 50 mm from the target, without the caller writing a wait loop.

Implements `docs/URWorkCell_개발사양서.md` v1.0.

---

## 1. Set up the environment

```bash
conda env create -f environment.yml
conda activate ur-workcell
```

Check the install without touching the robot:

```bash
python -c "import rtde_control, cv2, requests; print('ok')"
python main.py --help
```

---

## 2. Prepare the robot

Three things must be true before anything below works. The `status`
check in step 3 reports all three, so run that first and come back here
if it complains.

| Requirement | Why | Where |
|---|---|---|
| Pendant in **Remote Control** | e-Series refuses externally supplied scripts in local control, so motion is impossible without it | Top-right dropdown on the pendant. If absent, enable it under Settings → System → Remote Control |
| Ports **63352** and **4242** open | PolyScope 5.14 and newer close every eth0 port not tied to an enabled service, which blocks the gripper and camera URCaps | Settings → Security → General (Admin password). The field lists ports to **block**, so enter `1-4241,4243-63351,63353-65535` |
| Robot powered and brakes released | Motion scenarios need the arm running | Pendant power panel |

> **Safety.** Scenarios 1, 2 and 3 move the arm and the fingers. Clear
> the work area and keep the e-stop within reach. Every scenario that
> moves anything asks for confirmation first; `--yes` skips the prompt
> and should only be used once you have seen the motion.

---

## 3. Verify each feature

Each feature runs on its own. The robot IP defaults to
`192.168.1.238`; override it with `--ip`.

### Step 0 — status, moves nothing

Always start here. It reports every interface, the PolyScope version,
the safety state and whether Remote Control is on.

```bash
python main.py status
```

Expected: all four interfaces `OK`, and `remote control: True`.

```
  motion    : OK
  gripper   : OK
  camera    : OK
  dashboard : OK

  polyscope     : URSoftware 5.25.2.130406 (Apr 22 2026)
  safety        : Safetystatus: NORMAL
  remote control: True
```

If an interface says `NOT REACHABLE`, fix it with the table in section 2
before going further.

### Feature 1 — joint motion

Swings joint 1 by +20°, back, −20°, back, relative to wherever the arm
is standing. **The arm moves.**

```bash
python main.py joints
```

Expected: each offset within a few thousandths of a degree of ±20, and a
return to the start within about 1e-5 rad.

```
  positive: offset +20.000 deg   1.71 s
  negative: offset -20.007 deg   1.72 s
  returned to start within 0.000007 rad
```

### Feature 2 — TCP motion

Shifts the tool ±20 mm along the base Z axis with straight-line moves,
so the tool keeps its orientation. **The arm moves.**

```bash
python main.py tcp
```

Expected: about ±19.9 mm of travel and a return within a few hundredths
of a millimetre. The ~0.1 mm shortfall is the controller's own
convergence, well inside the 0.5 mm the specification asks for.

```
  up      : offset +19.87 mm   0.62 s
  down    : offset -19.91 mm   0.62 s
  returned to start within 0.0351 mm
```

### Feature 3 — gripper

Activates the gripper, opens it, holds three seconds, then closes.
**The fingers move.**

```bash
python main.py gripper
```

Expected: `POS 3` when open and `POS 249` when closed — those are the
Hand-E's calibrated end stops, not 0 and 255. `OBJ 3` means the fingers
reached the target; `OBJ 1` or `OBJ 2` means they stopped on an object.

```
  opened  : POS   3  OBJ 3
  closed  : POS 249  OBJ 3
```

Put an object between the fingers and run it again to see `OBJ 2`.

### Feature 4 — wrist camera

Records three seconds of frames and writes a video. Moves nothing.

```bash
python main.py camera --video wrist_camera_demo.mp4
```

Expected: about 20 frames at 640 × 480, saved at the measured rate so
the clip is the same length as the recording.

```
  captured 21 frames in 3.10 s
  resolution 640 x 480, 6.8 fps
  saved to wrist_camera_demo.mp4
```

The URCap serves one still per request rather than a stream, which is
why the rate is around 7 fps rather than video rate.

### Feature 5 — motion and gripper in one call

The specification's headline feature. Verified by a dedicated script
rather than a demo scenario, because it needs timing instrumentation.
**The arm and the fingers move.**

```bash
PYTHONPATH=.:claude_test python claude_test/verify_motion_gripper_sync.py
```

Expected: every trigger mode within its bound.

```
=== T03  trigger="start" ===          fire latency 6.6 ms (limit 50 ms)
=== T05  trigger="end" ===            gripper_pos 249, gripper_obj 3
=== T04  trigger="remaining_joint" ===  fired at 0.09972 of 0.10000 rad
=== 4.4-3  blocking=False ===         call returned in 18 ms
=== T09  gripper unavailable ===      motion continued, error recorded
V5 PASS
```

### All of them, in order

```bash
python main.py                 # asks for confirmation first
python main.py --yes           # skips the prompt
```

### Combinations

```bash
python main.py status camera   # the two that move nothing
python main.py joints tcp      # arm only, no gripper
python main.py --ip 192.168.0.100 status
```

---

## 4. Use the class directly

```python
from URRobotController import URRobotController, GripperAction

settings = dict(polyscope_x=False, use_ext_urcap=False,
                rtde_frequency=125.0)

with URRobotController("192.168.1.238", **settings) as robot:
    robot.gripper_activate()

    # Move and grip in one call: start closing once the TCP is 50 mm out.
    result = robot.move_l(
        pick_pose, speed=0.1,
        gripper=GripperAction(position=255, force=100,
                              trigger="remaining_dist", threshold=0.05,
                              wait_object=True))

    if result.gripper_obj in (1, 2):
        robot.move_l(place_pose, gripper=GripperAction.open(trigger="end"))
```

`with` is the recommended form: it tears down the RTDE control script
even when a motion raises.

### Settings for this cell

The constructor defaults follow the specification, but this controller
needs three of them overridden. `main.py` applies these already.

| Argument | Value | Why |
|---|---|---|
| `polyscope_x` | `False` | The controller runs PolyScope 5.25.2 and names programs `*.urp` |
| `use_ext_urcap` | `False` | No External Control URCap is installed, so `ur_rtde` uploads the control script itself |
| `rtde_frequency` | `125.0` | At 500 Hz this controller closes the RTDE stream after about 1.5 s |

### Errors

Everything raises from `WorkCellError`, and each carries the
specification's error code, the interface at fault, and the device's own
reply.

```
ModeConflictError: [E600] (rtde) the robot is in local control; switch
the pendant to Remote Control before commanding motion over RTDE
| response: 'MANUAL'
```

| Code | Class | Usual cause |
|---|---|---|
| E100 | `ConnectionFailedError` | Interface unreachable — check the firewall |
| E200 | `MotionError` | Target unreachable or outside the safety limits |
| E201 | `ProtectiveStopError` | Protective or emergency stop |
| E300 | `GripperError` | Gripper fault; the message names the fault code |
| E301 | `GripperNotActivatedError` | `gripper_activate()` was not called |
| E400 | `CameraError` | Camera endpoint silent or the image did not decode |
| E500 | `ProgramError` | Stored program failed to load or start |
| E600 | `ModeConflictError` | Local control, or a stored program owns the robot |

---

## 5. Repository layout

| Path | Contents |
|---|---|
| `URRobotController.py` | The controller class, the data types and the exception hierarchy |
| `main.py` | The five scenarios above |
| `claude_test/` | Hardware verification and diagnostic scripts, indexed in its own README |
| `docs/` | The specification, the verification report, and the UR manuals consulted |
| `environment.yml` | The conda environment |

## 6. Development

```bash
ruff check .
ruff format --check .
```

Both must pass before committing; the repository hooks enforce it. See
`CLAUDE.md` for the full conventions and `LearnedPatterns.md` for the
traps this cell has already sprung.

`docs/VerificationReport.md` records what has been measured against the
specification, including the three test items that have **not** been
exercised yet — object detection with something in the fingers, stored
program execution, and protective stop handling.
