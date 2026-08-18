# LearnedPatterns

Lessons carried forward from completed `ToDo.md` items, so they are
reused rather than rediscovered. Format per CLAUDE.md section 10.

---

## 1. Recurring Issues

**Reading a result register before the device has written it**
- **Problem**: A gripper move returned instantly and a URCap probe
  reported the wrong verdict, both because a status value was read
  before the device had refreshed it.
- **Cause**: `OBJ` still held the previous move's completion status,
  and the RTDE output registers still held the previous run's values.
- **Fix**: Wait for a positive handshake before trusting a status --
  the `PRE` echo for a gripper move, and a run-id register written last
  for a URScript probe.
- **Rule**: Always confirm a device accepted the new command before
  polling for its completion; never treat a stale register as an
  answer. (from ToDo#1 V3, ToDo#1 URCap diagnosis)
- **Recurrence**: the identical race reappeared in a second place,
  `_collect_gripper_status`, and made `wait_object` report the previous
  move's state. Fixing a race in one caller is not fixing it; find
  every reader of that status and give them one shared wait.
  (from ToDo#3 V5)

---

## 2. Solved Gotchas

**A timeout and a connection refused mean different things**
- **Problem**: Ports 63352 and 4242 were unreachable and it was unclear
  whether the URCap was missing or the traffic was filtered.
- **Cause**: Both failure modes look like "cannot connect" from the
  calling code.
- **Fix**: Probe a port that is certainly closed as a control. An
  immediate RST means the host is reachable and the service is absent;
  a timeout means a packet filter. Probing from inside the controller
  over loopback then separates daemon state from firewall state.
- **Rule**: Always include a known-closed control port when diagnosing
  an unreachable service. (from ToDo#1 firewall diagnosis)

**Gripper activation order**
- **Problem**: `gripper_activate()` failed with `FLT 7`.
- **Cause**: `GTO 1` requests motion, and it was sent before the
  activation sequence reported `STA 3`.
- **Fix**: Wait for `STA 3` first, then set `GTO`. Treat fault codes 5
  and 7 as expected while activation runs.
- **Rule**: Never request motion from a Robotiq gripper until
  activation reports finished. (from ToDo#1 V3)

**Timing a device event from the wrong instant**
- **Problem**: The `"start"` trigger looked like it missed its 50 ms
  budget by four times over.
- **Cause**: The clock started at entry into the call, which included
  the one-off RTDE control script upload of about 200 ms. The robot is
  not moving for any of that.
- **Fix**: Start the clock where the motion is actually dispatched, and
  offer `acquire_motion()` so the upload can be paid during setup.
- **Rule**: Always measure a latency budget from the event it is
  defined against, and separate one-off setup from steady-state cost.
  (from ToDo#3 V5)

---

## 3. Library Quirks

- **ur_rtde output registers**: only 12 to 19 are exposed to clients.
  `getOutputIntRegister(0)` raises `ValueError`. Always use 12 to 19
  for URScript-to-Python handshakes. (from ToDo#1 URCap diagnosis)
- **ur_rtde receive interface**: the constructor can return an object
  that reports itself disconnected instead of raising. Always check
  `isConnected()` after constructing and retry. (from ToDo#1 V1)
- **URScript string literals do not interpret escapes**: `"GET STA\n"`
  sends a literal backslash and n, so the peer never sees a terminated
  line. Always use `socket_send_line`. (from ToDo#1 loopback probe)
- **URScript `socket_read_string`** cannot distinguish "nothing has
  arrived yet" from an empty reply. Prefer `socket_read_byte_list`,
  which reports the byte count and survives binary payloads.
  (from ToDo#1 loopback probe)
- **URCap `rq_*` script functions** are injected by the installation
  node into programs built on the pendant, so they do not exist in a
  script sent to port 30002. Never plan on calling them from an
  externally supplied script. (from ToDo#1 URCap script API probe)
- **Ruff versus the style guide**: CLAUDE.md section 2 asks for a space
  around `=`, but Ruff's E251 forbids it around keyword arguments.
  Ruff is machine-checked and gated by a hook, so it wins.
  (from ToDo#1 implementation)

---

## 4. Workflow Lessons

- **Verify the environment before believing the specification**: this
  cell differed from the specification in three ways at once --
  PolyScope generation, External Control URCap presence, and a usable
  RTDE rate. Always run a read-only connection probe first and let it
  set the constructor arguments. (from ToDo#1 V1)
- **A blocked path is worth diagnosing to the root**: three cheap
  read-only probes turned "the URCaps seem missing" into an exact
  firewall setting and a one-line fix, and validated both protocols
  before the port was even open. (from ToDo#1 firewall diagnosis)
- **A blocked interface hides bugs behind it**: both gripper bugs were
  only reachable once port 63352 opened. Never read "cannot connect" as
  evidence that the code behind the connection is correct.
  (from ToDo#1 V3)

---

## 5. Environment Specifics

- **PolyScope closes eth0 ports by default from 5.14 onward**, leaving
  loopback untouched. Open ports on the pendant under Settings ->
  Security -> General with an Admin password; the field lists ports to
  **block**, so `1-4241,4243-63351,63353-65535` is what leaves 4242 and
  63352 reachable. (from ToDo#1 firewall diagnosis)
- **This controller drops the RTDE stream after about 1.5 s at 500 Hz**
  and is stable at 125 Hz, even though the specification allows 500.
  (from ToDo#1 V1)
- **e-Series refuses externally supplied scripts in local control.**
  The pendant must be in Remote Control before any RTDE motion; the
  Dashboard command `is in remote control` reports this and makes the
  failure diagnosable. (from ToDo#1 V2)
- **Hand-E end stops are POS 3 and 249**, not 0 and 255. Position
  checks need a tolerance rather than exact equality. (from ToDo#1 V3)
- **UR ships no official Python library beyond RTDE.** Dashboard,
  gripper and camera protocols have to be implemented directly; the
  library that spans RTDE plus Dashboard plus Interpreter is C++.
  (from ToDo#1 reference research)
