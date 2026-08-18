# ToDo

Cumulative command history for Claude Code sessions in this repository.
Append new tasks below; never overwrite or reorder existing entries.

---

## Task 1 — Bootstrap conventions and implement URWorkCell (2026-08-13)

**Request**: Add `coport-uni/CommonClaude` as a submodule under `external/`,
adopt its `CLAUDE.md` and hooks in this repository, then implement the UR
robot control class per `docs/URWorkCell_개발사양서.md` and verify it on the
real robot at `192.168.1.238`. Virtual/simulated testing is out of scope.

**Reference material reviewed**:
- `docs/URWorkCell_개발사양서.md` (v1.0, 2026-08-13)
- `external/CommonClaude/CLAUDE.md`

### Conventions bootstrap
- [x] Add `external/CommonClaude` as a git submodule
- [x] Copy `CLAUDE.md` to the repository root
- [x] Copy `.claude/settings.json` and `.claude/hooks/*.sh`
- [x] Add `.gitignore` (CLAUDE.md §13) and `pyproject.toml` with Ruff
      `line-length = 80` (§6)
- [x] Create GitHub issue via `gh issue create`
- [x] Cut working branch `feature/ur-workcell`

### URWorkCell implementation (spec §3 — §8)
- [x] `ur_workcell/errors.py` — `WorkCellError` hierarchy, codes E100 — E600
      (spec §6)
- [x] `ur_workcell/types.py` — `GripperAction`, `MotionResult`, `Mode`
      (spec §4.2, §4.3)
- [x] `ur_workcell/_gripper.py` — Robotiq socket client, port 63352
- [x] `ur_workcell/_camera.py` — Wrist Camera HTTP client, port 4242
- [x] `ur_workcell/_program.py` — Dashboard 29999 + Secondary 30002
- [x] `ur_workcell/_motion.py` — `ur_rtde` control/receive wrapper
- [x] `ur_workcell/workcell.py` — `URWorkCell` public API, mode state
      machine, trigger watch loop (spec §4.4)

### Hardware verification on 192.168.1.238 (spec §8, CLAUDE.md §5.1)
- [x] V1 — Connect all interfaces, print connection report
- [x] V2 — Move joint 1 by +20 deg and return, verify arrival error
      (PASS, arrival error 0.000121 rad against the 0.001 rad limit)
- [x] V3 — Gripper 0 % -> 100 % -> 0 % travel
      (PASS 2026-08-18 after the firewall was opened: POS 3 -> 249 -> 3,
      OBJ 3 at every stop, about 1.05 s per stroke)
- [x] V4 — Wrist Camera frame acquisition over URCap HTTP
      (PASS 2026-08-18: all four image types decoded at 640x480 BGR)
- [x] Identify the real cause of the two blocked ports. An
      on-controller URScript probe with a 29999 control
      (`claude_test/debug_urcap_daemons.py`) shows both ports OPEN from
      inside while they time out from outside, so the **controller
      firewall** drops external traffic to them. A first version of
      this probe read stale RTDE registers and wrongly concluded the
      URCaps were not installed; the operator's correction prompted
      the re-test.
- [x] Rule out driving the gripper by URCap `rq_*` URScript functions
      instead (`claude_test/debug_urcap_script_api.py`): none of the 14
      candidates exist in an externally supplied script, because the
      URCap injects them only into pendant-built programs.
- [x] Record the real console output for the PR `## Testing` section

### Consolidation into one class (follow-up request, 2026-08-13)

**Request**: The features are spread across a package; put them all into
a single god-class file named `URRobotController`.

- [x] Merge the seven `ur_workcell/*.py` modules into
      `URRobotController.py`, with the four internal modules dissolved
      into methods of one `URRobotController` class
- [x] Keep the exception hierarchy and the `GripperAction`,
      `MotionResult`, `Mode` data types as module-level definitions in
      the same file
- [x] Delete the `ur_workcell/` package and repoint `claude_test/`,
      `pyproject.toml` and `claude_test/README.md`
- [x] Re-run V1 against the real robot to confirm no behaviour changed

### Demo scenarios in main.py (follow-up request, 2026-08-13)

**Request**: Once the tests are complete, write demo scenarios in
`main.py` on top of the god class, covering four behaviours.

- [x] Scenario 1 — swing joint 1 by plus and minus 20 deg about its
      current angle (PASS on hardware: +20.007 / -20.004 deg, returned
      to start within 0.000012 rad)
- [x] Scenario 2 — shift the TCP by plus and minus 20 mm along base Z
      using `move_l` (PASS on hardware: +19.83 / -19.92 mm, returned to
      start within 0.0221 mm)
- [x] Scenario 3 — open the gripper, hold 3 s, close
      (PASS 2026-08-18)
- [x] Scenario 4 — record 3 s of wrist camera frames to a video file
      (PASS 2026-08-18: 21 frames over 3.10 s at 640x480, written at the
      measured 6.8 fps so the clip length matches the recording)
- [x] Operator confirmation prompt before any scenario that moves the
      hardware, with `--yes` to skip it (CLAUDE.md §5.1 rule 5)

### Wrap-up
- [x] Ruff check and format clean on all Python files
- [x] Update the GitHub issue, push branch, open PR
- [x] Create `LearnedPatterns.md` from completed items (CLAUDE.md §10)

---

## Task 2 — User-facing verification setup (2026-08-18)

**Request**: Make each feature verifiable by the user. Build a conda
environment around `main.py` and write up how to test each feature.

- [x] Add a `status` scenario that reports every interface, the safety
      state and Remote Control without moving anything, so the user has
      a safe first step
- [x] Add `environment.yml` for a conda environment `ur-workcell`,
      with OpenCV from conda-forge for its GUI backend and `ur_rtde`
      via pip
- [x] Create the environment and verify it drives the real robot
      end to end, not just that it imports
- [x] Rewrite `README.md` as a verification guide: setup, the three
      robot prerequisites, one section per feature with the expected
      output, and the error code table
- [x] Fix the camera stalling under back-to-back requests, found while
      checking the documented commands actually run: one retry in the
      fetch and per-frame tolerance in the recording scenario
