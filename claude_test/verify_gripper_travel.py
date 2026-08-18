# V3: drive the Hand-E gripper from 0 % to 100 % and back to 0 %.
# THIS SCRIPT MOVES THE GRIPPER FINGERS. Keep hands clear.
import sys
import time

from cell_settings import cell_kwargs, default_robot_ip

from URRobotController import (
    URRobotController,
    gripper_closed_position,
    gripper_open_position,
)

robot_ip = sys.argv[1] if len(sys.argv) > 1 else default_robot_ip

# Slow and gentle for a first run on real hardware.
speed = 100
force = 50

position_tolerance = 10  # counts, out of the 0 - 255 range


def to_counts(percent):
    span = gripper_closed_position - gripper_open_position
    return round(gripper_open_position + span * percent / 100.0)


def to_percent(counts):
    span = gripper_closed_position - gripper_open_position
    return 100.0 * (counts - gripper_open_position) / span


with URRobotController(robot_ip, **cell_kwargs) as cell:
    print(f"report         : {cell.get_connection_report()}")

    print("\n--- activate ---")
    cell.gripper_activate()
    start_counts = cell.gripper_position()
    print(
        f"  position       : {start_counts} ({to_percent(start_counts):.1f} %)"
    )

    results = []
    for percent in (0.0, 100.0, 0.0):
        target = to_counts(percent)
        print(f"\n--- move to {percent:.0f} % (POS {target}) ---")
        started = time.monotonic()
        position, obj = cell.gripper_move(target, speed=speed, force=force)
        elapsed = time.monotonic() - started
        error = abs(position - target)
        ok = error <= position_tolerance
        print(f"  final POS      : {position} ({to_percent(position):.1f} %)")
        print(f"  OBJ status     : {obj}")
        print(f"  error          : {error} counts (limit {position_tolerance})")
        print(f"  elapsed        : {elapsed:.3f} s")
        print(f"  verdict        : {'PASS' if ok else 'FAIL'}")
        results.append(ok)

    verdict = "PASS" if all(results) else "FAIL"
    print(f"\nV3 {verdict}")
    sys.exit(0 if verdict == "PASS" else 1)
