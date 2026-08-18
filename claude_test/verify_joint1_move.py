# V2: move joint 1 by +20 deg and back, measuring the arrival error.
# THIS SCRIPT MOVES THE ROBOT. Keep the e-stop within reach.
import math
import sys
import time

from cell_settings import cell_kwargs, default_robot_ip

from URRobotController import URRobotController

robot_ip = sys.argv[1] if len(sys.argv) > 1 else default_robot_ip

joint_index = 0  # Joint 1 is the base joint.
travel_deg = 20.0
travel_rad = math.radians(travel_deg)

# Deliberately slow: this is a first run on the real robot.
speed = 0.3  # rad/s
accel = 0.6  # rad/s^2

# Specification section 8, T02: joint arrival error must stay within
# this bound.
joint_tolerance_rad = 0.001


def fmt_deg(joints):
    return ", ".join(f"{math.degrees(v):+8.3f}" for v in joints)


def report_error(label, actual, target):
    errors = [abs(a - t) for a, t in zip(actual, target, strict=True)]
    worst = max(errors)
    ok = worst <= joint_tolerance_rad
    print(
        f"  {label} error : {worst:.6f} rad "
        f"(limit {joint_tolerance_rad}) {'PASS' if ok else 'FAIL'}"
    )
    return ok


with URRobotController(robot_ip, **cell_kwargs) as cell:
    print(f"polyscope      : {cell.polyscope_version()}")
    print(f"robot mode     : {cell.robot_mode()}")
    print(f"safety status  : {cell.safety_status()}")
    print(f"remote control : {cell.remote_control()}")
    print(f"operational    : {cell.operational_mode()}")
    print(f"report         : {cell.get_connection_report()}")

    home = cell.joints()
    target = list(home)
    target[joint_index] = home[joint_index] + travel_rad

    print(f"\nstart  [deg]   : {fmt_deg(home)}")
    print(f"target [deg]   : {fmt_deg(target)}")
    print(f"mode before    : {cell.mode()}")

    print(f"\n--- move joint 1 by {travel_deg:+.1f} deg ---")
    started = time.monotonic()
    forward = cell.move_j(target, speed=speed, accel=accel)
    reached = cell.joints()
    print(f"  motion_ok      : {forward.motion_ok}")
    print(f"  elapsed        : {forward.elapsed:.3f} s")
    print(f"  reached [deg]  : {fmt_deg(reached)}")
    forward_ok = report_error("outbound", reached, target)
    moved_deg = math.degrees(reached[joint_index] - home[joint_index])
    print(f"  joint 1 travel : {moved_deg:+.4f} deg")
    print(f"  mode during    : {cell.mode()}")

    time.sleep(0.5)

    print(f"\n--- return joint 1 by {-travel_deg:+.1f} deg ---")
    back = cell.move_j(home, speed=speed, accel=accel)
    returned = cell.joints()
    print(f"  motion_ok      : {back.motion_ok}")
    print(f"  elapsed        : {back.elapsed:.3f} s")
    print(f"  returned [deg] : {fmt_deg(returned)}")
    back_ok = report_error("return  ", returned, home)

    cell.release_motion()
    print(f"\nmode after     : {cell.mode()}")
    print(f"total elapsed  : {time.monotonic() - started:.3f} s")

    travel_ok = abs(abs(moved_deg) - travel_deg) <= math.degrees(
        joint_tolerance_rad
    )
    verdict = "PASS" if (forward_ok and back_ok and travel_ok) else "FAIL"
    print(f"\nV2 {verdict}")
    sys.exit(0 if verdict == "PASS" else 1)
