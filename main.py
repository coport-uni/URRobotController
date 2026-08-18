"""Demonstration scenarios driven by :class:`URRobotController`.

Four scenarios exercise the four interfaces of the work cell, each one
relative to wherever the robot happens to be standing when it starts,
so the demo needs no taught positions:

1. Swing joint 1 by plus and minus 20 degrees about its current angle.
2. Shift the TCP by plus and minus 20 mm along the base Z axis.
3. Open the gripper, hold for three seconds, then close it.
4. Record three seconds of wrist camera frames to a video file.

Every scenario returns the robot to where it started.

Run all four, or name the ones you want::

    python main.py --ip 192.168.1.238
    python main.py joints tcp
    python main.py camera --video demo.mp4
"""

import argparse
import math
import time

import cv2

from URRobotController import URRobotController, WorkCellError

# -- scenario parameters ----------------------------------------------

joint_travel_deg = 20.0
joint_index = 0  # Joint 1 is the base joint.
joint_speed = 0.3  # rad/s, deliberately slow for a demonstration.
joint_accel = 0.6  # rad/s^2

tcp_travel_m = 0.020
tcp_axis = 2  # Index 2 of the pose vector is the base Z axis.
tcp_speed = 0.05  # m/s
tcp_accel = 0.3  # m/s^2

gripper_hold_seconds = 3.0
gripper_speed = 100
gripper_force = 50

video_seconds = 3.0
video_poll_interval = 0.1  # The URCap serves single frames, not a stream.
default_video_path = "wrist_camera_demo.mp4"
video_fourcc = "mp4v"
fallback_fps = 10.0

pause_between_moves = 0.5

# Settings measured on the bench robot. The controller runs PolyScope
# 5.25, has no External Control URCap, and closes the RTDE stream when
# asked for 500 Hz, so these differ from the specification defaults.
default_robot_ip = "192.168.1.238"
default_cell_kwargs = {
    "polyscope_x": False,
    "use_ext_urcap": False,
    "rtde_frequency": 125.0,
}


def describe_joints(joints: list[float]) -> str:
    """Format a joint vector in degrees for the console.

    Returns:
        The six angles in degrees, comma separated.
    """
    return ", ".join(f"{math.degrees(value):+8.3f}" for value in joints)


def describe_pose(pose: list[float]) -> str:
    """Format a TCP pose for the console.

    Returns:
        Position in millimetres and rotation vector in radians.
    """
    position = ", ".join(f"{value * 1000.0:+8.2f}" for value in pose[:3])
    rotation = ", ".join(f"{value:+6.3f}" for value in pose[3:])
    return f"[{position}] mm  [{rotation}] rad"


def demo_joint_swing(robot: URRobotController) -> None:
    """Swing joint 1 by plus and minus 20 degrees, then return.

    The motion is relative to the pose the robot is already holding,
    so the demonstration is safe to start from any reachable posture
    that leaves room for the swing.

    Args:
        robot: A connected controller.

    Raises:
        WorkCellError: If a move is refused or the robot stops.
    """
    print(
        f"\n=== 1. joint 1 swing, plus and minus {joint_travel_deg:.0f} deg ==="
    )
    home = robot.joints()
    travel = math.radians(joint_travel_deg)
    print(f"  start   : {describe_joints(home)}")

    for label, offset in (
        ("positive", +travel),
        ("back", 0.0),
        ("negative", -travel),
        ("back", 0.0),
    ):
        target = list(home)
        target[joint_index] = home[joint_index] + offset
        result = robot.move_j(target, speed=joint_speed, accel=joint_accel)
        reached = robot.joints()
        moved = math.degrees(reached[joint_index] - home[joint_index])
        print(
            f"  {label:<8}: {describe_joints(reached)}"
            f"  offset {moved:+7.3f} deg  {result.elapsed:5.2f} s"
        )
        time.sleep(pause_between_moves)

    error = abs(robot.joints()[joint_index] - home[joint_index])
    print(f"  returned to start within {error:.6f} rad")


def demo_tcp_z_shift(robot: URRobotController) -> None:
    """Shift the TCP by plus and minus 20 mm along Z, then return.

    Uses straight-line moves so the tool keeps its orientation, which
    is what makes the vertical travel readable on the bench.

    Args:
        robot: A connected controller.

    Raises:
        WorkCellError: If a move is refused or the robot stops.
    """
    print(
        "\n=== 2. TCP Z shift, plus and minus "
        f"{tcp_travel_m * 1000.0:.0f} mm ==="
    )
    home = robot.tcp_pose()
    print(f"  start   : {describe_pose(home)}")

    for label, offset in (
        ("up", +tcp_travel_m),
        ("back", 0.0),
        ("down", -tcp_travel_m),
        ("back", 0.0),
    ):
        target = list(home)
        target[tcp_axis] = home[tcp_axis] + offset
        result = robot.move_l(target, speed=tcp_speed, accel=tcp_accel)
        reached = robot.tcp_pose()
        moved = (reached[tcp_axis] - home[tcp_axis]) * 1000.0
        print(
            f"  {label:<8}: {describe_pose(reached)}"
            f"  offset {moved:+7.2f} mm  {result.elapsed:5.2f} s"
        )
        time.sleep(pause_between_moves)

    error = abs(robot.tcp_pose()[tcp_axis] - home[tcp_axis]) * 1000.0
    print(f"  returned to start within {error:.4f} mm")


def demo_gripper_cycle(robot: URRobotController) -> None:
    """Open the gripper, hold three seconds, then close it.

    Args:
        robot: A connected controller.

    Raises:
        WorkCellError: If the gripper is unreachable or faulted.
    """
    print(
        f"\n=== 3. gripper open, hold {gripper_hold_seconds:.0f} s, close ==="
    )
    robot.gripper_activate()
    print(f"  activated, position {robot.gripper_position()}")

    position, status = robot.gripper_open(
        speed=gripper_speed, force=gripper_force
    )
    print(f"  opened  : POS {position:3d}  OBJ {status}")

    print(f"  holding open for {gripper_hold_seconds:.0f} s ...")
    time.sleep(gripper_hold_seconds)

    position, status = robot.gripper_close(
        speed=gripper_speed, force=gripper_force
    )
    print(f"  closed  : POS {position:3d}  OBJ {status}")
    if robot.gripper_object_detected():
        print("  an object is held between the fingers")


def demo_camera_clip(robot: URRobotController, path: str) -> None:
    """Record three seconds of wrist camera frames to a video file.

    The URCap serves one still image per request rather than a stream,
    so frames are polled and the real achieved rate is measured and
    used as the playback rate. That keeps the clip the same length as
    the recording.

    Args:
        robot: A connected controller.
        path: Destination video file path.

    Raises:
        CameraError: If the camera endpoint did not answer.
        RuntimeError: If no frame could be captured.
    """
    print(f"\n=== 4. record {video_seconds:.0f} s of wrist camera ===")
    frames = []
    started = time.monotonic()
    while time.monotonic() - started < video_seconds:
        frames.append(robot.camera_frame("color"))
        time.sleep(video_poll_interval)
    elapsed = time.monotonic() - started

    if not frames:
        raise RuntimeError("no frames were captured")

    fps = len(frames) / elapsed if elapsed > 0 else fallback_fps
    height, width = frames[0].shape[:2]
    writer = cv2.VideoWriter(
        path,
        cv2.VideoWriter_fourcc(*video_fourcc),
        fps,
        (width, height),
    )
    try:
        for frame in frames:
            writer.write(frame)
    finally:
        writer.release()

    print(f"  captured {len(frames)} frames in {elapsed:.2f} s")
    print(f"  resolution {width} x {height}, {fps:.1f} fps")
    print(f"  saved to {path}")


scenarios = {
    "joints": demo_joint_swing,
    "tcp": demo_tcp_z_shift,
    "gripper": demo_gripper_cycle,
    "camera": demo_camera_clip,
}

moving_scenarios = ("joints", "tcp", "gripper")


def parse_arguments() -> argparse.Namespace:
    """Read the command line.

    Returns:
        The parsed arguments.
    """
    parser = argparse.ArgumentParser(
        description="URRobotController demonstration scenarios"
    )
    parser.add_argument(
        "scenario",
        nargs="*",
        choices=list(scenarios) + [],
        help="scenarios to run; all four when omitted",
    )
    parser.add_argument(
        "--ip", default=default_robot_ip, help="robot IP address"
    )
    parser.add_argument(
        "--video",
        default=default_video_path,
        help="destination for the camera clip",
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="skip the confirmation prompt before moving the robot",
    )
    return parser.parse_args()


def confirm_motion(selected: list[str]) -> bool:
    """Ask the operator before anything on the bench moves.

    The robot and the gripper move under their own power here, so the
    operator gets the final say and a chance to reach the e-stop.

    Args:
        selected: Names of the scenarios about to run.

    Returns:
        True when the operator agreed, or when nothing moves.
    """
    moving = [name for name in selected if name in moving_scenarios]
    if not moving:
        return True
    print(f"About to run: {', '.join(selected)}")
    print(f"These move the hardware: {', '.join(moving)}")
    print("Clear the work area and keep the e-stop within reach.")
    return input("Continue? [y/N] ").strip().lower() in ("y", "yes")


def main() -> int:
    """Run the selected demonstration scenarios.

    Returns:
        Process exit status, 0 on success.
    """
    arguments = parse_arguments()
    selected = arguments.scenario or list(scenarios)

    if not arguments.yes and not confirm_motion(selected):
        print("Cancelled.")
        return 1

    with URRobotController(arguments.ip, **default_cell_kwargs) as robot:
        print(f"connected to {arguments.ip}")
        print(f"  report : {robot.get_connection_report()}")
        print(f"  mode   : {robot.mode()}")

        failures = []
        for name in selected:
            try:
                if name == "camera":
                    scenarios[name](robot, arguments.video)
                else:
                    scenarios[name](robot)
            except (WorkCellError, RuntimeError) as exc:
                # One unavailable interface should not cost the
                # operator the scenarios that do work.
                print(f"  FAILED: {type(exc).__name__}: {exc}")
                failures.append(name)

        robot.release_motion()

    print()
    if failures:
        print(f"finished with failures: {', '.join(failures)}")
        return 1
    print("all scenarios finished")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
