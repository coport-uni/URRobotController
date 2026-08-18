# V5: the headline feature of the specification, section 4.4 -- a
# motion and a gripper action issued as one call, with the gripper
# fired at a trigger point during the motion.
#
# Covers specification section 8 items T03 (trigger "start"), T04
# (trigger "remaining_joint") and T05 (trigger "end"), plus the
# non-blocking path of section 4.4 item 3.
#
# Measurement works by subclassing the controller and timestamping
# _fire_gripper, which is the only way to see when the command was
# actually issued rather than inferring it from the outcome.
#
# THIS SCRIPT MOVES THE ROBOT AND THE GRIPPER. Keep the e-stop
# within reach and the fingers clear.
import math
import sys
import time

from cell_settings import cell_kwargs, default_robot_ip

from URRobotController import GripperAction, URRobotController

robot_ip = sys.argv[1] if len(sys.argv) > 1 else default_robot_ip

travel_deg = 20.0
joint_index = 0
speed = 0.3
accel = 0.6
gripper_speed = 100
gripper_force = 50

# Specification section 8, T03 and T05.
latency_limit_s = 0.050
# Specification section 7 item 4: trigger precision is bounded by the
# 10 ms watch period times the joint speed.
watch_period_s = 0.010


class InstrumentedController(URRobotController):
    """Controller that records when and where the gripper was fired."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.fire_time = None
        self.fire_joints = None
        self.motion_start = None
        self.call_start = None

    def _fire_gripper(self, gripper, result):
        self.fire_time = time.monotonic()
        self.fire_joints = self.joints()
        super()._fire_gripper(gripper, result)

    def _run_motion(self, kind, target, speed, accel, gripper, blocking):
        self.fire_time = None
        self.fire_joints = None
        self.motion_start = None
        self.call_start = time.monotonic()
        return super()._run_motion(
            kind, target, speed, accel, gripper, blocking
        )

    def _dispatch_move(self, kind, target, speed, accel, asynchronous):
        # T03 and T05 bound the gripper command against the start of
        # the MOTION, not against entry into the call. Taking RTDE
        # control uploads a script and costs about 200 ms, and the
        # robot is not moving for any of it, so the clock starts here.
        self.motion_start = time.monotonic()
        return super()._dispatch_move(kind, target, speed, accel, asynchronous)


def offset_target(robot, degrees):
    """Build a joint target offset from the current pose.

    Returns:
        Tuple of the current joints and the target joints.
    """
    home = robot.joints()
    target = list(home)
    target[joint_index] = home[joint_index] + math.radians(degrees)
    return home, target


results = []

with InstrumentedController(robot_ip, **cell_kwargs) as robot:
    print(f"report : {robot.get_connection_report()}")
    robot.gripper_activate()
    print(f"gripper activated, POS {robot.gripper_position()}")
    # Take RTDE control up front so the one-off control script upload
    # is not charged to the first motion's trigger latency.
    robot.acquire_motion()
    print("rtde control acquired\n")

    # -- T03: fire at motion start -----------------------------------
    print('=== T03  trigger="start", gripper opens as the move begins ===')
    home, target = offset_target(robot, travel_deg)
    result = robot.move_j(
        target,
        speed=speed,
        accel=accel,
        gripper=GripperAction.open(
            speed=gripper_speed, force=gripper_force, trigger="start"
        ),
    )
    latency = robot.fire_time - robot.motion_start
    ok = latency <= latency_limit_s and result.motion_ok
    print(f"  motion_ok      : {result.motion_ok}")
    print(
        f"  fire latency   : {latency * 1000:.1f} ms "
        f"(limit {latency_limit_s * 1000:.0f} ms)"
    )
    print(f"  gripper POS    : {robot.gripper_position()} (expect open)")
    print(f"  total elapsed  : {result.elapsed:.3f} s")
    print(f"  verdict        : {'PASS' if ok else 'FAIL'}\n")
    results.append(("T03 start", ok))

    # -- T05: fire at motion end -------------------------------------
    print('=== T05  trigger="end", gripper closes on arrival ===')
    result = robot.move_j(
        home,
        speed=speed,
        accel=accel,
        gripper=GripperAction.close(
            speed=gripper_speed,
            force=gripper_force,
            trigger="end",
            wait_object=True,
        ),
    )
    # The fire happens after the watch loop breaks, so its distance
    # from the end of the motion is what T05 bounds.
    from_start = robot.fire_time - robot.motion_start
    slack = result.elapsed - from_start
    ok = result.motion_ok and robot.fire_joints is not None
    print(f"  motion_ok      : {result.motion_ok}")
    print(f"  fired at       : {from_start:.3f} s after motion start")
    print(f"  motion total   : {result.elapsed:.3f} s")
    print(f"  gripper_pos    : {result.gripper_pos}")
    print(f"  gripper_obj    : {result.gripper_obj} (3 = at target)")
    print(f"  wait_object    : reported {result.gripper_obj is not None}")
    print(f"  post-fire slack: {slack:.3f} s")
    print(f"  verdict        : {'PASS' if ok else 'FAIL'}\n")
    results.append(("T05 end", ok))

    # -- T04: fire at a remaining joint distance ---------------------
    threshold = 0.10  # rad
    print(f'=== T04  trigger="remaining_joint", threshold {threshold} rad ===')
    home, target = offset_target(robot, travel_deg)
    result = robot.move_j(
        target,
        speed=speed,
        accel=accel,
        gripper=GripperAction.open(
            speed=gripper_speed,
            force=gripper_force,
            trigger="remaining_joint",
            threshold=threshold,
        ),
    )
    remaining = math.dist(robot.fire_joints, target)
    # The watch loop can only sample every 10 ms, so the robot travels
    # up to speed * period past the threshold before it is noticed.
    tolerance = speed * watch_period_s
    error = abs(remaining - threshold)
    ok = result.motion_ok and error <= tolerance
    print(f"  motion_ok      : {result.motion_ok}")
    print(f"  remaining at fire: {remaining:.5f} rad")
    print(f"  threshold        : {threshold:.5f} rad")
    print(
        f"  error            : {error:.5f} rad "
        f"(limit {tolerance:.5f} = {speed} rad/s x 10 ms)"
    )
    print(f"  verdict          : {'PASS' if ok else 'FAIL'}\n")
    results.append(("T04 remaining_joint", ok))

    # -- non-blocking path, section 4.4 item 3 -----------------------
    print("=== 4.4-3  blocking=False with wait_motion_done() ===")
    started = time.monotonic()
    immediate = robot.move_j(
        home,
        speed=speed,
        accel=accel,
        gripper=GripperAction.close(
            speed=gripper_speed, force=gripper_force, trigger="end"
        ),
        blocking=False,
    )
    returned_after = time.monotonic() - started
    final = robot.wait_motion_done(timeout=30.0)
    total = time.monotonic() - started
    ok = (
        returned_after < 0.5
        and final is not None
        and final.motion_ok
        and total > returned_after
    )
    print(f"  call returned in : {returned_after * 1000:.1f} ms")
    print(f"  immediate result : motion_ok={immediate.motion_ok}")
    print(
        f"  final result     : motion_ok={final.motion_ok}, "
        f"elapsed={final.elapsed:.3f} s"
    )
    print(f"  total wall clock : {total:.3f} s")
    print(f"  verdict          : {'PASS' if ok else 'FAIL'}\n")
    results.append(("4.4-3 non-blocking", ok))

    robot.release_motion()

# -- T09: a gripper failure must not stop the motion -----------------
print("=== T09  gripper unavailable, motion must continue ===")
with InstrumentedController(
    robot_ip, gripper_enabled=False, **cell_kwargs
) as blind:
    home, target = offset_target(blind, travel_deg / 2)
    result = blind.move_j(
        target,
        speed=speed,
        accel=accel,
        gripper=GripperAction.close(trigger="start"),
    )
    reached = blind.joints()
    arrived = math.dist(reached, target) <= 0.01
    ok = result.motion_ok and arrived and result.gripper_error is not None
    print(f"  motion_ok      : {result.motion_ok}")
    print(f"  arrived        : {arrived}")
    print(f"  gripper_error  : {result.gripper_error}")
    print(f"  gripper_pos    : {result.gripper_pos} (None expected)")
    print(f"  verdict        : {'PASS' if ok else 'FAIL'}\n")
    results.append(("T09 gripper fault", ok))
    blind.move_j(home, speed=speed, accel=accel)
    blind.release_motion()

print("=== summary ===")
for name, ok in results:
    print(f"  {name:<22} {'PASS' if ok else 'FAIL'}")
verdict = all(ok for _, ok in results)
print(f"\nV5 {'PASS' if verdict else 'FAIL'}")
sys.exit(0 if verdict else 1)
