# Discover which URCap URScript functions the controller actually has.
#
# Purpose: port 63352 is refused even from the controller itself, yet
# the operator reports the gripper works from URScript. That points at
# a URCap generation that exposes rq_* script functions instead of the
# socket daemon. A URScript program is compiled as a whole before any
# line runs, so referencing a function inside "if False" proves the
# function exists WITHOUT executing it -- nothing moves.
#
# Register 12 carries the probe id. If it comes back as the id we sent,
# the program compiled, so every name it referenced exists.
#
# Expected lifetime: delete once the gripper and camera paths are done.
import socket
import sys
import time

from cell_settings import default_robot_ip
from rtde_receive import RTDEReceiveInterface

robot_ip = sys.argv[1] if len(sys.argv) > 1 else default_robot_ip
secondary_port = 30002
rtde_frequency = 125.0
settle_seconds = 1.5

# Each candidate is a group of names that a single URCap generation
# would provide together.
candidates = [
    ("rq_activate", ["rq_activate()"]),
    ("rq_activate_and_wait", ["rq_activate_and_wait()"]),
    ("rq_open / rq_close", ["rq_open()", "rq_close()"]),
    ("rq_open_and_wait", ["rq_open_and_wait()"]),
    ("rq_move", ["rq_move(128)"]),
    ("rq_move_and_wait", ["rq_move_and_wait(128)"]),
    ("rq_move_norm", ["rq_move_norm(50.0)"]),
    ("rq_set_pos", ["rq_set_pos(128)"]),
    ("rq_set_speed_norm", ["rq_set_speed_norm(50.0)"]),
    ("rq_set_force_norm", ["rq_set_force_norm(50.0)"]),
    ("rq_current_pos", ["rq_current_pos()"]),
    ("rq_is_object_detected", ["rq_is_object_detected()"]),
    ("rq_is_gripper_activated", ["rq_is_gripper_activated()"]),
    ("rq_gripper_act (variable)", ["rq_gripper_act"]),
]


def build_probe(probe_id, references):
    body = "\n".join(f"    {reference}" for reference in references)
    return (
        f"def probe_{probe_id}():\n"
        f"  write_output_integer_register(12, {probe_id})\n"
        f"  if False:\n"
        f"{body}\n"
        f"  end\n"
        f"end\n"
    )


print(f"=== URScript API probe on {robot_ip} ===")
print("Nothing is executed: every candidate sits inside 'if False'.\n")

receive = RTDEReceiveInterface(
    robot_ip, rtde_frequency, ["timestamp", "output_int_register_12"]
)
print(f"rtde connected : {receive.isConnected()}\n")

# A baseline probe with no candidate proves the transport works.
baseline_id = 100
with socket.create_connection((robot_ip, secondary_port), timeout=5.0) as sock:
    sock.sendall(
        f"def probe_baseline():\n"
        f"  write_output_integer_register(12, {baseline_id})\n"
        f"end\n".encode()
    )
time.sleep(settle_seconds)
baseline_ok = receive.getOutputIntRegister(12) == baseline_id
print(f"baseline (no URCap call) -> {'OK' if baseline_ok else 'NO RESPONSE'}")
if not baseline_ok:
    print("The controller is not running scripts; the rest is meaningless.")

print()
available = []
for index, (label, references) in enumerate(candidates):
    probe_id = 200 + index
    script = build_probe(probe_id, references)
    with socket.create_connection(
        (robot_ip, secondary_port), timeout=5.0
    ) as sock:
        sock.sendall(script.encode())
    time.sleep(settle_seconds)
    exists = receive.getOutputIntRegister(12) == probe_id
    print(f"  {label:<28} -> {'EXISTS' if exists else 'not defined'}")
    if exists:
        available.append(label)

receive.disconnect()

print(f"\navailable URCap script functions: {available or 'none'}")
