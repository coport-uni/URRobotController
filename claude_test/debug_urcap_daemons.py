# Ask the robot itself whether the URCap daemons are listening.
#
# Purpose: from the outside, ports 63352 and 4242 time out. That is
# ambiguous -- it could be a controller firewall dropping the SYN, or
# the URCap daemon simply not running. A URScript executed on the
# controller connects to 127.0.0.1 instead, which distinguishes the
# two. The script issues no move command.
#
# Port 29999 is probed as a control: it is known to be listening, so if
# it comes back refused the probe method itself is broken.
#
# Registers 12 to 14 carry the three results and register 15 carries a
# unique run id written LAST. Polling for that id is what makes the
# reading trustworthy -- an earlier version of this script polled the
# result registers directly and read leftover values from a previous
# run, which produced a confidently wrong "daemon not running" verdict.
#
# Expected lifetime: delete once the firewall situation is resolved.
import socket
import sys
import time

from cell_settings import default_robot_ip
from rtde_receive import RTDEReceiveInterface

robot_ip = sys.argv[1] if len(sys.argv) > 1 else default_robot_ip
secondary_port = 30002
rtde_frequency = 125.0
probe_timeout = 20.0

# Any value works as long as it differs from the previous run.
run_id = int(sys.argv[2]) if len(sys.argv) > 2 else int(time.time()) % 10000

probes = {
    12: ("29999 Dashboard (CONTROL)", 29999),
    13: ("63352 Robotiq gripper", 63352),
    14: ("4242 Wrist camera", 4242),
}


def build_script():
    lines = ["def check_urcap_daemons():"]
    for index, (_, port) in probes.items():
        name = f"s{port}"
        lines += [
            f'  ok = socket_open("127.0.0.1", {port}, "{name}")',
            "  if ok:",
            f"    write_output_integer_register({index}, 1)",
            f'    socket_close("{name}")',
            "  else:",
            f"    write_output_integer_register({index}, 0)",
            "  end",
        ]
    lines += [
        f"  write_output_integer_register(15, {run_id})",
        "end",
        "",
    ]
    return "\n".join(lines)


variables = ["timestamp"] + [
    f"output_int_register_{index}" for index in list(probes) + [15]
]

print(f"=== URCap daemon probe from inside {robot_ip} ===")
print(f"run id {run_id}\n")

receive = RTDEReceiveInterface(robot_ip, rtde_frequency, variables)
print(f"rtde connected : {receive.isConnected()}")

print("sending probe script to the secondary interface ...")
with socket.create_connection((robot_ip, secondary_port), timeout=5.0) as sock:
    sock.sendall(build_script().encode())

deadline = time.monotonic() + probe_timeout
completed = False
while time.monotonic() < deadline:
    if receive.getOutputIntRegister(15) == run_id:
        completed = True
        break
    time.sleep(0.2)

print(f"probe completed : {completed}\n")
if not completed:
    print("The script never finished; the results below are stale.")

meanings = {0: "REFUSED (nothing listening)", 1: "OPEN (daemon alive)"}
for index, (label, _) in probes.items():
    value = receive.getOutputIntRegister(index)
    print(f"  {label:<28} -> {meanings.get(value, f'unknown ({value})')}")

receive.disconnect()

print(
    "\nRead this together with an external port probe:"
    "\n  inside OPEN  + outside TIMEOUT -> controller firewall drops it"
    "\n  inside OPEN  + outside REFUSED -> daemon bound to loopback only"
    "\n  inside REFUSED               -> the URCap daemon is not running"
)
