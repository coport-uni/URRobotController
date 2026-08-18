# Validate the gripper and camera protocols through the robot's own
# loopback interface, before the eth0 firewall is opened.
#
# Purpose: from PolyScope 5.14 UR closes eth0 ports by default but
# leaves loopback untouched, so a URScript running on the controller
# still reaches 127.0.0.1:63352 and 127.0.0.1:4242. That lets us
# confirm the Robotiq ASCII protocol and, more valuably, find which of
# the candidate camera URLs actually answers -- specification section 7
# item 2 flags that URL as unofficial and unverified.
#
# Results come back through RTDE output registers on the already-open
# port 30004. Issues no move command.
#
# Two URScript traps found the hard way, both recorded here:
#   * String literals do NOT interpret escapes, so "GET STA\n" sends a
#     literal backslash-n and the server never sees a terminated line.
#     socket_send_line appends a real newline instead.
#   * socket_read_string returns empty when nothing has arrived yet and
#     gives no way to tell that from a genuine empty reply.
#     socket_read_byte_list reports the byte count, so it can tell
#     "no answer" from "answered", and it survives binary payloads.
#
# Expected lifetime: delete once the firewall is opened and the normal
# external paths are verified.
import socket
import sys
import time

from cell_settings import default_robot_ip
from rtde_receive import RTDEReceiveInterface

robot_ip = sys.argv[1] if len(sys.argv) > 1 else default_robot_ip
run_id = 606
secondary_port = 30002

# Registers 12 to 19 are the range ur_rtde exposes to clients.
register_gripper_bytes = 12
register_gripper_status = 13
camera_registers = {
    14: "/current.jpg?type=color",
    15: "/current.jpg",
    16: "/camera/current.jpg?type=color",
}
register_done = 17

ascii_zero = 48
# "HTTP/1.0 200 OK": with byte_list index 0 holding the count, the
# three status digits land on indices 10, 11 and 12.
status_digit_indices = (10, 11, 12)
minimum_status_line = 13
gripper_status_index = 5  # The digit in "STA <n>".

lines = ["def loopback_bridge():"]
for register in [
    register_gripper_bytes,
    register_gripper_status,
    *camera_registers,
]:
    lines.append(f"  write_output_integer_register({register}, -1)")

lines += [
    '  if socket_open("127.0.0.1", 63352, "grip"):',
    '    socket_send_line("GET STA", "grip")',
    "    sleep(1.2)",
    '    reply = socket_read_byte_list(8, "grip")',
    f"    write_output_integer_register({register_gripper_bytes}, reply[0])",
    f"    if reply[0] >= {gripper_status_index + 1}:",
    f"      write_output_integer_register({register_gripper_status},"
    f" reply[{gripper_status_index}] - {ascii_zero})",
    "    else:",
    f"      write_output_integer_register({register_gripper_status}, -4)",
    "    end",
    '    socket_close("grip")',
    "  else:",
    f"    write_output_integer_register({register_gripper_bytes}, -2)",
    "  end",
]

for register, path in camera_registers.items():
    name = f"c{register}"
    digits = " + ".join(
        f"(head[{index}] - {ascii_zero}) * {factor}"
        for index, factor in zip(
            status_digit_indices, (100, 10, 1), strict=True
        )
    )
    lines += [
        f'  if socket_open("127.0.0.1", 4242, "{name}"):',
        f'    socket_send_line("GET {path} HTTP/1.0", "{name}")',
        f'    socket_send_line("", "{name}")',
        "    sleep(2.0)",
        f'    head = socket_read_byte_list(16, "{name}")',
        f"    if head[0] >= {minimum_status_line}:",
        f"      write_output_integer_register({register}, {digits})",
        "    else:",
        f"      write_output_integer_register({register}, -4 - head[0])",
        "    end",
        f'    socket_close("{name}")',
        "  else:",
        f"    write_output_integer_register({register}, -2)",
        "  end",
    ]

lines += [f"  write_output_integer_register({register_done}, {run_id})"]
lines += ["end", ""]
script = "\n".join(lines)

registers = [
    register_gripper_bytes,
    register_gripper_status,
    *camera_registers,
    register_done,
]
receive = RTDEReceiveInterface(
    robot_ip,
    125.0,
    ["timestamp"] + [f"output_int_register_{i}" for i in registers],
)
print(f"=== loopback protocol probe on {robot_ip} ===")
print(f"rtde connected   : {receive.isConnected()}")

with socket.create_connection((robot_ip, secondary_port), timeout=5.0) as sock:
    sock.sendall(script.encode())

deadline = time.monotonic() + 40.0
while time.monotonic() < deadline:
    if receive.getOutputIntRegister(register_done) == run_id:
        break
    time.sleep(0.2)
print(
    "script completed : "
    f"{receive.getOutputIntRegister(register_done) == run_id}\n"
)

count = receive.getOutputIntRegister(register_gripper_bytes)
status = receive.getOutputIntRegister(register_gripper_status)
if count == -2:
    print("  gripper 63352  -> socket refused")
elif count <= 0:
    print("  gripper 63352  -> no reply")
else:
    activated = "ACTIVATED" if status == 3 else f"STA {status}"
    print(f"  gripper 63352  -> replied {count} bytes, {activated}")

print()
for register, path in camera_registers.items():
    value = receive.getOutputIntRegister(register)
    if value == -2:
        verdict = "socket refused"
    elif value < 0:
        verdict = f"short reply, {-4 - value} bytes"
    elif value == 200:
        verdict = "HTTP 200 OK  <-- this URL works"
    else:
        verdict = f"HTTP {value}"
    print(f"  {path:<32} -> {verdict}")

receive.disconnect()
