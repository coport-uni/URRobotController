# Characterise the controller's inbound port policy from the outside.
#
# Purpose: 63352 and 4242 time out while 29999 and 30004 connect. The
# question is whether that is a per-port block or a default-deny
# allow-list. Port 12345 is included as a control: nothing listens on
# it, so on an unfiltered host it must answer with an immediate RST.
# A timeout there proves a default-deny policy rather than a rule aimed
# at the URCap ports specifically.
#
# Read-only: this only opens TCP connections, it commands nothing.
#
# Expected lifetime: delete once the firewall situation is resolved.
import socket
import sys
from concurrent.futures import ThreadPoolExecutor

from cell_settings import default_robot_ip

robot_ip = sys.argv[1] if len(sys.argv) > 1 else default_robot_ip
connect_timeout = 6.0

ports = {
    22: ("SSH", "system"),
    80: ("HTTP", "system"),
    443: ("HTTPS", "system"),
    502: ("Modbus TCP", "UR native"),
    4242: ("Wrist camera", "URCap"),
    12345: ("nothing listens (CONTROL)", "control"),
    29999: ("Dashboard", "UR native"),
    30001: ("Primary", "UR native"),
    30002: ("Secondary", "UR native"),
    30003: ("Realtime", "UR native"),
    30004: ("RTDE", "UR native"),
    30020: ("Interpreter", "UR native"),
    50001: ("URCap misc", "URCap"),
    50002: ("External Control", "URCap"),
    63352: ("Robotiq gripper", "URCap"),
}


def probe(item):
    """Open one TCP connection and classify the outcome.

    Returns:
        Tuple of port, label, origin and result text.
    """
    port, (label, origin) = item
    sock = socket.socket()
    sock.settimeout(connect_timeout)
    try:
        sock.connect((robot_ip, port))
        return port, label, origin, "OPEN"
    except ConnectionRefusedError:
        return port, label, origin, "REFUSED (RST, reachable)"
    except TimeoutError:
        return port, label, origin, "dropped (no answer)"
    except OSError as error:
        return port, label, origin, type(error).__name__
    finally:
        sock.close()


print(f"=== inbound port policy of {robot_ip} ===\n")
with ThreadPoolExecutor(max_workers=len(ports)) as pool:
    results = sorted(pool.map(probe, ports.items()))

for port, label, origin, result in results:
    print(f"  {port:<6} {label:<26} {origin:<10} {result}")

open_ports = [port for port, _, _, result in results if result == "OPEN"]
blocked = [port for port, _, _, result in results if result.startswith("drop")]
print(f"\n  open    : {open_ports}")
print(f"  dropped : {blocked}")
print(
    "\nIf the control port 12345 is dropped rather than refused, the"
    "\ncontroller runs a default-deny policy and the open set is an"
    "\nallow-list. Compare that allow-list against UR's own standard"
    "\nservice ports to see whether URCap ports were simply never"
    "\nadded to it."
)
