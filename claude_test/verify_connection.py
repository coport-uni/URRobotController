# V1: connect every URWorkCell interface and print the report.
# Read-only diagnostic -- this script never commands motion.
import socket
import sys

from cell_settings import cell_kwargs, default_robot_ip

from URRobotController import URRobotController

robot_ip = sys.argv[1] if len(sys.argv) > 1 else default_robot_ip

ports = {
    29999: "Dashboard",
    30001: "Primary",
    30002: "Secondary",
    30004: "RTDE",
    63352: "Robotiq gripper URCap",
    4242: "Wrist camera URCap",
    50002: "External Control URCap",
}

print(f"=== port probe on {robot_ip} ===")
for port, label in sorted(ports.items()):
    sock = socket.socket()
    sock.settimeout(3.0)
    try:
        sock.connect((robot_ip, port))
        print(f"  {port:<6} {label:<24} OPEN")
    except OSError as exc:
        print(f"  {port:<6} {label:<24} FAIL ({type(exc).__name__}: {exc})")
    finally:
        sock.close()

print(f"\n=== URRobotController.connect() on {robot_ip} ===")
with URRobotController(robot_ip, **cell_kwargs) as cell:
    report = cell.get_connection_report()
    for name, ok in report.items():
        print(f"  {name:<10} {'OK' if ok else 'FAIL'}")

    print(f"\n  mode           : {cell.mode()}")
    print(f"  robot mode     : {cell.robot_mode()}")
    print(f"  safety status  : {cell.safety_status()}")
    print(f"  loaded program : {cell.loaded_program()}")
    print(f"  program running: {cell.program_running()}")

    joints = cell.joints()
    print("\n  joints [rad]   : " + ", ".join(f"{v:+.5f}" for v in joints))
    print(
        "  joints [deg]   : "
        + ", ".join(f"{v * 180.0 / 3.141592653589793:+8.3f}" for v in joints)
    )
    print(
        "  tcp pose       : " + ", ".join(f"{v:+.5f}" for v in cell.tcp_pose())
    )
    print(
        "  tcp force      : " + ", ".join(f"{v:+.3f}" for v in cell.tcp_force())
    )

print("\nV1 done -- no motion was commanded.")
