# V4: fetch wrist camera frames over the Vision URCap HTTP endpoint.
# Read-only diagnostic -- this script never commands motion.
import pathlib
import sys

from cell_settings import cell_kwargs, default_robot_ip

from URRobotController import (
    CameraError,
    URRobotController,
    camera_candidate_paths,
    camera_port,
)

robot_ip = sys.argv[1] if len(sys.argv) > 1 else default_robot_ip

output_dir = pathlib.Path(__file__).parent / "output"
output_dir.mkdir(exist_ok=True)

image_types = ("color", "edges", "magnitude", "annotations")

print(f"=== candidate endpoints on {robot_ip}:{camera_port} ===")
for path in camera_candidate_paths:
    print(f"  http://{robot_ip}:{camera_port}{path}")

with URRobotController(robot_ip, **cell_kwargs) as cell:
    print(f"\nreport         : {cell.get_connection_report()}")
    print(f"camera_ok()    : {cell.camera_ok()}")

    results = []
    for image_type in image_types:
        print(f"\n--- {image_type} ---")
        try:
            frame = cell.camera_frame(image_type)
        except CameraError as exc:
            print(f"  FAIL: {exc}")
            results.append(False)
            continue
        target = output_dir / f"wrist_{image_type}.png"
        cell.camera_snapshot(str(target), image_type)
        print(f"  shape          : {frame.shape}")
        print(f"  dtype          : {frame.dtype}")
        print(f"  mean intensity : {frame.mean():.2f}")
        print(f"  saved          : {target}")
        results.append(True)

    verdict = "PASS" if all(results) else "FAIL"
    print(f"\nV4 {verdict}")
    sys.exit(0 if verdict == "PASS" else 1)
