# Settings that match the robot on the bench, as measured by V1.
# Imported by the other verification scripts so the three runs agree.
#
# Why these differ from the specification defaults:
#   polyscope_x=False   the controller reports URSoftware 5.25.2 and
#                       names programs "*.urp", so it is PolyScope 5.
#   use_ext_urcap=False port 50002 is closed, so there is no External
#                       Control URCap; ur_rtde must upload the control
#                       script itself.
#   rtde_frequency=125  at 500 Hz the controller closes the RTDE
#                       stream after roughly 1.5 s; 125 Hz is stable.

default_robot_ip = "192.168.1.238"

cell_kwargs = {
    "polyscope_x": False,
    "use_ext_urcap": False,
    "rtde_frequency": 125.0,
}
