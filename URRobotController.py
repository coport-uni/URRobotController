"""Integrated UR7e work cell control in a single class.

:class:`URRobotController` owns every interface of the cell -- RTDE
motion, the Robotiq gripper socket, the wrist camera HTTP endpoint, and
the Dashboard server -- behind one object. Grouping them is what makes
the defining feature of this module possible: a motion and a gripper
action are a single call, ``move_l(pose, gripper=GripperAction.close())``,
with a watch loop deciding when during that motion to drive the fingers.

The module implements the development specification in
``docs/URWorkCell_개발사양서.md``.

Example:
    >>> with URRobotController("192.168.1.238") as robot:
    ...     robot.gripper_activate()
    ...     robot.move_j(home_q, gripper=GripperAction.open())
"""

import math
import socket
import threading
import time
from dataclasses import dataclass
from enum import Enum

import cv2
import numpy as np
import requests
from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface

__version__ = "0.1.0"

# -- interface ports (specification section 2.1) ----------------------

rtde_port = 30004
secondary_port = 30002
dashboard_port = 29999
gripper_port = 63352
camera_port = 4242

# -- timeouts (specification section 5) -------------------------------

default_connect_timeout = 5.0
gripper_timeout = 2.0
camera_timeout = 5.0
camera_attempts = 2
dashboard_timeout = 5.0

# -- motion defaults (specification section 4.3) ----------------------

move_j_speed = 1.05
move_j_accel = 1.4
move_l_speed = 0.25
move_l_accel = 1.2
stop_decel = 2.0

joint_count = 6
pose_length = 6
max_rtde_frequency = 500.0

# Settling tolerances for the asynchronous move completion check.
# These are deliberately looser than the accuracy the specification
# asks for in section 8, so that acceptance testing measures the
# controller's real convergence instead of the value used to decide
# that the move had ended.
joint_settle_tolerance = 0.01
position_settle_tolerance = 0.002
rotation_settle_tolerance = 0.01
joint_speed_tolerance = 0.001

# The controller sometimes drops the first RTDE handshake.
receive_connect_attempts = 3
receive_retry_delay = 0.5

# Specification section 4.4 item 1: the trigger watch loop runs at
# 100 Hz, five RTDE cycles at the default 500 Hz.
watch_interval = 0.01

# Specification section 4.7: a mode switch should settle within 2 s.
mode_switch_timeout = 2.0

# -- gripper protocol -------------------------------------------------

gripper_value_min = 0
gripper_value_max = 255
gripper_open_position = 0
gripper_closed_position = 255

# Values of the STA register: 3 means activation finished.
status_activated = 3

# Values of the OBJ register.
object_moving = 0
object_detected_opening = 1
object_detected_closing = 2

# Robotiq fault codes. 5 and 7 only mean that activation has not
# finished yet, so they are expected while the sequence runs.
activation_faults = (0, 5, 7)

gripper_fault_meanings = {
    0: "no fault",
    5: "action delayed, activation must complete first",
    7: "the activation bit must be set prior to performing action",
    8: "maximum operating temperature exceeded",
    9: "no communication for at least one second",
    10: "under minimum operating voltage",
    11: "automatic release in progress",
    12: "internal fault",
    13: "activation fault, check for interference",
    14: "overcurrent triggered",
    15: "automatic release completed",
}

gripper_echo_timeout = 1.0
gripper_activation_timeout = 10.0
gripper_move_timeout = 10.0
gripper_settle_timeout = 5.0

# -- camera protocol --------------------------------------------------

valid_image_types = ("color", "edges", "magnitude", "annotations")
min_poll_interval = 0.1

# Candidate paths, most likely first. "{image_type}" is substituted.
camera_candidate_paths = (
    "/current.jpg?type={image_type}",
    "/current.jpg?annotations={image_type}",
    "/current.jpg",
    "/camera/current.jpg?type={image_type}",
)

# -- dashboard protocol -----------------------------------------------

socket_buffer_bytes = 4096

# Dashboard replies that mean the request was refused.
failure_markers = (
    "could not",
    "failed",
    "file not found",
    "error",
    "not found",
)

valid_triggers = (
    "start",
    "end",
    "time",
    "remaining_dist",
    "remaining_joint",
)


# =====================================================================
# Exceptions (specification section 6)
# =====================================================================


class WorkCellError(Exception):
    """Base class for every UR work cell failure.

    Args:
        message: Human-readable description of what failed.
        interface: Name of the interface involved, such as ``"rtde"``
            or ``"gripper"``. Use ``None`` when no single interface is
            responsible.
        response: Raw reply from the device, when one was received.
            Preserved verbatim because Robotiq and Dashboard replies
            are the only diagnostic available for those protocols.
    """

    code = "E000"

    def __init__(
        self,
        message: str,
        interface: str | None = None,
        response: str | None = None,
    ) -> None:
        self.interface = interface
        self.response = response
        parts = [f"[{self.code}]"]
        if interface is not None:
            parts.append(f"({interface})")
        parts.append(message)
        if response is not None:
            parts.append(f"| response: {response!r}")
        super().__init__(" ".join(parts))


class ConnectionFailedError(WorkCellError):
    """An interface could not be reached, or reconnection failed."""

    code = "E100"


class MotionError(WorkCellError):
    """A move command was rejected by the controller or failed."""

    code = "E200"


class ProtectiveStopError(MotionError):
    """The robot entered protective stop while a command was active."""

    code = "E201"


class GripperError(WorkCellError):
    """The gripper did not acknowledge, or reported a fault code."""

    code = "E300"


class GripperNotActivatedError(GripperError):
    """A move was requested before the gripper finished activation."""

    code = "E301"


class CameraError(WorkCellError):
    """The camera endpoint did not answer, or the image failed to decode."""

    code = "E400"


class ProgramError(WorkCellError):
    """Loading, starting, or stopping a stored program failed."""

    code = "E500"


class ModeConflictError(WorkCellError):
    """A call was made that the current execution mode forbids."""

    code = "E600"


# =====================================================================
# Data types (specification sections 2.3, 4.2, 4.3)
# =====================================================================


class Mode(str, Enum):
    """Execution mode of the work cell.

    RTDE external control and a stored controller program cannot run at
    the same time, so the class tracks which of the two owns the robot.

    Attributes:
        idle: Connected, nothing driving the robot.
        motion: RTDE external control is active.
        program: A stored controller program is running.
    """

    idle = "IDLE"
    motion = "MOTION"
    program = "PROGRAM"


@dataclass
class GripperAction:
    """One gripper movement attached to a motion command.

    The point of this type is to let a caller express "move there and
    grasp" as a single call. The trigger decides *when* during the
    motion the gripper command is issued.

    Attributes:
        position: Target position, 0 open to 255 closed.
        speed: Travel speed, 0 slow to 255 fast.
        force: Grip force, 0 weak to 255 strong.
        trigger: One of ``"start"``, ``"end"``, ``"time"``,
            ``"remaining_dist"`` or ``"remaining_joint"``.
        threshold: Trigger comparison value. Seconds for ``"time"``,
            metres for ``"remaining_dist"``, radians for
            ``"remaining_joint"``. Unused by ``"start"`` and ``"end"``.
        wait_object: When true, the call waits for the gripper to
            settle and reports the object detection status.

    Raises:
        ValueError: If a field is outside its allowed range.
    """

    position: int
    speed: int = gripper_value_max
    force: int = 128
    trigger: str = "start"
    threshold: float = 0.0
    wait_object: bool = False

    def __post_init__(self) -> None:
        for name in ("position", "speed", "force"):
            value = getattr(self, name)
            if not gripper_value_min <= value <= gripper_value_max:
                raise ValueError(
                    f"GripperAction.{name} must be "
                    f"{gripper_value_min}-{gripper_value_max}, "
                    f"got {value}"
                )
        if self.trigger not in valid_triggers:
            raise ValueError(
                f"GripperAction.trigger must be one of "
                f"{valid_triggers}, got {self.trigger!r}"
            )
        if self.threshold < 0.0:
            raise ValueError(
                f"GripperAction.threshold must not be negative, "
                f"got {self.threshold}"
            )

    @classmethod
    def open(
        cls,
        speed: int = gripper_value_max,
        force: int = 128,
        trigger: str = "start",
        threshold: float = 0.0,
        wait_object: bool = False,
    ) -> "GripperAction":
        """Build an action that opens the gripper fully.

        Returns:
            A :class:`GripperAction` targeting the open position.
        """
        return cls(
            position=gripper_open_position,
            speed=speed,
            force=force,
            trigger=trigger,
            threshold=threshold,
            wait_object=wait_object,
        )

    @classmethod
    def close(
        cls,
        speed: int = gripper_value_max,
        force: int = 128,
        trigger: str = "end",
        threshold: float = 0.0,
        wait_object: bool = True,
    ) -> "GripperAction":
        """Build an action that closes the gripper fully.

        Closing defaults to the ``"end"`` trigger and to waiting for
        object detection, which is the common pick sequence.

        Returns:
            A :class:`GripperAction` targeting the closed position.
        """
        return cls(
            position=gripper_closed_position,
            speed=speed,
            force=force,
            trigger=trigger,
            threshold=threshold,
            wait_object=wait_object,
        )


@dataclass
class MotionResult:
    """Outcome of a motion, including its attached gripper action.

    Attributes:
        motion_ok: True when the motion ran to completion.
        gripper_pos: Final gripper position, or ``None`` when no
            gripper action was attached or the command failed.
        gripper_obj: Robotiq object status 0 to 3. Values 1 and 2 mean
            an object is held. ``None`` when not measured.
        elapsed: Wall clock duration of the call in seconds.
        gripper_error: Description of the gripper failure, when the
            motion continued despite the gripper not responding.
    """

    motion_ok: bool
    gripper_pos: int | None = None
    gripper_obj: int | None = None
    elapsed: float = 0.0
    gripper_error: str | None = None


# =====================================================================
# The controller
# =====================================================================


class URRobotController:
    """Motion, gripper, camera and program control for one UR7e.

    The class connects on :meth:`connect` and releases everything on
    :meth:`disconnect`. It also works as a context manager, which is
    the recommended form because it guarantees the RTDE control script
    is torn down even when a motion raises.

    Args:
        robot_ip: Address of the robot controller.
        rtde_frequency: RTDE update rate in Hz, at most 500. Some
            controllers close the stream when they cannot sustain the
            rate; 125 Hz is a safe fallback.
        use_ext_urcap: Reach the controller through the PolyScope X
            External Control URCap rather than uploading the control
            script directly.
        polyscope_x: Apply PolyScope X stored program naming, which
            omits the ``.urp`` extension.
        gripper_enabled: Connect the Robotiq gripper socket.
        camera_enabled: Probe the wrist camera endpoint.
        connect_timeout: Seconds allowed per interface at connect time.
        abort_motion_on_gripper_fault: Stop the robot when a gripper
            command fails mid-motion. The default of False follows
            specification section 4.4 item 4, where a gripper failure
            is recorded but the motion continues.

    Raises:
        ValueError: If ``rtde_frequency`` is outside 0 to 500 Hz.
    """

    def __init__(
        self,
        robot_ip: str,
        rtde_frequency: float = max_rtde_frequency,
        use_ext_urcap: bool = True,
        polyscope_x: bool = True,
        gripper_enabled: bool = True,
        camera_enabled: bool = True,
        connect_timeout: float = default_connect_timeout,
        abort_motion_on_gripper_fault: bool = False,
    ) -> None:
        if not 0.0 < rtde_frequency <= max_rtde_frequency:
            raise ValueError(
                f"rtde_frequency must be within 0 to "
                f"{max_rtde_frequency} Hz, got {rtde_frequency}"
            )

        self.robot_ip = robot_ip
        self.rtde_frequency = rtde_frequency
        self.use_ext_urcap = use_ext_urcap
        self.polyscope_x = polyscope_x
        self.gripper_enabled = gripper_enabled
        self.camera_enabled = camera_enabled
        self.connect_timeout = connect_timeout
        self.abort_motion_on_gripper_fault = abort_motion_on_gripper_fault

        self._control: RTDEControlInterface | None = None
        self._receive: RTDEReceiveInterface | None = None

        self._gripper_socket: socket.socket | None = None
        self._gripper_lock = threading.Lock()

        self._session = requests.Session()
        self._camera_path: str | None = None
        self._camera_ok = False

        self._dashboard_socket: socket.socket | None = None
        self._dashboard_lock = threading.Lock()
        self.dashboard_banner: str | None = None

        self._mode = Mode.idle
        self._watch_thread: threading.Thread | None = None
        self._watch_result: MotionResult | None = None
        self._watch_error: BaseException | None = None

        # State for the asynchronous move completion check, reset by
        # :meth:`_begin_async_move` before each move is dispatched.
        self._async_baseline = None
        self._async_observed_running = False
        self._async_kind = ""
        self._async_target: list[float] = []

    # =================================================================
    # Lifecycle (specification section 4.1)
    # =================================================================

    def __enter__(self) -> "URRobotController":
        """Connect on entry to a ``with`` block.

        Returns:
            The connected controller.
        """
        self.connect()
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        """Release every interface on exit from a ``with`` block."""
        self.disconnect()

    def connect(self) -> None:
        """Connect every enabled interface.

        The RTDE receive interface and the Dashboard server are
        required. The gripper and the camera are recorded as failed in
        the connection report rather than raising, because
        specification section 7 requires the remaining functions to
        keep working when a URCap is absent. RTDE *control* is opened
        lazily on the first motion, so connecting does not take the
        robot away from the pendant.

        Raises:
            ConnectionFailedError: If RTDE receive or the Dashboard
                server could not be reached.
        """
        self._connect_receive()
        self._connect_dashboard()

        if self.gripper_enabled:
            try:
                self._connect_gripper()
            except ConnectionFailedError:
                pass

        if self.camera_enabled:
            self._camera_ok = self.camera_ok()

    def disconnect(self) -> None:
        """Stop any motion and release every interface."""
        if self._watch_thread is not None:
            self._watch_thread.join(timeout=gripper_settle_timeout)
            self._watch_thread = None

        try:
            self.stop()
        except (MotionError, ConnectionFailedError):
            pass

        self._release_control()
        if self._receive is not None:
            try:
                self._receive.disconnect()
            except RuntimeError:
                pass
            finally:
                self._receive = None

        self._close_socket("_gripper_socket", self._gripper_lock)
        self._session.close()
        self._close_socket("_dashboard_socket", self._dashboard_lock)
        self._mode = Mode.idle

    def _close_socket(self, name: str, lock: threading.Lock) -> None:
        """Close one of the raw sockets held by the controller.

        Args:
            name: Attribute name holding the socket.
            lock: The lock that serialises access to that socket.
        """
        with lock:
            sock = getattr(self, name)
            if sock is not None:
                try:
                    sock.close()
                finally:
                    setattr(self, name, None)

    def get_connection_report(self) -> dict[str, bool]:
        """Report which interfaces are currently usable.

        Returns:
            Mapping of ``"motion"``, ``"gripper"``, ``"camera"`` and
            ``"dashboard"`` to their connection state. A disabled
            module reports False.
        """
        return {
            "motion": self._receive_connected(),
            "gripper": self._gripper_socket is not None,
            "camera": self._camera_ok,
            "dashboard": self._dashboard_socket is not None,
        }

    # =================================================================
    # Mode state machine (specification section 2.3)
    # =================================================================

    def mode(self) -> str:
        """Report the current execution mode.

        Returns:
            ``"IDLE"``, ``"MOTION"`` or ``"PROGRAM"``.
        """
        if self._mode is Mode.program and not self.program_running():
            # The program may have finished on its own; the state
            # machine follows the controller rather than the last
            # command issued.
            self._mode = Mode.idle
        return self._mode.value

    def _enter_motion_mode(self) -> None:
        """Take RTDE external control of the robot.

        Raises:
            ModeConflictError: If a stored program is running, or if
                the pendant is in local control.
            ConnectionFailedError: If RTDE control cannot be opened.
        """
        if self.mode() == Mode.program.value:
            raise ModeConflictError(
                "a stored program is running; call stop_program() "
                "before commanding motion over RTDE",
                interface="rtde",
            )
        try:
            self._connect_control()
        except ConnectionFailedError as exc:
            # Local control is by far the most common cause, and the
            # RTDE error alone does not say so.
            if self._dashboard_socket is not None and not (
                self.remote_control()
            ):
                raise ModeConflictError(
                    "the robot is in local control; switch the "
                    "pendant to Remote Control before commanding "
                    "motion over RTDE",
                    interface="rtde",
                    response=self.operational_mode(),
                ) from exc
            raise
        self._mode = Mode.motion

    def release_motion(self) -> None:
        """Give up RTDE external control and return to IDLE."""
        self._release_control()
        if self._mode is Mode.motion:
            self._mode = Mode.idle

    # =================================================================
    # RTDE connection
    # =================================================================

    def _connect_receive(self) -> None:
        """Open the RTDE receive interface.

        State polling must work even when nothing owns the robot, so
        the receive interface connects on its own.

        Raises:
            ConnectionFailedError: If the interface cannot be opened.
        """
        if self._receive_connected():
            return
        last_error = ""
        # The controller occasionally drops the first RTDE handshake
        # with an end-of-file, leaving an object that reports itself
        # disconnected rather than raising. Retrying covers both.
        for _ in range(receive_connect_attempts):
            try:
                candidate = RTDEReceiveInterface(
                    self.robot_ip, self.rtde_frequency
                )
            except RuntimeError as exc:
                last_error = str(exc)
                time.sleep(receive_retry_delay)
                continue
            if candidate.isConnected():
                self._receive = candidate
                return
            last_error = "interface reported itself disconnected"
            candidate.disconnect()
            time.sleep(receive_retry_delay)

        self._receive = None
        raise ConnectionFailedError(
            f"cannot open the RTDE receive interface on "
            f"{self.robot_ip}:{rtde_port}",
            interface="rtde",
            response=last_error,
        )

    def _connect_control(self) -> None:
        """Open the RTDE control interface and upload the script.

        Raises:
            ConnectionFailedError: If the interface cannot be opened.
                The usual causes are the robot not being in remote
                control, or the External Control URCap not running.
        """
        if self._control_connected():
            return
        flags = (
            RTDEControlInterface.FLAG_USE_EXT_UR_CAP
            if self.use_ext_urcap
            else RTDEControlInterface.FLAG_UPLOAD_SCRIPT
        )
        try:
            self._control = RTDEControlInterface(
                self.robot_ip, self.rtde_frequency, flags
            )
        except RuntimeError as exc:
            self._control = None
            raise ConnectionFailedError(
                f"cannot open the RTDE control interface on "
                f"{self.robot_ip}; check that the robot is in remote "
                f"control and that External Control is running",
                interface="rtde",
                response=str(exc),
            ) from exc

    def _release_control(self) -> None:
        """Stop the control script and drop the control interface.

        A stored program cannot start while the RTDE control script
        owns the robot, so this runs before every mode switch.
        """
        if self._control is None:
            return
        try:
            if self._control.isConnected():
                self._control.stopScript()
                self._control.disconnect()
        except RuntimeError:
            # The controller may already have torn the script down.
            pass
        finally:
            self._control = None

    def _control_connected(self) -> bool:
        """Report whether the control interface is usable.

        Returns:
            True when the control interface is connected.
        """
        return self._control is not None and self._control.isConnected()

    def _receive_connected(self) -> bool:
        """Report whether the receive interface is usable.

        Returns:
            True when the receive interface is connected.
        """
        return self._receive is not None and self._receive.isConnected()

    def _require_control(self) -> RTDEControlInterface:
        """Return the control interface, reconnecting once if needed.

        Returns:
            The live control interface.

        Raises:
            ConnectionFailedError: If the interface cannot be restored.
        """
        if not self._control_connected():
            self._control = None
            self._connect_control()
        assert self._control is not None
        return self._control

    def _require_receive(self) -> RTDEReceiveInterface:
        """Return the receive interface, reconnecting once if needed.

        Returns:
            The live receive interface.

        Raises:
            ConnectionFailedError: If the interface cannot be restored.
        """
        if not self._receive_connected():
            self._receive = None
            self._connect_receive()
        assert self._receive is not None
        return self._receive

    # =================================================================
    # State queries (specification section 4.8)
    # =================================================================

    def joints(self) -> list[float]:
        """Read the measured joint angles.

        Returns:
            Six joint angles in radians.
        """
        return list(self._require_receive().getActualQ())

    def joint_speeds(self) -> list[float]:
        """Read the measured joint velocities.

        Returns:
            Six joint velocities in rad/s.
        """
        return list(self._require_receive().getActualQd())

    def tcp_pose(self) -> list[float]:
        """Read the measured TCP pose.

        Returns:
            Six pose elements, metres then rotation vector radians.
        """
        return list(self._require_receive().getActualTCPPose())

    def tcp_force(self) -> list[float]:
        """Read the measured TCP force and torque.

        Returns:
            Three forces in newtons and three torques in newton metres.
        """
        return list(self._require_receive().getActualTCPForce())

    def is_steady(self) -> bool:
        """Report whether the robot has come to rest.

        Returns:
            True when the robot is fully at rest. Always False unless
            RTDE control is active, which is a limitation of the
            underlying controller query.
        """
        if not self._control_connected():
            return False
        assert self._control is not None
        return bool(self._control.isSteady())

    def check_protective_stop(self) -> None:
        """Raise when the robot sits in protective or emergency stop.

        Raises:
            ProtectiveStopError: If either stop is active.
        """
        receive = self._require_receive()
        if receive.isProtectiveStopped():
            raise ProtectiveStopError(
                "robot is in protective stop", interface="rtde"
            )
        if receive.isEmergencyStopped():
            raise ProtectiveStopError(
                "robot is in emergency stop", interface="rtde"
            )

    # =================================================================
    # Motion API (specification section 4.3)
    # =================================================================

    def move_j(
        self,
        q: list[float],
        speed: float = move_j_speed,
        accel: float = move_j_accel,
        gripper: GripperAction | None = None,
        blocking: bool = True,
    ) -> MotionResult:
        """Move in joint space, optionally driving the gripper.

        Args:
            q: Six target joint angles in radians.
            speed: Leading axis speed in rad/s.
            accel: Leading axis acceleration in rad/s^2.
            gripper: Gripper action to run alongside the motion.
            blocking: Wait for the motion, and for the gripper when
                the action asks for it, before returning.

        Returns:
            A :class:`MotionResult`. When ``blocking`` is False the
            result reports the motion as started, not finished; call
            :meth:`wait_motion_done` for the final result.

        Raises:
            MotionError: If the controller rejected the move.
            ProtectiveStopError: If the robot stopped protectively.
            ModeConflictError: If a stored program owns the robot.
        """
        target = self._validate_vector(q, joint_count, "q")
        return self._run_motion(
            "move_j", target, speed, accel, gripper, blocking
        )

    def move_l(
        self,
        pose: list[float],
        speed: float = move_l_speed,
        accel: float = move_l_accel,
        gripper: GripperAction | None = None,
        blocking: bool = True,
    ) -> MotionResult:
        """Move the TCP in a straight line, optionally gripping.

        Args:
            pose: Six pose elements, metres then rotation vector.
            speed: TCP speed in m/s.
            accel: TCP acceleration in m/s^2.
            gripper: Gripper action to run alongside the motion.
            blocking: Wait for the motion before returning.

        Returns:
            A :class:`MotionResult`.

        Raises:
            MotionError: If the controller rejected the move.
            ProtectiveStopError: If the robot stopped protectively.
            ModeConflictError: If a stored program owns the robot.
        """
        target = self._validate_vector(pose, pose_length, "pose")
        return self._run_motion(
            "move_l", target, speed, accel, gripper, blocking
        )

    def move_j_ik(
        self,
        pose: list[float],
        speed: float = move_j_speed,
        accel: float = move_j_accel,
        gripper: GripperAction | None = None,
        blocking: bool = True,
    ) -> MotionResult:
        """Move to a TCP pose along a joint-space path.

        Args:
            pose: Six pose elements, metres then rotation vector.
            speed: Leading axis speed in rad/s.
            accel: Leading axis acceleration in rad/s^2.
            gripper: Gripper action to run alongside the motion.
            blocking: Wait for the motion before returning.

        Returns:
            A :class:`MotionResult`.

        Raises:
            MotionError: If the controller rejected the move.
            ProtectiveStopError: If the robot stopped protectively.
            ModeConflictError: If a stored program owns the robot.
        """
        target = self._validate_vector(pose, pose_length, "pose")
        return self._run_motion(
            "move_j_ik", target, speed, accel, gripper, blocking
        )

    def stop(self, decel: float = stop_decel) -> None:
        """Decelerate the robot to a stop.

        Args:
            decel: Joint deceleration in rad/s^2.
        """
        if not self._control_connected():
            return
        assert self._control is not None
        try:
            self._control.stopJ(decel)
        except RuntimeError:
            # Nothing useful remains to do if the stop itself fails;
            # disconnect() will tear the script down.
            pass

    def wait_motion_done(self, timeout: float = 60.0) -> MotionResult | None:
        """Wait for a non-blocking motion to finish.

        Args:
            timeout: Seconds to wait for the watch thread.

        Returns:
            The result of the motion, or ``None`` when no non-blocking
            motion is outstanding.

        Raises:
            MotionError: If the motion did not finish in time.
            BaseException: Whatever the watch thread raised, re-raised
                here so a failure is not lost in the worker thread.
        """
        thread = self._watch_thread
        if thread is None:
            return None
        thread.join(timeout=timeout)
        if thread.is_alive():
            raise MotionError(
                f"motion did not finish within {timeout} s",
                interface="rtde",
            )
        self._watch_thread = None
        if self._watch_error is not None:
            error = self._watch_error
            self._watch_error = None
            raise error
        return self._watch_result

    @staticmethod
    def _validate_vector(
        values: list[float], length: int, name: str
    ) -> list[float]:
        """Check that a vector has the right length and finite values.

        Returns:
            The vector as a list of floats.

        Raises:
            ValueError: If the length or a value is wrong.
        """
        vector = [float(value) for value in values]
        if len(vector) != length:
            raise ValueError(
                f"{name} must have {length} elements, got {len(vector)}"
            )
        if not all(math.isfinite(value) for value in vector):
            raise ValueError(f"{name} contains a non-finite value")
        return vector

    # =================================================================
    # Motion and gripper coordination (specification section 4.4)
    # =================================================================

    def _run_motion(
        self,
        kind: str,
        target: list[float],
        speed: float,
        accel: float,
        gripper: GripperAction | None,
        blocking: bool,
    ) -> MotionResult:
        """Start a motion and coordinate the attached gripper action.

        Returns:
            A :class:`MotionResult`.

        Raises:
            MotionError: If the controller rejected the move.
            ModeConflictError: If a stored program owns the robot.
        """
        if self._watch_thread is not None:
            self.wait_motion_done()

        self._enter_motion_mode()
        started = time.monotonic()

        if gripper is None and blocking:
            # Nothing to coordinate: let ur_rtde block on the move.
            self._dispatch_move(kind, target, speed, accel, False)
            return MotionResult(
                motion_ok=True, elapsed=time.monotonic() - started
            )

        watch_target = (
            self._resolve_watch_target(kind, target, gripper)
            if gripper is not None
            else None
        )
        # The baseline must be sampled before the move is dispatched,
        # or a short move can finish unnoticed.
        self._begin_async_move(kind, target)
        self._dispatch_move(kind, target, speed, accel, True)

        if blocking:
            return self._watch(started, gripper, watch_target)

        self._watch_result = None
        self._watch_error = None
        self._watch_thread = threading.Thread(
            target=self._watch_in_thread,
            args=(started, gripper, watch_target),
            daemon=True,
        )
        self._watch_thread.start()
        return MotionResult(motion_ok=True, elapsed=time.monotonic() - started)

    def _dispatch_move(
        self,
        kind: str,
        target: list[float],
        speed: float,
        accel: float,
        asynchronous: bool,
    ) -> None:
        """Send one move command to the controller.

        Raises:
            ProtectiveStopError: If the robot is in a stop state.
            MotionError: If the controller rejected the move.
        """
        control = self._require_control()
        commands = {
            "move_j": ("moveJ", control.moveJ),
            "move_l": ("moveL", control.moveL),
            "move_j_ik": ("moveJ_IK", control.moveJ_IK),
        }
        name, command = commands[kind]

        self.check_protective_stop()
        try:
            accepted = command(target, speed, accel, asynchronous)
        except RuntimeError as exc:
            raise MotionError(
                f"{name} failed", interface="rtde", response=str(exc)
            ) from exc
        if not accepted:
            self.check_protective_stop()
            raise MotionError(
                f"{name} was rejected by the controller; the target "
                f"may be unreachable or outside the safety limits",
                interface="rtde",
                response=f"target={target}",
            )

    def _resolve_watch_target(
        self, kind: str, target: list[float], gripper: GripperAction
    ) -> list[float] | None:
        """Convert the move target into the space the trigger uses.

        A distance trigger compares TCP positions and a joint trigger
        compares joint vectors, so a joint-space move with a distance
        trigger needs forward kinematics, and the reverse needs inverse
        kinematics. Resolving this once before the move keeps the
        10 ms watch loop free of kinematics calls.

        Returns:
            The target in trigger space, or ``None`` when the trigger
            does not compare against a target.
        """
        control = self._require_control()
        joint_space_move = kind == "move_j"

        if gripper.trigger == "remaining_dist":
            if joint_space_move:
                return list(control.getForwardKinematics(target))
            return list(target)

        if gripper.trigger == "remaining_joint":
            if joint_space_move:
                return list(target)
            return list(control.getInverseKinematics(target, self.joints()))

        return None

    def _watch_in_thread(
        self,
        started: float,
        gripper: GripperAction | None,
        watch_target: list[float] | None,
    ) -> None:
        """Run :meth:`_watch` in the background thread.

        The result and any exception are stored for
        :meth:`wait_motion_done` to hand back to the caller.
        """
        try:
            self._watch_result = self._watch(started, gripper, watch_target)
        except BaseException as exc:  # noqa: BLE001
            self._watch_error = exc

    def _watch(
        self,
        started: float,
        gripper: GripperAction | None,
        watch_target: list[float] | None,
    ) -> MotionResult:
        """Poll the motion, fire the gripper, and build the result.

        The loop runs at the 10 ms period fixed by specification
        section 4.4 item 1. An ``"end"`` trigger is handled after the
        loop, which is what keeps it within the 50 ms of motion
        completion that section 8 item T05 asks for.

        Returns:
            A :class:`MotionResult`.

        Raises:
            ProtectiveStopError: If the robot stopped protectively.
            GripperError: If the gripper failed and
                ``abort_motion_on_gripper_fault`` is set.
        """
        result = MotionResult(motion_ok=False)
        pending = gripper is not None
        watch_during_motion = gripper is not None and gripper.trigger != "end"

        while True:
            self.check_protective_stop()
            if (
                pending
                and watch_during_motion
                and self._trigger_reached(gripper, watch_target, started)
            ):
                self._fire_gripper(gripper, result)
                pending = False
            if not self._async_move_running():
                break
            time.sleep(watch_interval)

        result.motion_ok = True
        if pending:
            self._fire_gripper(gripper, result)

        if gripper is not None and gripper.wait_object:
            self._collect_gripper_status(result)

        result.elapsed = time.monotonic() - started
        return result

    def _trigger_reached(
        self,
        gripper: GripperAction,
        watch_target: list[float] | None,
        started: float,
    ) -> bool:
        """Decide whether the gripper trigger condition now holds.

        Returns:
            True when the gripper command should be issued.
        """
        if gripper.trigger == "start":
            return True
        if gripper.trigger == "time":
            return time.monotonic() - started >= gripper.threshold
        if watch_target is None:
            return False
        if gripper.trigger == "remaining_dist":
            current = self.tcp_pose()
            remaining = math.dist(current[:3], watch_target[:3])
            return remaining <= gripper.threshold
        if gripper.trigger == "remaining_joint":
            current = self.joints()
            remaining = float(
                np.linalg.norm(np.asarray(current) - np.asarray(watch_target))
            )
            return remaining <= gripper.threshold
        return False

    def _fire_gripper(
        self, gripper: GripperAction, result: MotionResult
    ) -> None:
        """Issue the gripper command without waiting for it to finish.

        A gripper fault is recorded on the result and the motion
        continues, per specification section 4.4 item 4, unless the
        caller asked for the motion to abort.

        Raises:
            GripperError: If the command failed and
                ``abort_motion_on_gripper_fault`` is set.
        """
        if not self.gripper_enabled:
            result.gripper_error = "gripper module is disabled"
            return
        try:
            self.gripper_move(
                gripper.position,
                gripper.speed,
                gripper.force,
                wait=False,
            )
        except (GripperError, ConnectionFailedError) as exc:
            result.gripper_error = str(exc)
            if self.abort_motion_on_gripper_fault:
                self.stop()
                raise

    def _collect_gripper_status(self, result: MotionResult) -> None:
        """Wait for the fingers to settle and record their status."""
        if not self.gripper_enabled or result.gripper_error is not None:
            return
        deadline = time.monotonic() + gripper_settle_timeout
        try:
            while time.monotonic() < deadline:
                if self.gripper_get("OBJ") != object_moving:
                    break
                time.sleep(watch_interval)
            result.gripper_obj = self.gripper_get("OBJ")
            result.gripper_pos = self.gripper_get("POS")
        except (GripperError, ConnectionFailedError) as exc:
            result.gripper_error = str(exc)

    # -- asynchronous move completion ---------------------------------

    def _begin_async_move(self, kind: str, target: list[float]) -> None:
        """Arm the completion check for one asynchronous move.

        Call this immediately before dispatching the move.

        Args:
            kind: ``"move_j"``, ``"move_l"`` or ``"move_j_ik"``.
            target: The move target, in the space named by ``kind``.
        """
        self._async_kind = kind
        self._async_target = target
        self._async_observed_running = False
        self._async_baseline = (
            self._require_control().getAsyncOperationProgressEx()
        )

    def _async_move_running(self) -> bool:
        """Report whether the asynchronous move is still executing.

        ``ur_rtde`` reports async progress only for the operations that
        support progress feedback, and the report is a toggling counter
        rather than a flag, so a move that ends between two polls can
        be missed. Two independent signals are therefore combined: the
        controller's async status, and the robot itself having arrived
        near the target and stopped moving. Either one is enough to
        call the move finished.

        Returns:
            True while the move is in progress.
        """
        if self._async_status_finished():
            return False
        return not self._async_target_reached()

    def _async_status_finished(self) -> bool:
        """Ask the controller whether the async operation has ended.

        Returns:
            True when the controller reports the operation finished.
        """
        status = self._require_control().getAsyncOperationProgressEx()
        if status.isAsyncOperationRunning():
            self._async_observed_running = True
            return False
        if self._async_observed_running:
            return True
        # Not running and never seen running: the operation counter
        # having moved is the only proof it ran and ended between two
        # polls.
        return not status.equals(self._async_baseline)

    def _async_target_reached(self) -> bool:
        """Check whether the robot arrived and stopped.

        Returns:
            True when the robot sits near the target at rest.
        """
        if self._async_kind == "move_j":
            errors = [
                abs(actual - wanted)
                for actual, wanted in zip(
                    self.joints(), self._async_target, strict=True
                )
            ]
            reached = max(errors) <= joint_settle_tolerance
        else:
            pose = self.tcp_pose()
            reached = (
                math.dist(pose[:3], self._async_target[:3])
                <= position_settle_tolerance
                and math.dist(pose[3:], self._async_target[3:])
                <= rotation_settle_tolerance
            )
        if not reached:
            return False
        speeds = [abs(value) for value in self.joint_speeds()]
        return max(speeds) <= joint_speed_tolerance

    # =================================================================
    # Gripper (specification section 4.5)
    # =================================================================

    def _connect_gripper(self) -> None:
        """Open the socket to the URCap gripper server.

        Raises:
            ConnectionFailedError: If the port does not accept the
                connection within the timeout. The port only listens
                while the Robotiq URCap is running on the pendant.
        """
        try:
            sock = socket.create_connection(
                (self.robot_ip, gripper_port),
                timeout=self.connect_timeout,
            )
        except OSError as exc:
            raise ConnectionFailedError(
                f"cannot reach the Robotiq URCap socket on "
                f"{self.robot_ip}:{gripper_port}; check that the "
                f"Robotiq Grippers URCap is installed and running",
                interface="gripper",
                response=str(exc),
            ) from exc
        sock.settimeout(gripper_timeout)
        self._gripper_socket = sock

    def _require_gripper(self) -> None:
        """Ensure the gripper socket is open.

        Raises:
            ConnectionFailedError: If the gripper is disabled or the
                socket cannot be opened.
        """
        if not self.gripper_enabled:
            raise ConnectionFailedError(
                "the gripper is disabled; construct "
                "URRobotController with gripper_enabled=True",
                interface="gripper",
            )
        if self._gripper_socket is None:
            self._connect_gripper()

    def _gripper_exchange(self, command: str) -> str:
        """Send one gripper command and return the reply.

        The Robotiq URCap accepts one command at a time, so the
        exchange is serialised: the trigger watch thread and the
        caller thread both reach this socket.

        Args:
            command: Protocol line without its terminator.

        Returns:
            The decoded reply, stripped of whitespace.

        Raises:
            GripperError: If the exchange fails twice in a row.
        """
        with self._gripper_lock:
            try:
                return self._gripper_exchange_once(command)
            except OSError as first_error:
                # A dropped socket is common when the pendant restarts
                # the URCap; one silent retry keeps a motion alive.
                try:
                    if self._gripper_socket is not None:
                        self._gripper_socket.close()
                    self._gripper_socket = socket.create_connection(
                        (self.robot_ip, gripper_port),
                        timeout=self.connect_timeout,
                    )
                    self._gripper_socket.settimeout(gripper_timeout)
                    return self._gripper_exchange_once(command)
                except OSError as second_error:
                    self._gripper_socket = None
                    raise GripperError(
                        f"command {command!r} failed and reconnection "
                        f"did not recover it",
                        interface="gripper",
                        response=f"{first_error} / {second_error}",
                    ) from second_error

    def _gripper_exchange_once(self, command: str) -> str:
        """Perform a single gripper request and response round trip.

        Returns:
            The decoded reply, stripped of whitespace.

        Raises:
            OSError: If the socket is closed or times out.
        """
        if self._gripper_socket is None:
            raise OSError("gripper socket is not open")
        self._gripper_socket.sendall(f"{command}\n".encode())
        reply = self._gripper_socket.recv(socket_buffer_bytes)
        if not reply:
            raise OSError("gripper closed the connection")
        return reply.decode(errors="replace").strip()

    def gripper_get(self, name: str) -> int:
        """Read one gripper register.

        Args:
            name: Register name such as ``"POS"``, ``"STA"`` or
                ``"OBJ"``.

        Returns:
            The integer value of the register.

        Raises:
            GripperError: If the reply does not parse as ``NAME value``.
        """
        self._require_gripper()
        reply = self._gripper_exchange(f"GET {name}")
        parts = reply.split()
        if len(parts) != 2 or parts[0] != name:
            raise GripperError(
                f"malformed reply while reading {name}",
                interface="gripper",
                response=reply,
            )
        try:
            return int(parts[1])
        except ValueError as exc:
            raise GripperError(
                f"non-numeric value while reading {name}",
                interface="gripper",
                response=reply,
            ) from exc

    def gripper_set(self, name: str, value: int) -> None:
        """Write one gripper register.

        Args:
            name: Register name such as ``"POS"`` or ``"GTO"``.
            value: Value to write, 0 to 255.

        Raises:
            GripperError: If the gripper does not acknowledge.
        """
        self._require_gripper()
        reply = self._gripper_exchange(f"SET {name} {int(value)}")
        if reply.lower() != "ack":
            raise GripperError(
                f"gripper did not acknowledge SET {name}",
                interface="gripper",
                response=reply,
            )

    def gripper_check_fault(self, tolerated: tuple[int, ...] = (0,)) -> int:
        """Raise if the gripper reports a fault worth stopping for.

        Args:
            tolerated: Fault codes to accept without raising. During
                activation the "action delayed" and "activation bit
                must be set" codes are expected and pass through.

        Returns:
            The fault code that was read.

        Raises:
            GripperError: If the FLT register holds a code outside
                ``tolerated``.
        """
        fault = self.gripper_get("FLT")
        if fault not in tolerated:
            raise GripperError(
                f"gripper reports fault {fault}: "
                f"{gripper_fault_meanings.get(fault, 'unknown code')}",
                interface="gripper",
                response=f"FLT {fault}",
            )
        return fault

    def gripper_activate(self, force_reset: bool = False) -> None:
        """Run the Robotiq activation sequence.

        Activation is required once after the gripper powers up. It is
        skipped when the gripper already reports the activated status
        and no reset was asked for, because a reset drives the fingers
        and would be an unexpected motion.

        Args:
            force_reset: Clear the activation bit first, forcing the
                gripper through a full re-activation.

        Raises:
            GripperError: On a fault code, or if activation does not
                complete within the timeout.
        """
        if not force_reset and self.gripper_get("STA") == status_activated:
            self.gripper_set("GTO", 1)
            return

        self.gripper_set("ACT", 0)
        time.sleep(0.1)
        self.gripper_set("ACT", 1)

        # GTO must wait until activation reports finished. Requesting
        # motion first makes the gripper answer FLT 7, "the activation
        # bit must be set prior to performing action", and refuse.
        deadline = time.monotonic() + gripper_activation_timeout
        while time.monotonic() < deadline:
            self.gripper_check_fault(activation_faults)
            if self.gripper_get("STA") == status_activated:
                self.gripper_set("GTO", 1)
                return
            time.sleep(watch_interval * 10)
        raise GripperError(
            f"activation did not finish within {gripper_activation_timeout} s",
            interface="gripper",
            response=f"STA {self.gripper_get('STA')}",
        )

    def gripper_move(
        self,
        position: int,
        speed: int = gripper_value_max,
        force: int = 128,
        wait: bool = True,
        timeout: float = gripper_move_timeout,
    ) -> tuple[int, int]:
        """Command the fingers to a position.

        Args:
            position: Target position, 0 open to 255 closed.
            speed: Travel speed, 0 to 255.
            force: Grip force, 0 to 255.
            wait: Block until the gripper stops moving.
            timeout: Seconds to wait when ``wait`` is true.

        Returns:
            Tuple of the final position and the OBJ status.

        Raises:
            GripperNotActivatedError: If the gripper is not activated.
            GripperError: On a fault code or a wait timeout.
        """
        if self.gripper_get("STA") != status_activated:
            raise GripperNotActivatedError(
                "gripper must be activated before moving; call "
                "gripper_activate() first",
                interface="gripper",
                response=f"STA {self.gripper_get('STA')}",
            )

        self.gripper_set("SPE", speed)
        self.gripper_set("FOR", force)
        self.gripper_set("POS", position)
        self.gripper_set("GTO", 1)

        if not wait:
            return self.gripper_get("POS"), self.gripper_get("OBJ")

        # The gripper echoes the target it accepted in PRE. Polling OBJ
        # before that echo arrives reads the status left over from the
        # previous move, so a fresh command looks finished the instant
        # it is issued. Waiting for the echo closes that race. The echo
        # may never match exactly when the gripper clamps the target to
        # its calibrated range, so the wait is bounded rather than
        # unconditional.
        echo_deadline = time.monotonic() + gripper_echo_timeout
        while time.monotonic() < echo_deadline:
            if self.gripper_get("PRE") == position:
                break
            time.sleep(watch_interval)

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            status = self.gripper_get("OBJ")
            if status != object_moving:
                return self.gripper_get("POS"), status
            time.sleep(watch_interval)
        raise GripperError(
            f"gripper still moving after {timeout} s",
            interface="gripper",
            response=f"POS {self.gripper_get('POS')}",
        )

    def gripper_open(
        self,
        speed: int = gripper_value_max,
        force: int = 128,
        wait: bool = True,
    ) -> tuple[int, int]:
        """Open the fingers fully.

        Returns:
            Tuple of the final position and the OBJ status.
        """
        return self.gripper_move(gripper_open_position, speed, force, wait)

    def gripper_close(
        self,
        speed: int = gripper_value_max,
        force: int = 128,
        wait: bool = True,
    ) -> tuple[int, int]:
        """Close the fingers fully.

        Returns:
            Tuple of the final position and the OBJ status.
        """
        return self.gripper_move(gripper_closed_position, speed, force, wait)

    def gripper_position(self) -> int:
        """Read the current finger position.

        Returns:
            Position 0 open to 255 closed.
        """
        return self.gripper_get("POS")

    def gripper_object_detected(self) -> bool:
        """Report whether the fingers stopped on an object.

        Returns:
            True when the OBJ register reports 1 or 2.
        """
        return self.gripper_get("OBJ") in (
            object_detected_opening,
            object_detected_closing,
        )

    # =================================================================
    # Wrist camera (specification section 4.6)
    # =================================================================

    def _camera_url(self, path: str, image_type: str) -> str:
        """Build a full URL for one candidate camera path.

        Returns:
            The absolute URL to request.
        """
        return (
            f"http://{self.robot_ip}:{camera_port}"
            f"{path.format(image_type=image_type)}"
        )

    def _camera_fetch(self, image_type: str) -> bytes:
        """Download the current frame as encoded JPEG bytes.

        The Vision URCap image path is not part of any published API,
        so a short list of known paths is probed on first use and the
        one that answered is remembered.

        Returns:
            Raw JPEG bytes.

        Raises:
            CameraError: If no candidate path returned image data.
        """
        paths = (
            (self._camera_path,)
            if self._camera_path is not None
            else camera_candidate_paths
        )
        failures = []
        for path in paths:
            url = self._camera_url(path, image_type)
            # The URCap occasionally stalls past the timeout when
            # frames are requested back to back, then serves the next
            # request normally. One retry costs little and turns that
            # stall into a hiccup instead of a failed capture.
            for attempt in range(camera_attempts):
                try:
                    response = self._session.get(url, timeout=camera_timeout)
                except requests.RequestException as exc:
                    failures.append(
                        f"{url}: {type(exc).__name__} (attempt {attempt + 1})"
                    )
                    continue
                if response.status_code == 200 and response.content:
                    self._camera_path = path
                    return response.content
                failures.append(f"{url}: HTTP {response.status_code}")
                break

        # A cached path that stops working is worth retrying from
        # scratch on the next call.
        self._camera_path = None
        raise CameraError(
            f"no image returned from {self.robot_ip}:{camera_port}; "
            f"check that the Wrist Camera URCap is installed and that "
            f"its dashboard is reachable",
            interface="camera",
            response="; ".join(failures),
        )

    def camera_frame(self, image_type: str = "color") -> np.ndarray:
        """Fetch one wrist camera frame and decode it.

        Args:
            image_type: One of ``"color"``, ``"edges"``,
                ``"magnitude"`` or ``"annotations"``.

        Returns:
            The frame as a BGR ``numpy`` array.

        Raises:
            ValueError: If ``image_type`` is not a known type.
            ConnectionFailedError: If the camera is disabled.
            CameraError: If the request or the decode failed.
        """
        if not self.camera_enabled:
            raise ConnectionFailedError(
                "the camera is disabled; construct URRobotController "
                "with camera_enabled=True",
                interface="camera",
            )
        if image_type not in valid_image_types:
            raise ValueError(
                f"image_type must be one of {valid_image_types}, "
                f"got {image_type!r}"
            )
        payload = self._camera_fetch(image_type)
        image = cv2.imdecode(
            np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_COLOR
        )
        if image is None:
            raise CameraError(
                f"received {len(payload)} bytes that do not decode as an image",
                interface="camera",
                response=payload[:32].hex(),
            )
        self._camera_ok = True
        return image

    def camera_snapshot(self, path: str, image_type: str = "color") -> bool:
        """Fetch one frame and write it to disk.

        Args:
            path: Destination file path. The extension selects the
                encoder used by OpenCV.
            image_type: Image type to request.

        Returns:
            True when the file was written.

        Raises:
            CameraError: If the frame could not be fetched.
        """
        return bool(cv2.imwrite(path, self.camera_frame(image_type)))

    def camera_live(
        self, image_type: str = "color", poll_interval: float = 0.2
    ) -> None:
        """Show a polled live view until ``q`` is pressed.

        The camera serves single frames, so this is a polling loop
        rather than a stream. Polling faster than 100 ms only adds
        load without adding frames.

        Args:
            image_type: Image type to request.
            poll_interval: Seconds between requests.

        Raises:
            ValueError: If ``poll_interval`` is below 100 ms.
            CameraError: If a frame could not be fetched.
        """
        if poll_interval < min_poll_interval:
            raise ValueError(
                f"poll_interval must be at least {min_poll_interval} s, "
                f"got {poll_interval}"
            )
        window = f"wrist camera ({image_type})"
        try:
            while True:
                cv2.imshow(window, self.camera_frame(image_type))
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
                time.sleep(poll_interval)
        finally:
            cv2.destroyWindow(window)

    def camera_ok(self) -> bool:
        """Check the camera endpoint without raising.

        Returns:
            True when a frame could be fetched and decoded.
        """
        if not self.camera_enabled:
            return False
        try:
            self.camera_frame("color")
        except (CameraError, ConnectionFailedError, ValueError):
            self._camera_ok = False
        return self._camera_ok

    # =================================================================
    # Stored programs (specification section 4.7)
    # =================================================================

    def _connect_dashboard(self) -> None:
        """Open the Dashboard socket and consume its greeting.

        Raises:
            ConnectionFailedError: If the Dashboard server does not
                accept the connection.
        """
        try:
            sock = socket.create_connection(
                (self.robot_ip, dashboard_port),
                timeout=self.connect_timeout,
            )
            sock.settimeout(dashboard_timeout)
            # The server sends "Connected: Universal Robots Dashboard
            # Server" before accepting commands. Leaving it in the
            # buffer would offset every later reply by one line.
            self.dashboard_banner = (
                sock.recv(socket_buffer_bytes).decode(errors="replace").strip()
            )
        except OSError as exc:
            raise ConnectionFailedError(
                f"cannot reach the Dashboard server on "
                f"{self.robot_ip}:{dashboard_port}",
                interface="dashboard",
                response=str(exc),
            ) from exc
        self._dashboard_socket = sock

    def dashboard_command(self, line: str) -> str:
        """Send one Dashboard command and return its reply.

        Args:
            line: Command text without its terminator.

        Returns:
            The reply line, stripped of whitespace.

        Raises:
            ProgramError: If the socket is closed or the exchange
                failed.
        """
        with self._dashboard_lock:
            if self._dashboard_socket is None:
                raise ProgramError(
                    "Dashboard socket is not open",
                    interface="dashboard",
                )
            try:
                self._dashboard_socket.sendall(f"{line}\n".encode())
                reply = self._dashboard_socket.recv(socket_buffer_bytes)
            except OSError as exc:
                raise ProgramError(
                    f"Dashboard command {line!r} failed",
                    interface="dashboard",
                    response=str(exc),
                ) from exc
        return reply.decode(errors="replace").strip()

    def _checked_dashboard_command(self, line: str) -> str:
        """Send a Dashboard command and reject a failure reply.

        Returns:
            The reply line.

        Raises:
            ProgramError: If the reply contains a failure marker.
        """
        reply = self.dashboard_command(line)
        lowered = reply.lower()
        if any(marker in lowered for marker in failure_markers):
            raise ProgramError(
                f"Dashboard refused {line!r}",
                interface="dashboard",
                response=reply,
            )
        return reply

    def program_file_name(self, name: str) -> str:
        """Apply the generation-specific stored program naming rule.

        PolyScope 5 expects ``name.urp``; PolyScope X expects the bare
        name. Callers pass whichever form they have and this normalises
        it.

        Args:
            name: Program name, with or without the extension.

        Returns:
            The name in the form the controller expects.
        """
        if self.polyscope_x:
            return name[: -len(".urp")] if name.endswith(".urp") else name
        return name if name.endswith(".urp") else f"{name}.urp"

    def run_program(self, name: str, wait_until_done: bool = False) -> None:
        """Load and start a stored controller program.

        RTDE external control is released first, because the two
        cannot own the robot at the same time.

        Args:
            name: Stored program name. The extension is adjusted to
                the PolyScope generation configured on the class.
            wait_until_done: Block until the program stops running.

        Raises:
            ProgramError: If the program failed to load or start, or
                if it did not start within the mode switch timeout.
        """
        self.release_motion()
        self._checked_dashboard_command(f"load {self.program_file_name(name)}")
        self._checked_dashboard_command("play")

        deadline = time.monotonic() + mode_switch_timeout
        while time.monotonic() < deadline:
            if self.program_running():
                self._mode = Mode.program
                break
            time.sleep(watch_interval)
        else:
            raise ProgramError(
                f"program {name!r} did not start within "
                f"{mode_switch_timeout} s",
                interface="dashboard",
                response=self.loaded_program(),
            )

        if wait_until_done:
            while self.program_running():
                time.sleep(watch_interval * 10)
            self._mode = Mode.idle

    def stop_program(self) -> None:
        """Stop the running stored program and return to IDLE."""
        self.dashboard_command("stop")
        self._mode = Mode.idle

    def pause_program(self) -> None:
        """Pause the running stored program."""
        self.dashboard_command("pause")

    def program_running(self) -> bool:
        """Report whether a stored program is running.

        Returns:
            True when the Dashboard answers ``Program running: true``.
        """
        return "true" in self.dashboard_command("running").lower()

    def loaded_program(self) -> str:
        """Report the currently loaded stored program.

        Returns:
            The Dashboard reply describing the loaded program.
        """
        return self.dashboard_command("get loaded program")

    def send_script_file(self, path: str) -> None:
        """Stream a URScript file to the Secondary Client Interface.

        The interface is fire and forget: the controller never
        acknowledges, so a clean return only means the bytes were
        accepted by the socket.

        Args:
            path: Path to a ``.script`` file on the local machine.

        Raises:
            ProgramError: If the file cannot be read or sent.
        """
        try:
            with open(path, "rb") as handle:
                payload = handle.read()
        except OSError as exc:
            raise ProgramError(
                f"cannot read script file {path!r}",
                interface="secondary",
                response=str(exc),
            ) from exc

        if not payload.endswith(b"\n"):
            payload += b"\n"
        try:
            with socket.create_connection(
                (self.robot_ip, secondary_port),
                timeout=self.connect_timeout,
            ) as sock:
                sock.sendall(payload)
        except OSError as exc:
            raise ProgramError(
                f"cannot send script to {self.robot_ip}:{secondary_port}",
                interface="secondary",
                response=str(exc),
            ) from exc

    # -- controller status --------------------------------------------

    def robot_mode(self) -> str:
        """Report the controller robot mode.

        Returns:
            The Dashboard reply, such as ``Robotmode: RUNNING``.
        """
        return self.dashboard_command("robotmode")

    def safety_status(self) -> str:
        """Report the controller safety status.

        Returns:
            The Dashboard reply, such as ``Safetystatus: NORMAL``.
        """
        return self.dashboard_command("safetystatus")

    def remote_control(self) -> bool:
        """Report whether the pendant is in Remote Control.

        An e-Series controller refuses externally supplied scripts
        while it is in local control, so this is the first thing to
        check when RTDE control will not connect.

        Returns:
            True when external motion commands are permitted.
        """
        return "true" in self.dashboard_command("is in remote control").lower()

    def operational_mode(self) -> str:
        """Report the controller operational mode.

        Returns:
            The Dashboard reply, such as ``MANUAL``.
        """
        return self.dashboard_command("get operational mode")

    def polyscope_version(self) -> str:
        """Report the PolyScope software version.

        Returns:
            The Dashboard reply, such as ``URSoftware 5.25.2``.
        """
        return self.dashboard_command("PolyscopeVersion")
