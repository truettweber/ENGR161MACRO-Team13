#!/usr/bin/env python3
"""Simple straight-line following for the ENGR 161 MACRO robot.

AI-assisted development: OpenAI Codex, 2026-10-30.
This program was developed from the team's stated geometry and the project
documents in this repository. No external algorithm implementation was copied.

Hardware assumptions
--------------------
* Two SPIKE Large Angular Motors are mechanically coupled to one drivetrain.
* One SPIKE Small Angular Motor drives a rack-and-pinion steering system.
* The steering position at startup is treated as straight. +/-90 motor degrees
  from that center maps to +/-26.16 wheel degrees and is the commanded limit.
* A Grove button or keyboard Alt key toggles autonomous operation after
  motor initialization. Arrow keys immediately override autonomy for manual
  driving.

The autonomous controller is deliberately limited to a four-state bang-bang
response for one straight, unbroken black line. The IMU is not initialized or
read. Ctrl+C always exits and stops every motor through the main function's
finally block.
"""

from __future__ import annotations

import argparse
import csv
import fcntl
import math
import os
import select
import signal
import struct
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from enum import Enum
from pathlib import Path
from typing import NoReturn, Optional, Sequence, TextIO


SCRIPT_PATH = Path(__file__).resolve()


def project_library_candidates() -> list[Path]:
    """Return likely locations of the course-provided ``lib`` directory.

    Students commonly run this file from the repository, copy it to the
    repository root, or open a copy from the Raspberry Pi desktop. Do not rely
    on one fixed number of parent directories.
    """

    candidates: list[Path] = []
    for starting_directory in (SCRIPT_PATH.parent, Path.cwd().resolve()):
        for directory in (starting_directory, *starting_directory.parents):
            candidates.append(directory / "lib")

    home = Path.home()
    candidates.extend(
        (
            home / "Desktop" / "lib",
            home / "Desktop" / "ENGR161MACRO-Team13" / "lib",
            home / "ENGR161MACRO-Team13" / "lib",
        )
    )

    unique_candidates: list[Path] = []
    for candidate in candidates:
        if candidate not in unique_candidates:
            unique_candidates.append(candidate)
    return unique_candidates


def is_project_library(directory: Path) -> bool:
    return (
        (directory / "buildhat" / "__init__.py").is_file()
        and (directory / "basehat" / "__init__.py").is_file()
        and (directory / "basehat" / "LineFinder.py").is_file()
    )


def configure_project_library(explicit_directory: Optional[Path] = None) -> Optional[Path]:
    """Add the course library to ``sys.path`` and return its resolved path."""

    if explicit_directory is not None:
        candidate = explicit_directory.expanduser().resolve()
        if not is_project_library(candidate):
            raise ValueError(
                f"{candidate} is not the project lib directory; expected "
                "buildhat/__init__.py, basehat/__init__.py, "
                "and basehat/LineFinder.py inside it"
            )
        selected = candidate
    else:
        selected = next(
            (
                candidate
                for candidate in project_library_candidates()
                if is_project_library(candidate)
            ),
            None,
        )

    if selected is not None and str(selected) not in sys.path:
        sys.path.insert(0, str(selected))
    return selected


PROJECT_LIBRARY_DIRECTORY = configure_project_library()


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def move_toward(value: float, target: float, maximum_change: float) -> float:
    return value + clamp(target - value, -maximum_change, maximum_change)


def wrap_degrees(angle: float) -> float:
    return (angle + 180.0) % 360.0 - 180.0


def line_value_is_black(value: float) -> bool:
    """Convert the Grove signal convention: 0 is black and 1 is white."""

    return float(value) < 0.5


def require_direction(value: str) -> int:
    parsed = int(value)
    if parsed not in (-1, 1):
        raise argparse.ArgumentTypeError("direction must be either -1 or 1")
    return parsed


class ControllerState(Enum):
    WAITING = "waiting"
    MANUAL = "manual"
    ACQUIRE = "acquire"
    TRACK = "track"
    FAULT = "fault"


@dataclass(frozen=True)
class RobotConfig:
    # Robot geometry
    wheelbase_in: float = 5.5
    steering_motor_limit_deg: float = 90.0
    wheel_deg_at_90_motor_deg: float = 26.16

    # Loop and sensor filtering
    loop_hz: float = 40.0
    line_debounce_s: float = 0.010

    # Regulated motor commands, on the Build HAT -100..100 scale
    cruise_drive_command: float = 6.0
    curve_drive_command: float = 4.4

    # Manual arrow-key control
    manual_drive_command: float = 5.0
    manual_steering_fraction: float = 0.75

    # Straight-line bang-bang recovery
    line_loss_stop_s: float = 1.0

    # Steering motor position loop
    steering_position_kp: float = 1.75
    steering_min_command: float = 8.0
    steering_max_command: float = 70.0
    steering_deadband_motor_deg: float = 1.0
    steering_emergency_limit_deg: float = 90.0

    # Coupled-motor diagnostic
    motor_disagreement_limit_deg: float = 90.0
    motor_disagreement_trigger_s: float = 0.50

    @property
    def maximum_wheel_angle_deg(self) -> float:
        return (
            self.steering_motor_limit_deg
            * self.wheel_deg_at_90_motor_deg
            / 90.0
        )

@dataclass(frozen=True)
class HardwareConfig:
    drive_motor_1_port: str
    drive_motor_2_port: str
    steering_motor_port: str
    left_line_pin: int
    right_line_pin: int
    button_pin: int
    inches_per_motor_rotation: float
    drive_motor_1_direction: int = 1
    drive_motor_2_direction: int = 1
    steering_direction: int = 1

    def validate(self) -> None:
        ports = (
            self.drive_motor_1_port.upper(),
            self.drive_motor_2_port.upper(),
            self.steering_motor_port.upper(),
        )
        if any(port not in {"A", "B", "C", "D"} for port in ports):
            raise ValueError("Build HAT motor ports must be A, B, C, or D")
        if len(set(ports)) != 3:
            raise ValueError("all three motors must use different Build HAT ports")
        if self.left_line_pin == self.right_line_pin:
            raise ValueError("left and right line finders must use different pins")
        if self.button_pin in {self.left_line_pin, self.right_line_pin}:
            raise ValueError("the button and line finders must use different pins")
        if self.inches_per_motor_rotation <= 0:
            raise ValueError("inches per motor rotation must be greater than zero")
        for direction in (
            self.drive_motor_1_direction,
            self.drive_motor_2_direction,
            self.steering_direction,
        ):
            if direction not in (-1, 1):
                raise ValueError("all direction values must be -1 or 1")

    @property
    def inches_per_motor_degree(self) -> float:
        return self.inches_per_motor_rotation / 360.0


@dataclass(frozen=True)
class Observation:
    now: float
    dt: float
    line_left: bool
    line_right: bool
    distance_in: float
    odometer_in: float
    speed_ips: float
    drive_speed_1: float
    drive_speed_2: float
    drive_motor_disagreement_deg: float
    steering_motor_deg: float
    yaw_deg: float
    yaw_rate_dps: float
    pitch_deg: float
    roll_deg: float
    acceleration_deviation_mps2: float


@dataclass(frozen=True)
class ControlOutput:
    state: ControllerState
    drive_command: float
    steering_wheel_deg: float
    curvature_per_in: float
    line_confidence: float
    reason: str = ""


class OperatingMode(Enum):
    WAITING = "waiting"
    AUTONOMOUS = "autonomous"
    STRAIGHT_DRIVE = "straight_drive"
    MANUAL = "manual"


@dataclass(frozen=True)
class KeyboardSnapshot:
    alt_pressed: bool = False
    arrow_activity: bool = False
    emergency_stop: bool = False
    up: bool = False
    down: bool = False
    left: bool = False
    right: bool = False


class LinuxKeyboardInput:
    """Read global Alt and arrow-key events without an external package.

    A standalone Alt press is not sent through terminal stdin, including the
    Thonny shell. Linux input-event devices expose both presses and releases,
    which also lets manual commands stop immediately when an arrow is released.
    """

    EVENT = struct.Struct("@llHHi")
    EV_KEY = 0x01
    KEY_C = 46
    KEY_LEFTCTRL = 29
    KEY_RIGHTCTRL = 97
    KEY_LEFTALT = 56
    KEY_RIGHTALT = 100
    ARROW_CODES = {
        103: "up",
        108: "down",
        105: "left",
        106: "right",
    }

    # Linux ioctl encoding from asm-generic/ioctl.h. EVIOCGRAB prevents Thonny
    # or the desktop from also consuming arrows while this process owns them.
    IOC_WRITE = 1
    IOC_READ = 2
    EVIOCGRAB = (
        (IOC_WRITE << 30)
        | (struct.calcsize("i") << 16)
        | (ord("E") << 8)
        | 0x90
    )
    KEY_BITMAP_BYTES = 128

    def __init__(self, device_paths: Optional[Sequence[Path]] = None):
        paths = (
            list(device_paths)
            if device_paths is not None
            else sorted(Path("/dev/input").glob("event*"))
        )
        self._fds: list[int] = []
        self._paths: list[Path] = []
        self._grabbed_fds: set[int] = set()
        self._pressed_arrows: set[str] = set()
        self._pressed_ctrl: set[int] = set()
        self.permission_denied = False
        self.grab_failed = False
        for path in paths:
            try:
                fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
            except PermissionError:
                self.permission_denied = True
                continue
            except OSError:
                continue
            if not self._supports_robot_keys(fd):
                os.close(fd)
                continue
            try:
                fcntl.ioctl(fd, self.EVIOCGRAB, 1)
            except OSError:
                self.grab_failed = True
            else:
                self._grabbed_fds.add(fd)
            self._fds.append(fd)
            self._paths.append(path)

    @classmethod
    def _eviocgbit(cls, event_type: int, length: int) -> int:
        return (
            (cls.IOC_READ << 30)
            | (length << 16)
            | (ord("E") << 8)
            | (0x20 + event_type)
        )

    @classmethod
    def _supports_robot_keys(cls, fd: int) -> bool:
        bitmap = bytearray(cls.KEY_BITMAP_BYTES)
        try:
            fcntl.ioctl(
                fd,
                cls._eviocgbit(cls.EV_KEY, len(bitmap)),
                bitmap,
                True,
            )
        except OSError:
            return False

        def supports(code: int) -> bool:
            return bool(bitmap[code // 8] & (1 << (code % 8)))

        return (
            all(supports(code) for code in cls.ARROW_CODES)
            and any(supports(code) for code in (cls.KEY_LEFTALT, cls.KEY_RIGHTALT))
            and supports(cls.KEY_C)
            and any(supports(code) for code in (cls.KEY_LEFTCTRL, cls.KEY_RIGHTCTRL))
        )

    @property
    def available(self) -> bool:
        return bool(self._fds)

    @property
    def paths(self) -> tuple[Path, ...]:
        return tuple(self._paths)

    @property
    def exclusive(self) -> bool:
        return bool(self._fds) and len(self._grabbed_fds) == len(self._fds)

    def poll(self) -> KeyboardSnapshot:
        alt_pressed = False
        arrow_activity = False
        emergency_stop = False
        if self._fds:
            readable, _, _ = select.select(self._fds, [], [], 0.0)
        else:
            readable = []

        for fd in readable:
            try:
                data = os.read(fd, self.EVENT.size * 32)
            except BlockingIOError:
                continue
            except OSError:
                self._remove_fd(fd)
                self._pressed_arrows.clear()
                self._pressed_ctrl.clear()
                continue

            complete_bytes = len(data) - (len(data) % self.EVENT.size)
            for offset in range(0, complete_bytes, self.EVENT.size):
                _, _, event_type, code, value = self.EVENT.unpack_from(data, offset)
                if event_type != self.EV_KEY:
                    continue
                if code in (self.KEY_LEFTCTRL, self.KEY_RIGHTCTRL):
                    if value == 1:
                        self._pressed_ctrl.add(code)
                    elif value == 0:
                        self._pressed_ctrl.discard(code)
                    continue
                if code == self.KEY_C and value == 1 and self._pressed_ctrl:
                    emergency_stop = True
                    continue
                if code in (self.KEY_LEFTALT, self.KEY_RIGHTALT):
                    if value == 1:
                        alt_pressed = True
                    continue
                arrow = self.ARROW_CODES.get(code)
                if arrow is None:
                    continue
                if value == 1:
                    arrow_activity = True
                    self._pressed_arrows.add(arrow)
                elif value == 0:
                    self._pressed_arrows.discard(arrow)

        return KeyboardSnapshot(
            alt_pressed=alt_pressed,
            arrow_activity=arrow_activity,
            emergency_stop=emergency_stop,
            up="up" in self._pressed_arrows,
            down="down" in self._pressed_arrows,
            left="left" in self._pressed_arrows,
            right="right" in self._pressed_arrows,
        )

    def _remove_fd(self, fd: int) -> None:
        try:
            index = self._fds.index(fd)
        except ValueError:
            return
        self._fds.pop(index)
        self._paths.pop(index)
        if fd in self._grabbed_fds:
            try:
                fcntl.ioctl(fd, self.EVIOCGRAB, 0)
            except OSError:
                pass
            self._grabbed_fds.discard(fd)
        try:
            os.close(fd)
        except OSError:
            pass

    def close(self) -> None:
        for fd in self._fds:
            if fd in self._grabbed_fds:
                try:
                    fcntl.ioctl(fd, self.EVIOCGRAB, 0)
                except OSError:
                    pass
            try:
                os.close(fd)
            except OSError:
                pass
        self._fds.clear()
        self._paths.clear()
        self._grabbed_fds.clear()
        self._pressed_arrows.clear()
        self._pressed_ctrl.clear()


class TkKeyboardInput:
    """Desktop keyboard-control window used when Linux event devices are absent."""

    ARROW_KEYSYMS = {
        "Up": "up",
        "Down": "down",
        "Left": "left",
        "Right": "right",
    }
    ALT_KEYSYMS = {"Alt_L", "Alt_R"}
    CTRL_KEYSYMS = {"Control_L", "Control_R"}

    def __init__(self):
        import tkinter as tk

        self._tk = tk
        self._root = tk.Tk()
        self._root.title("MACRO Robot Keyboard Control")
        self._root.geometry("560x170")
        self._root.attributes("-topmost", True)
        self._pressed_arrows: set[str] = set()
        self._pressed_alt: set[str] = set()
        self._pressed_ctrl: set[str] = set()
        self._release_jobs: dict[str, str] = {}
        self._alt_pressed = False
        self._arrow_activity = False
        self._emergency_stop = False
        self._closed = False

        tk.Label(
            self._root,
            text="MACRO Robot Keyboard Control",
            font=("TkDefaultFont", 16, "bold"),
        ).pack(pady=(14, 4))
        tk.Label(
            self._root,
            text=(
                "Keep this window focused.  Alt: toggle autonomous\n"
                "Arrow keys: manual drive/steering    Ctrl+C: stop and exit"
            ),
            justify="center",
        ).pack(pady=4)
        self._status = tk.Label(
            self._root,
            text="Keyboard ready",
            foreground="dark green",
        )
        self._status.pack(pady=4)

        self._root.bind("<KeyPress>", self._on_press)
        self._root.bind("<KeyRelease>", self._on_release)
        self._root.bind("<FocusOut>", self._on_focus_out)
        self._root.protocol("WM_DELETE_WINDOW", self._request_close)
        self._root.update_idletasks()
        self._root.deiconify()
        self._root.lift()
        self._root.focus_force()

    def _cancel_delayed_release(self, keysym: str) -> None:
        job = self._release_jobs.pop(keysym, None)
        if job is not None:
            try:
                self._root.after_cancel(job)
            except self._tk.TclError:
                pass

    def _on_press(self, event: object) -> str:
        keysym = str(getattr(event, "keysym", ""))
        self._cancel_delayed_release(keysym)
        arrow = self.ARROW_KEYSYMS.get(keysym)
        if arrow is not None:
            if arrow not in self._pressed_arrows:
                self._arrow_activity = True
            self._pressed_arrows.add(arrow)
            self._status.configure(
                text=f"Manual key held: {keysym}", foreground="dark green"
            )
        elif keysym in self.ALT_KEYSYMS:
            if keysym not in self._pressed_alt:
                self._alt_pressed = True
            self._pressed_alt.add(keysym)
            self._status.configure(
                text="Autonomous toggle requested", foreground="dark green"
            )
        elif keysym in self.CTRL_KEYSYMS:
            self._pressed_ctrl.add(keysym)
        elif keysym.lower() == "c" and self._pressed_ctrl:
            self._emergency_stop = True
            self._status.configure(text="Emergency stop requested", foreground="red")
        return "break"

    def _on_release(self, event: object) -> str:
        keysym = str(getattr(event, "keysym", ""))
        self._cancel_delayed_release(keysym)
        # X11 may synthesize release/press pairs during key repeat. Delay the
        # release briefly so a following repeat press can cancel it.
        self._release_jobs[keysym] = self._root.after(
            30,
            lambda key=keysym: self._finish_release(key),
        )
        return "break"

    def _finish_release(self, keysym: str) -> None:
        self._release_jobs.pop(keysym, None)
        arrow = self.ARROW_KEYSYMS.get(keysym)
        if arrow is not None:
            self._pressed_arrows.discard(arrow)
            if not self._pressed_arrows:
                self._status.configure(text="Keyboard ready", foreground="dark green")
        elif keysym in self.ALT_KEYSYMS:
            self._pressed_alt.discard(keysym)
        elif keysym in self.CTRL_KEYSYMS:
            self._pressed_ctrl.discard(keysym)

    def _on_focus_out(self, _event: object) -> None:
        self._clear_held_keys()
        self._status.configure(
            text="Click this window before using robot controls",
            foreground="dark orange",
        )

    def _clear_held_keys(self) -> None:
        for job in self._release_jobs.values():
            try:
                self._root.after_cancel(job)
            except self._tk.TclError:
                pass
        self._release_jobs.clear()
        self._pressed_arrows.clear()
        self._pressed_alt.clear()
        self._pressed_ctrl.clear()

    def _request_close(self) -> None:
        self._clear_held_keys()
        self._emergency_stop = True

    def poll(self) -> KeyboardSnapshot:
        try:
            self._root.update_idletasks()
            self._root.update()
        except self._tk.TclError:
            self._closed = True
            self._emergency_stop = True

        snapshot = KeyboardSnapshot(
            alt_pressed=self._alt_pressed,
            arrow_activity=self._arrow_activity,
            emergency_stop=self._emergency_stop,
            up="up" in self._pressed_arrows,
            down="down" in self._pressed_arrows,
            left="left" in self._pressed_arrows,
            right="right" in self._pressed_arrows,
        )
        self._alt_pressed = False
        self._arrow_activity = False
        self._emergency_stop = False
        return snapshot

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._clear_held_keys()
        try:
            self._root.destroy()
        except self._tk.TclError:
            pass


class DebouncedButtonToggle:
    """Emit one toggle for each stable robot-button press."""

    def __init__(self, initial_pressed: bool, now: float, stable_s: float = 0.10):
        self.stable_s = stable_s
        self._stable = bool(initial_pressed)
        self._candidate = self._stable
        self._candidate_since = now
        self._armed = not self._stable

    def update(self, pressed: bool, now: float) -> bool:
        pressed = bool(pressed)
        if pressed != self._candidate:
            self._candidate = pressed
            self._candidate_since = now
            return False
        if pressed == self._stable or now - self._candidate_since < self.stable_s:
            return False

        self._stable = pressed
        if not pressed:
            self._armed = True
            return False
        if self._armed:
            self._armed = False
            return True
        return False


@dataclass(frozen=True)
class ButtonPressEvent:
    press_edge: bool = False
    single_press: bool = False
    double_press: bool = False


class ButtonPressClassifier:
    """Classify stable Grove-button presses without starting on the first tap."""

    def __init__(
        self,
        initial_pressed: bool,
        now: float,
        double_press_window_s: float = 0.35,
    ):
        self._button = DebouncedButtonToggle(initial_pressed, now)
        self.double_press_window_s = double_press_window_s
        self._first_press_at: Optional[float] = None

    def cancel(self) -> None:
        self._first_press_at = None

    def update(self, pressed: bool, now: float) -> ButtonPressEvent:
        press_edge = self._button.update(pressed, now)
        if press_edge:
            if (
                self._first_press_at is not None
                and now - self._first_press_at <= self.double_press_window_s
            ):
                self._first_press_at = None
                return ButtonPressEvent(press_edge=True, double_press=True)
            self._first_press_at = now
            return ButtonPressEvent(press_edge=True)

        if (
            self._first_press_at is not None
            and now - self._first_press_at > self.double_press_window_s
        ):
            self._first_press_at = None
            return ButtonPressEvent(single_press=True)
        return ButtonPressEvent()


def choose_operating_mode(
    current: OperatingMode,
    toggle_autonomous: bool,
    arrow_activity: bool,
) -> OperatingMode:
    """Apply user inputs, with an arrow press taking highest priority."""

    if arrow_activity:
        return OperatingMode.MANUAL
    if toggle_autonomous:
        if current == OperatingMode.AUTONOMOUS:
            return OperatingMode.WAITING
        return OperatingMode.AUTONOMOUS
    return current


def inactive_output(state: ControllerState, reason: str) -> ControlOutput:
    return ControlOutput(
        state=state,
        drive_command=0.0,
        steering_wheel_deg=0.0,
        curvature_per_in=0.0,
        line_confidence=0.0,
        reason=reason,
    )


def manual_output(keys: KeyboardSnapshot, config: RobotConfig) -> ControlOutput:
    drive_sign = int(keys.up) - int(keys.down)
    steering_sign = int(keys.right) - int(keys.left)
    return ControlOutput(
        state=ControllerState.MANUAL,
        drive_command=config.manual_drive_command * drive_sign,
        steering_wheel_deg=(
            config.maximum_wheel_angle_deg
            * config.manual_steering_fraction
            * steering_sign
        ),
        curvature_per_in=0.0,
        line_confidence=0.0,
        reason="arrow-key manual control",
    )


def straight_drive_output(config: RobotConfig) -> ControlOutput:
    return ControlOutput(
        state=ControllerState.TRACK,
        drive_command=config.cruise_drive_command,
        steering_wheel_deg=0.0,
        curvature_per_in=0.0,
        line_confidence=0.0,
        reason="double-press straight-drive mode",
    )


class LineDebouncer:
    """Debounce each line sensor independently to preserve departure order."""

    def __init__(self, debounce_s: float):
        self.debounce_s = debounce_s
        self._stable = [False, False]
        self._candidate = [False, False]
        self._candidate_since = [0.0, 0.0]
        self._initialized = False

    def update(self, left: bool, right: bool, now: float) -> tuple[bool, bool]:
        pair = (bool(left), bool(right))
        if not self._initialized:
            self._stable = list(pair)
            self._candidate = list(pair)
            self._candidate_since = [now, now]
            self._initialized = True
            return tuple(self._stable)
        for index, value in enumerate(pair):
            if value != self._candidate[index]:
                self._candidate[index] = value
                self._candidate_since[index] = now
            elif (
                value != self._stable[index]
                and now - self._candidate_since[index] >= self.debounce_s
            ):
                self._stable[index] = value
        return tuple(self._stable)


class LineFollowerController:
    """Simple bang-bang controller for one straight, unbroken black line."""

    def __init__(self, config: RobotConfig):
        self.config = config
        self.state = ControllerState.ACQUIRE
        self.reason = "waiting for a line at startup"
        self.last_line_sign = 0
        self.line_lost_since: Optional[float] = None
        self.disagreement_s = 0.0
        self.last_drive_command = 0.0

    @staticmethod
    def _line_sign(obs: Observation) -> int:
        if obs.line_left and not obs.line_right:
            return -1
        if obs.line_right and not obs.line_left:
            return 1
        return 0

    def _output(
        self,
        drive: float,
        wheel_angle: float,
        confidence: float,
    ) -> ControlOutput:
        wheel_angle = clamp(
            wheel_angle,
            -self.config.maximum_wheel_angle_deg,
            self.config.maximum_wheel_angle_deg,
        )
        curvature = math.tan(math.radians(wheel_angle)) / self.config.wheelbase_in
        self.last_drive_command = drive
        return ControlOutput(
            state=self.state,
            drive_command=drive,
            steering_wheel_deg=wheel_angle,
            curvature_per_in=curvature,
            line_confidence=confidence,
            reason=self.reason,
        )

    def update(self, obs: Observation) -> ControlOutput:
        if (
            abs(self.last_drive_command) >= self.config.curve_drive_command
            and obs.drive_motor_disagreement_deg
            > self.config.motor_disagreement_limit_deg
        ):
            self.disagreement_s += obs.dt
        else:
            self.disagreement_s = 0.0

        if self.disagreement_s >= self.config.motor_disagreement_trigger_s:
            self.state = ControllerState.FAULT
            self.reason = "coupled drive motor encoders disagree"

        if self.state == ControllerState.FAULT:
            return self._output(0.0, 0.0, 0.0)

        line_sign = self._line_sign(obs)

        if obs.line_left and obs.line_right:
            self.state = ControllerState.TRACK
            self.reason = "both sensors see black; driving straight"
            self.line_lost_since = None
            return self._output(self.config.cruise_drive_command, 0.0, 1.0)

        if line_sign != 0:
            self.state = ControllerState.TRACK
            self.last_line_sign = line_sign
            self.line_lost_since = None
            direction = "left" if line_sign < 0 else "right"
            self.reason = (
                f"only the {direction} sensor sees black; "
                f"commanding full {direction} steering"
            )
            return self._output(
                self.config.curve_drive_command,
                line_sign * self.config.maximum_wheel_angle_deg,
                0.75,
            )

        if self.state == ControllerState.ACQUIRE or self.last_line_sign == 0:
            self.state = ControllerState.ACQUIRE
            self.reason = "waiting for a line at startup"
            self.line_lost_since = None
            return self._output(0.0, 0.0, 0.0)

        if self.line_lost_since is None:
            self.line_lost_since = obs.now
        lost_for = max(0.0, obs.now - self.line_lost_since)
        if lost_for >= self.config.line_loss_stop_s:
            self.state = ControllerState.FAULT
            self.reason = "line remained lost; stopping instead of steering indefinitely"
            return self._output(0.0, 0.0, 0.0)

        self.state = ControllerState.TRACK
        direction = "left" if self.last_line_sign < 0 else "right"
        self.reason = (
            f"both sensors are white; continuing full {direction} correction "
            "toward the last observed line side"
        )
        confidence = clamp(1.0 - lost_for / self.config.line_loss_stop_s, 0.0, 1.0)
        return self._output(
            self.config.curve_drive_command,
            self.last_line_sign * self.config.maximum_wheel_angle_deg,
            confidence,
        )


class HardwareRobot:
    """Build HAT, Grove line-sensor, and button adapter."""

    def __init__(self, hardware: HardwareConfig, config: RobotConfig):
        hardware.validate()
        self.hardware = hardware
        self.config = config

        try:
            from buildhat import Motor
        except ImportError as error:
            self._raise_import_error("Build HAT motor driver", error)

        try:
            from basehat.LineFinder import LineFinder
        except ImportError as error:
            self._raise_import_error("Grove line-finder driver", error)

        try:
            from gpiozero import Button as GpioButton
        except ImportError as error:
            self._raise_import_error("Grove push-button driver", error)

        print("Initializing Build HAT motors; this can take about 10 seconds...")
        self.drive_motor_1 = Motor(hardware.drive_motor_1_port.upper())
        self.drive_motor_2 = Motor(hardware.drive_motor_2_port.upper())
        self.steering_motor = Motor(hardware.steering_motor_port.upper())
        self.left_line = LineFinder(hardware.left_line_pin)
        self.right_line = LineFinder(hardware.right_line_pin)
        self.start_stop_button = GpioButton(
            hardware.button_pin,
            # The Grove module drives its digital output HIGH while pressed.
            # pull_up=False makes gpiozero treat HIGH as the active state.
            pull_up=False,
            bounce_time=0.05,
        )
        self._last_drive_command: Optional[float] = None
        self._last_steering_command: Optional[float] = None
        self._last_steering_position = 0.0
        self._steering_center_absolute = 0.0
        self._last_time = time.monotonic()
        self._last_average_position = 0.0
        self._position_1_zero = 0.0
        self._position_2_zero = 0.0
        self._distance_in = 0.0
        self._odometer_in = 0.0
        self._filtered_speed_ips = 0.0
        self.stop_all()
        self._initialize_encoder_reference()

    @staticmethod
    def _raise_import_error(component: str, error: ImportError) -> NoReturn:
        missing_module = getattr(error, "name", None) or "unknown"
        selected = (
            str(PROJECT_LIBRARY_DIRECTORY)
            if PROJECT_LIBRARY_DIRECTORY is not None
            else "none found automatically"
        )
        raise RuntimeError(
            f"Unable to import the {component}; missing module: "
            f"{missing_module!r}. Project library directory: {selected}. "
            "Keep the full repository together or pass "
            "--library-directory /path/to/ENGR161MACRO-Team13/lib. "
            f"Original import error: {error}"
        ) from error

    @staticmethod
    def _motor_values(motor: object) -> tuple[float, float, float]:
        values = motor.get()
        if len(values) < 3:
            raise RuntimeError("an angular motor with an absolute encoder is required")
        return float(values[0]), float(values[1]), float(values[2])

    def _initialize_encoder_reference(self) -> None:
        _, position_1, _ = self._motor_values(self.drive_motor_1)
        _, position_2, _ = self._motor_values(self.drive_motor_2)
        position_1 *= self.hardware.drive_motor_1_direction
        position_2 *= self.hardware.drive_motor_2_direction
        self._position_1_zero = position_1
        self._position_2_zero = position_2
        self._last_average_position = 0.5 * (position_1 + position_2)

        _, _, steering_absolute = self._motor_values(self.steering_motor)
        self._steering_center_absolute = steering_absolute
        self._last_steering_position = 0.0
        print(
            f"Steering center recorded at absolute motor position "
            f"{steering_absolute:.1f} degrees."
        )

    def prepare_for_control(self) -> None:
        self._last_time = time.monotonic()
        print("Straight-line sensors ready; IMU is not used.")

    def read(self, debouncer: LineDebouncer) -> Observation:
        now = time.monotonic()
        dt = clamp(now - self._last_time, 1e-3, 0.25)
        self._last_time = now

        speed_1, position_1, _ = self._motor_values(self.drive_motor_1)
        speed_2, position_2, _ = self._motor_values(self.drive_motor_2)
        _, _, steering_absolute = self._motor_values(self.steering_motor)
        position_1 *= self.hardware.drive_motor_1_direction
        position_2 *= self.hardware.drive_motor_2_direction
        speed_1 *= self.hardware.drive_motor_1_direction
        speed_2 *= self.hardware.drive_motor_2_direction
        steering = (
            wrap_degrees(steering_absolute - self._steering_center_absolute)
            * self.hardware.steering_direction
        )

        average_position = 0.5 * (position_1 + position_2)
        delta_degrees = average_position - self._last_average_position
        self._last_average_position = average_position
        delta_distance = delta_degrees * self.hardware.inches_per_motor_degree
        self._distance_in += delta_distance
        self._odometer_in += abs(delta_distance)
        instantaneous_speed = delta_distance / dt
        self._filtered_speed_ips = 0.75 * self._filtered_speed_ips + 0.25 * instantaneous_speed

        normalized_1 = position_1 - self._position_1_zero
        normalized_2 = position_2 - self._position_2_zero
        disagreement = abs(normalized_1 - normalized_2)

        # Grove line finders output 0 over black and 1 over white. Controller
        # booleans mean "black line detected", so explicitly invert the raw
        # digital values instead of relying on GPIO active-state semantics.
        raw_left = line_value_is_black(self.left_line.value)
        raw_right = line_value_is_black(self.right_line.value)
        line_left, line_right = debouncer.update(raw_left, raw_right, now)
        self._last_steering_position = steering

        return Observation(
            now=now,
            dt=dt,
            line_left=line_left,
            line_right=line_right,
            distance_in=self._distance_in,
            odometer_in=self._odometer_in,
            speed_ips=self._filtered_speed_ips,
            drive_speed_1=speed_1,
            drive_speed_2=speed_2,
            drive_motor_disagreement_deg=disagreement,
            steering_motor_deg=steering,
            yaw_deg=0.0,
            yaw_rate_dps=0.0,
            pitch_deg=0.0,
            roll_deg=0.0,
            acceleration_deviation_mps2=0.0,
        )

    def apply(self, output: ControlOutput) -> None:
        drive = clamp(output.drive_command, -100.0, 100.0)
        if self._last_drive_command is None or abs(drive - self._last_drive_command) >= 0.5:
            self.drive_motor_1.start(
                drive * self.hardware.drive_motor_1_direction
            )
            self.drive_motor_2.start(
                drive * self.hardware.drive_motor_2_direction
            )
            self._last_drive_command = drive

        target_motor_deg = (
            output.steering_wheel_deg
            * 90.0
            / self.config.wheel_deg_at_90_motor_deg
        )
        target_motor_deg = clamp(
            target_motor_deg,
            -self.config.steering_motor_limit_deg,
            self.config.steering_motor_limit_deg,
        )
        error = target_motor_deg - self._last_steering_position
        if abs(error) <= self.config.steering_deadband_motor_deg:
            steering_command = 0.0
        else:
            magnitude = clamp(
                self.config.steering_position_kp * abs(error),
                self.config.steering_min_command,
                self.config.steering_max_command,
            )
            steering_command = math.copysign(magnitude, error)

        if abs(self._last_steering_position) >= self.config.steering_emergency_limit_deg:
            command_sign = math.copysign(
                1.0, steering_command or self._last_steering_position
            )
            position_sign = math.copysign(1.0, self._last_steering_position)
            if command_sign == position_sign:
                steering_command = 0.0

        physical_command = steering_command * self.hardware.steering_direction
        if (
            self._last_steering_command is None
            or abs(physical_command - self._last_steering_command) >= 0.5
        ):
            self.steering_motor.start(physical_command)
            self._last_steering_command = physical_command

    def stop_all(self) -> None:
        errors: list[Exception] = []
        for name in ("drive_motor_1", "drive_motor_2", "steering_motor"):
            motor = getattr(self, name, None)
            if motor is not None:
                try:
                    motor.stop()
                except Exception as error:  # Preserve attempts to stop every motor.
                    errors.append(error)
        self._last_drive_command = None
        self._last_steering_command = None
        if errors:
            print(f"Warning: {len(errors)} motor stop command(s) failed.", file=sys.stderr)

    @property
    def button_pressed(self) -> bool:
        return bool(self.start_stop_button.is_pressed)

    def close(self) -> None:
        self.stop_all()
        for sensor_name in ("left_line", "right_line", "start_stop_button"):
            sensor = getattr(self, sensor_name, None)
            close = getattr(sensor, "close", None)
            if callable(close):
                close()


class CsvRunLogger:
    FIELDNAMES = (
        "monotonic_s",
        "state",
        "reason",
        "line_left",
        "line_right",
        "distance_in",
        "odometer_in",
        "speed_ips",
        "drive_speed_1",
        "drive_speed_2",
        "motor_disagreement_deg",
        "steering_motor_deg",
        "steering_wheel_target_deg",
        "drive_command",
        "yaw_deg",
        "yaw_rate_dps",
        "pitch_deg",
        "roll_deg",
        "acceleration_deviation_mps2",
        "curvature_per_in",
        "line_confidence",
    )

    def __init__(self, path: Path):
        path.parent.mkdir(parents=True, exist_ok=True)
        self.path = path
        self._file: TextIO = path.open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._file, fieldnames=self.FIELDNAMES)
        self._writer.writeheader()
        self._rows_since_flush = 0

    def write(self, obs: Observation, output: ControlOutput) -> None:
        self._writer.writerow(
            {
                "monotonic_s": f"{obs.now:.6f}",
                "state": output.state.value,
                "reason": output.reason,
                "line_left": int(obs.line_left),
                "line_right": int(obs.line_right),
                "distance_in": f"{obs.distance_in:.4f}",
                "odometer_in": f"{obs.odometer_in:.4f}",
                "speed_ips": f"{obs.speed_ips:.4f}",
                "drive_speed_1": f"{obs.drive_speed_1:.3f}",
                "drive_speed_2": f"{obs.drive_speed_2:.3f}",
                "motor_disagreement_deg": f"{obs.drive_motor_disagreement_deg:.3f}",
                "steering_motor_deg": f"{obs.steering_motor_deg:.3f}",
                "steering_wheel_target_deg": f"{output.steering_wheel_deg:.3f}",
                "drive_command": f"{output.drive_command:.3f}",
                "yaw_deg": f"{obs.yaw_deg:.3f}",
                "yaw_rate_dps": f"{obs.yaw_rate_dps:.3f}",
                "pitch_deg": f"{obs.pitch_deg:.3f}",
                "roll_deg": f"{obs.roll_deg:.3f}",
                "acceleration_deviation_mps2": f"{obs.acceleration_deviation_mps2:.3f}",
                "curvature_per_in": f"{output.curvature_per_in:.6f}",
                "line_confidence": f"{output.line_confidence:.3f}",
            }
        )
        self._rows_since_flush += 1
        if self._rows_since_flush >= 20:
            self._file.flush()
            self._rows_since_flush = 0

    def close(self) -> None:
        self._file.flush()
        self._file.close()


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Simple bang-bang following for one straight black line"
    )
    parser.add_argument("--drive-motor-1-port", choices="ABCD", default="C")
    parser.add_argument("--drive-motor-2-port", choices="ABCD", default="D")
    parser.add_argument("--steering-motor-port", choices="ABCD", default="B")
    parser.add_argument("--left-line-pin", type=int, default=26)
    parser.add_argument("--right-line-pin", type=int, default=22)
    parser.add_argument("--button-pin", type=int, default=24)
    parser.add_argument(
        "--inches-per-motor-rotation",
        type=float,
        default=9.74,
        help="measured straight-line vehicle travel for 360 drive-motor degrees",
    )
    parser.add_argument("--drive-motor-1-direction", type=require_direction, default=1)
    parser.add_argument("--drive-motor-2-direction", type=require_direction, default=-1)
    parser.add_argument(
        "--steering-direction",
        type=require_direction,
        default=1,
        help="motor/encoder direction; defaults to 1 for the current linkage",
    )
    parser.add_argument(
        "--library-directory",
        type=Path,
        help="path to the course-provided lib directory; normally auto-detected",
    )
    parser.add_argument(
        "--log-file",
        type=Path,
        help="CSV output path; defaults to /tmp with a timestamp",
    )
    parser.add_argument(
        "--keyboard-device",
        type=Path,
        action="append",
        help=(
            "Linux /dev/input/event device to read; may be repeated. "
            "By default every readable event device is checked."
        ),
    )
    parser.add_argument(
        "--no-keyboard-control",
        action="store_true",
        help="disable Alt and arrow-key control and use only the Grove button",
    )
    return parser


def describe_keyboard(keyboard: LinuxKeyboardInput) -> None:
    if keyboard.available:
        devices = ", ".join(str(path) for path in keyboard.paths)
        if keyboard.exclusive:
            print(
                f"Keyboard control exclusively captured through: {devices}. "
                "Arrow keys will control only the robot until Ctrl+C exits."
            )
        else:
            print(
                f"WARNING: Keyboard detected at {devices}, but exclusive capture "
                "failed. Close other programs using that device. If needed, run "
                "'sudo usermod -aG input team_13', then log out and back in.",
                file=sys.stderr,
            )
        return
    if keyboard.permission_denied:
        print(
            "WARNING: Keyboard devices exist but cannot be read. The Grove "
            "button will still work. Add this account to the input group with "
            "'sudo usermod -aG input team_13', then log out and back in.",
            file=sys.stderr,
        )
        return
    print(
        "WARNING: No readable Linux keyboard input device was found; "
        "the Grove button will still work.",
        file=sys.stderr,
    )


def create_keyboard_input(
    device_paths: Optional[Sequence[Path]],
) -> Optional[LinuxKeyboardInput | TkKeyboardInput]:
    linux_keyboard = LinuxKeyboardInput(device_paths)
    if linux_keyboard.exclusive:
        describe_keyboard(linux_keyboard)
        return linux_keyboard

    try:
        desktop_keyboard = TkKeyboardInput()
    except Exception as error:
        describe_keyboard(linux_keyboard)
        print(
            "WARNING: The desktop keyboard-control window could not be opened: "
            f"{type(error).__name__}: {error}",
            file=sys.stderr,
        )
        if linux_keyboard.available:
            return linux_keyboard
        linux_keyboard.close()
        return None

    linux_keyboard.close()
    print(
        "Keyboard control window opened. Keep that window focused while using "
        "Alt or the arrow keys."
    )
    return desktop_keyboard


def run(args: argparse.Namespace) -> int:
    global PROJECT_LIBRARY_DIRECTORY
    if args.library_directory is not None:
        PROJECT_LIBRARY_DIRECTORY = configure_project_library(args.library_directory)

    config = RobotConfig()
    hardware_config = HardwareConfig(
        drive_motor_1_port=args.drive_motor_1_port,
        drive_motor_2_port=args.drive_motor_2_port,
        steering_motor_port=args.steering_motor_port,
        left_line_pin=args.left_line_pin,
        right_line_pin=args.right_line_pin,
        button_pin=args.button_pin,
        inches_per_motor_rotation=args.inches_per_motor_rotation,
        drive_motor_1_direction=args.drive_motor_1_direction,
        drive_motor_2_direction=args.drive_motor_2_direction,
        steering_direction=args.steering_direction,
    )
    hardware_config.validate()

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = args.log_file or Path(f"/tmp/macro_line_following_{timestamp}.csv")
    controller = LineFollowerController(config)
    debouncer = LineDebouncer(config.line_debounce_s)
    robot: Optional[HardwareRobot] = None
    keyboard: Optional[LinuxKeyboardInput | TkKeyboardInput] = None
    run_logger: Optional[CsvRunLogger] = None
    previous_state: Optional[ControllerState] = None
    loop_period = 1.0 / config.loop_hz

    try:
        robot = HardwareRobot(hardware_config, config)
        robot.prepare_for_control()
        robot.stop_all()

        if not args.no_keyboard_control:
            keyboard = create_keyboard_input(args.keyboard_device)

        now = time.monotonic()
        button_presses = ButtonPressClassifier(robot.button_pressed, now)
        operating_mode = OperatingMode.WAITING
        run_logger = CsvRunLogger(log_path)
        if keyboard is not None:
            print(
                "Waiting with all motors stopped. Press Alt or the Grove button "
                "once to start line following, or double-press the Grove button "
                "for straight drive. Press an arrow key for manual mode. "
                f"Ctrl+C exits. Log: {log_path}"
            )
        else:
            print(
                "Waiting with all motors stopped. Press the Grove button once "
                "to start line following, or double-press it for straight drive. "
                f"Ctrl+C exits. Log: {log_path}"
            )

        while True:
            loop_started = time.monotonic()
            keys = keyboard.poll() if keyboard is not None else KeyboardSnapshot()
            if keys.emergency_stop:
                raise KeyboardInterrupt
            button_event = button_presses.update(robot.button_pressed, loop_started)
            arrows_held = keys.up or keys.down or keys.left or keys.right
            arrow_override = keys.arrow_activity or arrows_held
            requested_mode = operating_mode

            if arrow_override:
                button_presses.cancel()
                requested_mode = OperatingMode.MANUAL
            elif operating_mode in {
                OperatingMode.AUTONOMOUS,
                OperatingMode.STRAIGHT_DRIVE,
            }:
                if button_event.press_edge or keys.alt_pressed:
                    button_presses.cancel()
                    requested_mode = OperatingMode.WAITING
            elif keys.alt_pressed:
                button_presses.cancel()
                requested_mode = OperatingMode.AUTONOMOUS
            elif button_event.double_press:
                requested_mode = OperatingMode.STRAIGHT_DRIVE
            elif button_event.single_press:
                requested_mode = OperatingMode.AUTONOMOUS

            if requested_mode != operating_mode:
                robot.stop_all()
                operating_mode = requested_mode
                previous_state = None
                if operating_mode == OperatingMode.AUTONOMOUS:
                    controller = LineFollowerController(config)
                    debouncer = LineDebouncer(config.line_debounce_s)
                    print(
                        "Line-following mode started. Press Alt, the Grove button, "
                        "or an arrow key to override it."
                    )
                elif operating_mode == OperatingMode.STRAIGHT_DRIVE:
                    print(
                        "Straight-drive mode started. Press Alt, the Grove button, "
                        "or an arrow key to stop it."
                    )
                elif operating_mode == OperatingMode.MANUAL:
                    print(
                        "Manual mode: hold arrows to steer and drive. "
                        "Press Alt or the Grove button to restart autonomous mode."
                    )
                else:
                    print(
                        "Waiting with all motors stopped. Press Alt or the Grove "
                        "button once for line following, or double-press the button "
                        "for straight drive."
                    )

            observation = robot.read(debouncer)
            if operating_mode == OperatingMode.AUTONOMOUS:
                output = controller.update(observation)
                robot.apply(output)
            elif operating_mode == OperatingMode.STRAIGHT_DRIVE:
                output = straight_drive_output(config)
                robot.apply(output)
            elif operating_mode == OperatingMode.MANUAL:
                output = manual_output(keys, config)
                robot.apply(output)
            else:
                output = inactive_output(
                    ControllerState.WAITING,
                    "waiting for Alt, Grove button, or arrow-key input",
                )
            run_logger.write(observation, output)

            if output.state != previous_state:
                print(f"State: {output.state.value} — {output.reason}")
                previous_state = output.state
            if output.state == ControllerState.FAULT:
                robot.stop_all()
                operating_mode = OperatingMode.WAITING
                previous_state = None
                print(
                    f"Safety stop: {output.reason}. Waiting for a new Alt or "
                    "Grove button press.",
                    file=sys.stderr,
                )

            remaining = loop_period - (time.monotonic() - loop_started)
            if remaining > 0:
                time.sleep(remaining)
    except KeyboardInterrupt:
        print("\nCtrl+C received; stopping all motors.")
        return 0
    finally:
        if keyboard is not None:
            keyboard.close()
        if robot is not None:
            robot.close()
        if run_logger is not None:
            run_logger.close()


def main(argv: Optional[Sequence[str]] = None) -> int:
    def request_stop(_signum: int, _frame: object) -> None:
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, request_stop)
    parser = build_argument_parser()
    args = parser.parse_args(argv)
    try:
        return run(args)
    except (RuntimeError, ValueError, OSError) as error:
        print(f"Startup/runtime error: {error}", file=sys.stderr)
        return 1
    except Exception as error:
        # Hardware libraries raise several project-specific exception classes.
        # The run() finally block has already attempted to stop all motors.
        print(
            f"Unexpected hardware/control error after safety stop: "
            f"{type(error).__name__}: {error}",
            file=sys.stderr,
        )
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
