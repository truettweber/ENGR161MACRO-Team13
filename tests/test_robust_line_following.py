"""Hardware-independent behavioral tests for the robust line follower."""

import os
import unittest
from pathlib import Path
from types import SimpleNamespace

from POC.POC_1_2.robust_line_following import (
    ControllerState,
    DebouncedButtonToggle,
    KeyboardSnapshot,
    LinuxKeyboardInput,
    LineDebouncer,
    LineFollowerController,
    Observation,
    OperatingMode,
    PROJECT_LIBRARY_DIRECTORY,
    RobotConfig,
    TkKeyboardInput,
    build_argument_parser,
    choose_operating_mode,
    manual_output,
    wrap_degrees,
)


def observation(
    now: float,
    *,
    left: bool = True,
    right: bool = True,
    odometer: float = 0.0,
    speed: float = 5.0,
    yaw: float = 0.0,
    yaw_rate: float = 0.0,
    pitch: float = 0.0,
    acceleration_deviation: float = 0.0,
) -> Observation:
    return Observation(
        now=now,
        dt=0.025,
        line_left=left,
        line_right=right,
        distance_in=odometer,
        odometer_in=odometer,
        speed_ips=speed,
        drive_speed_1=20.0,
        drive_speed_2=20.0,
        drive_motor_disagreement_deg=0.0,
        steering_motor_deg=0.0,
        yaw_deg=yaw,
        yaw_rate_dps=yaw_rate,
        pitch_deg=pitch,
        roll_deg=0.0,
        acceleration_deviation_mps2=acceleration_deviation,
    )


class LineFollowerControllerTests(unittest.TestCase):
    def test_relative_steering_position_handles_absolute_encoder_wrap(self) -> None:
        self.assertAlmostEqual(wrap_degrees(170.0 - (-170.0)), -20.0)
        self.assertAlmostEqual(wrap_degrees(-170.0 - 170.0), 20.0)

    def test_pi_script_defaults_match_supplied_robot_configuration(self) -> None:
        args = build_argument_parser().parse_args([])
        self.assertEqual(args.drive_motor_1_port, "C")
        self.assertEqual(args.drive_motor_2_port, "D")
        self.assertEqual(args.steering_motor_port, "B")
        self.assertEqual(args.left_line_pin, 26)
        self.assertEqual(args.right_line_pin, 22)
        self.assertEqual(args.button_pin, 24)
        self.assertEqual(args.drive_motor_2_direction, -1)
        self.assertEqual(args.steering_direction, 1)
        self.assertAlmostEqual(args.inches_per_motor_rotation, 9.74)

    def test_button_emits_one_toggle_per_stable_press(self) -> None:
        button = DebouncedButtonToggle(False, 0.0)
        self.assertFalse(button.update(True, 0.01))
        self.assertFalse(button.update(True, 0.05))
        self.assertTrue(button.update(True, 0.12))
        self.assertFalse(button.update(True, 0.25))
        self.assertFalse(button.update(False, 0.26))
        self.assertFalse(button.update(False, 0.37))
        self.assertFalse(button.update(True, 0.38))
        self.assertTrue(button.update(True, 0.49))

    def test_alt_toggles_autonomy_and_arrows_override_it(self) -> None:
        self.assertEqual(
            choose_operating_mode(OperatingMode.WAITING, True, False),
            OperatingMode.AUTONOMOUS,
        )
        self.assertEqual(
            choose_operating_mode(OperatingMode.AUTONOMOUS, True, False),
            OperatingMode.WAITING,
        )
        self.assertEqual(
            choose_operating_mode(OperatingMode.AUTONOMOUS, False, True),
            OperatingMode.MANUAL,
        )
        self.assertEqual(
            choose_operating_mode(OperatingMode.AUTONOMOUS, True, True),
            OperatingMode.MANUAL,
        )

    def test_arrow_keys_drive_and_steer_in_manual_mode(self) -> None:
        config = RobotConfig()
        forward_left = manual_output(
            KeyboardSnapshot(up=True, left=True),
            config,
        )
        self.assertEqual(forward_left.state, ControllerState.MANUAL)
        self.assertGreater(forward_left.drive_command, 0.0)
        self.assertLess(forward_left.steering_wheel_deg, 0.0)

        reverse_right = manual_output(
            KeyboardSnapshot(down=True, right=True),
            config,
        )
        self.assertLess(reverse_right.drive_command, 0.0)
        self.assertGreater(reverse_right.steering_wheel_deg, 0.0)

        released = manual_output(KeyboardSnapshot(), config)
        self.assertEqual(released.drive_command, 0.0)
        self.assertEqual(released.steering_wheel_deg, 0.0)

    def test_linux_input_events_report_alt_and_arrow_state(self) -> None:
        keyboard = LinuxKeyboardInput([])
        read_fd, write_fd = os.pipe()
        keyboard._fds.append(read_fd)
        keyboard._paths.append(Path("/test/keyboard"))
        try:
            os.write(
                write_fd,
                keyboard.EVENT.pack(0, 0, keyboard.EV_KEY, keyboard.KEY_LEFTALT, 1)
                + keyboard.EVENT.pack(0, 0, keyboard.EV_KEY, 103, 1),
            )
            pressed = keyboard.poll()
            self.assertTrue(pressed.alt_pressed)
            self.assertTrue(pressed.arrow_activity)
            self.assertTrue(pressed.up)

            os.write(
                write_fd,
                keyboard.EVENT.pack(0, 0, keyboard.EV_KEY, 103, 0),
            )
            released = keyboard.poll()
            self.assertFalse(released.alt_pressed)
            self.assertFalse(released.up)

            os.write(
                write_fd,
                keyboard.EVENT.pack(
                    0, 0, keyboard.EV_KEY, keyboard.KEY_LEFTCTRL, 1
                )
                + keyboard.EVENT.pack(0, 0, keyboard.EV_KEY, keyboard.KEY_C, 1),
            )
            self.assertTrue(keyboard.poll().emergency_stop)
        finally:
            keyboard.close()
            os.close(write_fd)

    def test_desktop_control_window_tracks_keys_and_focus_loss(self) -> None:
        class FakeRoot:
            def __init__(self) -> None:
                self.jobs = {}
                self.next_job = 0

            def after(self, _delay, callback):
                self.next_job += 1
                job = str(self.next_job)
                self.jobs[job] = callback
                return job

            def after_cancel(self, job):
                self.jobs.pop(job, None)

            def update_idletasks(self):
                pass

            def update(self):
                pass

            def run_jobs(self):
                jobs = list(self.jobs.values())
                self.jobs.clear()
                for callback in jobs:
                    callback()

        class FakeStatus:
            def configure(self, **_kwargs):
                pass

        class FakeTk:
            TclError = RuntimeError

        keyboard = TkKeyboardInput.__new__(TkKeyboardInput)
        keyboard._tk = FakeTk
        keyboard._root = FakeRoot()
        keyboard._status = FakeStatus()
        keyboard._pressed_arrows = set()
        keyboard._pressed_alt = set()
        keyboard._pressed_ctrl = set()
        keyboard._release_jobs = {}
        keyboard._alt_pressed = False
        keyboard._arrow_activity = False
        keyboard._emergency_stop = False
        keyboard._closed = False

        keyboard._on_press(SimpleNamespace(keysym="Up"))
        pressed = keyboard.poll()
        self.assertTrue(pressed.arrow_activity)
        self.assertTrue(pressed.up)

        keyboard._on_release(SimpleNamespace(keysym="Up"))
        keyboard._root.run_jobs()
        self.assertFalse(keyboard.poll().up)

        keyboard._on_press(SimpleNamespace(keysym="Alt_L"))
        self.assertTrue(keyboard.poll().alt_pressed)
        keyboard._on_press(SimpleNamespace(keysym="Control_L"))
        keyboard._on_press(SimpleNamespace(keysym="c"))
        self.assertTrue(keyboard.poll().emergency_stop)

        keyboard._on_press(SimpleNamespace(keysym="Right"))
        keyboard._on_focus_out(SimpleNamespace())
        self.assertFalse(keyboard.poll().right)

    def test_course_library_is_found_from_repository_layout(self) -> None:
        self.assertIsNotNone(PROJECT_LIBRARY_DIRECTORY)
        self.assertTrue((PROJECT_LIBRARY_DIRECTORY / "buildhat" / "__init__.py").is_file())
        self.assertTrue((PROJECT_LIBRARY_DIRECTORY / "basehat" / "__init__.py").is_file())
        self.assertTrue((PROJECT_LIBRARY_DIRECTORY / "basehat" / "LineFinder.py").is_file())

    def test_line_debouncer_rejects_a_short_change(self) -> None:
        debouncer = LineDebouncer(0.025)
        self.assertEqual(debouncer.update(True, True, 0.0), (True, True))
        self.assertEqual(debouncer.update(False, False, 0.010), (True, True))
        self.assertEqual(debouncer.update(True, True, 0.020), (True, True))
        self.assertEqual(debouncer.update(False, False, 0.030), (True, True))
        self.assertEqual(debouncer.update(False, False, 0.060), (False, False))

    def test_left_and_right_line_readings_command_correct_directions(self) -> None:
        left_controller = LineFollowerController(RobotConfig())
        left_controller.update(observation(0.0))
        left = left_controller.update(observation(0.025, left=True, right=False))
        self.assertEqual(left.state, ControllerState.TRACK)
        self.assertLess(left.steering_wheel_deg, 0.0)

        right_controller = LineFollowerController(RobotConfig())
        right_controller.update(observation(0.0))
        right = right_controller.update(observation(0.025, left=False, right=True))
        self.assertEqual(right.state, ControllerState.TRACK)
        self.assertGreater(right.steering_wheel_deg, 0.0)

    def test_short_gap_preserves_curve_then_reacquires(self) -> None:
        controller = LineFollowerController(RobotConfig())
        controller.update(observation(0.0, left=False, right=True))
        for index in range(1, 12):
            output = controller.update(
                observation(index * 0.025, left=False, right=True, odometer=index * 0.1)
            )
        self.assertGreater(output.curvature_per_in, 0.0)

        gap = controller.update(
            observation(0.4, left=False, right=False, odometer=1.3)
        )
        self.assertEqual(gap.state, ControllerState.GAP_PREDICT)
        self.assertGreater(gap.curvature_per_in, 0.0)

        reacquired = controller.update(
            observation(0.5, left=True, right=True, odometer=1.8)
        )
        self.assertEqual(reacquired.state, ControllerState.TRACK)

    def test_gap_longer_than_specification_enters_search(self) -> None:
        controller = LineFollowerController(RobotConfig())
        controller.update(observation(0.0))
        controller.update(observation(0.1, left=False, right=False, odometer=0.1))
        output = controller.update(observation(0.5, left=False, right=False, odometer=3.7))
        self.assertEqual(output.state, ControllerState.SEARCH)

    def test_obstacle_tilt_changes_state_and_reduces_speed(self) -> None:
        controller = LineFollowerController(RobotConfig())
        controller.update(observation(0.0))
        output = controller.update(observation(0.025, pitch=10.0))
        self.assertEqual(output.state, ControllerState.OBSTACLE)
        self.assertEqual(output.drive_command, RobotConfig().obstacle_drive_command)

    def test_search_is_bounded_by_distance(self) -> None:
        controller = LineFollowerController(RobotConfig())
        controller.update(observation(0.0, left=False, right=False, speed=5.0))
        controller.update(observation(1.1, left=False, right=False, odometer=1.0))
        self.assertEqual(controller.state, ControllerState.SEARCH)
        output = controller.update(observation(2.0, left=False, right=False, odometer=13.1))
        self.assertEqual(output.state, ControllerState.FAULT)
        self.assertEqual(output.drive_command, 0.0)

    def test_coupled_motor_disagreement_causes_safety_stop(self) -> None:
        controller = LineFollowerController(RobotConfig())
        controller.update(observation(0.0))
        output = None
        for index in range(1, 22):
            sample = observation(index * 0.025, odometer=index * 0.1)
            sample = Observation(
                **{
                    **sample.__dict__,
                    "drive_motor_disagreement_deg": 100.0,
                }
            )
            output = controller.update(sample)
        self.assertIsNotNone(output)
        self.assertEqual(output.state, ControllerState.FAULT)
        self.assertEqual(output.drive_command, 0.0)


if __name__ == "__main__":
    unittest.main()
