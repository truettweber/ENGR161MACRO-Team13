"""Hardware-independent tests for straight-line robot control."""

import os
import unittest
from pathlib import Path

from POC.POC_1_2.robust_line_following import (
    ControllerState,
    ButtonPressClassifier,
    DebouncedButtonToggle,
    KeyboardSnapshot,
    LinuxKeyboardInput,
    LineDebouncer,
    LineFollowerController,
    Observation,
    OperatingMode,
    PROJECT_LIBRARY_DIRECTORY,
    RobotConfig,
    build_argument_parser,
    choose_operating_mode,
    line_value_is_black,
    manual_output,
    straight_drive_output,
    wrap_degrees,
)


def observation(
    now: float,
    *,
    left: bool = True,
    right: bool = True,
    disagreement: float = 0.0,
) -> Observation:
    return Observation(
        now=now,
        dt=0.025,
        line_left=left,
        line_right=right,
        distance_in=0.0,
        odometer_in=0.0,
        speed_ips=1.0,
        drive_speed_1=5.0,
        drive_speed_2=5.0,
        drive_motor_disagreement_deg=disagreement,
        steering_motor_deg=0.0,
        yaw_deg=0.0,
        yaw_rate_dps=0.0,
        pitch_deg=0.0,
        roll_deg=0.0,
        acceleration_deviation_mps2=0.0,
    )


class StraightLineControllerTests(unittest.TestCase):
    def test_defaults_match_robot_and_nonstalling_test_speed(self) -> None:
        args = build_argument_parser().parse_args([])
        self.assertEqual(args.drive_motor_1_port, "C")
        self.assertEqual(args.drive_motor_2_port, "D")
        self.assertEqual(args.steering_motor_port, "B")
        self.assertEqual(args.left_line_pin, 26)
        self.assertEqual(args.right_line_pin, 22)
        self.assertEqual(args.button_pin, 24)
        self.assertEqual(args.drive_motor_2_direction, -1)
        self.assertEqual(args.steering_direction, 1)
        config = RobotConfig()
        self.assertEqual(config.cruise_drive_command, 30.0)
        self.assertEqual(config.curve_drive_command, 22.0)
        self.assertEqual(config.manual_drive_command, 25.0)
        self.assertEqual(config.steering_max_command, 70.0)

    def test_course_library_is_found(self) -> None:
        self.assertIsNotNone(PROJECT_LIBRARY_DIRECTORY)
        self.assertTrue((PROJECT_LIBRARY_DIRECTORY / "buildhat" / "__init__.py").is_file())
        self.assertTrue((PROJECT_LIBRARY_DIRECTORY / "basehat" / "LineFinder.py").is_file())

    def test_zero_is_black_and_one_is_white(self) -> None:
        self.assertTrue(line_value_is_black(0))
        self.assertTrue(line_value_is_black(0.49))
        self.assertFalse(line_value_is_black(0.5))
        self.assertFalse(line_value_is_black(1))

    def test_relative_steering_wrap(self) -> None:
        self.assertAlmostEqual(wrap_degrees(170.0 - (-170.0)), -20.0)
        self.assertAlmostEqual(wrap_degrees(-170.0 - 170.0), 20.0)

    def test_line_debouncer_rejects_short_changes(self) -> None:
        debouncer = LineDebouncer(0.010)
        self.assertEqual(debouncer.update(True, True, 0.000), (True, True))
        self.assertEqual(debouncer.update(False, False, 0.005), (True, True))
        self.assertEqual(debouncer.update(True, True, 0.009), (True, True))

    def test_line_debouncer_preserves_departure_order(self) -> None:
        debouncer = LineDebouncer(0.010)
        self.assertEqual(debouncer.update(True, True, 0.000), (True, True))
        self.assertEqual(debouncer.update(False, True, 0.005), (True, True))
        self.assertEqual(debouncer.update(False, False, 0.010), (True, True))
        self.assertEqual(debouncer.update(False, False, 0.016), (False, True))
        self.assertEqual(debouncer.update(False, False, 0.021), (False, False))

    def test_both_black_drives_straight(self) -> None:
        output = LineFollowerController(RobotConfig()).update(observation(0.0))
        self.assertEqual(output.state, ControllerState.TRACK)
        self.assertEqual(output.drive_command, 30.0)
        self.assertEqual(output.steering_wheel_deg, 0.0)

    def test_one_sensor_commands_immediate_full_turn(self) -> None:
        left = LineFollowerController(RobotConfig()).update(
            observation(0.0, left=True, right=False)
        )
        right = LineFollowerController(RobotConfig()).update(
            observation(0.0, left=False, right=True)
        )
        self.assertAlmostEqual(left.steering_wheel_deg, -26.16)
        self.assertAlmostEqual(right.steering_wheel_deg, 26.16)
        self.assertEqual(left.drive_command, 22.0)
        self.assertEqual(right.drive_command, 22.0)

    def test_no_line_at_startup_stays_stopped(self) -> None:
        controller = LineFollowerController(RobotConfig())
        output = controller.update(observation(5.0, left=False, right=False))
        self.assertEqual(output.state, ControllerState.ACQUIRE)
        self.assertEqual(output.drive_command, 0.0)
        self.assertEqual(output.steering_wheel_deg, 0.0)

    def test_line_loss_continues_last_turn_then_stops(self) -> None:
        controller = LineFollowerController(RobotConfig())
        controller.update(observation(0.0, left=True, right=False))
        recovering = controller.update(observation(0.1, left=False, right=False))
        self.assertAlmostEqual(recovering.steering_wheel_deg, -26.16)
        stopped = controller.update(observation(1.11, left=False, right=False))
        self.assertEqual(stopped.state, ControllerState.FAULT)
        self.assertEqual(stopped.drive_command, 0.0)
        self.assertEqual(stopped.steering_wheel_deg, 0.0)

    def test_reacquiring_both_sensors_immediately_straightens(self) -> None:
        controller = LineFollowerController(RobotConfig())
        controller.update(observation(0.0, left=False, right=True))
        output = controller.update(observation(0.025))
        self.assertEqual(output.steering_wheel_deg, 0.0)
        self.assertEqual(output.drive_command, 30.0)

    def test_encoder_disagreement_faults(self) -> None:
        controller = LineFollowerController(RobotConfig())
        output = None
        for index in range(25):
            output = controller.update(
                observation(index * 0.025, disagreement=100.0)
            )
        self.assertIsNotNone(output)
        self.assertEqual(output.state, ControllerState.FAULT)
        self.assertEqual(output.drive_command, 0.0)

    def test_manual_arrows_drive_and_steer(self) -> None:
        config = RobotConfig()
        output = manual_output(KeyboardSnapshot(up=True, left=True), config)
        self.assertEqual(output.state, ControllerState.MANUAL)
        self.assertEqual(output.drive_command, 25.0)
        self.assertLess(output.steering_wheel_deg, 0.0)

    def test_alt_toggles_and_arrows_override(self) -> None:
        self.assertEqual(
            choose_operating_mode(OperatingMode.WAITING, True, False),
            OperatingMode.AUTONOMOUS,
        )
        self.assertEqual(
            choose_operating_mode(OperatingMode.AUTONOMOUS, False, True),
            OperatingMode.MANUAL,
        )

    def test_button_emits_one_toggle_per_press(self) -> None:
        button = DebouncedButtonToggle(False, 0.0)
        self.assertFalse(button.update(True, 0.01))
        self.assertTrue(button.update(True, 0.12))
        self.assertFalse(button.update(True, 0.25))

    def test_button_single_and_double_press_actions(self) -> None:
        button = ButtonPressClassifier(False, 0.0)
        self.assertFalse(button.update(True, 0.01).press_edge)
        self.assertTrue(button.update(True, 0.06).press_edge)
        self.assertFalse(button.update(False, 0.07).press_edge)
        self.assertFalse(button.update(False, 0.12).press_edge)
        self.assertFalse(button.update(True, 0.50).press_edge)
        double = button.update(True, 0.55)
        self.assertTrue(double.double_press)

        single = ButtonPressClassifier(False, 1.0)
        single.update(True, 1.01)
        single.update(True, 1.06)
        single.update(False, 1.07)
        single.update(False, 1.12)
        self.assertTrue(single.update(False, 1.72).single_press)

    def test_double_press_straight_drive_output(self) -> None:
        output = straight_drive_output(RobotConfig())
        self.assertEqual(output.drive_command, RobotConfig().cruise_drive_command)
        self.assertEqual(output.steering_wheel_deg, 0.0)

    def test_linux_key_events_track_alt_and_arrow(self) -> None:
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
            snapshot = keyboard.poll()
            self.assertTrue(snapshot.alt_pressed)
            self.assertTrue(snapshot.up)
        finally:
            keyboard.close()
            os.close(write_fd)


if __name__ == "__main__":
    unittest.main()
