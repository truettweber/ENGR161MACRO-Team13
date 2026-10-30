"""Hardware-independent behavioral tests for the robust line follower."""

import unittest

from POC.POC_1_2.robust_line_following import (
    ControllerState,
    LineDebouncer,
    LineFollowerController,
    Observation,
    PROJECT_LIBRARY_DIRECTORY,
    RobotConfig,
    build_argument_parser,
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
    def test_pi_script_defaults_match_supplied_robot_configuration(self) -> None:
        args = build_argument_parser().parse_args([])
        self.assertEqual(args.drive_motor_1_port, "C")
        self.assertEqual(args.drive_motor_2_port, "D")
        self.assertEqual(args.steering_motor_port, "B")
        self.assertEqual(args.left_line_pin, 26)
        self.assertEqual(args.right_line_pin, 22)
        self.assertEqual(args.drive_motor_2_direction, -1)
        self.assertAlmostEqual(args.inches_per_motor_rotation, 9.74)

    def test_course_library_is_found_from_repository_layout(self) -> None:
        self.assertIsNotNone(PROJECT_LIBRARY_DIRECTORY)
        self.assertTrue((PROJECT_LIBRARY_DIRECTORY / "buildhat" / "__init__.py").is_file())
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
