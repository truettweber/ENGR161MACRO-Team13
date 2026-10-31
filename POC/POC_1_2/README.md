# Robust line follower

`robust_line_following.py` integrates PoC Task 1 (solid-line obstacle
traversal) and Task 2 (curved dashed/broken-line following). It uses both
drive-motor encoders, the steering encoder, two Grove line finders, and the
Grove IMU. It deliberately does not use the magnetometer.

## Safety before the first run

1. Lift the driven wheels clear of the table.
2. Point both front wheels physically straight ahead. The program records the
   current absolute steering encoder position as its relative 0-degree center.
3. Confirm that the physical steering hard stops safely accommodate the
   program's full +/-90 motor-degree commanded range.
4. Determine each motor direction at a low command. Both drive motors must
   propel the vehicle forward together without fighting one another.
5. Turn the robot right by hand and select the yaw direction that produces a
   positive yaw rate.
6. Place each line finder over black and white paper. The raw Grove convention
   is `0 = black` and `1 = white`; the controller explicitly converts `0` into
   a detected black line.
7. Connect the Grove start/stop button to D24. Verify that the Grove button or
   Alt switches from autonomous mode to the stopped waiting mode while the
   wheels are still lifted. Ctrl+C must stop all motors and exit.
8. Test manual control with the wheels lifted: Up/Down drive forward/reverse,
   Left/Right steer, and releasing the arrows stops/centers those commands.

Keep the robot still and level during the two-second IMU calibration every time
the program starts.

All drive commands are currently 60% below the preceding half-speed settings
(20% of the original commands) to make the startup and tracking behavior easier
to diagnose. This includes cruise, curves, gaps, searching, obstacle traversal,
reverse recovery, and manual Up/Down control.

Line tracking is tuned to reduce oscillation: curvature ramps at `0.075`,
command slew is `0.120`, and centered steering decays at `0.250`. Ordinary
tracking is capped at 60% of the full curvature range. When both sensors become
white, the controller turns toward whichever sensor was the last one still on
black. Each sensor is debounced independently so a short one-sensor interval is
not discarded when the other sensor changes. Line-loss recovery applies full
steering curvature immediately. If the control loop still observes both
sensors leaving simultaneously, it uses the most recently known line side; it
commands straight only when no side has ever been observed. The steering
position gain is `1.75` and its motor command is capped at `70` for faster rack
motion. IMU yaw-rate and heading steering gains are currently zero until the
IMU signal and yaw sign are verified; IMU values are still logged.

At autonomous startup the robot remains stopped with straight steering until at
least one line sensor detects black. It no longer times out into an arbitrary
rightward search when the robot begins with both sensors on white.

## Measure encoder distance

The gearing cannot be inferred from wheel diameter because both motors, both
axles, and the different wheel sizes are mechanically linked. Mark the motor
position and vehicle position, roll the robot straight for several motor
rotations, and calculate:

```
inches per motor rotation = measured vehicle travel / motor rotations
```

Use the average result of several forward trials. The controller requires this
value because the course permits broken-line gaps up to 3 inches.

## Run

The current robot values from the team's Pi script are defaults: drive motors
C and D, steering motor B, left line finder D26, right line finder D22,
start/stop button D24, 9.74 inches per drive-motor rotation, and reversed
direction for drive motor D. The steering direction defaults to `1`. Running
without arguments uses those values:

```text
python3 POC/POC_1_2/robust_line_following.py
```

They can still be overridden at launch:

```text
python3 POC/POC_1_2/robust_line_following.py \
  --drive-motor-1-port A \
  --drive-motor-2-port B \
  --steering-motor-port C \
  --left-line-pin 24 \
  --right-line-pin 26 \
  --button-pin 18 \
  --inches-per-motor-rotation MEASURED_VALUE
```

Direction values can be overridden with `-1` or `1`:

```text
--drive-motor-1-direction -1
--drive-motor-2-direction 1
--steering-direction 1
--yaw-direction -1
--pitch-direction -1
--roll-direction -1
```

The steering sign must make a positive steering target turn the vehicle right.
The yaw sign must make a physical right turn produce positive yaw.

After the two-second IMU calibration, the program enters a waiting state with
all motors stopped. Press the Grove button or either Alt key to start
autonomous mode. Press either control again to return to waiting without
exiting. Pressing any arrow key immediately overrides autonomous mode and
enters manual mode:

- Up: drive forward
- Down: drive backward
- Left/Right: steer left/right
- Multiple arrows: combine steering and drive
- Release the arrows: command zero drive and centered steering while remaining
  in manual mode

From manual mode, press Alt or the Grove button to start autonomous mode again.
Ctrl+C remains the emergency stop: it stops all motors and exits. The Grove
button is active-high: released reads 0 and pressed reads 1.

The program first tries to read Linux `/dev/input/event*` devices because a
terminal cannot report a standalone Alt press. When those devices are
available, it exclusively captures the physical keyboard while running. If the
Pi does not expose a readable keyboard device, the program opens a small
**MACRO Robot Keyboard Control** window instead. Keep that window focused while
driving; losing focus immediately clears held arrows and stops manual motion.
This also prevents the arrows from moving the cursor in Thonny. Ctrl+C remains
the emergency exit in either input mode, and the device/window is released
automatically when the program exits. If both the control window and direct
keyboard capture fail, run:

```text
sudo usermod -aG input team_13
```

Then log out and back in (or reboot) before running the program again. Use
`--keyboard-device /dev/input/eventN` to select one device explicitly, or
`--no-keyboard-control` to run with only the Grove button.

## Import troubleshooting

Keep this script inside the full repository. The program searches its own
ancestors, the current working directory's ancestors, and common Raspberry Pi
Desktop locations for the course-provided `lib` directory. Copying only the
Python file discards the supplied `basehat` and vendored `buildhat` drivers.

If the repository or library has a different location, specify it explicitly:

```text
python3 Poc_task1.py --library-directory /full/path/to/ENGR161MACRO-Team13/lib
```

Import failures now identify the exact missing module. If the error names
`libakicm.so`, the Python files were found but the native course IMU driver is
missing from the Raspberry Pi installation.

Run data is written by default to a timestamped CSV file under `/tmp`. Use
`--log-file PATH` to retain it elsewhere. The Ctrl+C shutdown path issues stop
commands to all three motors even if another motor stop command fails.

## Controller states

- `waiting`: all motors are stopped pending Alt, button, or arrow-key input.
- `manual`: arrow keys directly control drive and steering.
- `acquire`: move slowly and find the starting line.
- `track`: follow visible line readings with a capped correction and fast
  recentering. IMU yaw steering correction is currently disabled.
- `gap_predict`: turn toward the sensor that last left the line while crossing
  a gap of up to 3.5 inches.
- `search`: perform a slow, bounded widening sweep toward the last line side.
- `obstacle`: reduce speed and retain the entry curvature while pitch, roll,
  acceleration, and encoder progress indicate rough terrain.
- `recover_reverse`: back up briefly and retry a blocked obstacle.
- `fault`: stop after encoder disagreement, failed recovery, or an exhausted
  line search.

The course specification permits a 2-inch line radius, but this vehicle's
5.5-inch wheelbase and full steering limit produce an approximate 11.2-inch
minimum center turning radius. Software cannot make the vehicle follow curves
tighter than its mechanical turning radius.
