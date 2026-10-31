# Robust line follower

`robust_line_following.py` integrates PoC Task 1 (solid-line obstacle
traversal) and Task 2 (curved dashed/broken-line following). It uses both
drive-motor encoders, the steering encoder, two Grove line finders, and the
Grove IMU. It deliberately does not use the magnetometer.

## Safety before the first run

1. Lift the driven wheels clear of the table.
2. Point both front wheels physically straight ahead. The program records the
   current absolute steering encoder position as its relative 0-degree center.
3. Confirm that neither steering hard stop can be reached inside the program's
   normal +/-80 motor-degree limit.
4. Determine each motor direction at a low command. Both drive motors must
   propel the vehicle forward together without fighting one another.
5. Turn the robot right by hand and select the yaw direction that produces a
   positive yaw rate.
6. Place each line finder over black and white paper. The program uses the
   supplied `line_detected` property, which should be true over black.
7. Verify that Ctrl+C stops both drive motors and the steering motor while the
   wheels are still lifted.

Keep the robot still and level during the two-second IMU calibration every time
the program starts.

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
C and D, steering motor B, left line finder D26, right line finder D22, 9.74
inches per drive-motor rotation, and reversed direction for drive motor D.
Running without arguments uses those values:

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
  --inches-per-motor-rotation MEASURED_VALUE
```

If another motor or IMU axis is reversed, add the relevant option with `-1`:

```text
--drive-motor-1-direction -1
--drive-motor-2-direction -1
--steering-direction -1
--yaw-direction -1
--pitch-direction -1
--roll-direction -1
```

The steering sign must make a positive steering target turn the vehicle right.
The yaw sign must make a physical right turn produce positive yaw.

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
`--log-file PATH` to retain it elsewhere. Press Ctrl+C for the manual stop; the
shutdown path issues stop commands to all three motors even if another motor
stop command fails.

## Controller states

- `acquire`: move slowly and find the starting line.
- `track`: follow visible line readings and damp turns with IMU yaw rate.
- `gap_predict`: preserve measured curvature for up to 3.5 inches.
- `search`: perform a slow, bounded widening sweep toward the last line side.
- `obstacle`: reduce speed and preserve heading while pitch, roll, acceleration,
  and encoder progress indicate rough terrain.
- `recover_reverse`: back up briefly and retry a blocked obstacle.
- `fault`: stop after encoder disagreement, failed recovery, or an exhausted
  line search.

The course specification permits a 2-inch line radius, but this vehicle's
5.5-inch wheelbase and safe steering limit produce an approximate 12.8-inch
minimum center turning radius. Software cannot make the vehicle follow curves
tighter than its mechanical turning radius.
