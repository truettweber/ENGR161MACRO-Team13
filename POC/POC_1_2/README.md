# Straight-line follower

`robust_line_following.py` is intentionally reduced to a simple bang-bang
controller for one straight, unbroken black line. It uses the two Grove line
sensors and steering/drive encoders. IMU initialization and calibration are
skipped in this mode.

## Sensor and steering behavior

The existing sensor locations are unchanged. Grove readings are interpreted as
`0 = black` and `1 = white`. Each sensor is debounced independently for 0.010
second.

| Left sensor | Right sensor | Command |
| --- | --- | --- |
| Black | Black | Drive straight |
| Black | White | Full left steering |
| White | Black | Full right steering |
| White | White at startup | Stop and wait for the line |
| White | White after tracking | Continue the last full correction for up to 1 second |

If the line remains lost for one second, the controller enters `fault` and
stops instead of turning indefinitely. When both sensors see black again, the
steering target returns to zero immediately.

Full correction is +/-90 steering-motor degrees, corresponding to +/-26.16
wheel degrees. The steering position gain is 1.75 and its Build HAT command is
capped at 70.

## Current drive speeds

The drive commands have been doubled from the preceding non-stalling settings:

- Straight cruise: 30.0
- Corrective turn: 22.0
- Manual Up/Down: 25.0

These are Build HAT command values on the -100 to 100 scale.

## Before running

1. Lift the driven wheels clear of the table for the first test.
2. Point the front wheels physically straight. Their startup position becomes
   the relative steering center.
3. Confirm Left Arrow physically turns left and Right Arrow turns right.
4. Confirm the physical left sensor produces `line_left=1` in the CSV when it
   alone is over black; repeat for the right sensor.
5. Place the robot so at least one sensor sees the black line before starting
   autonomy. With both sensors white, the robot deliberately remains stopped.

## Run

Defaults are drive motors C and D, steering motor B, left sensor D26, right
sensor D22, button D24, drive motor D direction -1, and steering direction 1:

```text
python3 POC/POC_1_2/robust_line_following.py
```

The program searches its repository ancestors and common Desktop locations for
the supplied `lib` directory. If needed, provide it explicitly:

```text
python3 POC/POC_1_2/robust_line_following.py \
  --library-directory /path/to/ENGR161MACRO-Team13/lib
```

After motor initialization, the program waits with all motors stopped:

- Press Alt to start line following immediately.
- Press the Grove button once to start line following. It begins after a
  0.65-second double-press window.
- Double-press the Grove button to enter straight-drive mode. It drives at the
  current cruise command with centered steering and ignores the line sensors.
- Press Alt or the Grove button while either autonomous mode is active to stop
  and return to waiting.
- Any arrow key enters manual mode.

- Up/Down: forward/reverse
- Left/Right: left/right steering
- Alt: restart line following
- Grove button once/double press: restart line following/straight drive
- Ctrl+C: stop all motors and exit

When direct Linux keyboard access is unavailable, a **MACRO Robot Keyboard
Control** window opens. Keep that window focused while using Alt or arrows.

Run data is written to `/tmp/macro_line_following_TIMESTAMP.csv` unless
`--log-file PATH` is supplied.

## Autonomous states

- `acquire`: stopped until at least one sensor sees black
- `track`: the four-state bang-bang behavior above
- `fault`: stopped after prolonged line loss or coupled-motor disagreement
