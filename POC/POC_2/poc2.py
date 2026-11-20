import time

from buildhat import Motor
from basehat import HallSensor, IMUSensor, LineFinder

print("Initializing buildHAT and baseHAT...")
print("This may take approximately 10 seconds.")
print()

# ==================================================
# Hardware
# ==================================================

drive_motor = Motor("C")
steering_motor = Motor("B")
payload_motor = Motor("D")

lf_left = LineFinder(24)
lf_right = LineFinder(5)

hall_sensor = HallSensor(26)

#imu = IMUSensor(16)

# ==================================================
# Configuration
# ==================================================

DRIVE_SPEED = 3
HILL_SPEED = 3

STEERING_STEP = 35
MAX_STEERING = 100

PAYLOAD_OPEN_ANGLE = 180
LINE_MEMORY_TIME = 0.30

HILL_ACCEL_THRESHOLD = 0.30
HILL_DURATION = 2.0

# Encoder distance traveled after beacon detection
# before releasing cargo.
DROP_DELAY_DEGREES = 90

# ==================================================
# Calibration
# ==================================================

print("Calibration")
print()

input("Center steering and press ENTER...")

STEERING_CENTER = steering_motor.get_position()

print(f"Steering center recorded at {STEERING_CENTER}")

input("Close payload door and press ENTER...")

PAYLOAD_CLOSED = payload_motor.get_position()
PAYLOAD_OPEN = PAYLOAD_CLOSED + PAYLOAD_OPEN_ANGLE

print(f"Payload closed position recorded at {PAYLOAD_CLOSED}")
print()

input("Press ENTER to begin demonstration...")

# ==================================================
# State Variables
# ==================================================

payload_deployed = False

hall_armed = True

waiting_for_drop = False
drop_position = 0

last_direction = 0
last_line_seen = time.time()

hill_boost_end = 0

# ==================================================
# Functions
# ==================================================

def hill_detected():
    """
    Uses accelerometer readings from the Grove IMU.
    """

    try:

        x_accel, y_accel, z_accel = imu.getAccel()

        return abs(x_accel) > HILL_ACCEL_THRESHOLD

    except Exception:

        return False


def straighten():

    current_pos = steering_motor.get_position()

    error = STEERING_CENTER - current_pos

    if abs(error) > 3:
        steering_motor.run_for_degrees(error)


def turn_left():

    current_pos = steering_motor.get_position()

    if current_pos > (STEERING_CENTER - MAX_STEERING):

        steering_motor.run_for_degrees(
            -STEERING_STEP
        )


def turn_right():

    current_pos = steering_motor.get_position()

    if current_pos < (STEERING_CENTER + MAX_STEERING):

        steering_motor.run_for_degrees(
            STEERING_STEP
        )


def deploy_payload():

    global payload_deployed

    if payload_deployed:
        return

    print("Deploying payload...")

    drive_motor.stop()

    payload_motor.run_to_position(
        PAYLOAD_OPEN
    )

    time.sleep(1.5)

    payload_deployed = True

    print("Payload deployment complete.")

# ==================================================
# Main Program
# ==================================================

try:

    drive_motor.start(DRIVE_SPEED)

    while True:

        current_time = time.time()

        left_black = lf_left.line_detected
        right_black = lf_right.line_detected

        # ------------------------------------------
        # Hill / Obstacle Assist
        # ------------------------------------------

        if hill_detected():

            hill_boost_end = (
                current_time + HILL_DURATION
            )

        if current_time < hill_boost_end:

            drive_motor.start(HILL_SPEED)

        else:

            drive_motor.start(DRIVE_SPEED)

        # ------------------------------------------
        # Hall Sensor Detection
        # ------------------------------------------

        hall_detected = hall_sensor.value

        if (
            hall_detected and
            hall_armed and
            not payload_deployed and
            not waiting_for_drop
        ):

            print("Beacon detected.")

            hall_armed = False

            waiting_for_drop = True

            drop_position = (
                drive_motor.get_position()
                + DROP_DELAY_DEGREES
            )

            print(
                f"Waiting until drive position "
                f"{drop_position}"
            )

        elif not hall_detected:

            hall_armed = True

        # ------------------------------------------
        # Delayed Payload Deployment
        # ------------------------------------------

        if waiting_for_drop:

            if drive_motor.get_position() >= drop_position:

                deploy_payload()

                waiting_for_drop = False

                drive_motor.start(DRIVE_SPEED)

        # ------------------------------------------
        # Line Following
        # ------------------------------------------

        if left_black and not right_black:

            turn_left()

            last_direction = -1
            last_line_seen = current_time

        elif right_black and not left_black:

            turn_right()

            last_direction = 1
            last_line_seen = current_time

        elif not left_black and not right_black:

           straighten()

        else:
            straighten()
            
           
        # ------------------------------------------
        # Dotted Line Recovery
        # ------------------------------------------

        if (
            not left_black and
            not right_black and
            current_time - last_line_seen <
            LINE_MEMORY_TIME
        ):

            if last_direction == -1:

                turn_left()

            elif last_direction == 1:

                turn_right()

        time.sleep(0.05)

except KeyboardInterrupt:

    print()
    print("Ctrl+C received.")
    print("Stopping robot...")

finally:

    drive_motor.stop()
    steering_motor.stop()
    payload_motor.stop()

    print("Robot stopped safely.")
