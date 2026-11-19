import time

from buildhat import Motor
from basehat import HallSensor, IMUSensor, LineFinder

print("Initializing buildHAT and baseHAT...")
print("This may take approximately 10 seconds.")
print()

# ---------- Hardware ----------

drive_motor = Motor("B")
steering_motor = Motor("C")
payload_motor = Motor("D")

lf_left = LineFinder(5)
lf_right = LineFinder(16)

hall_sensor = HallSensor(24)
imu = IMUSensor()

# ---------- Constants ----------

DRIVE_SPEED = 25
HILL_SPEED = 35

STEERING_STEP = 8
MAX_STEERING = 100

PAYLOAD_OPEN_ANGLE = 90

HILL_THRESHOLD = 25
HILL_DURATION = 1.5

# ---------- Calibration ----------

print("Begin Calibration.")
print()

input("Please center the front wheels, then press ENTER...")

STEERING_CENTER = steering_motor.get_position()

input("Please close the payload door, then press ENTER...")

PAYLOAD_CLOSED = payload_motor.get_position()
PAYLOAD_OPEN = PAYLOAD_CLOSED + PAYLOAD_OPEN_ANGLE

print()
print(f"Steering center: {STEERING_CENTER}")
print(f"Payload closed position: {PAYLOAD_CLOSED}")
print("Calibration is complete.")
print()

input("Please press the button to begin operation...")

# ---------- State ----------

hall_armed = True
payload_deployed = False

hill_boost_end = 0
hill_counter = 0

# ---------- Functions ----------

def get_pitch():
    """
    Replace this function with the proper IMU call once
    the IMU API is confirmed.
    """

    try:
        roll, pitch, yaw = imu.getAHRS()
        return pitch
    except:
        return 0


def straighten():
    position = steering_motor.get_position()

    error = STEERING_CENTER - position

    if abs(error) > 3:
        steering_motor.run_for_degrees(error)


def turn_left():
    position = steering_motor.get_position()

    if position > (STEERING_CENTER - MAX_STEERING):
        steering_motor.run_for_degrees(-STEERING_STEP)


def turn_right():
    position = steering_motor.get_position()

    if position < (STEERING_CENTER + MAX_STEERING):
        steering_motor.run_for_degrees(STEERING_STEP)


def deploy_payload():

    global payload_deployed

    if payload_deployed:
        return

    print("Magnetic Beacon Detected!.")
    print("Deploying payload...")

    drive_motor.stop()

    payload_motor.run_to_position(PAYLOAD_OPEN)

    # Allow payload time to clear robot

    time.sleep(3)

    payload_motor.run_to_position(PAYLOAD_CLOSED)

    payload_deployed = True

    print("Payload Deployed. Continuing Operation...")

# ---------- Main ----------

try:

    drive_motor.start(DRIVE_SPEED)

    while True:

        current_time = time.time()

        left_black = lf_left.line_detected
        right_black = lf_right.line_detected

        hall_detected = hall_sensor.value

        # ----- Hill Detection -----

        pitch = get_pitch()

        if abs(pitch) > HILL_THRESHOLD:
            hill_counter += 1
        else:
            hill_counter = 0

        if hill_counter >= 5:
            hill_boost_end = current_time + HILL_DURATION

        if current_time < hill_boost_end:
            drive_motor.start(HILL_SPEED)
        else:
            drive_motor.start(DRIVE_SPEED)

        # ----- Payload Deployment -----

        if hall_detected and hall_armed and not payload_deployed:

            deploy_payload()

            hall_armed = False

            drive_motor.start(DRIVE_SPEED)

        if not hall_detected:
            hall_armed = True

        # ----- Line Following -----

        # Stop marker
        if left_black and right_black:

            print("Stop marker detected.")

            drive_motor.stop()

            break

        # Turn left
        elif left_black and not right_black:

            turn_left()

        # Turn right
        elif right_black and not left_black:

            turn_right()

        # Straight section
        else:

            straighten()

        time.sleep(0.05)

except KeyboardInterrupt:

    print()
    print("Ctrl+C received.")

finally:

    drive_motor.stop()
    steering_motor.stop()
    payload_motor.stop()

    print("Robot stopped safely.")
