from buildhat import Motor
from basehat import LineFinder, Button
import time

# Steering settings
maxAngle = 40      # maximum angle the front motor should turn to prevent driveshaft damage
steeringStep = 5      # degrees per correction

def main():

    # Motors
    motorLeft = Motor('A')
    motorRight = Motor('B')
    motorFront = Motor('C')

    # Inputs
    button = Button(26)
    lineFinder = LineFinder(24)

    # Keep track of steering angle
    steering_angle = 0

    # Search direction
    last_direction = 1

    print("Press button to start.")

    while not button.value:
        time.sleep(0.05)

    print("Robot started.")

    try:
        while True:

            sensor_value = lineFinder.value

            print(f"Sensor: {sensor_value} | Steering Angle: {steering_angle}")
        
            if sensor_value:

                # Drive forward
                motorLeft.start(30)
                motorRight.start(30)

                # Gradually return steering toward center
                if steering_angle > 0:

                    correction = min(steeringStep, steering_angle)

                    motorFront.run_for_degrees(-correction)
                    steering_angle -= correction

                elif steering_angle < 0:

                    correction = min(steeringStep, abs(steering_angle))

                    motorFront.run_for_degrees(correction)
                    steering_angle += correction
            else:

                # Alternate search direction each loss
                last_direction *= -1

                if last_direction > 0:

                    motorLeft.start(20)
                    motorRight.start(-20)

                    # Turn wheels right if safe
                    if steering_angle < maxAngle:

                        turn_amount = min(steeringStep, maxAngle - steering_angle)

                        motorFront.run_for_degrees(turn_amount)
                        steering_angle += turn_amount

                else:

                    motorLeft.start(-20)
                    motorRight.start(20)

                    # Turn wheels left if safe
                    if steering_angle > -maxAngle:

                        turn_amount = min(steeringStep, steering_angle + maxAngle)

                        motorFront.run_for_degrees(-turn_amount)
                        steering_angle -= turn_amount

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nCtrl+C detected.")

    finally:
        print("Stopping motors...")

        motorLeft.stop()
        motorRight.stop()
        motorFront.stop()

        print("Robot safely stopped.")

if __name__ == "__main__":
    main()
