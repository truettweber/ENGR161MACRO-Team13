import time
import asyncio
from buildhat import Motor
from basehat import HallSensor, IMUSensor, LineFinder

forward_motor = Motor("B") # Large motor, use for the rear (or front) wheel drive.
turn_motor = Motor("C") # Pretty much as we have it now, just swap medium motor for large motor
payload_motor = Motor("D") # Medium motor, used for deploying the payload (because trying to integrate payload unload with driving would be irritating)
lineL = LineFinder(5)
lineR = LineFinder(16)
imu = IMUSensor()
def_speed = 20
turnMotor_speed = 8
max_turn_angle = 100

# The following async def functions are basically taking steps that I can't be bothered to write over and over again, and converting
# them into mini functions that I can use later down the line. Yes, I am that lazy.

async def monitor_imu():
    x_mag, y_mag, z_mag = imu.getMag() # Gets X, Y, and Z magnitude from a default base position based on how the IMU is situated at the beginning of running the program
    print(f"x_mag: {x_mag} y_mag: {y_mag} z_mag: {z_mag}")

async def start():
    forward_motor.start(speed)

async def stop():
    turn_motor.stop()
    forward_motor.stop()

async def left():
    turn_motor.stop()
    turn_motor.start(-turn_speed)

async def right():
    turn_motor.stop()
    turn_motor.start(turn_speed)

async def stop_turn():
    turn_motor.stop()

async def reverse():
    forward_motor.stop()
    forward_motor.start(-speed)

async def deploy_payload():
    payload_motor.start(2)

async def reverse_payload():
    payload_motor.start(-2)

async def stop_payload():
    payload_motor.stop()

async def straighten(pos, target):
    turn_motor.run_for_degrees(target - pos, turn_speed)
    return 0

async def turn_left(current_pos, central_pos, amount):
    if (current_pos > central_pos - max_turn):
        turn_motor.run_for_degrees(0 - amount)
    return 0
    
async def turn_right(current_pos, central_pos, amount):
    if (current_pos < central_pos + max_turn):
        turn_motor.run_for_degrees(amount)
    return 0

async def auto_controller(central_pos):
    while True:
        left_in = lf_left.value
        right_in = lf_right.value
        turn_pos = turn_motor.get_position()
        #print(f"left: {left_in} right:{right_in} turn_pos: {turn_pos}")
        if left_in and right_in:
            #asyncio.run(straighten(turn_pos, central_pos))
            if (turn_pos < central_pos):
                await turn_left(turn_pos, central_pos, 20)
            else:
                await turn_right(turn_pos, central_pos, 20)
        elif left_in:
            await turn_left(turn_pos, central_pos, 20)
        elif right_in:
            await turn_right(turn_pos, central_pos, 20)
        else:
            await straighten(turn_pos, central_pos)
        await time.sleep(0.1)


async def manual_controller(central_pos): # This area of stuff can be used (according to el gpt) if we want to manually control how the robot works, instead of having it work autonomously.
    while True:
        command = await input("current command: ")
        turn_pos = await turn_motor.get_position() # I HATE IF-ELIF-ELSE NOW
        if command == "w":
            asyncio.run(start())
            print("Started motors going forward")
        elif command == "s":
            asyncio.run(stop())
            print("Stopped motors")
        elif command == "r":
            asyncio.run(reverse())
            print("Started motors going reverse")
        elif command == "a":
            asyncio.run(turn_left(turn_pos, 45))
            print("Started turning left")
        elif command == "d":
            asyncio.run(turn_right(turn_pos, 45))
            print("Started turning right")
        elif command == "q":
            asyncio.run(straighten(turn_pos, central_pos))
            print(f"Re-straightned. Current pos:{turn_pos}")
        elif command == "wp":
            asyncio.run(deploy_payload())
            print("deploying payload")
        elif command == "rp":
            asyncio.run(reverse_payload())
            print("retrieving payload")
        elif command == "sp":
            asyncio.run(stop_payload())
            print("stopping payload motor")
        elif command == "p":
            speed += 1
            asyncio.run(start())
            print(f"Increasing speed to {speed}")
        elif command == "l":
            speed -= 1
            asyncio.run(start())
            print(f"Decreasing speed to {speed}")

mode = input("mode: ")
if mode == "monitorIMU":
    while True:
        asyncio.run(monitor_hall())
        time.sleep(1)
elif mode == "manual":
    central_pos = turn_motor.get_position()
    while True:
        command = input("current command: ")
        turn_pos = turn_motor.get_position()
        if command == "w":
            asyncio.run(start())
            print("Started motors going forward")
        elif command == "s":
            asyncio.run(stop())
            print("Stopped motors")
        elif command == "r":
            asyncio.run(reverse())
            print("Started motors going reverse")
        elif command == "a":
            asyncio.run(turn_left(turn_pos, central_pos, 45))
            print("Started turning left")
        elif command == "d":
            asyncio.run(turn_right(turn_pos, central_pos, 45))
            print("Started turning right")
        elif command == "q":
            turn_pos = turn_motor.get_position()
            asyncio.run(straighten(turn_pos, central_pos))
            print(f"Re-straightned. Current pos:{turn_pos}")
        elif command == "wp":
            asyncio.run(deploy_payload())
            print("deploying payload")
        elif command == "rp":
            asyncio.run(reverse_payload())
            print("retrieving payload")
        elif command == "sp":
            asyncio.run(stop_payload())
            print("stopping payload motor")
        elif command == "p":
            speed += 1
            asyncio.run(start())
            print(f"Increasing speed to {speed}")
        elif command == "l":
            speed -= 1
            asyncio.run(start())
            print(f"Decreasing speed to {speed}")
elif mode == "auto":
    forward_motor.stop()
    forward_motor.start(speed)
    central_pos = turn_motor.get_position()
    direction = 0
    while True:
        left_in = lf_left.value
        right_in = lf_right.value
        turn_pos = turn_motor.get_position()
        print(f"left: {left_in} right:{right_in} turn_pos: {turn_pos}")
        if left_in and right_in:
            #asyncio.run(straighten(turn_pos, central_pos))
            if (turn_pos < central_pos):
                asyncio.run(turn_left(turn_pos, central_pos, 20))
            else:
                asyncio.run(turn_right(turn_pos, central_pos, 20))
        elif left_in:
            asyncio.run(turn_left(turn_pos, central_pos, 20))
        elif right_in:
            asyncio.run(turn_right(turn_pos, central_pos, 20))
        else:
            asyncio.run(straighten(turn_pos, central_pos))
        time.sleep(0.1)

elif mode == "assist":
    forward_motor.stop()
    forward_motor.start(speed)
    central_pos = turn_motor.get_position()
    asyncio.run(auto_controller(central_pos))
    asyncio.run(manual_controller(central_pos))
