from buildhat import Motor
from basehat import LineFinder, Button
import time

def main():
	# For this code, please plug the motors into ports A, B, and C on the buildhat.
	# The line finder uses 
	motorLeft = Motor('A')
	motorRight = Motor('B')
	motorFront = Motor('C')
	button = Button(22)
	linePin = 16

	lineFinder = LineFinder(linePin)
	direction = 0
	while True:
		button_value = Button.value
	
		print(f"lineFinder value: {sensor_value}, Direction: {direction}")

		if lineFinder.line_detected:
			if direction == 0:
				motorLeft.start(20)
				motorRight.start(-20) # the right turn 
				motorFront.run_for_degrees(45) 
				direction = 1
			elif direction > 0:
				if direction > 10:
					direction += 1
				else: 
					direction = -1
					motorLeft.start(-20)
					motorRight.start(-20)
					motorFront.start()
			else:
				if direction > 10:
					direction -= 1
				else:
					direction = 1
					motorLeft.start(10)
					motorRight.start(-10)
					motorFront.start() 
		else:
			direction = 0
			motorLeft.start()
			motorRight.start()
			motorFront.start()
		time.sleep(0.1)

if __name__ == "__main__":
	main()
			
				
			
