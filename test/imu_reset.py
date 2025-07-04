
import RPi.GPIO as GPIO
import board
from BNO085 import IMUandColorSensor

GPIO.setmode(GPIO.BCM)

GPIO.setup(5, GPIO.IN, pull_up_down=GPIO.PUD_UP)#Button to GPIO23

imu = IMUandColorSensor(board.SCL, board.SDA)

previous_state = 0
button_state = 0
button  = False
reset = False
while True:
	previous_state = button_state
	button_state = GPIO.input(5)
	if(previous_state == 1 and button_state == 0):
		button = not(button)

	if button:
		reset = True

	else:
		reset = False

	head = imu.read_imu(reset)
	print(head)

GPIO.cleanup()
