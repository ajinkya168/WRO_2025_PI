import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

GPIO.setup(5, GPIO.IN, pull_up_down=GPIO.PUD_UP)#Button to GPIO23

previous_state = 0
button_state = 0
button  = False

while True:
	time.sleep(0.01)
	previous_state = button_state
	button_state = GPIO.input(5)
	if(previous_state == 1 and button_state == 0):
		button = not(button)
		
	print(button)


GPIO.cleanup()
