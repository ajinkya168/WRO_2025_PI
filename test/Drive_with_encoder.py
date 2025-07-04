import pigpio
import RPi.GPIO as GPIO
import time
import math
import serial
GPIO.setmode(GPIO.BCM)
GPIO.setup(20, GPIO.OUT) # Connected to AIN2
GPIO.setup(12, GPIO.OUT)
pwm12 = GPIO.PWM(12, 100)

# Set up GPIO pins
pi = pigpio.pi()
pi.set_mode(7, pigpio.INPUT)
pi.set_pull_up_down(7, pigpio.PUD_UP) # channel A
pi.set_mode(8, pigpio.INPUT)
pi.set_pull_up_down(8, pigpio.PUD_UP) # channel B
#ser = serial.Serial('/dev/ttyUSB0', 115200)

prev_state_A = pi.read(7)
prev_state_B = pi.read(8)
count = 0
r = 2
multiplier = 1.1
const = 52/(2*math.pi*r)

"""
def edge_detected(gpio, level, tick):
    global prev_state_A, prev_state_B, count
    state_A = pi.read(7)
    state_B = pi.read(8)
    #print(prev_state_A)
    if prev_state_A == 0 and state_A == 1:
       # if state_B == 1:
            count += 1
        #else:
            #count -= 1

    prev_state_A = state_A
# Set up callbacks for rising edge
cb_A = pi.callback(7, pigpio.RISING_EDGE, edge_detected)
cb_B = pi.callback(8, pigpio.RISING_EDGE, edge_detected)
"""

try:
	#dist = int(input())
	#total = dist * 52 * multiplier
	#print(total)
	pwm12.start(0)
	while True:
		# Do something with the count value
		#if(total>0):
		pwm12.ChangeDutyCycle(60)
				#total = total - 1 
		#else:
				#pwm12.ChangeDutyCycle(0)						
		GPIO.output(20, GPIO.HIGH)  
		print("Count : ", total)
		#time.sleep(0.0005)
except KeyboardInterrupt:
    cb_A.cancel()
    cb_B.cancel()
    pi.stop() # Clean up pigpio on exit
    GPIO.cleanup()
finally:
	GPIO.cleanup()
