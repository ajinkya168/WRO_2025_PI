import pigpio
import RPi.GPIO as GPIO
import math
import board
import busio
from math import atan2, sqrt, pi
from robotModule.classes.Servo import Servo
from adafruit_bno08x import (
 BNO_REPORT_ACCELEROMETER,
 BNO_REPORT_GYROSCOPE,
 BNO_REPORT_MAGNETOMETER,
 BNO_REPORT_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C
i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
bno = BNO08X_I2C(i2c)
bno.enable_feature(BNO_REPORT_ACCELEROMETER)
bno.enable_feature(BNO_REPORT_GYROSCOPE)
bno.enable_feature(BNO_REPORT_MAGNETOMETER)
bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)




GPIO.setmode(GPIO.BCM)
GPIO.setup(20, GPIO.OUT) # Connected to AIN2
GPIO.setup(12, GPIO.OUT)
pwm12 = GPIO.PWM(12, 100)
servo = Servo(8)
# Set up GPIO pins
pi = pigpio.pi()
pi.set_mode(9, pigpio.INPUT)
pi.set_pull_up_down(9, pigpio.PUD_UP) # channel A
pi.set_mode(11, pigpio.INPUT)
pi.set_pull_up_down(11, pigpio.PUD_UP) # channel B

prev_state_A = pi.read(9)
prev_state_B = pi.read(11)
count = 0
r = 2
x1  = 0
y1 = 0
multiplier = 2
prev_count = 0
cpr = 1040
const = 2*math.pi*2
error_x = 0
error_y = 0
prev_distance = 0
def find_heading(dqw, dqx, dqy, dqz):
    norm = sqrt(dqw * dqw + dqx * dqx + dqy * dqy + dqz * dqz)
    dqw = dqw / norm
    dqx = dqx / norm
    dqy = dqy / norm
    dqz = dqz / norm

    ysqr = dqy * dqy

    t3 = +2.0 * (dqw * dqz + dqx * dqy)
    t4 = +1.0 - 2.0 * (ysqr + dqz * dqz)
    yaw_raw = atan2(t3, t4)
    
   
    yaw = yaw_raw * 180.0 / math.pi
    yaw = yaw - 180

    if yaw > 0:
        yaw = 360 - yaw
    else:
        yaw = abs(yaw)
    return yaw  # heading in 360 clockwise



def edge_detected(gpio, level, tick):
	global prev_state_A, prev_state_B, count
	state_A = pi.read(9)
	state_B = pi.read(11)
	#print(prev_state_A)
		    
	if state_B != state_A :

		count -= 1
	else:
		count += 1
	#prev_state_A = state_A
	#prev_state_B = state_B

# Set up callbacks for rising edge



pwm12.start(0)
try:
	dist = int(input())

	cb_A = pi.callback(9, pigpio.EITHER_EDGE, edge_detected)
	cb_B = pi.callback(11, pigpio.EITHER_EDGE, edge_detected)

	

	servo.setAngle(90)
	while True:
		revolution = count/cpr
		distance_cm = revolution*const*1.2
		quat_i, quat_j, quat_k, quat_real = bno.quaternion
		GPIO.output(20, GPIO.HIGH)	
	#	pwm12.ChangeDutyCycle(60)
		heading = find_heading(quat_real, quat_i, quat_j, quat_k)
		# Do something with the count value
		if(distance_cm < dist):

				pwm12.ChangeDutyCycle(60)
				
		else:
				pwm12.ChangeDutyCycle(0)
				
				break
		change = (distance_cm - prev_distance)
		dx = math.cos(math.radians(heading)) * change
		dy = math.sin(math.radians(heading)) * change
		error_x += dx - math.cos(math.radians(heading - error_x)) * change
		error_y += dy - math.sin(math.radians(heading - error_y)) * change
		x1 = dx + x1
		y1 = dy + y1
		#print("Count = {}".format(count))		
		#print("Count = {}".format(count))
		print("x: {}, y: {}, heading : {}, dist : {}, count:{}, rev: {}".format(x1, y1, heading, distance_cm, count, revolution))
		prev_distance = distance_cm
		#time.sleep(0.0001)
except KeyboardInterrupt:
    cb_A.cancel()
    cb_B.cancel()
    pi.stop() # Clean up pigpio on exit
    GPIO.cleanup()
finally:
	GPIO.cleanup()
