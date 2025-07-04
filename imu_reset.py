import time
import board
import busio
from math import atan2, sqrt, pi
import adafruit_tcs34725
import keyboard
import RPi.GPIO as GPIO
from digitalio import DigitalInOut

from adafruit_bno08x import (
	BNO_REPORT_ACCELEROMETER,
	BNO_REPORT_GYROSCOPE,
	BNO_REPORT_MAGNETOMETER,
	BNO_REPORT_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C


reset_pin = DigitalInOut(board.D19)

i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
i2c1 = busio.I2C(board.SCL, board.SDA)

sensor = adafruit_tcs34725.TCS34725(i2c)
bno = BNO08X_I2C(i2c1,  reset = reset_pin)
bno.begin_calibration()

bno.enable_feature(BNO_REPORT_ACCELEROMETER)
bno.enable_feature(BNO_REPORT_GYROSCOPE)
bno.enable_feature(BNO_REPORT_MAGNETOMETER)
bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
sensor.gain = 60
prev_time = 0
quat_i =0 
quat_j = 0
quat_k =0
quat_real = 0

#GPIO.output(RESET_PIN, GPIO.LOW) 

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

	yaw = yaw_raw * 180.0 / pi
	yaw = yaw - 180

	if yaw > 0:
		yaw = 360 - yaw
	else:
		yaw = abs(yaw)
	return yaw  # heading in 360 clockwise

def reset_imu():
    bno.soft_reset()
    print("IMU reset")

'''def on_key_press(event):
    if event.name == 'q':
    	reset_imu()
    	
    	print("You pressed 'q'!")'''
imu_reset = False
def on_key_press(event):
	if event.name == 'q':
		print("q is pressed")
		global imu_reset
		imu_reset = True
		# return

if __name__ == '__main__':
	keyboard.on_press(on_key_press)	
	while True:
	
		#print(1 / (time.time() - prev_time))
		#prev_time = time.time()
		#res = int(input())
		if imu_reset:
			bno.soft_reset()
		
			print(f"{bno.quaternion}") 
			print("Reset Complete") 
			imu_reset = False
		
		try:
			color_rgb = sensor.color_rgb_bytes
			#time.sleep(0.1)
			if not imu_reset:
				quat_i, quat_j, quat_k, quat_real = bno.quaternion
			# print("Rotation Vector Quaternion:")
			
			heading = find_heading(quat_real, quat_i, quat_j, quat_k)

			print(f"Heading: {heading} {bno.quaternion}")					
				
			if (color_rgb[0] == 0 and color_rgb[1] == 0 and color_rgb[2] == 0):
				continue
			elif color_rgb[2]< 15 and (color_rgb[0] < 55 and color_rgb[0] < 80) and color_rgb[1] < 25 :
				color_n = "Blue"
			elif color_rgb[2] < 10 and (color_rgb[0] > 70 and color_rgb[0]< 80) and color_rgb[1] < 15:
				color_n = "Orange"
			else:
				color_n = "White"
			#rint(f"Color: {color_n}, {color_rgb}".format(color_n))
			#print(f"r: {color_rgb[0]} g:{color_rgb[1]} b:{color_rgb[2]}")
		except Exception as e:
			print(f"Exception: {e}")
			pass	

	GPIO.cleanup()
