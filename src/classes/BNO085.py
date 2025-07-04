import time
import board
import busio
from math import atan2, sqrt, pi
import adafruit_tcs34725
import math
from adafruit_bno08x import (
	BNO_REPORT_ACCELEROMETER,
	BNO_REPORT_GYROSCOPE,
	BNO_REPORT_MAGNETOMETER,
	BNO_REPORT_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C


class IMUandColorSensor:
	def __init__(self, scl, sda, frequency=400000):
		self.i2c = busio.I2C(scl, sda, frequency=frequency)
		self.i2c1 = busio.I2C(scl, sda)
		self.bno = BNO08X_I2C(self.i2c)
		self.prev_imu = 0
		#self.bno.begin_caliberation()
		#print(f"Calibaretion:{self.bno.begin_calibration()}")
		self.sensor = adafruit_tcs34725.TCS34725(self.i2c1)
		self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
		self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
		self.bno.enable_feature(BNO_REPORT_MAGNETOMETER)
		self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
		self.sensor.gain = 60
		self.color_rgb = [0, 0, 0]


	def quaternion_to_heading(self, w, x, y, z):
		# Compute the yaw (heading) from the quaternion
		heading = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

		# Convert from radians to degrees
		heading_degrees = math.degrees(heading)

		# Normalize the heading to 0-360 degrees
		if heading_degrees > 0:
			heading_degrees = 360 - heading_degrees
		else:
			heading_degrees = abs(heading_degrees)


		return heading_degrees


	def read_imu(self):
		time.sleep(0.01)
		quat_i, quat_j, quat_k, quat_real = self.bno.quaternion
		heading = self.quaternion_to_heading(quat_real, quat_i, quat_j, quat_k)

		'''if (abs((heading - prev_heading)/(time.time() - prev_time)) > 10) :
			heading = prev_heading
			print("Wrong imu...")'''
		self.prev_imu = heading
		#self.prev_time = time.time()
		#print(f"{abs((heading - prev_heading)/(time.time() - prev_time))}")
		return heading

	def get_color(self):
		#time.sleep(0.001)
		self.color_rgb = self.sensor.color_rgb_bytes
		if self.color_rgb[2] < 20 and (self.color_rgb[0] < 30 and self.color_rgb[0] > 20) and self.color_rgb[1] < 20:
			return "Blue"
		elif self.color_rgb[2] < 20 and (self.color_rgb[0] >= 125 and self.color_rgb[0] <= 255) and self.color_rgb[1] < 50:
			return  "Orange"
		else:
			return "White"

	def close(self):
		GPIO.cleanup()
