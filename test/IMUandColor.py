import time
import board
import busio
from math import atan2, sqrt, pi
import math
import RPi.GPIO as GPIO
import colorsys
from BNO085 import IMUandColorSensor
import serial

GPIO.setmode(GPIO.BCM)
GPIO.setup(14, GPIO.OUT)
GPIO.output(14, GPIO.LOW)
time.sleep(1)  
GPIO.output(14, GPIO.HIGH)


imu = IMUandColorSensor(board.SCL, board.SDA)
while True:
	#print(1 / (time.time() - prev_time))
	#prev_time = time.time()
	try:
		color_sensor = imu.get_color()
		'''if (imu.color_rgb[0] == 0 and imu.color_rgb[1] == 0 and imu.color_rgb[2] == 0) or (imu.hsv_r == 0 and imu.hsv_g == 0 and imu.hsv_b == 0):
			continue'''
		print(f"{color_sensor}")
		#print(f"0: {color_hsv[0]//1} 1:{color_hsv[1]//1} 2:{color_hsv[2]//1}")
		#print(f"r: {color_rgb[0]} g:{color_rgb[1]} b:{color_rgb[2]}")
	except Exception as e:
		print(e)
		pass	

GPIO.cleanup()
