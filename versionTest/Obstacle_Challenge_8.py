import os

os.system('sudo pkill pigpiod')
os.system('sudo pigpiod')
import sys
import logging

import numpy as np
import RPi.GPIO as GPIO
import cv2
from picamera2 import Picamera2
import time

import multiprocessing
import pigpio
import board

import math
from Encoder import EncoderCounter
from BNO085 import IMUandColorSensor
from Servo import Servo
import serial
import RPi.GPIO as GPIO
from TFmini import TFmini

log_file = open('/home/pi/WRO_CODE/logs/log_8.txt', 'w')
sys.stdout = log_file
time.sleep(5)
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

#### PINS

RX_Head = 23
RX_Left = 24
RX_Right = 25
RX_Back = 27
button_pin = 5
servo_pin = 8
blue_led = 26
red_led = 10
green_led = 6
reset_pin = 8

#### INITIALIZATION

GPIO.setup(reset_pin, GPIO.OUT)
GPIO.setup(blue_led, GPIO.OUT)  # blue
GPIO.setup(red_led, GPIO.OUT)  # red
GPIO.setup(green_led, GPIO.OUT)  # green
GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Button to GPIO5

#####################################

GPIO.output(blue_led, GPIO.LOW)
GPIO.output(green_led, GPIO.LOW)
GPIO.output(red_led, GPIO.LOW)

############ RESETTING ARDUINO #########3

print("Resetting....")
GPIO.output(reset_pin, GPIO.LOW)
GPIO.output(green_led, GPIO.HIGH)
time.sleep(1)
GPIO.output(reset_pin, GPIO.HIGH)
GPIO.output(green_led, GPIO.LOW)
time.sleep(1)
print("Reset Complete")

print("######################## LOGS ############################")


########### IMPORTING CLASSES ###############
servo = Servo(servo_pin)
imu = IMUandColorSensor(board.SCL, board.SDA)
tfmini = TFmini(RX_Head, RX_Left, RX_Right, RX_Back)

############ PID VARIABLES #############
currentAngle = 0
error_gyro = 0
prevErrorGyro = 0
totalErrorGyro = 0
correcion = 0
totalError = 0
prevError = 0

kp = 0.6
ki = 0
kd = 0.1

kp_e = 3  # 12
ki_e = 0
kd_e = 40  # 40if

corr = 0
corr_pos = 0


##############################################


def correctPosition(setPoint, head, x, y, counter, blue, orange, reset, reverse, heading, centr_x, finish):
	# print("INSIDE CORRECT")
	# getTFminiData()
	global prevError, totalError, prevErrorGyro, totalErrorGyro, corr_pos

	error = 0
	correction = 0
	pTerm_e = 0
	dTerm_e = 0
	iTerm_e = 0
	lane = counter % 4
	# if(time.time() - last_time > 0.001):
	if lane == 0:
		error = setPoint - y
		print(f"2 lane: {lane}, error: {error} target:{(setPoint)}, x:{x} y:{y} not reverse")
	elif lane == 1:
		if orange:
			error = x - (100 - setPoint)
			print(f"lane:{lane}, error:{error} target:{(100 - setPoint)}, x:{x}, y:{y}")

		elif blue:
			error = (100 + setPoint) - x
			print(f"lane:{lane}, error:{error} target:{(100 + setPoint)}, x:{x} y:{y} Bluee")
	# print(f" trigger : {flag_t} setPoint: {setPoint} lane: {lane} correction:{correction}, error:{error} x:{x}, y:{y}, prevError :{prevError} angle:{head - correction}")
	elif lane == 2:
		if orange:
			error = y - (200 - setPoint)
			print(f"lane:{lane} error:{error} target:{(200 - setPoint)},  x: {x} y{y}")
		elif blue:
			error = y - (-200 - setPoint)
			print(f"lane:{lane} error:{error} target:{(-200 - setPoint)}, x: {x} y{y}")
	# print(f"setPoint: {flag_t} lane: {lane} correction:{correction}, error:{error} x:{x}, y:{y}, prevError :{prevError} angle:{head - correction}")
	elif lane == 3:
		if orange:
			error = (setPoint - 100) - x
			print(f"lane:{lane} error:{error} target:{(setPoint - 100)}, x: {x} y {y}")

		elif blue:
			error = x + (100 + setPoint)
			print(f"lane:{lane} error:{error} target:{(100 + setPoint)}, x:{x} y {y}")

	corr_pos = error
	pTerm_e = kp_e * error
	dTerm_e = kd_e * (error - prevError)
	totalError += error
	iTerm_e = ki_e * totalError
	correction = pTerm_e + iTerm_e + dTerm_e

	if setPoint == 0:
		if abs(error) < 10:
			print("absolute is 0")
			correction = 0

	if not reset:
		tfmini.getTFminiData()
		print(f"centr_pink:{centr_x_pink.value}")
		if ((setPoint == -35 and orange) or (counter == 0 and (centr_x_pink.value < 800 and centr_x_pink.value > 0) and ((centr_y.value or centr_y_red.value) <= centr_y_pink.value) and not blue and not orange) and not finish):
			if tfmini.distance_left <= 30:
				correction = 20
				print(f"Avoiding pink wall {correction}")
				
			elif tfmini.distance_right < 50:
				if tfmini.distance_right <= 35:
					correction = -45
					print(f"Avoiding pink wall {correction}")

				else:
					correction = -10
					print(f"Avoiding pink wall {correction}")

			else:
				correction = 0

		elif ((setPoint == 35 and blue) or (counter == 0 and (centr_x_pink.value < 800 and centr_x_pink.value > 0) and ((centr_y.value or centr_y_red.value) <= centr_y_pink.value) and not blue and not orange) and not finish):

			if tfmini.distance_right <= 30:
				correction = -20
				print(f"Avoiding pink wall {correction}")
				
			elif tfmini.distance_left < 50 or tfmini.distance_left > 100:
				if tfmini.distance_left <= 35:
					correction = 45
					print(f"Avoiding pink wall {correction}")

				else:
					correction = 10
					print(f"Avoiding pink wall {correction}")
			else:
				correction = 0

		if not blue:
			if (setPoint <= -70) and tfmini.distance_left <= 22:
				print(f"Correcting Green Wall Orange")
				correction = 10
			else:
				pass

			if setPoint >= 70 and (tfmini.distance_right <= 20 or (tfmini.distance_head <= 18)):
				print(f"Wall detected..making correction")
				correction = -10
			else:
				pass

		else:
			if setPoint <= -70 and (tfmini.distance_left <= 20 or (tfmini.distance_head <= 18)):
				print(f"Wall detected..making correction")
				correction = 10
			else:
				pass

			if setPoint >= 70 and tfmini.distance_right <= 22:
				print(f"correctng red wall in blue")
				correction = -10
			else:
				pass

	if setPoint == 0:
		if correction > 25:
			correction = 25
		elif correction < -25:
			correction = -25
	else:
		if correction > 45:
			correction = 45
		elif correction < -45:
			correction = -45

	prevError = error
	correctAngle(head + correction, heading)


def correctAngle(setPoint_gyro, heading):
	global corr
	error_gyro = 0
	prevErrorGyro = 0
	totalErrorGyro = 0
	correction = 0
	totalError = 0
	prevError = 0

	error_gyro = heading - setPoint_gyro

	if error_gyro > 180:
		error_gyro = error_gyro - 360
	corr = error_gyro
	# print("Error : ", error_gyro)
	pTerm = 0
	dTerm = 0
	iTerm = 0

	pTerm = kp * error_gyro
	dTerm = kd * (error_gyro - prevErrorGyro)
	totalErrorGyro += error_gyro
	iTerm = ki * totalErrorGyro
	correction = pTerm + iTerm + dTerm

	if correction > 30:
		correction = 30
	elif correction < -30:
		correction = -30

	prevErrorGyro = error_gyro
	servo.setAngle(90 - correction)


# Extract Frames

# basic constants for opencv Functs
kernel = np.ones((3, 3), 'uint8')
font = cv2.FONT_HERSHEY_SIMPLEX
org = (0, 20)
fontScale = 0.6
color = (0, 0, 255)
thickness = 2


# loop to capture video frames
def Live_Feed(color_b, stop_b, red_b, green_b, pink_b, centr_y, centr_x, centr_y_red, centr_x_red, centr_x_pink, centr_y_pink, centr_y_b, blue_b, prev_b, orange_o, centr_y_o):
	print('Image Process started')
	both_flag = False
	all_flag = False
	only_red = False
	only_green = False
	only_pink = False
	pink_red = False
	pink_green = False
	white_present = False
	blue_present = False
	orange_present = False
	dist = -1
	centroid_x_b = 0
	centroid_y_b = 0
	centroid_y_o = 0
	centroid_y_o = 0
	fps_time = 0
	max_cnt = np.array([[[0, 0]]])
	max_cnt1 = np.array([[[0, 0]]])
	max_cnt2 = np.array([[[0, 0]]])
	max_cnt_b = np.array([[[0, 0]]])
	max_cnt_o = np.array([[[0, 0]]])
	picam2 = Picamera2()

	picam2.preview_configuration.main.size = (1600, 1000)  # (1300, 800)
	picam2.preview_configuration.main.format = 'RGB888'

	picam2.preview_configuration.align()
	picam2.configure('preview')

	picam2.start()

	cv2.namedWindow('Object Frame', cv2.WINDOW_NORMAL)
	cv2.resizeWindow('Object Frame', 400, 200)
	time.sleep(2)
	GPIO.output(green_led, GPIO.HIGH)

	while True:
		print(f"fps:{1/(time.time() - fps_time)}")
		fps_time = time.time() 
		prev_b.value = centroid_y_b
		img = picam2.capture_array()

		hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # green
		hsv_img1 = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # red
		hsv_img2 = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # pink
		hsv_img3 = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # white
		hsv_img_blue = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # blue
		hsv_img_orange = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # orange

		# predefined mask for green colour detection
		# For Green Color
		lower = np.array([55, 54, 50])  # green
		upper = np.array([73, 130, 122])
		mask = cv2.inRange(hsv_img, lower, upper)

		# For Red Color
		lower1 = np.array([0, 125, 83])  # red
		upper1 = np.array([4, 228, 163])
		r1lower = np.array([171, 125, 83])
		r1upper = np.array([179, 228, 163])
		mask1 = cv2.inRange(hsv_img1, lower1, upper1)
		mask_red = cv2.inRange(hsv_img1, r1lower, r1upper)
		mask1 = mask1 + mask_red

		# For Pink Color
		lower2 = np.array([160, 74, 79])  # pink
		upper2 = np.array([174, 180, 210])
		mask2 = cv2.inRange(hsv_img2, lower2, upper2)
		
		# For white color
		lower3 = np.array([0, 0, 69])  # white                              
		upper3 = np.array([0, 68, 255])
		r1lower1 = np.array([38, 0, 69])
		r1upper1 = np.array([179, 68, 255])
		mask3 = cv2.inRange(hsv_img3, lower1, upper1)
		mask_white = cv2.inRange(hsv_img3, r1lower1, r1upper1)
		mask3 = mask3 + mask_white
		
		# blue line
		lower_blue = np.array([106, 110, 41])  # blue
		upper_blue = np.array([126, 222, 114])
		mask_blue = cv2.inRange(hsv_img_blue, lower_blue, upper_blue)
		
		# Remove Extra garbage from image
		d_img = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)  # green
		d_img = cv2.morphologyEx(d_img, cv2.MORPH_CLOSE, kernel, iterations=2)  # green
		d_img1 = cv2.morphologyEx(mask1, cv2.MORPH_OPEN, kernel, iterations=2)  # red
		d_img1 = cv2.morphologyEx(d_img1, cv2.MORPH_CLOSE, kernel, iterations=2)  # red
		d_img2 = cv2.morphologyEx(mask2, cv2.MORPH_OPEN, kernel, iterations=2)  # pink
		d_img2 = cv2.morphologyEx(d_img2, cv2.MORPH_CLOSE, kernel, iterations=2)  # pink
		d_img3 = cv2.morphologyEx(mask3, cv2.MORPH_CLOSE, kernel, iterations=2)  # white
		d_img3 = cv2.morphologyEx(d_img3, cv2.MORPH_CLOSE, kernel, iterations=2)  # white
		d_img_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_CLOSE, kernel, iterations=2)  # blue
		d_img_blue = cv2.morphologyEx(d_img_blue, cv2.MORPH_CLOSE, kernel, iterations=2)  # blue
		# find the histogram
		cont, hei = cv2.findContours(d_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		cont = sorted(cont, key=cv2.contourArea, reverse=True)[:1]

		cont1, hei1 = cv2.findContours(d_img1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		cont1 = sorted(cont1, key=cv2.contourArea, reverse=True)[:1]

		cont2, hei2 = cv2.findContours(d_img2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		cont2 = sorted(cont2, key=cv2.contourArea, reverse=True)[:1]

		cont3, hei3 = cv2.findContours(d_img3, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		cont3 = sorted(cont3, key=cv2.contourArea, reverse=True)[:1]
		
		cont_b, hei_b = cv2.findContours(d_img_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		cont_b = sorted(cont_b, key=cv2.contourArea, reverse=True)[:1]

		# ----------------------------------------------
		if len(cont) == 0:
			green_present = False
		else:
			max_cnt = max(cont, key=cv2.contourArea)
			if cv2.contourArea(max_cnt) > 2000:
				green_present = True
			else:
				green_present = False

		# ---------------------------------------
		if len(cont1) == 0:
			red_present = False

		else:
			max_cnt1 = max(cont1, key=cv2.contourArea)
			if cv2.contourArea(max_cnt1) > 2000:
				red_present = True
			else:
				red_present = False
		# ----------------------------------------------------
		if len(cont2) == 0:
			pink_present = False

		else:
			max_cnt2 = max(cont2, key=cv2.contourArea)
			if cv2.contourArea(max_cnt2) > 2000:
				pink_present = True
			else:
				pink_present = False
		# --------------------------------------------------------
		
		if len(cont3) == 0:
			white_present = False

		else:
			max_cnt3 = max(cont3, key=cv2.contourArea)
			white_present = True
			
		if len(cont_b) == 0:
			blue_present = False
		else:
			max_cnt_b = max(cont_b, key=cv2.contourArea)
			if cv2.contourArea(max_cnt_b) > 2000:
				blue_present = True
			else:
				blue_present = False
				
		'''if len(cont_o) == 0:
			orange_present = False
		else:
			max_cnt_o = max(cont_o, key=cv2.contourArea)
			if cv2.contourArea(max_cnt_o) > 2000:
				orange_present = True
			else:
				orange_present = False'''
				
		#print(f"{type(cont3)} {type(max_cnt3)}, {type(np.array(cont3))}")		
			
		if not red_present and not green_present and not pink_present:
			color_b.value = False
			red_b.value = False
			green_b.value = False
			pink_b.value = False
			all_flag = False
			both_flag = False
			pink_red = False
			pink_green = False
			only_red = False
			only_green = False
			only_pink = False

		if (red_present and green_present and pink_present):
			all_flag = True
			both_flag = False
			pink_red = False
			pink_green = False
			only_red = False
			only_green = False
			only_pink = False
		elif (red_present and green_present) and not pink_present:
			both_flag = True
			all_flag = False
			only_red = False
			only_green = False
			only_pink = False
			pink_red = False
			pink_green = False
		elif red_present and (not pink_present and not green_present):
			only_red = True
			both_flag = False
			all_flag = False
			only_green = False
			only_pink = False
			pink_red = False
			pink_green = False
		elif green_present and (not pink_present and not red_present):
			only_green = True
			both_flag = False
			all_flag = False
			only_red = False
			only_pink = False
			pink_red = False
			pink_green = False
		elif pink_present and (not green_present and not red_present):
			only_pink = True
			both_flag = False
			all_flag = False
			only_red = False
			only_green = False
			pink_red = False
			pink_green = False
		elif (pink_present and green_present) and not red_present:
			only_pink = False
			both_flag = False
			all_flag = False
			only_red = False
			only_green = False
			pink_red = False
			pink_green = True

		elif (pink_present and red_present) and not green_present:
			only_pink = False
			both_flag = False
			all_flag = False
			only_red = False
			only_green = False
			pink_red = True
			pink_green = False
			
		if (cv2.contourArea(max_cnt_b) > 10000 and cv2.contourArea(max_cnt_b) < 80000):
			# Draw a rectange on the contour
			rect_b = cv2.minAreaRect(max_cnt_b)
			box = cv2.boxPoints(rect_b)
			box = np.intp(box)
			cv2.drawContours(img, [box], -1, (0, 255, 0), 3)

			(x, y, w, h) = cv2.boundingRect(box)

			centroid_y_b = y + h // 2
			centroid_x_b = x + w // 2
			
			centr_y_b.value = centroid_y_b
			blue_b.value = True
		else:
			blue_b.value = False
			centroid_y_b = 0
			centr_y_b.value = centroid_y_b
			
		'''if (cv2.contourArea(max_cnt_o) > 10000 and cv2.contourArea(max_cnt_o) < 35000):
			# Draw a rectange on the contour
			rect_o = cv2.minAreaRect(max_cnt_o)
			box = cv2.boxPoints(rect_o)
			box = np.intp(box)
			cv2.drawContours(img, [box], -1, (0, 255, 0), 3)

			(x, y, w, h) = cv2.boundingRect(box)

			centroid_y_o = y + h // 2
			centroid_x_o = x + w // 2
			
			centr_y_o.value = centroid_y_o
			orange_o.value = True
		else:
			orange_o.value = False
			centroid_y_o = 0
			centr_y_o.value = centroid_y_o'''

		if all_flag:
			color_b.value = True
			# print("ITS THE FIRST LOOP")
			### FOR GREEN BOX
			if cv2.contourArea(max_cnt) > cv2.contourArea(max_cnt1):
				if cv2.contourArea(max_cnt) > 1000 and cv2.contourArea(max_cnt) < 306000:
					# Draw a rectange on the contour
					rect = cv2.minAreaRect(max_cnt)
					
					box = cv2.boxPoints(rect)
					box = np.intp(box)
					#print(f"{box}, {type(box)}")
					cv2.drawContours(img, [box], -1, (255, 0, 0), 3)
					(x, y, w, h) = cv2.boundingRect(box)
					centroid_y = y + h // 2
					centroid_x = x + w // 2
					centr_y.value = centroid_y
					centr_x.value = centroid_x
					centr_y_red.value = 0
					centr_x_red.value = 0
					green_b.value = True
					red_b.value = False



			### FOR RED BOX
			elif cv2.contourArea(max_cnt1) > cv2.contourArea(max_cnt):
				if (cv2.contourArea(max_cnt1) > 1000 and cv2.contourArea(max_cnt1) < 306000):
					# Draw a rectange on the contour
					rect1 = cv2.minAreaRect(max_cnt1)
					box = cv2.boxPoints(rect1)
					box = np.intp(box)
					cv2.drawContours(img, [box], -1, (255, 0, 0), 3)

					(x, y, w, h) = cv2.boundingRect(box)

					centroid_y_red = y + h // 2
					centroid_x_red = x + w // 2

					centr_y_red.value = centroid_y_red
					centr_x_red.value = centroid_x_red
					centr_y.value = 0
					centr_x.value = 0
					red_b.value = True
					green_b.value = False

			#### FOR PINK BOX
			# elif cv2.contourArea(max_cnt2) > cv2.contourArea(max_cnt) and cv2.contourArea(max_cnt2) > cv2.contourArea(max_cnt1):
			if (cv2.contourArea(max_cnt2) > 2000 and cv2.contourArea(max_cnt2) < 306000):
				# Draw a rectange on the contour
				rect2 = cv2.minAreaRect(max_cnt2)
				box = cv2.boxPoints(rect2)
				box = np.intp(box)
				cv2.drawContours(img, [box], -1, (255, 0, 0), 3)

				(x, y, w, h) = cv2.boundingRect(box)

				centroid_x = x + w // 2
				centroid_y = y + h // 2
				centr_x_pink.value = centroid_x
				centr_y_pink.value = centroid_y
				pink_b.value = True


		### FOR RED BOX
		elif only_red:
			color_b.value = True
			# print(cv2.contourArea(max_cnt1))
			if cv2.contourArea(max_cnt1) > 1000 and cv2.contourArea(max_cnt1) < 306000:
				# Draw a rectange on the contour
				rect1 = cv2.minAreaRect(max_cnt1)
				box = cv2.boxPoints(rect1)
				box = np.intp(box)
				cv2.drawContours(img, [box], -1, (255, 0, 0), 3)
				(x, y, w, h) = cv2.boundingRect(box)

				centroid_y_red = y + h // 2
				centroid_x_red = x + w // 2

				centr_y_red.value = centroid_y_red
				centr_x_red.value = centroid_x_red
				centr_y.value = 0
				centr_x.value = 0
				centr_x_pink.value = 0
				centr_y_pink.value = 0

				red_b.value = True
				pink_b.value = False
				green_b.value = False



		### FOR GREEN BOX
		elif only_green:
			color_b.value = True
			# print(cv2.contourArea(max_cnt))
			if cv2.contourArea(max_cnt) > 1000 and cv2.contourArea(max_cnt) < 306000:
				# Draw a rectange on the contour
				rect = cv2.minAreaRect(max_cnt)
				box = cv2.boxPoints(rect)
				box = np.intp(box)
				cv2.drawContours(img, [box], -1, (255, 0, 0), 3)
				(x, y, w, h) = cv2.boundingRect(box)

				centroid_y = y + h // 2
				centroid_x = x + w // 2
				centr_y.value = centroid_y
				centr_x.value = centroid_x
				centr_y_red.value = 0
				centr_x_red.value = 0
				centr_x_pink.value = 0
				centr_y_pink.value = 0
				# if(centroid_y > 100):
				# if(counter_green >= max_count):
				green_b.value = True
				pink_b.value = False
				red_b.value = False

		### FOR PINK BOX
		elif only_pink:
			if (cv2.contourArea(max_cnt2) > 2000 and cv2.contourArea(max_cnt2) < 306000):
				# Draw a rectange on the contour
				rect2 = cv2.minAreaRect(max_cnt2)
				box = cv2.boxPoints(rect2)
				box = np.intp(box)
				cv2.drawContours(img, [box], -1, (255, 0, 0), 3)

				(x, y, w, h) = cv2.boundingRect(box)

				centroid_x = x + w // 2
				centroid_y = y + h // 2
				centr_x_pink.value = centroid_x
				centr_y_pink.value = centroid_y
				centr_y_red.value = 0
				centr_x_red.value = 0
				centr_x.value = 0
				centr_y.value = 0
				red_b.value = False
				green_b.value = False
				pink_b.value = True

		elif both_flag:
			color_b.value = True
			# print("BOTH ARE PRESENT...")
			### FOR GREEN BOX
			if cv2.contourArea(max_cnt) > cv2.contourArea(max_cnt1):
				if cv2.contourArea(max_cnt) > 1000 and cv2.contourArea(max_cnt) < 306000:
					# Draw a rectange on the contour
					rect = cv2.minAreaRect(max_cnt)
					box = cv2.boxPoints(rect)
					box = np.intp(box)
					cv2.drawContours(img, [box], -1, (255, 0, 0), 3)
					(x, y, w, h) = cv2.boundingRect(box)
					centroid_y = y + h // 2
					centroid_x = x + w // 2
					centr_y.value = centroid_y
					centr_x.value = centroid_x
					centr_x_red.value = 0
					centr_y_red.value = 0
					centr_x_pink.value = 0
					centr_y_pink.value = 0
					green_b.value = True

					red_b.value = False
					pink_b.value = False

			### FOR RED BOX
			elif cv2.contourArea(max_cnt1) > cv2.contourArea(max_cnt):
				if (cv2.contourArea(max_cnt1) > 1000 and cv2.contourArea(max_cnt1) < 306000):
					# Draw a rectange on the contour
					rect1 = cv2.minAreaRect(max_cnt1)
					box = cv2.boxPoints(rect1)
					box = np.intp(box)
					cv2.drawContours(img, [box], -1, (255, 0, 0), 3)

					(x, y, w, h) = cv2.boundingRect(box)

					centroid_y_red = y + h // 2
					centroid_x_red = x + w // 2

					centr_y_red.value = centroid_y_red
					centr_x_red.value = centroid_x_red
					centr_x.value = 0
					centr_y.value = 0
					centr_x_pink.value = 0
					centr_y_pink.value = 0
					red_b.value = True
					green_b.value = False
					pink_b.value = False

		elif pink_red:
			### FOR RED BOX
			# if cv2.contourArea(max_cnt1) > cv2.contourArea(max_cnt2):
			if (cv2.contourArea(max_cnt1) > 1000 and cv2.contourArea(max_cnt1) < 306000):
				# Draw a rectange on the contour
				rect1 = cv2.minAreaRect(max_cnt1)
				box = cv2.boxPoints(rect1)
				box = np.intp(box)
				cv2.drawContours(img, [box], -1, (255, 0, 0), 3)

				(x, y, w, h) = cv2.boundingRect(box)

				centroid_y_red = y + h // 2
				centroid_x_red = x + w // 2

				centr_y_red.value = centroid_y_red
				centr_x_red.value = centroid_x_red
				centr_x.value = 0
				centr_y.value = 0
				red_b.value = True
				green_b.value = False  # pink_b.value = False
			# elif cv2.contourArea(max_cnt2) > cv2.contourArea(max_cnt1):
			if (cv2.contourArea(max_cnt2) > 2000 and cv2.contourArea(max_cnt2) < 306000):
				# Draw a rectange on the contour
				rect2 = cv2.minAreaRect(max_cnt2)
				box = cv2.boxPoints(rect2)
				box = np.intp(box)
				cv2.drawContours(img, [box], -1, (255, 0, 0), 3)

				(x, y, w, h) = cv2.boundingRect(box)

				centroid_x = x + w // 2
				centroid_y = y + h // 2
				centr_x_pink.value = centroid_x
				centr_y_pink.value = centroid_y
				centr_x.value = 0
				centr_y.value = 0
				# if centroid_y > 500:
				pink_b.value = True
				# red_b.value = False
				green_b.value = False

		elif pink_green:
			# if cv2.contourArea(max_cnt) > cv2.contourArea(max_cnt2):
			if (cv2.contourArea(max_cnt) > 1000 and cv2.contourArea(max_cnt) < 306000):
				# Draw a rectange on the contour
				rect = cv2.minAreaRect(max_cnt)
				box = cv2.boxPoints(rect)
				box = np.intp(box)
				cv2.drawContours(img, [box], -1, (255, 0, 0), 3)

				(x, y, w, h) = cv2.boundingRect(box)
				centroid_y = y + h // 2
				centroid_x = x + w // 2
				centr_y.value = centroid_y
				centr_x.value = centroid_x
				centr_x_red.value = 0
				centr_x_red.value = 0
				red_b.value = False
				green_b.value = True  # pink_b.value = False
			# elif cv2.contourArea(max_cnt2) > cv2.contourArea(max_cnt):
			if (cv2.contourArea(max_cnt2) > 2000 and cv2.contourArea(max_cnt2) < 306000):
				# Draw a rectange on the contour
				rect2 = cv2.minAreaRect(max_cnt2)
				box = cv2.boxPoints(rect2)
				box = np.intp(box)
				cv2.drawContours(img, [box], -1, (255, 0, 0), 3)
				(x, y, w, h) = cv2.boundingRect(box)
				centroid_x = x + w // 2
				centroid_y = y + h // 2
				centr_x_pink.value = centroid_x
				centr_y_pink.value = centroid_y
				centr_x_red.value = 0
				centr_y_red.value = 0
				# if centroid_y > 500:
				pink_b.value = True
				red_b.value = False  # green_b.value = False
		else:
			centr_x_pink.value = 0
			centr_y_red.value = 0
			centr_y.value = 0
			centr_x.value = 0
			centr_x_red.value = 0
		# print(f"Green:{green_present}, red:{red_present}, pink:{pink_present}")
		# print(f"all:{all_flag}, only_red:{only_red}, only_green:{only_green}, only_pink:{only_pink}, pink_green:{pink_green}, pink_red:{pink_red}, both:{both_flag}")
		# print(f"g_next:{g_next.value}, r_next:{r_next.value}")
		# print(f"green:{green_b.value}  red:{red_b.value}, pink:{pink_b.value}")
		#print(f"green centr :{centr_x.value}, red_centr:{centr_x_red.value}, pink_centr:{centr_x_pink.value}")
		# print(f"{centr_y_pink.value}")
		#print(f"blue centr x:{centroid_x_b} centr_y = {centroid_y_b} blue: {blue_b.value} prev_b.value:{prev_b.value}")
		#print(f"blue: {centr_y_b.value} {cv2.contourArea(max_cnt_b)} ")
		cv2.imshow('Object Frame', img)

		if cv2.waitKey(1) & 0xFF == ord('q'):
			stop_b.value = True
			break

	cv2.destroyAllWindows()
	picam2.stop()


def servoDrive(pwm, color_b, stop_b, red_b, green_b, pink_b, counts, centr_y, centr_x, centr_y_red, centr_x_red, centr_x_pink, centr_y_pink, head, centr_y_b, blue_b, prev_b, orange_o, centr_y_o, blue_c, orange_c, white_c):
	global imu, corr, corr_pos

	pb_time = 0
	pwm_pin = 12
	direction_pin = 20

	pwm.set_mode(pwm_pin, pigpio.OUTPUT)  # Set pin 12 as an output
	pwm.set_mode(direction_pin, pigpio.OUTPUT)  # Set pin 20 as an output
	pwm.hardware_PWM(pwm_pin, 100, 0)

	pwm.set_PWM_dutycycle(pwm_pin, 0)  # Set duty cycle to 50% (128/255)

	enc = EncoderCounter()

	############# FLAGS ###############
	button = False
	trigger = False
	reset_f = False
	blue_flag = False
	orange_flag = False
	change_path = False
	green_turn = False
	timer_started = False
	g_flag = False
	r_flag = False
	p_flag = False
	g_past = False
	r_past = False
	p_past = False
	red_stored = False
	green_stored = False
	parking_flag = False
	red_turn = False
	reverse = False
	parking_heading = False
	stop_flag = False
	calc_time = False
	continue_parking = False
	lap_finish = False
	counter_reset = False
	turn_flag = False
	reset_flags = False
	finished = False
	red_time = False
	green_time = False
	finish = False
	pink_detected = False
	last_red = False
	cw = False
	ccw = False
	reset_heading = False
	previous_heading_stored = False
	reverse_trigger = False
	pink_timer = False
	back_bot = False
	green_timer = False
	red_timer = False
	g_last_flag = False	
	r_last_flag = False	
	pink_r = False
	reverse_complete = False
	blue_on = False
	finish_flag = False
	reset_servo = False	
	############ VARIABLES ##################
	color_n = ""
	setPointL = -70
	setPointR = 70
	setPointC = 0
	power = 100
	prev_power = 0
	last_counter = 12
	change_counter = 7 # 3
	rev_counter = 7
	heading_angle = 0
	counter = 0
	turn_t = 0
	current_time = 0
	gp_time = 0
	rp_time = 0
	buff = 0
	c_time = 0
	green_count = 0
	red_count = 0
	i = 0
	l = 0
	lap_finish_time = 0
	prev_distance = 0
	turn_trigger_distance = 0
	target_count = 0
	offset = 0
	button_state = 0
	past_time = 0
	correctAngle(heading_angle, head.value)
	stop_time = 0
	previous_heading = -1
	turn_cos_theta = 0
	parking_done = 0
	pink_d = 0
	g_time = 0
	r_time = 0
	u = 0
	avg_right = 0
	avg_head = 0
	avg_left = 0
	time_p = 0
	prev_time = 0
	prev_restore = 0
	finish_timer = 0
	prev_blue = 0
	prev_orange = 0
	avg_blue = 0
	avg_orange = 0
	c = 0
	c_time = 0
	fps_time2 = 0
	color_s = ""
	while True:
		print(f"fps 2222:{1/(time.time() - fps_time2)}")
		fps_time2 = time.time()
		if centr_y_b.value > 400 and (counter == 0 or blue_flag):
			power = 50

		else:
			power = 100
			
		#print(f"c_time:{c}")
		#color_sensor = imu.get_color()
		#color_sensor="None"
		#print(color_sensor)		
		'''if centr_y_b.value > 100 or centr_y_o.value > 100 or c > 10:
			GPIO.output(blue_led, GPIO.HIGH)
			c =  c + 1
			
			color_sensor = imu.get_color()
		if time.time() - c_time > 1:
			c = 0
			GPIO.output(blue_led, GPIO.LOW)'''

		tfmini.getTFminiData()		
		##### STOP CONDITION ######
		#print(f"rgb:{imu.color_rgb} color:{color_sensor}")
		#print(f"pink detected:{pink_detected}")
		if counter == last_counter and not lap_finish:
			print(f"centr_y :{centr_y.value} centr_y_red:{centr_y_red.value}")
			if not finished:
				target_count = counts.value + 35000
				finished = True
			if counts.value >= target_count and not reverse_trigger:
				power = 0
				pwm.set_PWM_dutycycle(pwm_pin, power)  # Set duty cycle to 50% (128/255)
				time.sleep(3)
				power = 100
				prev_power = 0
				lap_finish = True
				reverse_trigger = True
				print(f"Vehicle is stopped,..reverse_trigger: {reverse_trigger}")

		if lap_finish:
			if not counter_reset:
				counter = counter % 12
				counter_reset = True

		if lap_finish and not continue_parking:
			if orange_flag:
				if counter % 4 != 0:
					setPointR = -100
					setPointC = -100
			elif blue_flag:
				if counter % 4 != 0:
					setPointL = 100
					setPointC = 100

		if continue_parking:  ### THIS SETPOINT IS WHEN THE ROBOT IS IN THE PARKING MODE
			green_b.value = False
			red_b.value = False
			g_past = False
			r_past = False
			g_flag = False
			r_flag = False
			if orange_flag and (((centr_x_pink.value < centr_x.value) and (centr_x.value > 0 and centr_x_pink.value > 0)) or (centr_x_pink.value < centr_x_red.value and (centr_x_red.value > 0 and centr_x_pink.value > 0))):
				setPointR = -35
				setPointC = -35
				finish = True
			elif blue_flag and (((centr_x_pink.value > centr_x.value) and (centr_x.value > 0 and centr_x_pink.value > 0)) or (centr_x_pink.value > centr_x_red.value and (centr_x_red.value > 0 and centr_x_pink.value > 0))):
				setPointL = 35
				setPointC = 35
				finish = True

		if pink_b.value:  ### DECIDES SETPOINT WHENEVER PINK IS IN THE FRAME
			if orange_flag:
				if (centr_x_pink.value < centr_x.value) and (centr_x_pink.value > 0 and centr_x.value > 0) and not continue_parking:
					setPointL = -35
					setPointR = 70
					#print(f"setPointL: {setPointL}")
			elif blue_flag:
				if (centr_x_red.value < centr_x_pink.value) and (centr_x_pink.value > 0 and centr_x_red.value > 0) and not continue_parking:
					setPointR = 35
					setPointL = -70
					#print(f"setPointR: {setPointR}")

			elif counter % 4 == 0 and not blue_flag and not orange_flag:
				if ((centr_x_pink.value < 800 and centr_x_pink.value > 0) and ((centr_y.value or centr_y_red.value) <= centr_y_pink.value)) and not continue_parking:
					#print(f"at 0 counter orange:{setPointR} {setPointL}")
					setPointR = 35
					setPointL = -70
				if ((centr_x_pink.value > 800) and ((centr_y.value or centr_y_red.value) <= centr_y_pink.value)) and not continue_parking:
					#print(f"at 0 counter blue:{setPointR} {setPointL}")
					setPointL = -35
					setPointR = 70

			if lap_finish and not continue_parking:
				pink_detected = False
				print("Starting Parking...")
				continue_parking = True
			pb_time = time.time()
		elif not pink_b.value and time.time() - pb_time > 1 and not lap_finish:  ### IF DOES NOT SEE PINK, KEEP THE SAME SETPOINT FOR 1 SECOND AND THEN CHANGE
			print(f"Resetting setPoints...{pink_detected}")
			if g_flag and not continue_parking:
				print(f"away from green {g_past}")
				setPointL = setPointL - 1
				setPointR = 70
			elif r_flag and not continue_parking:
				print(f"away from red {r_past}")
				setPointR = setPointR + 1
				setPointL = -70

		try:

			avg_right_pass = (tfmini.distance_right* 0.1) + (avg_right * 0.9)
			avg_left_pass = (tfmini.distance_left* 0.1) + (avg_left * 0.9)
			avg_head = (tfmini.distance_head* 0.10) + (avg_head*0.90)
			avg_left = (tfmini.distance_left* 0.15) + (avg_left*0.85)
			avg_right = (tfmini.distance_right* 0.15) + (avg_right*0.85)

			print(f"average ::: r_pass:{avg_right_pass} l_pass:{avg_left_pass} h:{avg_head} l:{avg_left} r:{avg_right}")
			previous_state = button_state
			time.sleep(0.05)
			button_state = GPIO.input(5)
			if previous_state == 1 and button_state == 0:
				button = not (button)
				power = 100

			if button:  ##### THIS BLOCK OF CODE WHEN BUTTON IS PRESSED
				if not reset_servo:
					time.sleep(0.5)
					servo.setAngle(130)
					time.sleep(0.5)
					servo.setAngle(90)	
					reset_servo = True

				if not reverse:
					imu_head = head.value
				else:
					#print(f"Changing imu..{imu_head} {cw} {ccw}")
					imu_head = head.value - 180

				x, y = enc.get_position(imu_head, counts.value)

				total_power = (power * 0.1) + (prev_power * 0.9)
				prev_power = total_power
				pwm.set_PWM_dutycycle(pwm_pin, 2.55 * total_power)  # Set duty cycle to 50% (128/255)

				pwm.write(direction_pin, 1)  # Set pin 20 high

				if stop_b.value:
					power = 0
					prev_power = 0

				######      DIRECTION DECISION (CLOCKWISE OR ANTICLOCKWISE)     #####
				if blue_c.value:
					color_s = "Blue"
				elif orange_c.value:
					color_s = "Orange"
				elif white_c.value:
					color_s = "White"
					
				if not blue_flag and not orange_flag:
					if color_s == "Orange":
						orange_flag = True
						blue_flag = False
						color_n = "Orange"

					elif color_s == "Blue":
						blue_flag = True
						orange_flag = False
						color_n = "Blue"
						


				################        PARKING         ################

				if parking_flag and not stop_flag:
					print(f"PARKING ------> distance_head : {tfmini.distance_head}")
					print("Inside Parking Loop")

					if not calc_time:
						c_time = time.time()
						calc_time = True
					if pink_b.value and not pink_r:
						print("Time is same")
						time_p = 0.7
						pink_r = True
					elif not pink_b.value and not pink_r:
						print("increasing time..")
						time_p = 2.5
						pink_r = True
					while time.time() - c_time < time_p and not reverse_complete:
						print("Reversing backward...")
						power = 100
						prev_power = 0
						pwm.set_PWM_dutycycle(pwm_pin, power)  # Set duty cycle to 50% (128/255)
						pwm.write(direction_pin, 0)  # Set pin 20 hig '''
						prev_time = time.time()
					reverse_complete = True	
					while time.time() - prev_time < 0.5:
						print("Robot is stopped")
						power = 0
						prev_power = 0
						pwm.set_PWM_dutycycle(pwm_pin, power)  # Set duty cycle to 50% (128/255)


					if orange_flag:
						if not parking_heading:
							heading_angle = heading_angle - 90
							calc_time = False
							parking_heading = True

					elif blue_flag:
						if not parking_heading:
							heading_angle = heading_angle + 90
							calc_time = False
							parking_heading = True

					print(f"Correcting angle..{abs(corr)}")
					if parking_heading:
						print("Moving slowly..")
						power = 85
						pwm.set_PWM_dutycycle(pwm_pin, power)
						correctAngle(heading_angle, head.value)
					back_bot = False
					#tfmini.getTFminiData()
					stop_flag = False
					if (abs(corr) < 15) and (tfmini.distance_head <= 5 and tfmini.distance_head >= 0) and not finish_flag:
						finish_timer = time.time()
						finish_flag = True
					print(f"finish flag:{finish_flag}")
					if time.time() - finish_timer > 1 and finish_flag and not stop_flag:
						power = 0
						prev_power = 0
						servo.setAngle(90)
						pwm.set_PWM_dutycycle(pwm_pin, power)
						stop_flag = True
						stop_time = time.time()
						print("Succesfully Parked...")
						sys.exit(0)

						back_bot = True
				else:
					if reset_f:
						
						setPointL = -70
						setPointR = 70
						g_past = False

						if not last_red:
							g_last_flag = False
							r_last_flag = False
						print(f"green: {green_count} red:{red_count}")
						if counter == change_counter and green_count == 1:
							last_red = False
						elif counter == change_counter and red_count == 1:
							print("Changing path...")
							last_red = True
						print(f"last_red: {last_red}")
						if blue_flag:  ### BLUE RESET BLOCK

							print(f"BLUE RESET...{reverse_trigger}")

							if (red_b.value or red_turn) or reverse_trigger:  # red after trigger
								if counter != rev_counter:
									green_count = 0
									red_count = 1
									GPIO.output(red_led, GPIO.HIGH)
									GPIO.output(green_led, GPIO.LOW)
								#tfmini.getTFminiData()
								x, y = enc.get_position(imu_head, counts.value)
								print(f"Red Detected after trigger...green: {g_flag} {g_past} red:{r_flag} {r_past} {setPointR} {setPointL}")
								red_turn = True
								if pink_b.value and not red_b.value:
									red_turn = False
									red_time = False
									reverse_trigger = False
								elif (tfmini.distance_head < 30 and time.time() - g_time > 0.8) and not red_b.value and not pink_b.value:
									red_turn = False
									reverse_trigger = False
									red_time = True
								correctPosition(setPointC, heading_angle, x, y, counter, blue_flag, orange_flag, reset_f, reverse, head.value, centr_x_pink.value, finish)

							else:
								if red_time:
									time_g = 1.2

								else:
									time_g = 0.5

								if not timer_started:
									current_time = time.time()
									timer_started = True

								if not green_b.value and not red_b.value:
									print('reversing diection red')
									while (time.time() - current_time < time_g):
										servo.setAngle(100)
										x, y = enc.get_position(imu_head, counts.value)
										power = 100
										prev_power = 95
										pwm.set_PWM_dutycycle(pwm_pin, power)  # Set duty cycle to 50% (128/255)
										pwm.write(direction_pin, 0)  # Set pin 20 hig
									print('reversing diection red complete')
									print('Stopping Motor...')
									#tfmini.getTFminiData()
									turn_trigger_distance = tfmini.distance_head
									turn_cos_theta = math.cos(math.radians(abs(corr)))
									timer_started = False

								elif (green_b.value or green_turn) or g_past:
									green_turn = True
									if counter != rev_counter:
										green_count = 1
										red_count = 0
										GPIO.output(red_led, GPIO.LOW)
										GPIO.output(green_led, GPIO.HIGH)
									while 1 and green_b.value:
										tfmini.getTFminiData()
										correctAngle(heading_angle, head.value)
										if abs(corr) < 15:
											turn_trigger_distance = tfmini.distance_head
											turn_cos_theta = math.cos(math.radians(abs(corr)))
											break
									print('reversing diection Green')
									if not timer_started:
										current_time = time.time()
										timer_started = True
									while 1:
										servo.setAngle(110)
										if (green_b.value and (centr_y.value < 500 and centr_y.value > 0)):
											print(f"Breaking the loop...")
											break
										elif (time.time() - current_time > 1.5) and not green_b.value:
											print("Green is not there breaking the loop...")
											break
										x, y = enc.get_position(imu_head, counts.value)
										power = 100
										prev_power = 0
										pwm.set_PWM_dutycycle(pwm_pin, power)  # Set duty cycle to 50% (128/255)
										pwm.write(direction_pin, 0)  # Set pin 20 hig
									print('Green reversing diection complete')
									print('Stopping Motor...')
									buff = 4
									time_started = False
								power = 0
								prev_power = 0
								pwm.set_PWM_dutycycle(pwm_pin, power)  # Set duty cycle to 50% (128/255)

								#tfmini.getTFminiData()
								print(f"head: {tfmini.distance_head}")
								print(f"before update: {x} {y}")
								time.sleep(0.8)
								counter = counter + 1
								c_time = time.time()
								lane_reset = counter % 4
								print(f"in Lane {lane_reset}")
								if lane_reset == 1:
									enc.x = (150 - abs(turn_trigger_distance * turn_cos_theta)) - 10
								if lane_reset == 2:
									enc.y = (abs(turn_trigger_distance * turn_cos_theta) - 250) + 10
								if lane_reset == 3:
									enc.x = (abs(turn_trigger_distance * turn_cos_theta) - 150) + 10
								if lane_reset == 0:
									enc.y = (50 - abs(turn_trigger_distance * turn_cos_theta)) - 10
								print(f'Resuming Motor...{x} {y}')

								power = 100
								if reverse == True:
									print("In blue reverse...")
									offset = 180
									heading_angle = -((90 * counter) % 360) - offset
									if abs(heading_angle) >= 360:
										heading_angle = (heading_angle % 360)
								else:
									print("In blue reverse else...")
									heading_angle = -((90 * counter) % 360)
								green_turn = False
								red_turn = False
								reset_f = False
								red_time = False
								r_flag = False
								r_past = False

						if orange_flag:  ### ORANGE RESET BLOCK
							print(f"ORANGE RESET...reverse_trigger:{reverse_trigger}")

							if (green_b.value or green_turn) or (reverse_trigger):  # green after trigger
								#tfmini.getTFminiData()
								if counter != rev_counter:
									green_count = 1
									red_count = 0
									GPIO.output(red_led, GPIO.LOW)
									GPIO.output(green_led, GPIO.HIGH)
								x, y = enc.get_position(imu_head, counts.value)

								print(f"Green Detected after trigger... ")
								green_turn = True
								if pink_b.value and not green_b.value:
									green_turn = False
									green_time = False
									reverse_trigger = False
								elif tfmini.distance_head < 30 and not green_b.value and not pink_b.value:
									green_turn = False
									reverse_trigger = False
									green_time = True
								correctPosition(setPointC, heading_angle, x, y, counter, blue_flag, orange_flag, reset_f, reverse, head.value, centr_x_pink.value, finish)
							else:
								print("ORANGE RESET ELSE..")
								if green_time:
									time_g = 1.2
								else:
									time_g = 0.5
								if not timer_started:
									current_time = time.time()
									timer_started = True

								if not red_b.value and not r_past:
									print('reversing diection green')
									#tfmini.getTFminiData()
									turn_trigger_distance = tfmini.distance_head
									while time.time() - current_time < time_g:
										servo.setAngle(70)
										# getTFminiData()
										x, y = enc.get_position(imu_head, counts.value)
										power = 100
										prev_power = 95
										pwm.set_PWM_dutycycle(pwm_pin, power)  # Set duty cycle to 50% (128/255)
										pwm.write(direction_pin, 0)  # Set pin 20 hig
									print('reversing diection green complete')
									print('Stopping Motor...')
									#tfmini.getTFminiData()
									turn_trigger_distance = tfmini.distance_head
									turn_cos_theta = math.cos(math.radians(abs(corr)))
									timer_started = False
								elif (red_b.value or red_turn) or r_past:
									red_turn = True
									if counter != rev_counter:
										red_count = 1
										green_count = 0
										GPIO.output(red_led, GPIO.HIGH)
										GPIO.output(green_led, GPIO.LOW)
									while 1:
										tfmini.getTFminiData()
										print(f"correct red heading..")
										correctAngle(heading_angle, head.value)
										if abs(corr) < 15:
											turn_trigger_distance = tfmini.distance_head
											print(f"turn_trigger: {turn_trigger_distance}")
											turn_cos_theta = math.cos(math.radians(corr))
											break
									if not timer_started:
										current_time = time.time()
										timer_started = True
									print(f'reversing diection red pink color: {pink_b.value} pink flag: {p_flag} {p_past}  red color:{red_b.value} red flag: {r_past} {r_flag}')
									while 1:
										print("RED IS SEEN..")
										servo.setAngle(80)
										print(f"centr y: {centr_y_red.value}")
										if (red_b.value and (centr_y_red.value < 475 and centr_y_red.value > 0)):
											print(f"Breaking the loop...")
											break
										elif (time.time() - current_time > 1.5) and not red_b.value:
											print("Green is not there breaking the loop...")
											break

										# getTFminiData()
										x, y = enc.get_position(imu_head, counts.value)
										# print(f"x: {x}, y: {y},  count:{counts.value} distance_head : {distance_head}")
										power = 100
										prev_power = 0
										pwm.set_PWM_dutycycle(pwm_pin, power)  # Set duty cycle to 50% (128/255)
										pwm.write(direction_pin, 0)  # Set pin 20 hig
									print('red reversing diection complete')
									buff = 4
									timer_started = False
								print('Stopping Motor...')

								power = 0
								prev_power = 0
								pwm.set_PWM_dutycycle(pwm_pin, power)  # Set duty cycle to 50% (128/255)

								# getTFminiData()
								x, y = enc.get_position(imu_head, counts.value)

								time.sleep(0.8)
								counter = counter + 1
								c_time = time.time()
								lane_reset = counter % 4
								print(f"head: {turn_trigger_distance}, corr: {turn_cos_theta}")
								if lane_reset == 1:
									enc.x = (150 - (turn_trigger_distance * turn_cos_theta)) - 10
									print(f"x: {enc.x}")
								if lane_reset == 2:
									enc.y = (250 - (turn_trigger_distance * turn_cos_theta)) - 10
								if lane_reset == 3:
									enc.x = ((turn_trigger_distance * turn_cos_theta) - 150) + 10
								if lane_reset == 0:
									enc.y = ((turn_trigger_distance * turn_cos_theta) - 50) + 10
								print(f'Resuming Motor...{offset}')

								power = 100
								if reverse == True:
									offset = -180
									heading_angle = ((90 * counter) % 360) + offset
									if abs(heading_angle) >= 360:
										heading_angle = (heading_angle % 360)
								else:
									heading_angle = ((90 * counter) % 360)
								red_turn = False
								green_time = False
								reset_f = False
								g_flag = False
								g_past = False
								#centr_y_b.value = 0

					else:
						#tfmini.getTFminiData()
						print(f"power :{power} prev_power{prev_power}")
						if avg_head < 10:
							prev_restore = time.time()
							print(f"counter: {counter} Trigger detected...")
							power = 100
							prev_power = 0
							while time.time() - prev_restore < 2:
								tfmini.getTFminiData()
								servo.setAngle(90)
								GPIO.output(blue_led, GPIO.HIGH)
								pwm.set_PWM_dutycycle(pwm_pin, power)  # Set duty cycle to 50% (128/255)
								pwm.write(direction_pin, 0)  # Set pin 20 hig
								any_color = True
						else:
							blue_on = False
							GPIO.output(blue_led, GPIO.LOW)
							
						avg_blue = (prev_b.value*0.1 + avg_blue*0.9)
						avg_orange = (prev_b.value*0.1 + avg_blue*0.9)
						
						if color_s == color_n and not trigger and (time.time() - turn_t) > (4 + buff):
							buff = 0
							timer_started = False
							trigger = True
							reset_f = True
							GPIO.output(blue_led, GPIO.HIGH)
							turn_t = time.time()

						elif color_s == 'White':
							trigger = False
							GPIO.output(blue_led, GPIO.LOW)
						
						prev_blue = centr_y_b.value
						prev_orange = centr_y_o.value

						if last_red:
					
							print(f"tf mini back: {tfmini.distance_back} front:{tfmini.distance_head} sum:{tfmini.distance_head + tfmini.distance_back} corr:{abs(corr)}")

						if g_last_flag:
							if ((imu_head < 10 or imu_head > 350) and heading_angle == 0) and ((tfmini.distance_back*math.cos(math.radians(abs(imu_head))) > 150 and imu_head > 350) or (tfmini.distance_head*math.cos(math.radians(abs(imu_head))) < 160 and imu_head < 10)) and last_red:
								u = u + 1
								if u > 3:
									print(f"Change path is true after 4.2 seconds")
									reset_flags = True
									change_path = True
									reverse = True
						elif r_last_flag:
							if ((imu_head < 10 or imu_head > 350) and heading_angle == 0) and ((tfmini.distance_back*math.cos(math.radians(abs(imu_head)))) > 150 and imu_head < 10) or (tfmini.distance_head*math.cos(math.radians(abs(imu_head))) < 160 and imu_head > 350) and last_red:
								u = u + 1
								if u > 3:
									print(f"Change path is true after 4.2 seconds")
									reset_flags = True
									change_path = True
									reverse = True

						if reset_flags:
							if orange_flag:
								blue_flag = True
								orange_flag = False
								color_n = "Blue"
								reset_flags = False
							elif blue_flag:
								orange_flag = True
								blue_flag = False
								color_n = "Orange"
								reset_flags = False
								
								

				################### PANDAV 2.0 ####################

						if green_b.value and not r_flag and not continue_parking and not g_past:
							print(f"centr x: {centr_x.value} centr y: {centr_y.value}")
							g_flag = True
							if (centr_x.value > 1500 or  centr_y.value > 900):
								g_past = True
							GPIO.output(red_led, GPIO.LOW)
							GPIO.output(green_led, GPIO.LOW)
							print('1')

						elif (g_past or time.time() - gp_time < 0.5) and not continue_parking:
							print("Avoiding green...")
	
							if tfmini.distance_right <= 50 : #and ((avg_left_pass < 60 or avg_left_pass > 120) or counter!=rev_counter):
								print("Green Avoid Complete")
								g_past = False
								g_flag = False
								red_count = 0
								green_count = 1
								GPIO.output(green_led, GPIO.HIGH)
								GPIO.output(red_led, GPIO.LOW)
								buff = 0
								gp_time = time.time()
							g_flag = True
							print('2')

						elif red_b.value and not g_flag and not continue_parking and not r_past:
							r_flag = True
							print(f"centr x red: {centr_x_red.value} centr y red: {centr_y_red.value}")
							if ((centr_x_red.value < 100 and centr_x_red.value > 0) or  centr_y_red.value > 900):
								r_past = True
							#GPIO.output(blue_led, GPIO.LOW)
							GPIO.output(green_led, GPIO.LOW)
							GPIO.output(red_led, GPIO.LOW)
							print('3')

						elif (r_past or time.time() - rp_time < 0.5) and not continue_parking:
							print("Avoiding red...")
							if tfmini.distance_left <= 50 : #and ((avg_right_pass < 60 or avg_right_pass > 120) or counter!=rev_counter):
								print(f"red Avoid complete")
								r_past = False
								r_flag = False
								red_stored = False
								red_count = 1
								green_count = 0
								GPIO.output(red_led, GPIO.HIGH)
								GPIO.output(green_led, GPIO.LOW)
								buff = 0
								rp_time = time.time()
							r_flag = True
							print('4')

						elif pink_b.value and not p_past and continue_parking and not p_flag:
							if (centr_x_pink.value < 800 and orange_flag) or (centr_x_pink.value > 800 and blue_flag):
								p_flag = True
								p_past = True
							print('5')

						elif p_past and continue_parking and not parking_flag:

							if orange_flag:
								print(f"prev_distance: {prev_distance}, distance_left: {tfmini.distance_left} diff: {abs(prev_distance - tfmini.distance_left)}")
								p_flag = True
								if tfmini.distance_left <= 30 and (abs(prev_distance - tfmini.distance_left) >= 7 and prev_distance > 0) and p_past:
									p_past = False
									p_flag = False
									parking_flag = True
									print("Pink Avoidance Complete...")
								prev_distance = tfmini.distance_left

							elif blue_flag:
								print(f"prev_distance: {prev_distance}, distance_right: {tfmini.distance_right}  diff: {abs(prev_distance - tfmini.distance_right)}")
								if tfmini.distance_right <= 30 and (abs(prev_distance - tfmini.distance_right) >= 7 and prev_distance > 0) and p_past:
									p_past = False
									p_flag = False
									parking_flag = True
									print("Pink Avoidance Complete Blue...")
								prev_distance = tfmini.distance_right

						
							print('6')

						else:
							g_flag = False
							r_flag = False
							p_flag = False
							r_past = False
							g_past = False
							p_past = False

							print('7')

							GPIO.output(red_led, GPIO.LOW)
							GPIO.output(green_led, GPIO.LOW)

						if not change_path:
							if g_flag or g_last_flag:
								if last_red:
									g_last_flag = True
									r_last_flag = False
								print("avoiding green..")
								correctPosition(setPointL, heading_angle, x, y, counter, blue_flag, orange_flag, reset_f, reverse, head.value, centr_x_pink.value, finish)
							elif r_flag or r_last_flag:
								if last_red:
									r_last_flag = True
									g_last_flag = False
								print("avoiding red...")
								correctPosition(setPointR, heading_angle, x, y, counter, blue_flag, orange_flag, reset_f, reverse, head.value, centr_x_pink.value, finish)
							elif p_flag:
								print("avoiding pink..")
								if orange_flag:
									correctPosition(setPointR, heading_angle, x, y, counter, blue_flag, orange_flag, reset_f, reverse, head.value, centr_x_pink.value, finish)
								elif blue_flag:
									correctPosition(setPointL, heading_angle, x, y, counter, blue_flag, orange_flag, reset_f, reverse, head.value, centr_x_pink.value, finish)
							else:
								correctPosition(setPointC, heading_angle, x, y, counter, blue_flag, orange_flag, reset_f, reverse, head.value, centr_x_pink.value, finish)

							print(f"g_last:{g_last_flag} r_last:{r_last_flag}")
						else:

							print(f"Turning 180...{abs(corr)} {i}")
							print(f"heading_angle:{heading_angle} prev_heading {previous_heading}")
							if abs(heading_angle) == 180 and abs(corr) < 15:
								power = 0
								pwm.set_PWM_dutycycle(pwm_pin, power)  # Set duty cycle to 50% (128/255)
								time.sleep(0.5)
								power = 100
								prev_power = 0
								heading_angle = -180
								change_path = False
								last_red = False
								print("Change path is false")
							#tfmini.getTFminiData()
							if r_last_flag or cw:
								r_flag = True
								g_flag = True
								g_past = False
								print("Turning anticlockwise")
								cw = True
								offset = -90
							elif g_last_flag or ccw:
								g_flag = True
								r_flag = False
								r_past = False
								print("Turning clockwise")
								ccw = True
								offset = 90
							if abs(corr) < 15 and i < 2 and heading_angle != previous_heading:
								print(f"off:{offset}")
								print(f"heading_angle before change {heading_angle}")
								previous_heading = heading_angle
								heading_angle = heading_angle + offset
								print(f"heading_angle after change {heading_angle}")
								previous_heading_stored = True
								i = i + 1
							correctAngle(heading_angle, head.value)
				print(f"green:{green_count} r_count:{red_count}")
				print(f"color_s:{color_s} color_n:{color_n} centr_y_b.value: {centr_y_b.value} centr_x:{centr_x.value} centr_red: {centr_x_red.value} centr_pink:{centr_x_pink.value} setPointL:{setPointL} setPointR:{setPointR} g_count:{green_count} r_count:{red_count} color_n:{color_n} color_sensor:{color_s} x: {x}, y: {y} counts: {counts.value}, prev_distance: {prev_distance}, head_d: {tfmini.distance_head} right_d: {tfmini.distance_right}, left_d: {tfmini.distance_left}, back_d:{tfmini.distance_back} imu: {imu_head}, heading: {heading_angle}, cp: {continue_parking}, counter: {counter}, pink_b: {pink_b.value} p_flag = {p_flag}, g_flag: {g_flag} r_flag: {r_flag} p_past: {p_past}, g_past: {g_past}, r_past: {r_past} , red_stored:{red_stored} green_stored:{green_stored}")
			else:
				power = 0
				pwm.hardware_PWM(12, 100, 0)
				heading_angle = 0
				counter = 0
				correctAngle(heading_angle, head.value)
				color_b.Value = False
				stop_b.value = False
				red_b.value = False
				green_b.value = False

		except Exception as e:
			print(f"Exception: {e}")
			if isinstance(e, KeyboardInterrupt):
				power = 0
				pwm.hardware_PWM(12, 100, 0)
				heading_angle = 0
				counter = 0
				correctAngle(heading_angle, head.value)
				color_b.Value = False
				stop_b.value = False
				red_b.value = False
				green_b.value = False


def runEncoder(counts, head):
	print("Encoder Process Started")
	ser = serial.Serial('/dev/UART_USB', 115200)
	print("created uart")


	try:
		while True:
			line = ser.readline().decode().strip()
			esp_data = line.split(" ")

			if len(esp_data) >= 2:
				try:
					head.value = float(esp_data[0])
					counts.value = int(esp_data[1])   
					# ✅ Print clean summary
					#print("📊 LIDAR Snapshot:")
					#print(f"0: {dist_0} 90: {dist_90} 270:{dist_270}")
					#print(f"angle: {lidar_angle.value} distance: {lidar_distance.value}")
					#print(f"array:{specific_angle}, imu: {esp_angle}")

				except ValueError:
					print(f"⚠️ Malformed ESP data: {esp_data}")
			else:
				print(f"⚠️ Incomplete ESP data: {esp_data}")

	except KeyboardInterrupt:
		print("👋 Shutting down...")
		ser.close()

def color_SP(blue_c, orange_c, white_c):
	fps3 = 0
	while True:
			#color_sensor = imu.get_color()
			color_sensor = "None"
			print(f" fps33:{(1/(time.time() - fps3))} color: {color_sensor}")
			fps3 = time.time()

			if color_sensor == "Orange":
				orange_c.value = True
				blue_c.value = False
				white_c.value = False
			elif color_sensor == "Blue":
				blue_c.value = True
				orange_c.value = False
				white_c.value = False
			else:
				white_c.value = True
				blue_c.value = False
				orange_c.value = False
			print(f"color:{color_sensor}")

if __name__ == '__main__':
	try:
		pwm = pigpio.pi()
		counts = multiprocessing.Value('i', 0)
		color_b = multiprocessing.Value('b', False)
		stop_b = multiprocessing.Value('b', False)
		red_b = multiprocessing.Value('b', False)
		green_b = multiprocessing.Value('b', False)
		pink_b = multiprocessing.Value('b', False)
		blue_b = multiprocessing.Value('b', False)
		orange_o = multiprocessing.Value('b', False)
		blue_c = multiprocessing.Value('b', False)
		orange_c = multiprocessing.Value('b', False)
		white_c = multiprocessing.Value('b', False)
		centr_y = multiprocessing.Value('f', 0.0)
		centr_x = multiprocessing.Value('f', 0.0)
		centr_y_red = multiprocessing.Value('f', 0.0)
		centr_x_red = multiprocessing.Value('f', 0.0)
		centr_x_pink = multiprocessing.Value('f', 0.0)
		centr_y_pink = multiprocessing.Value('f', 0.0)
		centr_y_b = multiprocessing.Value('f', 0.0)
		centr_y_o = multiprocessing.Value('f', 0.0)
		prev_b = multiprocessing.Value('f', 0.0)
		head = multiprocessing.Value('f', 0.0)

		
		P = multiprocessing.Process(target=Live_Feed, args=(color_b, stop_b, red_b, green_b, pink_b, centr_y, centr_x, centr_y_red, centr_x_red, centr_x_pink, centr_y_pink, centr_y_b, blue_b, prev_b, orange_o, centr_y_o))
		S = multiprocessing.Process(target=servoDrive, args=(pwm, color_b, stop_b, red_b, green_b, pink_b, counts, centr_y, centr_x, centr_y_red, centr_x_red, centr_x_pink, centr_y_pink, head, centr_y_b, blue_b, prev_b, orange_o, centr_y_o, blue_c, orange_c, white_c))
		E = multiprocessing.Process(target=runEncoder, args=(counts, head,))
		C = multiprocessing.Process(target=color_SP, args=(blue_c, orange_c, white_c))
		E.start()
		C.start()
		S.start()
		P.start()


	except KeyboardInterrupt:
		pwm.hardware_PWM(12, 100, 0)
		pwm.bb_serial_read_close(RX_Head)
		pwm.bb_serial_read_close(RX_Left)
		pwm.bb_serial_read_close(RX_Right)
		pwm.stop()
		imu.close()
		GPIO.cleanup()
