import os

os.system('sudo pkill pigpiod')
os.system('sudo pigpiod')

import numpy as np
import RPi.GPIO as GPIO
import cv2
from picamera2 import Picamera2
import time

import multiprocessing
import pigpio
import board

import math
from classes.Encoder import EncoderCounter

from classes.BNO085 import IMUandColorSensor
from classes.Servo import Servo
import serial

ser = serial.Serial('/dev/UART_USB', 115200)
print("Connection Established...")
ser.write(b"1")
print("Command sent: b'1'")
ser.flush()
time.sleep(5)
#imu = IMUandColorSensor(board.SCL, board.SDA)

GPIO.setwarnings(False)

glob = 0

GPIO.setmode(GPIO.BCM)

"""GPIO.setup(20, GPIO.OUT) # Connected to AIN2
GPIO.setup(12, GPIO.OUT)
pwm12 = GPIO.PWM(12, 100)"""
GPIO.setup(5, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Button to GPIO23

# pwm = pigpio.pi()
# Parameters for servo
servo = Servo(8)

RX_Head = 23
RX_Left = 24
RX_Right = 25
# pi = pigpio.pi()

# Define object specific variables for gree

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

kp_b = 0.02
ki_b = 0
kd_b = 0.003

kp_us = 0.55
ki_us = 0
kd_us = 0.2

kp_e = 3  # 12
ki_e = 0
kd_e = 40  # 40if

setPoint_flag = 0

distance_head = 0
distance_left = 0
distance_right = 0

corr = 0
corr_pos = 0


def getTFminiData():
	# while True:
	global distance_head
	global distance_right
	global distance_left

	# while True:
	# time.sleep(0.01)  # change the value if needed
	# (count, recv) = pi.bb_serial_read(RX)
	(count_head, recv_head) = pwm.bb_serial_read(RX_Head)
	(count_left, recv_left) = pwm.bb_serial_read(RX_Left)
	(count_right, recv_right) = pwm.bb_serial_read(RX_Right)

	if count_head > 8:
		for i in range(0, count_head - 9):
			if recv_head[i] == 89 and recv_head[i + 1] == 89:  # 0x59 is 89
				checksum = 0
				for j in range(0, 8):
					checksum = checksum + recv_head[i + j]
				checksum = checksum % 256
				if checksum == recv_head[i + 8]:
					global distance_head
					distance_head = recv_head[i + 2] + recv_head[i + 3] * 256
					strength_head = recv_head[i + 4] + recv_head[i + 5] * 256
	# print("DDDD : ", distance_head)

	if count_left > 8:
		for i in range(0, count_left - 9):
			if recv_left[i] == 89 and recv_left[i + 1] == 89:  # 0x59 is 89
				checksum = 0
				for j in range(0, 8):
					checksum = checksum + recv_left[i + j]
				checksum = checksum % 256
				if checksum == recv_left[i + 8]:
					global distance_left
					distance_left = recv_left[i + 2] + recv_left[i + 3] * 256
					strength_left = recv_left[i + 4] + recv_left[i + 5] * 256
	# print("distance_left : ", distance_left)

	if count_right > 8:
		for i in range(0, count_right - 9):
			if recv_right[i] == 89 and recv_right[i + 1] == 89:  # 0x59 is 89
				checksum = 0
				for j in range(0, 8):
					checksum = checksum + recv_right[i + j]
				checksum = checksum % 256
				if checksum == recv_right[i + 8]:
					global distance_right
					distance_right = recv_right[i + 2] + recv_right[i + 3] * 256
					strength_right = recv_right[i + 4] + recv_right[i + 5] * 256


# print("distance_right : ", distance_right)


def correctPosition(setPoint, head, x, y, trigger, counter, green, red, blue, orange, reset, reverse):
	# print("INSIDE CORRECT")
	# getTFminiData()
	global glob, prevError, totalError, prevErrorGyro, totalErrorGyro, distance_left, distance_right, corr_pos

	error = 0
	correction = 0
	pTerm_e = 0
	dTerm_e = 0
	iTerm_e = 0
	lane = counter % 4
	# if(time.time() - last_time > 0.001):
	if lane == 0:
		if reverse:
			error = y - setPoint
		else:
			error = setPoint - y
	#print(f"lane:{lane} error: {error} tagret:{setPoint}")
	# print(f"trigger : {flag_t} setPoint: {setPoint} lane: {lane} correction:{correction}, error:{error} x:{x}, y:{y}, prevError :{prevError} angle:{head - correction}")
	elif lane == 1:
		if reverse:
			error = x - (setPoint - 100)

		else:
			error = x - (100 - setPoint)
	#print(f"lane:{lane}, error:{error} target:{(100 - setPoint)}")
	# print(f" trigger : {flag_t} setPoint: {setPoint} lane: {lane} correction:{correction}, error:{error} x:{x}, y:{y}, prevError :{prevError} angle:{head - correction}")
	elif lane == 2:
		if reverse:
			error = y - (setPoint - 200)
		else:
			error = y - (200 - setPoint)
	#print(f"lane:{lane} error:{error} target:{(200 - setPoint)}")
	# print(f"setPoint: {flag_t} lane: {lane} correction:{correction}, error:{error} x:{x}, y:{y}, prevError :{prevError} angle:{head - correction}")
	elif lane == 3:
		if reverse:
			error = (setPoint - 100) - x
		else:
			error = (100 - setPoint) - x

	#print(f"lane:{lane} error: {error} target:{-100 - setPoint}")
	# print(f"setPoint: {flag_t} lane: {lane} correction:{correction}, error:{error} x:{x}, y:{y}, prevError :{prevError} angle:{head - correction}")
	# last_time = time.time()
	corr_pos = error
	pTerm_e = kp_e * error
	dTerm_e = kd_e * (error - prevError)
	totalError += error
	iTerm_e = ki_e * totalError
	correction = pTerm_e + iTerm_e + dTerm_e

	# print("correction: {}, x:{}, y:{}, heading:{} ".format(correction, x, y, glob))

	# if (setPoint_flag == 0)

	if setPoint == 0:
		if abs(error) < 15:
			correction = 0
	if setPoint == -10 or setPoint == 10:
		if abs(error) < 10:
			correction = 0
	# print("In the  correct Position")

	if not reset:
		getTFminiData()
		if setPoint == -70:
			if distance_left < 20 or (distance_head < 70 and distance_right < 30):
				print(f"Correcting Green Wall")
				correction = 10
			else:
				pass
		# print(f"Correction : {correction}")

		if setPoint == 70:
			# print(f"distance_left: {distance_left}")
			if (distance_right < 20) or (distance_head < 70 and distance_left < 30):
				print(f"Correcting Red Wall")
				correction = -10
			else:
				pass

	if setPoint == 0:
		if correction > 30:
			correction = 30
		elif correction < -30:
			correction = -30
	else:
		if correction > 45:
			correction = 45
		elif correction < -45:
			correction = -45

	#print(f"Correction in position:{correction}")

	# print(f"lane: {lane} correction:{correction}, error:{error} x:{x}, y:{y}, prevError :{prevError} angle:{head - correction}")
	# print("correction: ", correction)

	prevError = error
	# print(f"Correction: {head - correction}")
	correctAngle(head + correction, reverse)


def correctAngle(setPoint_gyro, reverse):
	# print("INSIDE CORRECT")
	# time.sleep(0.001)
	global glob, corr

	error_gyro = 0
	prevErrorGyro = 0
	totalErrorGyro = 0
	correction = 0
	totalError = 0
	prevError = 0

	heading = imu.read_imu(reverse)

	glob = heading

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
	# print(f"correction in imu: {correction}")

	if correction > 30:
		correction = 30
	elif correction < -30:
		correction = -30

	# print("correction: ", e)

	prevErrorGyro = error_gyro
	# print("Error1 : {}, correction1 : {}, error : {}, correction : {} ".format(error1, correction1, error_gyro, correctio n ))
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
def Live_Feed(color_b, stop_b, red_b, green_b, pink_b, avoid_park, centr_y, centr_y_red, centr_y_pink):
	print('Image Process started')
	both_flag = False
	all_flag = False
	only_red = False
	only_green = False
	only_pink = False
	pink_red = False
	pink_green = False
	while True:
		try:
			picam2 = Picamera2()
			break
		except RuntimeError:
			picam2.uninit()
			picam2 = Picamera2()
	picam2.preview_configuration.main.size = (1280, 720)
	picam2.preview_configuration.main.format = 'RGB888'

	picam2.preview_configuration.align()
	picam2.configure('preview')

	picam2.start()
	# picam2.set_controls({"AfMode":0, "LensPosition": 1})

	# color_b.value = False
	cv2.namedWindow('Object Dist Measure ', cv2.WINDOW_NORMAL)
	cv2.resizeWindow('Object Dist Measure ', 1280, 720)

	while True:
		img = picam2.capture_array()

		hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # green
		hsv_img1 = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # red
		hsv_img2 = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # pink

		# predefined mask for green colour detection
		# For Green Color
		lower = np.array([40, 54, 36])  # green
		upper = np.array([73, 150, 155])
		mask = cv2.inRange(hsv_img, lower, upper)
		mask = cv2.dilate(mask, kernel, iterations=3)
		mask = cv2.erode(mask, kernel, iterations=3)

		# For Red Color
		lower1 = np.array([171, 159, 61])  # red
		upper1 = np.array([179, 255, 147])
		mask1 = cv2.inRange(hsv_img1, lower1, upper1)
		mask1 = cv2.dilate(mask1, kernel, iterations=5)
		mask1 = cv2.erode(mask1, kernel, iterations=5)

		# For Pink Color
		lower2 = np.array([154, 168, 59])  # pink
		upper2 = np.array([171, 217, 192])
		mask2 = cv2.inRange(hsv_img2, lower2, upper2)
		mask2 = cv2.dilate(mask2, kernel, iterations=5)
		mask2 = cv2.erode(mask2, kernel, iterations=5)

		# Remove Extra garbage from image
		d_img = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=5)  # green
		d_img1 = cv2.morphologyEx(mask1, cv2.MORPH_OPEN, kernel, iterations=5)  # red
		d_img2 = cv2.morphologyEx(mask2, cv2.MORPH_OPEN, kernel, iterations=5)  # pink

		# find the histogram
		cont, hei = cv2.findContours(d_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		cont = sorted(cont, key=cv2.contourArea, reverse=True)[:1]

		cont1, hei1 = cv2.findContours(d_img1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		cont1 = sorted(cont1, key=cv2.contourArea, reverse=True)[:1]

		cont2, hei2 = cv2.findContours(d_img2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		cont2 = sorted(cont2, key=cv2.contourArea, reverse=True)[:1]

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
			avoid_park.value = True
			pink_green = False
		if all_flag:
			color_b.value = True
			# print("ITS THE FIRST LOOP")
			### FOR GREEN BOX
			if cv2.contourArea(max_cnt) > cv2.contourArea(max_cnt1) and cv2.contourArea(max_cnt) > cv2.contourArea(max_cnt2):
				if cv2.contourArea(max_cnt) > 1000 and cv2.contourArea(max_cnt) < 306000:
					# Draw a rectange on the contour
					rect = cv2.minAreaRect(max_cnt)
					box = cv2.boxPoints(rect)
					box = np.intp(box)
					cv2.drawContours(img, [box], -1, (255, 0, 0), 3)
					(x, y, w, h) = cv2.boundingRect(box)
					centroid_y = y + h // 2
					centr_y.value = centroid_y
					green_b.value = True
					red_b.value = False
					pink_b.value = False

			### FOR RED BOX
			if cv2.contourArea(max_cnt1) > cv2.contourArea(max_cnt) and cv2.contourArea(max_cnt1) > cv2.contourArea(max_cnt2):
				if (cv2.contourArea(max_cnt1) > 1000 and cv2.contourArea(max_cnt1) < 306000):
					# Draw a rectange on the contour
					rect1 = cv2.minAreaRect(max_cnt1)
					box = cv2.boxPoints(rect1)
					box = np.intp(box)
					cv2.drawContours(img, [box], -1, (255, 0, 0), 3)

					(x, y, w, h) = cv2.boundingRect(box)

					centroid_y_red = y + h // 2

					centr_y_red.value = centroid_y_red
					red_b.value = True
					green_b.value = False
					pink_b.value = False

			#### FOR PINK BOX
			if cv2.contourArea(max_cnt2) > cv2.contourArea(max_cnt) and cv2.contourArea(max_cnt2) > cv2.contourArea(max_cnt1):
				if (cv2.contourArea(max_cnt2) > 2000 and cv2.contourArea(max_cnt2) < 306000):
					# Draw a rectange on the contour
					rect2 = cv2.minAreaRect(max_cnt2)
					box = cv2.boxPoints(rect2)
					box = np.intp(box)
					cv2.drawContours(img, [box], -1, (255, 0, 0), 3)

					(x, y, w, h) = cv2.boundingRect(box)

					centroid_y = y + h // 2

					centr_y_pink.value = centroid_y
					# if centroid_y > 500:
					pink_b.value = True
					red_b.value = False
					green_b.value = False


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
				centr_y_red.value = centroid_y_red
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
				centr_y.value = centroid_y
				# if(centroid_y > 100):
				# if(counter_green >= max_count):
				green_b.value = True
				pink_b.value = False
				red_b.value = False
				counter_red = 0

		### FOR PINK BOX
		elif only_pink:
			if (cv2.contourArea(max_cnt2) > 2000 and cv2.contourArea(max_cnt2) < 306000):
				# Draw a rectange on the contour
				rect2 = cv2.minAreaRect(max_cnt2)
				box = cv2.boxPoints(rect2)
				box = np.intp(box)
				cv2.drawContours(img, [box], -1, (255, 0, 0), 3)

				(x, y, w, h) = cv2.boundingRect(box)

				centroid_y = y + h // 2

				centr_y_pink.value = centroid_y

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
					centr_y.value = centroid_y
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

					centr_y_red.value = centroid_y_red
					red_b.value = True
					green_b.value = False
					pink_b.value = False

		elif pink_red:
			### FOR RED BOX
			if cv2.contourArea(max_cnt1) > cv2.contourArea(max_cnt2):
				if (cv2.contourArea(max_cnt1) > 1000 and cv2.contourArea(max_cnt1) < 306000):
					# Draw a rectange on the contour
					rect1 = cv2.minAreaRect(max_cnt1)
					box = cv2.boxPoints(rect1)
					box = np.intp(box)
					cv2.drawContours(img, [box], -1, (255, 0, 0), 3)

					(x, y, w, h) = cv2.boundingRect(box)

					centroid_y_red = y + h // 2

					centr_y_red.value = centroid_y_red
					red_b.value = True
					green_b.value = False
					pink_b.value = False
			elif cv2.contourArea(max_cnt2) > cv2.contourArea(max_cnt1):
				if (cv2.contourArea(max_cnt2) > 1000 and cv2.contourArea(max_cnt2) < 306000):
					# Draw a rectange on the contour
					rect2 = cv2.minAreaRect(max_cnt2)
					box = cv2.boxPoints(rect2)
					box = np.intp(box)
					cv2.drawContours(img, [box], -1, (255, 0, 0), 3)

					(x, y, w, h) = cv2.boundingRect(box)

					centroid_y = y + h // 2

					centr_y_pink.value = centroid_y
					# if centroid_y > 500:
					pink_b.value = False
					red_b.value = True
					green_b.value = False

		elif pink_green:
			if cv2.contourArea(max_cnt) > cv2.contourArea(max_cnt2):
				if (cv2.contourArea(max_cnt) > 2000 and cv2.contourArea(max_cnt) < 306000):
					# Draw a rectange on the contour
					rect = cv2.minAreaRect(max_cnt)
					box = cv2.boxPoints(rect)
					box = np.intp(box)
					cv2.drawContours(img, [box], -1, (255, 0, 0), 3)

					(x, y, w, h) = cv2.boundingRect(box)

					centroid_y = y + h // 2

					centr_y.value = centroid_y
					red_b.value = False
					green_b.value = True
					pink_b.value = False
			if cv2.contourArea(max_cnt2) > cv2.contourArea(max_cnt):
				if (cv2.contourArea(max_cnt2) > 2000 and cv2.contourArea(max_cnt2) < 306000):
					# Draw a rectange on the contour
					rect2 = cv2.minAreaRect(max_cnt2)
					box = cv2.boxPoints(rect2)
					box = np.intp(box)
					cv2.drawContours(img, [box], -1, (255, 0, 0), 3)
					(x, y, w, h) = cv2.boundingRect(box)
					centroid_y = y + h // 2
					centr_y_pink.value = centroid_y
					# if centroid_y > 500:
					pink_b.value = True
					red_b.value = False
					green_b.value = False
		# print(f"Green:{green_present}, red:{red_present}, pink:{pink_present}")
		#print(f"all:{all_flag}, only_red:{only_red}, only_green:{only_green}, only_pink:{only_pink}, pink_green:{pink_green}, pink_red:{pink_red}, both:{both_flag}")
		#print(f"green:{green_b.value}  red:{red_b.value}, pink:{pink_b.value}")
		cv2.imshow('Object Dist Measure ', img)

		if cv2.waitKey(1) & 0xFF == ord('q'):
			stop_b.value = True
			break
	# print(dist2)

	cv2.destroyAllWindows()
	picam2.stop()


# pwm.set_PWM_dutycycle(12, power.value)


def servoDrive(pwm, color_b, stop_b, red_b, green_b, pink_b, avoid_park, counts, centr_y, centr_y_red, centr_y_pink):
	# print("ServoProcess started")
	global heading, glob, distance_head, imu, corr, corr_pos
	# lobal x, y
	flag_t = 0
	pb_time = 0
	pwm.set_mode(RX_Head, pigpio.INPUT)
	pwm.set_mode(RX_Left, pigpio.INPUT)
	pwm.set_mode(RX_Right, pigpio.INPUT)
	pwm.set_mode(12, pigpio.OUTPUT)  # Set pin 12 as an output
	pwm.set_mode(20, pigpio.OUTPUT)  # Set pin 20 as an output
	pwm.hardware_PWM(12, 100, 0)
	try:
		pwm.bb_serial_read_open(RX_Head, 115200)
		pwm.bb_serial_read_open(RX_Left, 115200)
		pwm.bb_serial_read_open(RX_Right, 115200)
	except pigpio.error as e:
		if e == 'GPIO already in use':
			pwm.stop()
			pwm.bb_serial_read_open(RX_Head, 115200)
			pwm.bb_serial_read_open(RX_Left, 115200)
			pwm.bb_serial_read_open(RX_Right, 115200)
	count = 0
	x = 0
	y = 0
	previous_state = 0
	button_state = 0
	button = False
	pwm.set_PWM_dutycycle(12, 0)  # Set duty cycle to 50% (128/255)

	heading_angle = 0

	red_stored = False
	green_stored = False

	trigger = False
	counter = 0
	reverse = False
	correctAngle(heading_angle, reverse)
	enc = EncoderCounter()
	enc.start_counter()

	setPointL = -70
	setPointR = 70
	reset_f = False
	change_path = False
	green_turn = False
	turn_t = 0
	current_time = 0
	timer_started = False
	block_list = [0]
	power = 55
	prev_power = 0

	g_flag = False
	r_flag = False
	p_flag = False

	gp_time = 0
	rp_time = 0
	pp_time = 0

	g_past = False
	r_past = False
	p_past = False

	green_stored = False
	prev_time = 0
	red_turn = False
	buff = 0

	blue_flag = False
	orange_flag = False

	color_n = ""
	reset_done = False
	parking_flag = False
	correct_heading = False

	parking_counter = -1
	green_count = 0
	red_count = 0
	time_g = 1
	change_heading = False
	new_head = 0
	imu_head = 0
	change_path_complete = False
	p_turn = False
	pink_stored = False
	sp_change = False
	parking_heading = False
	last_counter = 12
	c_time = 0
	stop_flag = False
	calc_time = False
	start_parking = False
	continue_parking = False
	adjust = False
	p_count = 0
	parking_counter_store = False
	lap_finish = False
	pink_and_red = False
	change_counter = -1
	pink_and_green = False
	pink_avoid = False
	prev_distance = 0
	turn_red = False
	p_time = 0
	counter_reset = False
	while True:
		#print(f"Pink detetcted at counter: {parking_counter}")
		#print(f"pink:{pink_b.value}, green:{green_b.value}, red:{red_b.value}")
		#print(f"distance_left:{distance_left} imu: {glob} heading:{heading_angle} last_counter:{last_counter}, coutner: {counter}, parking_counter:{parking_counter}")
		if counter == last_counter:
			if not lap_finish:
				print("last counter detected")
				if not start_parking:
					start_parking = True
				lap_finish = True

		if lap_finish and start_parking and not counter_reset:
			counter = counter % 12
			counter_reset = True

		if counter == parking_counter and start_parking and not continue_parking:
			r_past = False
			green_stored = False
			red_stored = False
			r_flag = False
			p_flag = True
			p_past = True
			getTFminiData()
			prev_distance = distance_left
			print("Starting Parking...")
			continue_parking = True

		if (pink_b.value):### DECIDES SETPOINT WHENEVER PINK IS IN THE FRAME
			#print(f"Pink detected while green is there :{counter}")
			if orange_flag:
				setPointL = -30
				setPointR = 70
				if not pink_and_green:
					pink_and_green = True

			elif blue_flag:
				setPointR = 30
				setPointL = -70
				if not pink_and_red:
					pink_and_red = True
			pb_time = time.time()
		elif not pink_b.value and time.time() - pb_time > 1:         ### IF DOES NOT SEE PINK, KEEP THE SAME SETPOINT FOR 1 SECOND AND THEN CHANGE

			setPointL = -70
			setPointR = 70
		else:

			pass




		if continue_parking: ### THIS SETPOINT IS WHEN THE ROBOT IS IN THE PARKING MODE
			p_flag = True
			p_past = True
			g_past = False
			g_flag = False
			r_flag = False
			r_past = False
			trigger = False
			reset_f = False
			if orange_flag:
				setPointR = -20

			elif blue_flag:
				setPointL = 15


		try:
			color_sensor = imu.get_color()          ### RETURNS COLOR SENSOR VALUES
			if (imu.color_rgb[0] == 0 and imu.color_rgb[1] == 0 and imu.color_rgb[2] == 0):
				continue

			# print(f"Color: {color_sensor}, {imu.color_rgb}")
			previous_state = button_state
			button_state = GPIO.input(5)

			if previous_state == 1 and button_state == 0:
				button = not (button)
				power = 60

			if button:              ##### THIS BLOCK OF CODE WHEN BUTTON IS PRESSED
				################ PARKING ################
				if not change_path:
					imu_head = glob
				else:
					print("in else of change path...")
					heading_angle = heading_angle - 180
					blue_flag = False
					orange_flag = False
					reverse = True
					change_counter = -1
					change_path = False
					print("Path Change Complete...")

				x, y = enc.get_position(imu_head, counts.value)
				# print(f"power:{power}")
				total_power = (power * 0.1) + (prev_power * 0.9)

				prev_power = total_power
				pwm.set_PWM_dutycycle(
					12, 2.55 * total_power
				)  # Set duty cycle to 50% (128/255)

				pwm.write(20, 1)  # Set pin 20 high
				if stop_b.value:
					power = 0
					prev_power = 0
				###### DECIDES THE DIRECTION
				if (not blue_flag and not orange_flag) or change_path:
					if color_sensor == "Orange":
						orange_flag = True
						blue_flag = False
						color_n = "Orange"

					elif color_sensor == "Blue":
						blue_flag = True
						orange_flag = False
						color_n = "Blue"
				###### REVERSE PATH IF RED BLOCK

				if parking_flag and not stop_flag:
					p_flag = False
					p_past = False
					correctAngle(heading_angle, reverse)

					print(f"distance_head : {distance_head}, distance_left:{distance_left}")
					if not stop_flag and abs(corr) < 10:

						print("Inside Parking Loop")

						if not calc_time:
							c_time = time.time()
							calc_time = True
						while time.time() - c_time < 0.5:
							print("Reversing backward...")
							power = 100
							prev_power = 0
							pwm.set_PWM_dutycycle(
								12, power
							)  # Set duty cycle to 50% (128/255)
							# GPIO.output(20, GPIO.LOW) # Set AIN
							pwm.write(20, 0)  # Set pin 20 hig

						'''while 1 and not adjust:
							print("Reversing backward...")
							power = 100
							prev_power = 0
							pwm.set_PWM_dutycycle(
								12, power
							)  # Set duty cycle to 50% (128/255)
							# GPIO.output(20, GPIO.LOW) # Set AIN
							pwm.write(20, 0)  # Set pin 20 hig
							if(centr_y_pink.value > 100  and pink_b.value):
								power = 0
								prev_power = 0
								pwm.set_PWM_dutycycle(12, power)
								time.sleep(0.5)
								adjust = True
								break'''

						power = 30
						pwm.set_PWM_dutycycle(12, power)
						if orange_flag:
							#print(f"time111: {time.time() - c_time}")
							if not parking_heading:
								heading_angle = heading_angle - 90
								parking_heading = True

						elif blue_flag:
							if not parking_heading:
								heading_angle = heading_angle + 90
								parking_heading = True
						correctAngle(heading_angle, reverse)
						getTFminiData()
						if (abs(corr) < 15) and ((distance_head < 5 and distance_head>=0)):
							power = 0
							prev_power = 0
							correctAngle(heading_angle, reverse)
							pwm.set_PWM_dutycycle(12, power)
							stop_flag = True
							print("Succesfully Parked...")


				###### FOR
				else:
					if reset_f:
						getTFminiData()
						x, y = enc.get_position(glob, counts.value)
						if blue_flag:               ### BLUE RESET BLOCK
							print("BLUE RESET...")
							if counter == change_counter and green_count == 1:
								change_path = False
								power = 60

							elif counter == change_counter and red_count == 1:
								change_path = True
								power = 60
								reset_f = False

							if (red_b.value or r_past) and (distance_head > 20): #red after trigger
								# if((green_b.value or green_turn) and centr_y.value < 700):
								print(f"Red Detected after trigger...")
								red_turn = True
								p_turn = False
								setPointR = 10
								correctPosition(setPointR, heading_angle, x, y, trigger, counter, g_flag, r_flag, blue_flag, orange_flag, reset_f, reverse)

							else:
								setPointR = 70
								if red_turn:
									time_g = 1
									if not red_stored:
										red_stored = True
								else:
									time_g = 0.5

								while 1:
									correctAngle(heading_angle, reverse)
									if (abs(corr) < 5):
										break

								if not timer_started:
									current_time = time.time()
									timer_started = True
								# getTFminiData()
								if not green_b.value and not g_past:
									# .g_past = True
									print('reversing diection red')
									while time.time() - current_time < time_g:
										# correctAngle(heading_angle)
										getTFminiData()
										x, y = enc.get_position(glob, counts.value)
										# print(f"distance_head : {distance_head} x: {x}, y: {y}, count:{counts.value}")
										# print(f"x: {x}, y: {y},  count:{counts.value} distance_head : {distance_head}")
										power = 100
										prev_power = 0
										pwm.set_PWM_dutycycle(
											12, power
										)  # Set duty cycle to 50% (128/255)
										# GPIO.output(20, GPIO.LOW) # Set AIN
										pwm.write(20, 0)  # Set pin 20 hig
									print('reversing diection red complete')
									# print(f"distance_head : {distance_head} x: {x}, y: {y}, count:{counts.value}")
									print('Stopping Motor...')

								elif ((green_b.value or green_turn) or g_past):
									green_turn = True
									# print(f"red_b: {red_b.value}, red_turn:{red_turn}, r_past:{r_past}")
									print('reversing diection Green')
									while 1:
										# correctAngle(heading_angle)
										buff = 4
										if (green_b.value and centr_y.value < 350) or (r_past and (time.time() - current_time > 2)):
											print(f"Breaking the loop...")
											break
										getTFminiData()
										x, y = enc.get_position(glob, counts.value)
										# print(f"x: {x}, y: {y},  count:{counts.value} distance_head : {distance_head}")
										power = 100
										prev_power = 0
										pwm.set_PWM_dutycycle(
											12, power
										)  # Set duty cycle to 50% (128/255)
										# GPIO.output(20, GPIO.LOW) # Set AIN
										pwm.write(20, 0)  # Set pin 20 hig
									print('Green reversing diection complete')
									print('Stopping Motor...')

								power = 0
								prev_power = 0
								pwm.set_PWM_dutycycle(
									12, power
								)  # Set duty cycle to 50% (128/255)

								getTFminiData()
								print(f"head : {distance_head}")
								time.sleep(1)

								counter = counter + 1
								lane_reset = counter % 4
								if lane_reset == 1:
									enc.x = (150 - distance_head * math.cos(math.radians(
										abs(corr)))) - 10
								if lane_reset == 2:
									enc.y = (250 - distance_head * math.cos(math.radians(abs(corr)))) - 10
								if lane_reset == 3:
									enc.x = (distance_head * math.cos(math.radians(abs(corr))) - 150) + 10
								if lane_reset == 0:
									enc.y = (distance_head * math.cos(math.radians(abs(corr))) - 50) + 10
								print('Resuming Motor...')

								power = 60

								heading_angle = -((90 * counter) % 360)

								reset_f = False
								green_turn = False
								red_turn = False
								p_turn = False

						if orange_flag:         ### ORANGE RESET BLOCK

							if counter == change_counter and green_count == 1:
								change_path = False
								power = 60
							elif counter == change_counter and red_count == 1 and reset_f:
								print("Changing path...")
								change_path = True
								power = 60
								reset_f = False
							getTFminiData()
							x, y = enc.get_position(glob, counts.value)
							if ((g_flag or green_turn) and distance_head > 20) : #green after trigger
								# if((green_b.value or green_turn) and centr_y.value < 700):
								print(f"Green Detected after trigger... ")
								if start_parking:
									p_flag = True
									p_past = True
									green_turn = False
								else:
									green_turn = True
								setPointL = -10
								p_turn = False
								#pink_turn = False

								correctPosition(setPointL, heading_angle, x, y, trigger, counter, g_flag, r_flag, blue_flag, orange_flag, reset_f,reverse)
							else:
								setPointL = -70
								if green_turn:
									time_g = 1
									if not green_stored:
										green_stored = True
								else:
									time_g = 0.5
								#print(f"IN ELSE ORANGE {time_g}")
								while 1:
									correctAngle(heading_angle, reverse)
									if (abs(corr) <5):
										break

								#print("Heading is under 5")
								if not timer_started:
									current_time = time.time()
									timer_started = True
								# getTFminiData()
								if not red_b.value and not r_past:
									# .g_past = True
									print('reversing diection green')
									while time.time() - current_time < time_g + 0.3 :
										# correctAngle(heading_angle)
										getTFminiData()
										x, y = enc.get_position(glob, counts.value)
										# print(f"distance_head : {distance_head} x: {x}, y: {y}, count:{counts.value}")
										# print(f"x: {x}, y: {y},  count:{counts.value} distance_head : {distance_head}")
										power = 100
										prev_power = 0
										pwm.set_PWM_dutycycle(12, power)  # Set duty cycle to 50% (128/255)
										# GPIO.output(20, GPIO.LOW) # Set AIN
										pwm.write(20, 0)  # Set pin 20 hig
									print('reversing diection green complete')
									# print(f"distance_head : {distance_head} x: {x}, y: {y}, count:{counts.value}")
									print('Stopping Motor...')

								elif ((red_b.value or red_turn) or r_past):
									red_turn = True
									# print(f"red_b: {red_b.value}, red_turn:{red_turn}, r_past:{r_past}")
									print(f'reversing diection red pink color: {pink_b.value} pink flag: {p_flag} {p_past}  red color:{red_b.value} red flag: {r_past} {r_flag}')
									while 1 :
										# correctAngle(heading_angle)
										buff = 4
										if (red_b.value and centr_y_red.value < 350) or (r_past and (time.time() - current_time > 2)):
											print(f"Breaking the loop...")
											break
										getTFminiData()
										x, y = enc.get_position(glob, counts.value)
										# print(f"x: {x}, y: {y},  count:{counts.value} distance_head : {distance_head}")
										power = 100
										prev_power = 0
										pwm.set_PWM_dutycycle(12, power)  # Set duty cycle to 50% (128/255)
										pwm.write(20, 0)  # Set pin 20 hig
									print('red reversing diection complete')
								print('Stopping Motor...')

								power = 0
								prev_power = 0
								pwm.set_PWM_dutycycle(
									12, power
								)  # Set duty cycle to 50% (128/255)

								getTFminiData()
								print(f"head : {distance_head}, counter:{counter}, change_counter:{change_counter} , red_count:{red_count}")
								time.sleep(1)

								counter = counter + 1
								lane_reset = counter % 4

								if lane_reset == 1:
									enc.x = (150 - distance_head * math.cos(math.radians(
										abs(corr)))) - 10
								if lane_reset == 2:
									enc.y = (
											        250 - distance_head * math.cos(math.radians(abs(corr)))) - 10
								if lane_reset == 3:
									enc.x = (
											        distance_head * math.cos(math.radians(abs(corr))) - 150) + 10
								if lane_reset == 0:
									enc.y = (
											        distance_head * math.cos(math.radians(abs(corr))) - 50) + 10
								print('Resuming Motor...')

								power = 60

								heading_angle = (90 * counter) % 360
								correct_heading = False

								reset_f = False
								green_turn = False
								red_turn = False
								p_turn = False
					else:
						if (color_sensor == color_n and not trigger and (time.time() - turn_t) > (4 + buff)):
							buff = 0
							reset_done = False
							timer_started = False
							correct_heading = False
							trigger = True
							reset_f = True
							setPointL = -70
							# lane = counter % 4
							# counter = counter + 1
							turn_t = time.time()

						elif color_sensor == 'White':
							trigger = False
					#print(f"\np_count: {p_count} x: {x}, y: {y}, imu: {imu_head} {heading_angle}\nreset_f :{reset_f} counter:{counter} trigger:{trigger}\ncolor:{color_sensor} {imu.color_rgb}\ngreen: {g_flag}, red: {r_flag}, pink: {p_flag}, green_stored: {green_stored}, g_past:{g_past}, r_past:{r_past}, p_past:{p_past}\nleft:{distance_left} right:{distance_right}, head:{distance_head}")
					#print(f"setPointL: {setPointL} setPointR: {setPointR}")


					if start_parking:
						g_past = False
						r_past = False
						green_stored = False
						red_stored = False
						g_flag = False
						r_flag = False



					###### PANDAV


					if green_b.value and (not r_flag and not p_flag):
						g_flag = True
						g_past = True
						if pink_and_green or pink_and_red:
							if not parking_counter_store:
								parking_counter = counter
								#print("pink counter is stored when red is present")
								parking_counter_store = True
						print('1')
					elif (not green_b.value and g_past) or time.time() - gp_time < 1:

						g_flag = True
						getTFminiData()
						# print(f"distance_right:{distance_right}")
						if distance_right < 50 and g_past:
							g_past = False
							green_stored = False
							g_flag = False


							buff = 0
							gp_time = time.time()
						green_count = 1
						red_count = 0
						print('2')
					elif red_b.value and (not g_flag and not p_flag):
						r_flag = True
						#green_stored = False
						r_past = True
						if pink_and_green or pink_and_red:
							if not parking_counter_store:
								parking_counter = counter
								#print("pink counter is stored when red is present")
								parking_counter_store = True

						# getTFminiData()


						print('3')
					elif (not red_b.value and r_past) or time.time() - rp_time < 1:
						r_flag = True
						getTFminiData()
						# print(f"distance_left:{distance_left}")
						if distance_left < 50 and r_past:
							print(f"red Avoid complete")
							r_past = False
							r_flag = False
							red_stored = False

							buff = 0
							rp_time = time.time()
						red_count = 1
						green_count = 0
						print('4')
					elif pink_b.value and not p_past and (not g_flag and not r_flag) and continue_parking:
						p_flag = True
						p_past = True
						print('5')
					elif p_past and continue_parking or time.time() - pp_time < 3:
						p_flag = True
						parking_flag = False
						getTFminiData()

						if distance_left <= 15 and (prev_distance - distance_left > 40) and p_past:
							# print(f"red Avoid complete")
							p_past = False
							p_flag = False
							pink_avoid = True
							parking_flag = True
							print("Pink Avoidance Complete...")
						#prev_distance = distance_left

						print('6')

					else:
						g_flag = False
						r_flag = False
						p_flag = False
						r_past = False
						g_past = False
						p_past = False
						if green_stored:
							g_flag = True
							g_past = True
							green_stored = False
						if red_stored:
							r_flag = True
							r_past = True
							red_stored = False
						if pink_stored:
							p_flag = True
							p_past = True
							pink_stored = False

						print('7')

					if g_flag:
						red_stored = False
						correctPosition(setPointL, heading_angle, x, y, trigger, counter, g_flag, r_flag, blue_flag, orange_flag, reset_f, reverse)
					elif r_flag:
						green_stored = False
						correctPosition(setPointR, heading_angle, x, y, trigger, counter, g_flag, r_flag, blue_flag, orange_flag, reset_f, reverse)
					elif p_flag:
						if orange_flag:
							#print("Avoiding Pink...")
							correctPosition(setPointR, heading_angle, x, y, trigger, counter, g_flag, r_flag, blue_flag, orange_flag, reset_f, reverse)
						elif blue_flag:
							#print("Avoiding Pink in Blue...")
							correctPosition(setPointL, heading_angle, x, y, trigger, counter, g_flag, r_flag, blue_flag, orange_flag, reset_f, reverse)
					else:
						correctPosition(0, heading_angle, x, y, trigger, counter, g_flag, r_flag, blue_flag, orange_flag, reset_f, reverse)
				print(f"x:{x}, y:{y}, prev_distance:{prev_distance}, distance_left:{distance_left}, imu:{glob}, heading:{heading_angle}, cp: {continue_parking}, counter:{counter}, last_c:{last_counter}, p_flag = {p_flag}, p_past:{p_past}, g_past:{g_past}, r_past: {r_past}")
			#print(f" reverse:{reverse}, imu: {imu_head}, counter:{counter} heading:{heading_angle} x:{x} y:{y}, green_count: {green_count}, red_count:{red_count}, change_path:{change_path}, last_counter:{last_counter}, blue:{blue_flag}, orange:{orange_flag}")

			else:
				power = 0
				pwm.hardware_PWM(12, 100, 0)
				heading_angle = 0

				counter = 0
				correctAngle(heading_angle, reverse)
				color_b.Value = False
				stop_b.value = False
				red_b.value = False
				green_b.value = False

		except Exception as e:
			print(f"Exception: {e}")
			imu = IMUandColorSensor(board.SCL, board.SDA)
			print("Reset Complete...")
			if isinstance(e, KeyboardInterrupt):
				power = 0
				pwm.hardware_PWM(12, 100, 0)
				heading_angle = 0
				counter = 0
				count = 0
				x = 0
				y = 0
				correctAngle(heading_angle, reverse)
				color_b.Value = False
				stop_b.value = False
				red_b.value = False
				green_b.value = False



def runEncoder(counts):
	print("Encoder Process Started")

	try:
		while True:

			line = ser.readline().decode().strip()
			# print(f"Line:{line}")
			try:
				counts.value = int(line)
			# print(f"Received data: {counts.value}")
			except ValueError:
				# print("Value Error")
				pass


	except KeyboardInterrupt:
		ser.close()


if __name__ == '__main__':
	try:
		pwm = pigpio.pi()

		counts = multiprocessing.Value('i', 0)
		color_b = multiprocessing.Value('b', False)
		stop_b = multiprocessing.Value('b', False)
		red_b = multiprocessing.Value('b', False)
		green_b = multiprocessing.Value('b', False)
		pink_b = multiprocessing.Value('b', False)
		avoid_park = multiprocessing.Value('b', False)
		centr_y = multiprocessing.Value('f', 0.0)
		centr_y_red = multiprocessing.Value('f', 0.0)
		centr_y_pink = multiprocessing.Value('f', 0.0)
		P = multiprocessing.Process(target=Live_Feed, args=(
			color_b, stop_b, red_b, green_b, pink_b, avoid_park, centr_y, centr_y_red, centr_y_pink))
		S = multiprocessing.Process(target=servoDrive, args=(
			pwm, color_b, stop_b, red_b, green_b, pink_b, avoid_park, counts, centr_y, centr_y_red, centr_y_pink,))
		E = multiprocessing.Process(target=runEncoder, args=(counts,))
		E.start()

		S.start()
		P.start()


	except KeyboardInterrupt:
		pwm.hardware_PWM(12, 100, 0)
		pwm.bb_serial_read_close(RX_Head)
		pwm.bb_serial_read_close(RX_Left)
		pwm.bb_serial_read_close(RX_Right)
		pwm.stop()
		# pwm12.stop()
		imu.close()

		GPIO.cleanup()
