import numpy as np
import RPi.GPIO as GPIO
import cv2
from picamera2 import Picamera2
import time
import multiprocessing
import pigpio
import board
import busio
from math import atan2, sqrt, pi
import os
from robotModule.classes.Encoder import EncoderCounter

os.system("sudo pkill pigpiod")
os.system("sudo pigpiod")
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

time.sleep(5)
glob = 0

GPIO.setmode(GPIO.BCM)

GPIO.setup(20, GPIO.OUT)  # Connected to AIN2
GPIO.setup(12, GPIO.OUT)
GPIO.setup(5, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Button to GPIO23
pwm12 = GPIO.PWM(12, 100)

# pwm = pigpio.pi()
# Parameters for servo
servo = 8

RX_Head = 23
RX_Left = 24
RX_Right = 25
# pi = pigpio.pi()

# Define object specific variables for green
dist = 15
focal = 1120
pixels = 30
width = 4

dist1 = 0
dist2 = 0

currentAngle = 0
error_gyro = 0
prevErrorGyro = 0
totalErrorGyro = 0
correcion = 0
totalError = 0
prevError = 0
kp = 0.6
ki = 0.1
kd = 0.5

kp_b = 0.02
ki_b = 0
kd_b = 0.003

kp_us = 0.55
ki_us = 0
kd_us = 0.2
setPoint_flag = 0

# pi = pigpio.pi()

distance_head = 0
distance_left = 0
distance_right = 0

center_x_green = 0
cneter_x_red = 0
center_y = 0
distance_x = 0
distance_y = 0
distance_c = 0
centroid_x = 0
centroid_y = 0
finish = 0

red_detect = 0
green_detect = 0


# green_present = False
# red_present = False

# pwm = pigpio.pi()


def Centre(frame):
	height, width, _ = frame.shape
	global center_x_green
	global center_x_red
	global center_y
	global centroid_y
	global power
	global distance_c
	global finish
	block_identified = False
	global distance_x
	if centroid_y < 360:
		distance_c = 0 - distance_c
	# print("green  =", green_b.value)
	# print("distance_c :", distance_c)
	# Calculate the center coordinates
	center_x_green = width // 2 + 470
	center_y = height // 2
	center_x_red = width // 2 - 470
	# Define the cube vertices (8 points)
	cube_size = 5
	vertices_green = np.array(
		[
			[center_x_green - cube_size, center_y - cube_size],
			[center_x_green + cube_size, center_y - cube_size],
			[center_x_green + cube_size, center_y + cube_size],
			[center_x_green - cube_size, center_y + cube_size],
			[center_x_green - cube_size, center_y - cube_size],
			[center_x_green + cube_size, center_y - cube_size],
			[center_x_green + cube_size, center_y + cube_size],
			[center_x_green - cube_size, center_y + cube_size],
		]
	)

	vertices_red = np.array(
		[
			[center_x_red - cube_size, center_y - cube_size],
			[center_x_red + cube_size, center_y - cube_size],
			[center_x_red + cube_size, center_y + cube_size],
			[center_x_red - cube_size, center_y + cube_size],
			[center_x_red - cube_size, center_y - cube_size],
			[center_x_red + cube_size, center_y - cube_size],
			[center_x_red + cube_size, center_y + cube_size],
			[center_x_red - cube_size, center_y + cube_size],
		]
	)

	# Define the cube edges (12 lines)
	edges = np.array(
		[
			[0, 1],
			[1, 2],
			[2, 3],
			[3, 0],
			[4, 5],
			[5, 6],
			[6, 7],
			[7, 4],
			[0, 4],
			[1, 5],
			[2, 6],
			[3, 7],
		]
	)

	# Draw the cube
	for edge in edges:
		thickness_green = 1
		thickness_red = 1
		# thickness = 1
		finish = 0
		if centroid_y >= 650 and green_b.value:
			thickness_green = 5
			finish = 1
		if centroid_y >= 650 and red_b.value:
			thickness_red = 5
			finish = 1

		cv2.line(
			frame,
			tuple(vertices_green[edge[0]]),
			tuple(vertices_green[edge[1]]),
			(0, 255, 0),
			thickness_green,
		)
		cv2.line(
			frame,
			tuple(vertices_red[edge[0]]),
			tuple(vertices_red[edge[1]]),
			(0, 255, 0),
			thickness_red,
		)

	return frame


distance_head = 0
distance_right = 0
distance_left = 0


def getTFminiData():
	# while True:
	global distance_head
	global distance_right
	global distance_left

	# while True:
	time.sleep(0.01)  # change the value if needed
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
					# print("distance_head : ", distance_head)

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


def correctAngle(setPoint_gyro):
	# print("INSIDE CORRECT")
	global distance_right
	global distance_c
	global centroid_x
	global centroid_y
	global red_detect
	global green_detect
	global glob

	error_gyro = 0
	prevErrorGyro = 0
	totalErrorGyro = 0
	correction = 0
	totalError = 0
	prevError = 0

	quat_i, quat_j, quat_k, quat_real = bno.quaternion
	heading = find_heading(quat_real, quat_i, quat_j, quat_k)
	glob = heading
	# if(heading > 180 and setPoint_gyro < 180):
	# heading =  heading - 360

	# print("Heading :", heading)
	error_gyro = heading - setPoint_gyro
	if error_gyro > 180:
		error_gyro = error_gyro - 360

	# print("Error : ", error_gyro)
	pTerm = 0
	dTerm = 0
	iTerm = 0

	pTerm = kp * error_gyro
	dTerm = kd * (error_gyro - prevErrorGyro)
	totalErrorGyro += error_gyro
	iTerm = ki * totalErrorGyro
	correction = pTerm + iTerm + dTerm
	# print("correction 1: ", correction)

	if setPoint_flag == 0:
		if correction > 30:
			correction = 30
		elif correction < -30:
			correction = -30

	else:

		if correction > 30:
			correction = 30
		elif correction < -30:
			correction = -30

	# print("correction: ", e)

	prevErrorGyro = error_gyro
	# print("Error1 : {}, correction1 : {}, error : {}, correction : {} ".format(error1, correction1, error_gyro, correctio n ))
	setAngle(90 - correction)


def correctBlock(distance_val, centroid_val):
	# print("INSIDE CORRECT")
	global distance_x
	global distance_c
	global centroid_x
	global centroid_y
	global red_detect
	global green_detect
	# print("Red : {}. Green : {}".format(red_detect, green_detect))
	error = 0
	prevError = 0
	totalError = 0
	correction = 0
	setPoint_gyro = 0

	# quat_i, quat_j, quat_k, quat_real = bno.quaternion
	# heading = find_heading(quat_real, quat_i, quat_j, quat_k)
	# glob = heading
	if green_detect:
		if centroid_val < 1180:
			# print("INSIDE LOOP")
			distance_val = distance_val - 0
		else:
			distance_val = 0 - distance_val
	elif red_detect:
		if centroid_val < 150:
			# print("INSIDE LOOP")
			distance_val = distance_val - 0
		else:
			distance_val = 0 - distance_val
		# distance_y = 0 - distance_y
	# print("Distance :", distance_val)
	error = distance_val - 0

	# print("Error : ", error)
	pTerm = 0
	dTerm = 0
	iTerm = 0

	pTerm = kp_b * error
	dTerm = -kd_b * (error - prevError)
	totalError += error
	iTerm = ki_b * totalError
	correction = pTerm + iTerm + dTerm
	# print("correction : ", correction)
	# print("Distance : {}, Centorid: {}, correction : {}".format(distance_val, centroid_val, correction))
	# if(heading > 180 and setPoint < 180):
	# heading =  heading - 360
	if setPoint_flag == 0:
		if correction > 30:
			correction = 30
		elif correction < -30:
			correction = -30

	else:

		if correction > 30:
			correction = 30
		elif correction < -30:
			correction = -30

	prevError = error

	setAngle(90 - correction)


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

	yaw = yaw_raw * 180 / pi
	yaw = yaw - 180

	if yaw > 0:
		yaw = 360 - yaw
	else:
		yaw = abs(yaw)
	return yaw  # heading in 360 clockwise


def setAngle(angle):
	pwm.set_servo_pulsewidth(servo, 500 + round(angle * 11.11))  # 0 degree


def BrightnessContrast(brightness=0):
	# getTrackbarPos returns the
	# current position of the specified trackbar.
	brightness = cv2.getTrackbarPos("Brightness", "GEEK")

	contrast = cv2.getTrackbarPos("Contrast", "GEEK")

	effect = controller(img, brightness, contrast)

	# The function imshow displays
	# an image in the specified window
	cv2.imshow("Effect", effect)


# find the distance from then camera
def get_dist(rectange_params, image, block):
	# find no of pixels covered
	pixels = rectange_params[1][0]
	global dist1
	global dist2
	dist1 = (width * focal) / pixels
	dist2 = (width * focal) / pixels
	# print(pixels)

	# calculate distance
	if block == "green":

		image = cv2.putText(
			image,
			"Distance from Camera GREEN in CM :",
			org,
			font,
			1,
			color,
			2,
			cv2.LINE_AA,
		)

		image = cv2.putText(
			image, str(dist1), (110, 50), font, fontScale, color, 1, cv2.LINE_AA
		)

	else:

		image = cv2.putText(
			image,
			"Distance from Camera RED in CM :",
			(700, 20),
			font,
			1,
			color,
			2,
			cv2.LINE_AA,
		)

		image = cv2.putText(
			image, str(dist2), (1000, 50), font, fontScale, color, 1, cv2.LINE_AA
		)
		# Wrtie n the image
	return image


# Extract Frames
# cap = cv2.VideoCapture(0)

# basic constants for opencv Functs
kernel = np.ones((3, 3), "uint8")
font = cv2.FONT_HERSHEY_SIMPLEX
org = (0, 20)
fontScale = 0.6
color = (0, 0, 255)
thickness = 2


# loop to capture video frames
def Live_Feed(
		distance, block, distance_center, centroid_x_val, color_b, stop_b, red_b, green_b
):
	global distance_x
	global distance_y
	global distance_c
	global centroid_x
	global centroid_y
	global finish
	global center_x_red
	global center_x_green
	print("Image Process started")
	picam2 = Picamera2()
	picam2.preview_configuration.main.size = (1280, 720)
	picam2.preview_configuration.main.format = "RGB888"
	picam2.preview_configuration.align()
	picam2.configure("preview")
	picam2.start()

	# color_b.value = False
	cv2.namedWindow("Object Dist Measure ", cv2.WINDOW_NORMAL)
	cv2.resizeWindow("Object Dist Measure ", 1280, 720)

	while True:

		img = picam2.capture_array()
		img = Centre(img)
		# print("STOP_B :", finish)
		if finish:
			stop_b.value = True
		else:
			stop_b.value = False
		# print("STOP_B :", stop_b.value)
		hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
		hsv_img1 = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

		# predefined mask for green colour detection

		lower = np.array([40, 88, 38])  # green
		upper = np.array([59, 163, 122])
		mask = cv2.inRange(hsv_img, lower, upper)
		mask = cv2.dilate(mask, kernel, iterations=3)
		mask = cv2.erode(mask, kernel, iterations=3)

		lower1 = np.array([155, 99, 56])  # red
		upper1 = np.array([179, 190, 149])
		mask1 = cv2.inRange(hsv_img1, lower1, upper1)
		mask1 = cv2.dilate(mask1, kernel, iterations=3)
		mask1 = cv2.erode(mask1, kernel, iterations=3)

		# Remove Extra garbage from image
		d_img = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=5)
		d_img1 = cv2.morphologyEx(mask1, cv2.MORPH_OPEN, kernel, iterations=5)
		# final_img = resize_final_img(300,300, mask, d_img)
		# final_img1 = resize_final_img(300,300, mask1, d_img1)

		# cv2.imshow("Image", final_img )
		# cv2.imshow("Image1", final_img1 )
		# find the histogram
		cont, hei = cv2.findContours(d_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		cont = sorted(cont, key=cv2.contourArea, reverse=True)[:1]

		cont1, hei1 = cv2.findContours(
			d_img1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
		)
		cont1 = sorted(cont1, key=cv2.contourArea, reverse=True)[:1]

		if len(cont) == 0:
			# print("Cant find contour for green....")
			green_present = False
			# color_b.value = False

		else:

			max_cnt = max(cont, key=cv2.contourArea)
			green_present = True
			# color_b.value = True

		if len(cont1) == 0:
			# print("Cant find contour for red....")
			red_present = False
			# color_b.value = False
		else:

			max_cnt1 = max(cont1, key=cv2.contourArea)
			red_present = True
			# color_b.value = True
		# max_cnt1 = max(cont1, key = cv2.contourArea)
		if not red_present and not green_present:
			color_b.value = False
			red_b.value = False
			green_b.value = False

		if red_present and green_present:
			color_b.value = True
			# green_b.value = True
			if cv2.contourArea(max_cnt) > cv2.contourArea(max_cnt1):

				if cv2.contourArea(max_cnt) > 100 and cv2.contourArea(max_cnt) < 306000:
					# Draw a rectange on the contour
					rect = cv2.minAreaRect(max_cnt)
					box = cv2.boxPoints(rect)
					box = np.int0(box)
					cv2.drawContours(img, [box], -1, (255, 0, 0), 3)
					(x, y, w, h) = cv2.boundingRect(box)
					centroid_x = x + w // 2
					centroid_y = y + h // 2

					# color_b.value = True

					centroid_x_val.value = centroid_x
					distance_x = centroid_x - center_x_green
					distance_y = centroid_y - center_y
					distance_c = sqrt(distance_x ** 2 + distance_y ** 2)
					distance_center.value = distance_c
					# correctBlock()
					# print("Distance from center: ", distance_c)
					# print("center_x : {}, center_y : {}".format(centroid_x, centroid_y))
					img = get_dist(rect, img, "green")
					distance.value = dist1
					block.value = 1
					green_b.value = True

			# check for contour area
			else:

				if (
						cv2.contourArea(max_cnt1) > 100
						and cv2.contourArea(max_cnt1) < 306000
				):
					# Draw a rectange on the contour
					rect1 = cv2.minAreaRect(max_cnt1)
					box = cv2.boxPoints(rect1)
					box = np.int0(box)
					cv2.drawContours(img, [box], -1, (255, 0, 0), 3)
					(x, y, w, h) = cv2.boundingRect(box)
					centroid_x = x + w // 2
					centroid_x_val.value = centroid_x
					centroid_y = y + h // 2

					distance_x = centroid_x - center_x_red
					distance_y = centroid_y - center_y
					distance_c = sqrt(distance_x ** 2 + distance_y ** 2)
					distance_center.value = distance_c
					# correctBlock()
					img = get_dist(rect1, img, "red")
					distance.value = dist2
					block.value = 2
					red_b.value = True

		elif red_present:
			color_b.value = True

			if cv2.contourArea(max_cnt1) > 100 and cv2.contourArea(max_cnt1) < 306000:
				# Draw a rectange on the contour
				rect1 = cv2.minAreaRect(max_cnt1)
				box = cv2.boxPoints(rect1)
				box = np.int0(box)
				cv2.drawContours(img, [box], -1, (255, 0, 0), 3)
				(x, y, w, h) = cv2.boundingRect(box)
				centroid_x = x + w // 2
				centroid_x_val.value = centroid_x
				centroid_y = y + h // 2
				# if(centroid_y > 50):
				# color_b.value = True
				# red_b.value = True
				distance_x = centroid_x - center_x_red
				distance_y = centroid_y - center_y

				distance_c = sqrt(distance_x ** 2 + distance_y ** 2)
				distance_center.value = distance_c
				# correctBlock()
				img = get_dist(rect1, img, "red")
				distance.value = dist2
				block.value = 2
				red_b.value = True

		elif green_present:
			color_b.value = True

			if cv2.contourArea(max_cnt) > 100 and cv2.contourArea(max_cnt) < 306000:
				# Draw a rectange on the contour
				rect = cv2.minAreaRect(max_cnt)
				box = cv2.boxPoints(rect)
				box = np.int0(box)
				cv2.drawContours(img, [box], -1, (255, 0, 0), 3)
				(x, y, w, h) = cv2.boundingRect(box)
				centroid_x = x + w // 2
				centroid_y = y + h // 2

				centroid_x_val.value = centroid_x
				distance_x = centroid_x - center_x_green
				distance_y = centroid_y - center_y
				distance_c = sqrt(distance_x ** 2 + distance_y ** 2)
				distance_center.value = distance_c
				# correctBlock()
				img = get_dist(rect, img, "green")
				distance.value = dist1
				block.value = 1
				green_b.value = True

		cv2.imshow("Object Dist Measure ", img)

		if cv2.waitKey(1) & 0xFF == ord("q"):
			break
		# print(dist2)

	cv2.destroyAllWindows()
	picam2.stop()
	GPIO.cleanup()


prev_state_A = 0
prev_state_B = 0

count = 0
prev_count = 0


def servoDrive(
		distance,
		block,
		pwm,
		distance_center,
		centroid_x_val,
		color_b,
		stop_b,
		red_b,
		green_b,
):
	# print("ServoProcess started")

	global heading
	global red_detect
	global green_detect
	global prev_state_A, prev_state_B, count, prev_count, glob
	pwm.set_mode(servo, pigpio.OUTPUT)

	pwm.set_PWM_frequency(servo, 50)

	pwm.set_mode(RX_Head, pigpio.INPUT)
	pwm.set_mode(RX_Left, pigpio.INPUT)
	pwm.set_mode(RX_Right, pigpio.INPUT)
	pwm.bb_serial_read_open(RX_Head, 115200)
	pwm.bb_serial_read_open(RX_Left, 115200)
	pwm.bb_serial_read_open(RX_Right, 115200)

	previous_state = 0
	button_state = 0
	button = False
	correctAngle(0)
	pwm12.start(0)
	power = 0
	turn_flag = False

	heading_angle = 0

	trigger = False
	counter = 0
	tf_flag = False
	tf_off_time = 0
	turn_threshold = 0
	block_follow_off = 0
	block_flag = False

	enc = EncoderCounter()
	enc.start_counter()

	while True:

		count = enc.get_count()
		x, y = enc.get_position(glob)
		time.sleep(0.00005)
		print("x: {}, y:{}, heading:{}, count:{}".format(x, y, glob, count))
		# tf_flag = False
		init_flag = False
		# print("Color : ", color_b.value)

		# print("STOP_B :", stop_b.value)
		tf_flag = False
		block_flag = False
		previous_state = button_state
		button_state = GPIO.input(5)
		# print("Trigger: ",trigger)

		if previous_state == 1 and button_state == 0:
			button = not (button)
			init_flag = True

		if button:

			current_time = time.time()
			correctAngle(heading_angle)
			if tf_flag and current_time - tf_off_time > 3:
				tf_flag = False

			if block_flag and current_time - block_off_time > 0.2:
				block_flag = False

			if not block_flag:
				pass
			else:
				correctAngle(heading_angle)

			if not tf_flag:
				getTFminiData()

			else:

				correctAngle(heading_angle)

			if stop_b.value:

				correctAngle(heading_angle)

			else:

				if color_b.value and not tf_flag:
					tf_flag = True
					if green_b.value:

						green_detect = 1
						red_detect = 0

					elif red_b.value:

						red_detect = 1
						green_detect = 0

					correctBlock(distance_center.value, centroid_x_val.value)

					# print("Followig Block...")
				else:
					correctAngle(heading_angle)

					# print("Following IMu...")

			# print("Button is pressed")
			if init_flag:
				for power in np.arange(0, 60, 0.01):
					pwm12.ChangeDutyCycle(power)  # Set PWMA
				init_flag = False
			pwm12.ChangeDutyCycle(power)

			if counter == -1:
				global start_time
				if time.time() - start_time > 1.1:
					power = 0
			# print("trigger : {}, counter : {} , heading: {}, glob : {},  stop_b : {}, tf_flag : {}".format(trigger, counter, heading_angle, glob, stop_b.value, tf_flag))

			GPIO.output(20, GPIO.HIGH)  # Set AIN1
			if counter == 12:

				if distance_head < 150 and distance_right < 95 and not trigger:
					start_time = time.time()
					counter = -1

			if (
					distance_right > 100
					and not trigger
					and distance_head < 150
					and not tf_flag
					and not block_flag
			):
				counter = counter + 1
				heading_angle = (90 * counter) % 360
				trigger = True
				tf_flag = True

				tf_off_time = time.time()
				if stop_b.value:
					block_flag = True
					block_off_time = time.time()

			if distance_right < 100 and distance_head > 100:
				trigger = False

		else:
			if init_flag:
				init_flag = False

			pwm12.ChangeDutyCycle(0)
			tf_flag = False
			heading_angle = 0
			counter = 0
			correctAngle(heading_angle)
			color_b.Value = False
			stop_b.value = False
			red_b.value = False
			green_b.value = False


if __name__ == "__main__":

	try:
		pwm = pigpio.pi()
		distance = multiprocessing.Value("f", 0.0)
		block = multiprocessing.Value("i", 0)
		distance_center = multiprocessing.Value("f", 0.0)
		centroid_x_val = multiprocessing.Value("f", 0.0)
		color_b = multiprocessing.Value("b", False)
		stop_b = multiprocessing.Value("b", False)
		red_b = multiprocessing.Value("b", False)
		green_b = multiprocessing.Value("b", False)
		P = multiprocessing.Process(
			target=Live_Feed,
			args=(
				distance,
				block,
				distance_center,
				centroid_x_val,
				color_b,
				stop_b,
				red_b,
				green_b,
			),
		)
		S = multiprocessing.Process(
			target=servoDrive,
			args=(
				distance,
				block,
				pwm,
				distance_center,
				centroid_x_val,
				color_b,
				stop_b,
				red_b,
				green_b,
			),
		)

		P.start()
		S.start()

	except:
		pwm.bb_serial_read_close(RX_Head)
		pwm.bb_serial_read_close(RX_Left)
		pwm.bb_serial_read_close(RX_Right)
		pwm.stop()
		cb_A.cancel()
		cb_B.cancel()
		GPIO.cleanup()
