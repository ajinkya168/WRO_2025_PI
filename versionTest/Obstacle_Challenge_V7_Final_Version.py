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
from robotModule.classes.Encoder import EncoderCounter

from robotModule.classes.BNO085 import IMUandColorSensor
from robotModule.classes.Servo import Servo
import serial

ser = serial.Serial('/dev/UART_USB', 115200)
print("Connection Established...")
ser.write(b"1")
print("Command sent: b'1'")
ser.flush()
time.sleep(5)
imu = IMUandColorSensor(board.SCL, board.SDA)

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
kd_e = 40  # 40

setPoint_flag = 0

distance_head = 0
distance_left = 0
distance_right = 0

corr = 0


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


def correctPosition(setPoint, head, x, y, trigger, counter, green, red, blue, orange, reset):
	# print("INSIDE CORRECT")
	# getTFminiData()
	global glob, prevError, totalError, prevErrorGyro, totalErrorGyro, distance_left, distance_right

	error = 0
	correction = 0
	pTerm_e = 0
	dTerm_e = 0
	iTerm_e = 0
	lane = counter % 4
	# if(time.time() - last_time > 0.001):
	if lane == 0:
			error = setPoint - y
			print(f"error: {error} tagret:{setPoint}")
	# print(f"trigger : {flag_t} setPoint: {setPoint} lane: {lane} correction:{correction}, error:{error} x:{x}, y:{y}, prevError :{prevError} angle:{head - correction}")
	elif lane == 1:
			error = x - (100 - setPoint)

			print(f"error:{error} target:{(100 - setPoint)}")
	# print(f" trigger : {flag_t} setPoint: {setPoint} lane: {lane} correction:{correction}, error:{error} x:{x}, y:{y}, prevError :{prevError} angle:{head - correction}")
	elif lane == 2:
			error = y - (200 - setPoint)

	# print(f"setPoint: {flag_t} lane: {lane} correction:{correction}, error:{error} x:{x}, y:{y}, prevError :{prevError} angle:{head - correction}")
	elif lane == 3:
		if orange:
			error = (setPoint - 100) - x
		if blue:
			error =	x - (- 100 - setPoint)
 
		print(f"error: {error} target:{-100 - setPoint}")			
	# print(f"setPoint: {flag_t} lane: {lane} correction:{correction}, error:{error} x:{x}, y:{y}, prevError :{prevError} angle:{head - correction}")
	# last_time = time.time()

	pTerm_e = kp_e * error
	dTerm_e = kd_e * (error - prevError)
	totalError += error
	iTerm_e = ki_e * totalError
	correction = pTerm_e + iTerm_e + dTerm_e

	# print("correction: {}, x:{}, y:{}, heading:{} ".format(correction, x, y, glob))

	# if (setPoint_flag == 0)
	
	if(setPoint == 0):

		if abs(error) < 2:
			correction = 0

	#print("In the  correct Position")

	if  not reset:
		getTFminiData()

		if green or lane == 0 :
			print(f"Correcting Green Wall")
			if (distance_left  < 25 ):
				correction = 10
			else:
				pass
		#print(f"Correction : {correction}")

		if red or lane == 0 :
			#print(f"distance_left: {distance_left}")
			print(f"Correcting Red Wall")
			if(distance_right < 25):
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

	print(f"Correction in position:{correction}")

	# print(f"lane: {lane} correction:{correction}, error:{error} x:{x}, y:{y}, prevError :{prevError} angle:{head - correction}")
	# print("correction: ", correction)

	prevError = error
	# print(f"Correction: {head - correction}")
	correctAngle(head + correction)


def correctAngle(setPoint_gyro):
	# print("INSIDE CORRECT")
	# time.sleep(0.001)
	global glob, corr

	error_gyro = 0
	prevErrorGyro = 0
	totalErrorGyro = 0
	correction = 0
	totalError = 0
	prevError = 0

	heading = imu.read_imu()

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
	print(f"correction in imu: {correction}")

	if correction > 30:
		correction = 30
	elif correction < -30:
		correction = -30

	# print("correction: ", e)

	prevErrorGyro = error_gyro
	# print("Error1 : {}, correction1 : {}, error : {}, correction : {} ".format(error1, correction1, error_gyro, correctio n ))
	servo.setAngle(90 - correction)


def BrightnessContrast(brightness=0):
	# getTrackbarPos returns the
	# current position of the specified trackbar.
	brightness = cv2.getTrackbarPos('Brightness', 'GEEK')

	contrast = cv2.getTrackbarPos('Contrast', 'GEEK')

	effect = controller(img, brightness, contrast)

	# The function imshow displays
	# an image in the specified window
	cv2.imshow('Effect', effect)


# Extract Frames

# basic constants for opencv Functs
kernel = np.ones((3, 3), 'uint8')
font = cv2.FONT_HERSHEY_SIMPLEX
org = (0, 20)
fontScale = 0.6
color = (0, 0, 255)
thickness = 2


# loop to capture video frames
def Live_Feed(color_b, stop_b, red_b, green_b, centr_y):
	counter_red = 0
	counter_green = 0
	max_count = 1
	print('Image Process started')
	try:
		picam2 = Picamera2()
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
		# img = Centre(img)
		# print("STOP_B :", finish)

		# print("STOP_B :", stop_b.value)
		hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
		hsv_img1 = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

		# predefined mask for green colour detection

		lower = np.array([40, 54, 36])  # green
		upper = np.array([73, 150, 155])
		mask = cv2.inRange(hsv_img, lower, upper)
		mask = cv2.dilate(mask, kernel, iterations=3)
		mask = cv2.erode(mask, kernel, iterations=3)

		lower1 = np.array([131, 136, 56])  # red
		upper1 = np.array([179, 240, 180])
		mask1 = cv2.inRange(hsv_img1, lower1, upper1)
		mask1 = cv2.dilate(mask1, kernel, iterations=5)
		mask1 = cv2.erode(mask1, kernel, iterations=5)

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
			counter_green = 0
		# color_b.value = False

		else:
			max_cnt = max(cont, key=cv2.contourArea)
			green_present = True
		# color_b.value = True

		if len(cont1) == 0:
			# print("Cant find contour for red....")
			red_present = False
			counter_red = 0
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
			counter_red = 0
			counter_green = 0

		if red_present and green_present:
			color_b.value = True
			# green_b.value = True
			#print(cv2.contourArea(max_cnt))
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
					# if(centroid_y > 100):
					green_b.value = True
					# if(counter_green >= max_count):
					# green_b.value = True
					red_b.value = False
					counter_red = 0

			# check for contour area
			else:
				#print(cv2.contourArea(max_cnt1))
				if (
						cv2.contourArea(max_cnt1) > 1000
						and cv2.contourArea(max_cnt1) < 306000
				):
					# Draw a rectange on the contour
					rect1 = cv2.minAreaRect(max_cnt1)
					box = cv2.boxPoints(rect1)
					box = np.intp(box)
					cv2.drawContours(img, [box], -1, (255, 0, 0), 3)

					(x, y, w, h) = cv2.boundingRect(box)

					centroid_y = y + h // 2

					centr_y.value = centroid_y
					# if(centroid_y > 100):
					red_b.value = True
					# if(counter_red >= max_count):

					green_b.value = False
					counter_green = 0

		elif red_present:
			color_b.value = True
			#print(cv2.contourArea(max_cnt1))
			if cv2.contourArea(max_cnt1) > 1000 and cv2.contourArea(max_cnt1) < 306000:
				# Draw a rectange on the contour
				rect1 = cv2.minAreaRect(max_cnt1)
				box = cv2.boxPoints(rect1)
				box = np.intp(box)
				cv2.drawContours(img, [box], -1, (255, 0, 0), 3)
				(x, y, w, h) = cv2.boundingRect(box)

				centroid_y = y + h // 2
				centr_y.value = centroid_y
				# if(centroid_y > 100):
				red_b.value = True
				# if(counter_red >= max_count):

				green_b.value = False
				counter_green = 0

		elif green_present:
			color_b.value = True
			#print(cv2.contourArea(max_cnt))
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

				red_b.value = False
				counter_red = 0

		cv2.imshow('Object Dist Measure ', img)

		if cv2.waitKey(1) & 0xFF == ord('q'):
			stop_b.value = True
			break
	# print(dist2)

	cv2.destroyAllWindows()
	picam2.stop()

# pwm.set_PWM_dutycycle(12, power.value)


def servoDrive(pwm, color_b, stop_b, red_b, green_b, counts, centr_y):
	# print("ServoProcess started")
	global heading, glob, distance_head, imu, corr
	# lobal x, y
	flag_t = 0
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
	init_flag = False
	# pwm12.start(0)
	pwm.set_PWM_dutycycle(12, 0)  # Set duty cycle to 50% (128/255)

	heading_angle = 0

	red_stored = False
	green_stored = False

	trigger = False
	counter = 0
	correctAngle(heading_angle)
	enc = EncoderCounter()
	enc.start_counter()
	trig_counter = 0
	green_time = 0.5
	red_time = 0.5
	setPointL = -90
	setPointR = 90
	reset_f = False
	turn_f = False
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
	g_time = 0
	r_time = 0

	gp_time = 0
	rp_time = 0
	g_past = False
	r_past = False
	green_stored = False
	prev_time = 0
	red_turn = False
	buff = 0
	blue_flag = False
	orange_flag = False
	color_n = ""
	reset_done = False
	while True:
		"""print(1/(time.time() - prev_time))
		prev_time = time.time()"""
		# time.sleep(1)
		#print(f"Counts:{counts.value}")
		try:

			# init_flag = False
			color_sensor = imu.get_color()
			if (imu.color_rgb[0] == 0 and imu.color_rgb[1] == 0 and imu.color_rgb[2] == 0):
				continue
			#print(f"{imu.color_rgb}, {color_sensor}")
			previous_state = button_state
			button_state = GPIO.input(5)
			# print("Trigger: ",trigger)



			#print(f"red_tunr: {red_turn}, r_past:{r_past}, red:{r_flag}")
			# print(f"color: {color_sensor} x: {x}, y: {y} , trigger:{trigger} distance_head : {distance_head}")
			print(f"Color: {color_sensor}, {imu.color_rgb}")
			if previous_state == 1 and button_state == 0:
				button = not (button)
				init_flag = True
				power = 60

			if button:


				if stop_b.value:
					power = 0
					prev_power = 0

				x, y = enc.get_position(glob, counts.value)

				# print(f"power:{power}")
				total_power = (power * 0.1) + (prev_power * 0.9)

				prev_power = total_power
				pwm.set_PWM_dutycycle(
					12, 2.55 * total_power
				)  # Set duty cycle to 50% (128/255)

				pwm.write(20, 1)  # Set pin 20 high




###### DECIDES THE DIRECTION
				if not blue_flag and not orange_flag:
					if color_sensor == "Orange":
						orange_flag = True
						blue_flag = False
						color_n = "Orange"

					elif color_sensor == "Blue":
						blue_flag = True
						orange_flag = False
						color_n = "Blue"
				#print(color_n)
###### REVERSE PATH IF RED BLOCK
				if change_path:
					heading_angle = heading_angle - 180
					#### TODO: RESET THE IMU
					if orange_flag:
						blue_flag = True
						orange_flag = False
					elif blue_flag:
						orange_flag = True
						blue_flag = False

###### FOR

				if reset_f:
					getTFminiData()
					lane_reset = counter % 4
					if blue_flag:
						print("BLUE RESET...")
						#print(f"distance_head : {distance_head} x: {x}, y: {y}")
						if ((red_b.value or red_turn or r_past) and (color_sensor == "White")) and (distance_head > 10):
							# if((green_b.value or green_turn) and centr_y.value < 700):
							print(f"Green Detected after trigger...")
							red_turn = True
							if not red_stored:
								red_stored = True
							correctPosition(0, heading_angle, x, y, trigger, counter, g_flag, r_flag, blue_flag, orange_flag, reset_f)
						else:
							while 1:
								correctAngle(heading_angle)
								if(abs(corr) < 5):
									break

							if (abs(corr) < 5):  # s(heading_angle - glob) < 10 or abs(heading_angle - glob) > 355):
								if not timer_started:
									current_time = time.time()
									timer_started = True
								# getTFminiData()
								if not green_b.value and not g_past:
									#.g_past = True
									print('reversing diection red')
									while time.time() - current_time < 1:
										#correctAngle(heading_angle)
										getTFminiData()
										x, y = enc.get_position(glob, counts.value)
										#print(f"distance_head : {distance_head} x: {x}, y: {y}, count:{counts.value}")
										#print(f"x: {x}, y: {y},  count:{counts.value} distance_head : {distance_head}")
										power = 100
										prev_power = 0
										pwm.set_PWM_dutycycle(
											12, power
										)  # Set duty cycle to 50% (128/255)
										# GPIO.output(20, GPIO.LOW) # Set AIN
										pwm.write(20, 0)  # Set pin 20 hig
									print('reversing diection red complete')
									#print(f"distance_head : {distance_head} x: {x}, y: {y}, count:{counts.value}")
									print('Stopping Motor...')

								elif ((green_b.value or green_turn) or g_past):
									green_turn = True
									#print(f"red_b: {red_b.value}, red_turn:{red_turn}, r_past:{r_past}")
									print('reversing diection Green')
									while 1:
										#correctAngle(heading_angle)
										buff = 4
										if(green_b.value and centr_y.value < 350):

											print(f"Breaking the loop...")
											break
										getTFminiData()
										x, y = enc.get_position(glob, counts.value)
										#print(f"x: {x}, y: {y},  count:{counts.value} distance_head : {distance_head}")
										power = 100
										prev_power = 0
										pwm.set_PWM_dutycycle(
											12, power
										)  # Set duty cycle to 50% (128/255)
										# GPIO.output(20, GPIO.LOW) # Set AIN
										pwm.write(20, 0)  # Set pin 20 hig
									print('Green reversing diection complete')
									reset_done = True
									g_flag = True
									print('Stopping Motor...')

								power = 0
								prev_power = 0
								pwm.set_PWM_dutycycle(
									12, power
								)  # Set duty cycle to 50% (128/255)


								getTFminiData()
								print(f"head : {distance_head}")
								time.sleep(1)
								if lane_reset == 1:
									enc.x = (150 - distance_head*math.cos(math.radians(
										abs(corr)))) -10
								if lane_reset == 2:
									enc.y = (250 - distance_head*math.cos(math.radians(abs(corr)))) -10
								if lane_reset == 3:
									enc.x = (distance_head*math.cos(math.radians(abs(corr))) - 150) +10
								if lane_reset == 0:
									enc.y = (distance_head*math.cos(math.radians(abs(corr))) - 50) +10
								print('Resuming Motor...')

								power = 60

								heading_angle = -((90 * counter) % 360)

								reset_f = False
								green_turn = False 
								red_turn = False

					if orange_flag :
						#print("ORANGE RESET..")
						#print("COUNTINGGGG:")
						#print(f"distance_head : {distance_head} x: {x}, y: {y}")
						if (((green_b.value or green_turn) or g_past) and (color_sensor == "White" or color_sensor == "Orange")) and (distance_head > 2):
							# if((green_b.value or green_turn) and centr_y.value < 700):
							print(f"Green Detected after trigger...")
							green_turn = True
							if not green_stored:
								green_stored = True
								g_past = False
							correctPosition(0, heading_angle, x, y, trigger, counter, g_flag, r_flag, blue_flag, orange_flag, reset_f)
						else:
							print("IN ELSE ORANGE")
							while 1:
								correctAngle(heading_angle)
								if(abs(corr) < 5):
									break
							print(f"corr: {corr}")
							if (abs(corr) < 5):  # s(heading_angle - glob) < 10 or abs(heading_angle - glob) > 355):
								print("Heading is under 5")
								if not timer_started:
									current_time = time.time()
									timer_started = True
								# getTFminiData()
								if not red_b.value and not r_past:
									#.g_past = True
									print('reversing diection green')
									while time.time() - current_time < 1:
										#correctAngle(heading_angle)
										getTFminiData()
										x, y = enc.get_position(glob, counts.value)
										#print(f"distance_head : {distance_head} x: {x}, y: {y}, count:{counts.value}")
										#print(f"x: {x}, y: {y},  count:{counts.value} distance_head : {distance_head}")
										power = 100
										prev_power = 0
										pwm.set_PWM_dutycycle(12, power)  # Set duty cycle to 50% (128/255)
										# GPIO.output(20, GPIO.LOW) # Set AIN
										pwm.write(20, 0)  # Set pin 20 hig
									print('reversing diection green complete')
									#print(f"distance_head : {distance_head} x: {x}, y: {y}, count:{counts.value}")
									print('Stopping Motor...')

								elif ((red_b.value or red_turn) or r_past):
									red_turn = True
									#print(f"red_b: {red_b.value}, red_turn:{red_turn}, r_past:{r_past}")
									print('reversing diection red')
									while 1:
										#correctAngle(heading_angle)
										buff = 4
										if(red_b.value and centr_y.value < 350):

											print(f"Breaking the loop...")
											break
										getTFminiData()
										x, y = enc.get_position(glob, counts.value)
										#print(f"x: {x}, y: {y},  count:{counts.value} distance_head : {distance_head}")
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
								print(f"head : {distance_head}")
								time.sleep(1)
								if lane_reset == 1:
									enc.x = (150 - distance_head*math.cos(math.radians(
										abs(corr)))) -10
								if lane_reset == 2:
									enc.y = (250 - distance_head*math.cos(math.radians(abs(corr)))) -10
								if lane_reset == 3:
									enc.x = (distance_head*math.cos(math.radians(abs(corr))) - 150) +10
								if lane_reset == 0:
									enc.y = (distance_head*math.cos(math.radians(abs(corr))) - 50) +10
								print('Resuming Motor...')

								power = 60
								heading_angle = (90 * counter) % 360


								reset_f = False
								green_turn = False 
								red_turn = False
				else:

					if counter == 12 and not trigger:
						power = 0


					if (color_sensor == color_n and not trigger and (time.time() - turn_t) > (4 + buff)):
						buff = 0
						reset_done = False
						timer_started = False
						trigger = True
						reset_f = True
						counter = counter + 1
						turn_t = time.time()


					elif color_sensor == 'White':
						trigger = False
					print(f"\nx: {x}, y: {y}, imu: {glob} {heading_angle}\nreset_f :{reset_f} counter:{counter} trigger:{trigger}\ncolor:{color_sensor} {imu.color_rgb}\ngreen: {g_flag}, red: {r_flag}, green_stored: {green_stored}, g_past:{g_past}, r_past:{r_past}\nleft:{distance_left} right:{distance_right}, head:{distance_head}")


###### PANDAV
				if green_b.value and not r_flag:
					g_flag = True
					g_past = True
					print('1')
				elif not green_b.value and g_past or time.time() - gp_time < 0.5:
					g_flag = True
					getTFminiData()
					# print(f"distance_right:{distance_right}")
					if distance_right < 50 and g_past:
						#print(f"Green Avoid complete")
						g_past = False
						green_stored = False
						g_flag = False
						gp_time = time.time()
					print('2')
				elif red_b.value and not g_flag:
					r_flag = True

					green_stored = False
					# getTFminiData()
					r_past = True
					print('3')
				elif not red_b.value and r_past or time.time() - rp_time < 0.5:
					r_flag = True
					g_past = False
					getTFminiData()
					# print(f"distance_left:{distance_left}")
					if distance_left < 50 and r_past:
						#print(f"red Avoid complete")
						r_past = False
						r_flag = False
						rp_time = time.time()
					print('4')

				else:
					g_flag = False
					r_flag = False
					r_past = False
					g_past = False
					if green_stored:
						g_flag = True
						g_time = time.time()
						g_past = True
						green_stored = False
					print('5')

				if g_flag:
					green_time = time.time()
					#print("Correcting green.....")
					correctPosition(setPointL, heading_angle, x, y, trigger, counter, g_flag, r_flag, blue_flag, orange_flag, reset_f)

				elif r_flag:
					red_time = time.time()
					# print("correcting red...")
					green_stored = False
					correctPosition(setPointR, heading_angle, x, y, trigger, counter, g_flag, r_flag, blue_flag, orange_flag, reset_f)
				else:
					# print("Correcting 0...")
					correctPosition(0, heading_angle, x, y, trigger, counter, g_flag, r_flag, blue_flag, orange_flag, reset_f)
			else:
				if init_flag:
					init_flag = False

				power = 0
				pwm.hardware_PWM(12, 100, 0)
				heading_angle = 0

				counter = 0
				count = 0
				x = 0
				y = 0
				correctAngle(heading_angle)
				color_b.Value = False
				stop_b.value = False
				red_b.value = False
				green_b.value = False
		except Exception as e:
			print(f"Exception: {e}")
			imu = IMUandColorSensor(board.SCL, board.SDA)
			print("Reset Complete...")

def runEncoder(counts):
	print("Encoder Process Started")

	try:
		while True:

			line = ser.readline().decode().strip()
			#print(f"Line:{line}")
			try:
				counts.value = int(line)
			#print(f"Received data: {counts.value}")
			except ValueError:
				#print("Value Error")
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
		trig = multiprocessing.Value('b', False)
		centr_y = multiprocessing.Value('f', 0.0)

		P = multiprocessing.Process(target=Live_Feed, args=(color_b, stop_b, red_b, green_b, centr_y,))
		S = multiprocessing.Process( target=servoDrive, args=(pwm, color_b, stop_b, red_b, green_b, counts, centr_y,))
		E = multiprocessing.Process(target=runEncoder, args=(counts,))
		E.start()
		#S.start()
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

