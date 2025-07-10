import os

os.system("sudo pkill pigpiod")
os.system("sudo pigpiod")

import RPi.GPIO as GPIO
import time
import multiprocessing
import pigpio
import board
from ctypes import c_float
import subprocess
from BNO085 import IMUandColorSensor
from Encoder import EncoderCounter
import serial
import sys
from BNO085 import IMUandColorSensor

log_file = open('/home/pi/WRO_CODE/logs/log_open.txt', 'w')
sys.stdout = log_file


blue_led = 26
red_led = 10
green_led = 6
reset_pin = 14
button_pin = 5
angle = 0

pwm = pigpio.pi()
if not pwm.connected:
    print("Could not connect to pigpio daemon")
    exit(1)

#### INITIALIZATION ####

# Set pin modes for LEDs and reset
for pin in [reset_pin, blue_led, red_led, green_led]:
    pwm.set_mode(pin, pigpio.OUTPUT)
    pwm.write(pin, 0)  # Set LOW

# Set button pin as input with pull-up
pwm.set_mode(button_pin, pigpio.INPUT)
pwm.set_pull_up_down(button_pin, pigpio.PUD_UP)

#### RESETTING ARDUINO ####

print("Resetting....")

pwm.write(reset_pin, 0)          # Pull reset LOW
pwm.write(green_led, 1)          # Turn on green LED
time.sleep(1)

pwm.write(reset_pin, 1)          # Release reset (HIGH)
pwm.write(green_led, 0)          # Turn off green LED
time.sleep(1)

print("Reset Complete")

glob = 0
ser = serial.Serial('/dev/UART_USB', 115200)

################# MULTIPROCESSING VARIABLES ############
distance = multiprocessing.Value("f", 0.0)
block = multiprocessing.Value("i", 0)
counts = multiprocessing.Value('i', 0)
head = multiprocessing.Value('f', 0.0)
head = multiprocessing.Value('f', 0.0)
sp_angle = multiprocessing.Value('i', 0)
turn_trigger = multiprocessing.Value('b', False)
# Shared memory for LIDAR and IMU
previous_angle = multiprocessing.Value('d', 0.0)
lidar_angle = multiprocessing.Value('d', 0.0)
lidar_distance = multiprocessing.Value('d', 0.0)
imu_shared = multiprocessing.Value('d', 0.0)
specific_angle = multiprocessing.Array(c_float, 3)  # shared array of 3 integers



rplidar = [None]*360
previous_distance = 0
dist_0 = 0
dist_90 = 0
dist_270 = 0
angle = 0
lidar_front = 0
lidar_left = 0
lidar_right = 0

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
setPoint_flag = 0

# pi = pigpio.pi()

distance_head = 0
distance_left = 0
distance_right = 0


def getTFminiData():
	# while True:

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
					strength_head = recv_head[i + 4] + recv_head[i + 5] * 256  # print("distance_head : ", distance_head)

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
					strength_left = recv_left[i + 4] + recv_left[i + 5] * 256  # print("distance_left : ", distance_left)

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
					strength_right = recv_right[i + 4] + recv_right[i + 5] * 256  # print("distance_right : ", distance_right)


corr = 0
prev_time = 0
reset_imu = False


def correctAngle(setPoint_gyro, left, right, trigger, heading):
	time.sleep(0.01)
	global glob, distance_right, distance_head, distance_left, corr, prev_time
	error_gyro = 0
	prevErrorGyro = 0
	totalErrorGyro = 0
	correction = 0
	totalError = 0
	prevError = 0
	buff = 0

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

	"""if(heading > 180 and setPoint_gyro < 180):	
	heading =  heading - 360"""
	print(f"before correction: {correction}")

	getTFminiData()
	if (distance_left < 15 and distance_left >= 0):
		print("Inside Left")
		# corr -=1
		# setPoint_gyro -= corr
		correction = correction - 20

	# distance_right = -1

	elif (distance_right < 15 and distance_right >= 0):
		print("inside right")
		# print(f"correction before decrement: {correction}")
		# corr += 1
		# setPoint_gyro -= corr
		# print(f"before correction: {correction}")

		correction = correction + 20

	else:
		correction = correction - 0

	if correction > 30:
		correction = 30

	elif correction < -30:
		correction = -30

	#print(f"time : {time.time() - prev_time} imu: {glob} correction : {correction} error: {error_gyro} left: {distance_left}, right:{distance_right}")
	prev_time = time.time()
	# print(f"setPoint:{setPoint_gyro} Correction: {correction}, error:{error_gyro} left:{left}, right:{right}, left_d:{distance_left}, right_d :{distance_right}")
	# print("correction: ", e)
	# print("heading: {}, error: {}, correction: {}, left:{}, right: {}".format(heading, error_gyro, correction, left, right))
	prevErrorGyro = error_gyro
	setAngle(90 - correction)


def setAngle(angle):
	pwm.set_servo_pulsewidth(servo, 500 + round(angle * 11.11))  # 0 degree


def servoDrive(distance, block, pwm, counts, head):
	print("ServoProcess started")
	global heading
	global distance_right, distance_head, distance_left

	pwm.set_mode(servo, pigpio.OUTPUT)
	imu = IMUandColorSensor(board.SCL, board.SDA)

	pwm.set_PWM_frequency(servo, 50)

	pwm.set_mode(RX_Head, pigpio.INPUT)
	pwm.set_mode(RX_Left, pigpio.INPUT)
	pwm.set_mode(RX_Right, pigpio.INPUT)

	pwm.bb_serial_read_open(RX_Head, 115200)
	pwm.bb_serial_read_open(RX_Left, 115200)
	pwm.bb_serial_read_open(RX_Right, 115200)

	pwm.set_mode(12, pigpio.OUTPUT)  # Set pin 12 as an output
	pwm.set_mode(20, pigpio.OUTPUT)  # Set pin 20 as an output
	pwm.hardware_PWM(12, 100, 0)
	pwm.set_PWM_dutycycle(12, 0)  # Set duty cycle to 50% (128/255)

	previous_state = 0
	button_state = 0
	button = False
	reset_servo = False
	start_time = 0
	power = 100
	prev_power = 0
	count = 0
	turn_flag = False
	heading_angle = 0
	target_angle = 0
	trigger = False
	counter = 0
	left_flag = False
	right_flag = False
	correctAngle(0, left_flag, right_flag, trigger, head.value)
	target_count = 0
	turn_t = 0
	try:
		while True:
			init_flag = False
			getTFminiData()
			previous_state = button_state
			button_state = pwm.read(5)
			color_sensor = imu.get_color()
			if previous_state == 1 and button_state == 0:
				button = not (button)
				init_flag = True
				power = 100

				print("Button is pressed")

			if button:
				if not reset_servo:
					setAngle(130)
					time.sleep(0.5)
					setAngle(0)
					reset_servo = True
					
					
				total_power = (power * 0.1) + (prev_power * 0.9)
				prev_power = total_power
				pwm.set_PWM_dutycycle(12, 2.55 * total_power)  # Set duty cycle to 50% (128/255)

				pwm.write(20, 1)  # Set pin 20 high

				if not right_flag and not left_flag:
					if distance_right > 100:
						right_flag = True
					elif distance_left > 100:
						left_flag = True

				correctAngle(heading_angle, left_flag, right_flag, trigger, head.value),

				if counter == 12:
					if distance_head < 150 and heading_angle == 0 and (head.value < 10 or head.value > 350):
						power = 0
						pwm.set_PWM_dutycycle(12, power)  # Set duty cycle to 50% (128/255)
						sys.exit()

				if right_flag:
					if(turn_trigger.value and not trigger and (time.time() - turn_t) > (4 + buff)):
						buff = 0
						heading_angle = (90 * counter) % 360
						trigger = True
						turn_t = time.time()
					elif not turn_trigger.value:
						trigger = False
						pwm.write(blue_led, 0)
					'''if (distance_right > 100 and distance_head < 75) and not trigger and (time.time() - turn_t) > 3:
						# time.sleep(0.5)
						counter = counter + 1
						heading_angle = (90 * counter) % 360
						trigger = True
						turn_t = time.time()

					if distance_right < 85 and distance_head > 75:
						trigger = False'''
				elif left_flag:
					'''if (distance_left > 100 and distance_head < 75) and not trigger and (time.time() - turn_t) > 3:
						# time.sleep(0.5)
						counter = counter + 1
						heading_angle = -((90 * counter) % 360)
						trigger = True
						turn_t = time.time()

					elif distance_left < 85 and distance_head > 75:
						trigger = False'''
				




			else:
				if init_flag:
					init_flag = False
				power = 0
				pwm.set_PWM_dutycycle(12, 0)
				pwm.hardware_PWM(12, 100, 0)
				heading_angle = 0
				counter = 0
				left_flag = False
				right_flag = False
				# counts.value = 0
				correctAngle(heading_angle, left_flag, right_flag, trigger, head.value)
			print(f"trigger:{trigger} ")
			#print(f"heading:{head.value} {heading_angle}  counter:{counter} {trigger},  target_count:{target_count}, encoder_c:{counts.value}, L C R:{distance_left} {distance_head} {distance_right}")
	except KeyboardInterrupt:
		#imu = IMUandColorSensor(board.SCL, board.SDA)
		power = 0
		pwm.set_PWM_dutycycle(12, 0)
	except Exception as e:
		print(f"Exception: {e}")

def runEncoder(counts, head):
	print("Encoder Process Started")

	try:
		while True:
			line = ser.readline().decode().strip()
			# print(f"Line:{line}")
			data = line.split(" ")
			try:
				if data[0].isdigit() or data[1].isdigit():
					counts.value = int(data[1])
					head.value = float(data[0])
				else:
					pass
			except ValueError:
				continue


	except KeyboardInterrupt:
		ser.close()


def read_lidar(lidar_angle, lidar_distance, previous_angle, imu_shared, sp_angle, turn_trigger, specific_angle):
	#print("This is first line")
	global CalledProcessError
	lidar_binary_path = '/home/pi/rplidar_sdk/output/Linux/Release/ultra_simple'
	print("⏳ Waiting for LIDAR output...")
	
	global previous_distance, lidar_front, lidar_left, lidar_right, angle  
	if not os.path.isfile(lidar_binary_path):
		print(f"❌ File not found: {lidar_binary_path}")
		return


	print("🚀 Launching ultra_simple...")

	process = subprocess.Popen(
		[lidar_binary_path, '--channel', '--serial', '/dev/LIDAR_USB', '460800'],
		stdout=subprocess.PIPE,
		stderr=subprocess.STDOUT,
		text=True
	)

	#try:
	for line in process.stdout:
		line = line.strip()
		#print(line)
		if "theta" in line and "Dist" in line:
			try:
				angle_part = line.split()
				#print(angle_part)
				
				angle_index = angle_part.index("theta:") + 1
				dist_index = angle_part.index("Dist:") + 1

				angle = float(angle_part[angle_index])
				distance = float(angle_part[dist_index])
				angle = int(angle)					
				imu_r = int(imu_shared.value)
				#print(f"📍 Angle: {angle:.2f}°, Distance: {distance:.2f} mm")
			except Exception as e:
				print("⚠️ Parse error:", e)
		else:
			print("ℹ️", line)


		if previous_angle.value != angle:
	
			while(angle - previous_angle.value > 1):
				lidar_angle.value = previous_angle.value + 1
				lidar_distance.value = previous_distance
				previous_angle.value = previous_angle.value + 1
				rplidar[int(lidar_angle.value)] = lidar_distance.value
				if(int(lidar_angle.value) == (0 + imu_r ) % 360):
					specific_angle[0] = lidar_distance.value
					lidar_front = lidar_distance.value
				if(int(lidar_angle.value) == (90 + imu_r ) % 360):
					specific_angle[1] = lidar_distance.value
					lidar_left = lidar_distance.value

				if(int(lidar_angle.value) == (270 + imu_r ) % 360):
					specific_angle[2] = lidar_distance.value
					lidar_right = lidar_distance.value

				#print(f"angles: {specific_angle} imu: {imu_shared.value} total:{imu_r + lidar_angle.value} sp_angle:{sp_angle.value}")
				
			if(distance != 0): 
				with lidar_angle.get_lock(), lidar_distance.get_lock(), previous_angle.get_lock(), imu_shared.get_lock():
					lidar_angle.value = angle
					lidar_distance.value = distance
					previous_distance = distance
					previous_angle.value = angle
					rplidar[int(lidar_angle.value)] = lidar_distance.value
					if(int(lidar_angle.value) == (0 + imu_r) % 360):
						specific_angle[0] = lidar_distance.value
						lidar_front = lidar_distance.value
					if(int(lidar_angle.value) == (90 + imu_r) % 360):
						specific_angle[1] = lidar_distance.value
						lidar_left = lidar_distance.value
					if(int(lidar_angle.value) == (270 + imu_r) % 360  ):
						specific_angle[2] = lidar_distance.value 
						lidar_right = lidar_distance.value                                      
					#print(f"angles: {specific_angle}, imu: {imu_shared.value} total:{imu_r + lidar_angle.value}")
			
			if(lidar_front < 800 and lidar_left < 900 and lidar_right > 1800):
				turn_trigger.value = True
			elif (lidar_front > 2000 and (lidar_left < 1000 or lidar_right < 1000)):
				turn_trigger.value = False
			#print(f"front: {lidar_front}. right:{lidar_right} left:{lidar_left} sp_angle:{sp_angle.value}, turn_trigger:{turn_trigger.value}")
					#print(f"angle: {lidar_angle.value} distance:{rplidar[int(lidar_angle.value)]}")

	'''except KeyboardInterrupt:
		print("🛑 Ctrl+C received. Stopping LIDAR.")
		process.terminate()
	finally:
		print("🔌 Lidar process ended.")'''



if __name__ == "__main__":
	try:

		
		S = multiprocessing.Process(target=servoDrive, args=(distance, block, pwm, counts, head,))
		E = multiprocessing.Process(target=runEncoder, args=(counts, head,))
		lidar_proc = multiprocessing.Process(target=read_lidar, args=(lidar_angle, lidar_distance, previous_angle, imu_shared, sp_angle, turn_trigger, specific_angle))

		S.start()
		E.start()
		lidar_proc.start()

	except KeyboardInterrupt:
		ser.close()
		E.terminate()
		S.terminate()
		lidar_proc.terminate()
		E.join()
		S.join()
		lidar_proc.join()
		pwm.hardware_PWM(12, 100, 0)
		pwm.bb_serial_read_close(RX_Head)
		pwm.bb_serial_read_close(RX_Left)
		pwm.bb_serial_read_close(RX_Right)
		pwm.stop()
		imu.close()
		GPIO.cleanup()