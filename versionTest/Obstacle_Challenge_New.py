from TFmini import TFmini
import RPi.GPIO as GPIO
import serial
from Servo import Servo
from Encoder import EncoderCounter
import math
import board
import pigpio
import multiprocessing
from ctypes import c_float
import cv2
import numpy as np
import logging
import sys
import os
import time
import subprocess
import camera_utilities

os.system('sudo pkill pigpiod')
os.system('sudo pigpiod')


def kill_process_on_port(port):
    try:
        # Find PID using the port
        result = subprocess.check_output(
            f"lsof -t -i:{port}", shell=True
        ).decode().strip()

        if result:
            pids = result.split('\n')
            for pid in pids:
                print(f"Killing PID {pid} using port {port}")
                os.system(f"kill -9 {pid}")
    except subprocess.CalledProcessError:
        print(f"No process is using port {port}")


# kill_process_on_port(5003)
time.sleep(5)


# import RPi.GPIO as GPIO
# import time


#log_file = open('/home/pi/WRO_2025_PI/logs/log_9.txt', 'w')
#sys.stdout = log_file

# PINS

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

# INITIALIZATION
# pwm = pigpio.pi()
process = None
ser = serial.Serial('/dev/UART_USB', 115200)
print("created uart")

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

########### IMPORTING CLASSES ###############
servo = Servo(servo_pin)
tfmini_lock = multiprocessing.Lock()

tfmini = TFmini(RX_Head, RX_Left, RX_Right, RX_Back)
# app = Flask(__name__)

rplidar = [None]*360
previous_distance = 0
dist_0 = 0
dist_90 = 0
dist_270 = 0
angle = 0
lidar_front = 0
lidar_left = 0
lidar_right = 0

#########  MULTIPROCESSING VARIABLE ###########

counts = multiprocessing.Value('i', 0)
color_b = multiprocessing.Value('b', False)
stop_b = multiprocessing.Value('b', False)
red_b = multiprocessing.Value('b', False)
green_b = multiprocessing.Value('b', False)
pink_b = multiprocessing.Value('b', False)
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
sp_angle = multiprocessing.Value('i', 0)
turn_trigger = multiprocessing.Value('b', False)
# Shared memory for LIDAR and IMU
previous_angle = multiprocessing.Value('d', 0.0)
lidar_angle = multiprocessing.Value('d', 0.0)
lidar_distance = multiprocessing.Value('d', 0.0)
imu_shared = multiprocessing.Value('d', 0.0)
specific_angle = multiprocessing.Array(c_float, 3)  # shared array of 3 integers
lidar_f = multiprocessing.Value('d', 0.0)

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

###################################################


def correctPosition(setPoint, head, x, y, counter, blue, orange, reset, reverse, heading, centr_x_p, centr_y_g, centr_y_r, centr_y_p, finish, distance_h, distance_l, distance_r):
    # print("INSIDE CORRECT")
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
        print(
            f"lane: {lane}, error: {error} target:{(setPoint)}, x:{x} y:{y} not reverse")
    elif lane == 1:
        if orange:
            error = x - (100 - setPoint)
            print(
                f"lane:{lane}, error:{error} target:{(100 - setPoint)}, x:{x}, y:{y}")

        elif blue:
            error = (100 + setPoint) - x
            # print(f"lane:{lane}, error:{error} target:{(100 + setPoint)}, x:{x} y:{y} Bluee")
    # print(f" trigger : {flag_t} setPoint: {setPoint} lane: {lane} correction:{correction}, error:{error} x:{x}, y:{y}, prevError :{prevError} angle:{head - correction}")
    elif lane == 2:
        if orange:
            error = y - (200 - setPoint)
            print(f"lane:{lane} error:{error} target:{(200 - setPoint)},  x: {x} y: {y} setPoint:{setPoint}")
        elif blue:
            error = y - (-200 - setPoint)
            # print(f"lane:{lane} error:{error} target:{(-200 - setPoint)}, x: {x} y{y}")
    # print(f"setPoint: {flag_t} lane: {lane} correction:{correction}, error:{error} x:{x}, y:{y}, prevError :{prevError} angle:{head - correction}")
    elif lane == 3:
        if orange:
            error = (setPoint - 100) - x
            print(f"lane:{lane} error:{error} target:{(setPoint - 100)}, x: {x} y {y}")
        elif blue:
            error = x + (100 + setPoint)
            # print(f"lane:{lane} error:{error} target:{(100 + setPoint)}, x:{x} y {y}")

    corr_pos = error
    pTerm_e = kp_e * error
    dTerm_e = kd_e * (error - prevError)
    totalError += error
    iTerm_e = ki_e * totalError
    correction = pTerm_e + iTerm_e + dTerm_e

    if setPoint == 0:
        if abs(error) < 10:
            # print("absolute is 0")
            correction = 0

    if not reset:
        if (((setPoint == -35 or setPoint == 0) and orange) or (counter == 0 and (centr_x_p < 800 and centr_x_p > 0) and ((centr_y_g or centr_y_r) <= centr_y_p) and not blue and not orange) and not finish):
            if distance_l <= 30:
                correction = 20
                print(f"Avoiding pink wall {correction}")

            elif distance_r < 50:
                if distance_r <= 35:
                    correction = -45
                    print(f"Avoiding pink wall {correction}")

                else:
                    correction = -10
                    print(f"Avoiding pink wall {correction}")

            else:
                correction = 0

        elif (((setPoint == 35 or setPoint == 0) and blue) or (counter == 0 and (centr_x_p < 800 and centr_x_p > 0) and ((centr_y_g or centr_y_r) <= centr_y_p) and not blue and not orange) and not finish):
            
            if distance_r <= 30:
                correction = -20
                print(f"Avoiding pink wall {correction}")

            elif distance_l < 50 or distance_l > 100:
                if distance_l <= 35:
                    correction = 45
                    print(f"Avoiding pink wall {correction}")

                else:
                    correction = 10
                    print(f"Avoiding pink wall {correction}")
            else:
                correction = 0

        if not blue:
            if (setPoint <= -70 or setPoint == 0) and distance_l <= 22:
                print(f"Correcting Green Wall Orange")
                correction = 10
            else:
                pass

            if (setPoint >= 70 or setPoint == 0) and (distance_r <= 20 or distance_h <= 18):
                print(f"Correcting Red Wall...")
                correction = -10
            else:
                pass

        else:
            if (setPoint <= -70 or sertPoint == 0) and (distance_l <= 20 or distance_h <= 18):
                print(f"Correcting Green Wall Blue")
                correction = 10
            else:
                pass

            if (setPoint >= 70 or setPoint == 0) and distance_r <= 22:
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
def Live_Feed(color_b, stop_b, red_b, green_b, pink_b, centr_y, centr_x, centr_y_red, centr_x_red, centr_x_pink, centr_y_pink, centr_y_b, orange_o, centr_y_o):
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


    # cv2.namedWindow('Object Frame', cv2.WINDOW_NORMAL)
    # cv2.resizeWindow('Object Frame', 400, 200)
    time.sleep(2)
    pwm.write(green_led, 1)
    cap = cv2.VideoCapture(0)  # Open the default camera
    try:
        while True:
            
            all_flag = False
            both_flag = False
            pink_red = False
            pink_green = False
            only_red = False
            only_green = False
            only_pink = False

            red_b.value = False
            green_b.value = False
            pink_b.value = False

            centr_x_pink.value = 0
            centr_y_pink.value = 0
            centr_y.value = 0
            centr_x.value = 0

            # print(f"fps:{1/(time.time() - fps_time)}")
            fps_time = time.time()
            # prev_b.value = centroid_y_b
            ret, img = cap.read()

            hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # green
            hsv_img1 = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # red
            hsv_img2 = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # pink

            # predefined mask for green colour detection
             # For Green Color
            lower = np.array([100, 86, 16])  # green
            upper = np.array([107, 255, 184])
            mask = cv2.inRange(hsv_img, lower, upper)

            # For Red Color
            lower1 = np.array([0, 193, 35])  # red
            upper1 = np.array([96, 255, 255])
            r1lower = np.array([167, 193, 35])
            r1upper = np.array([179, 255, 255])
            mask1 = cv2.inRange(hsv_img1, lower1, upper1)
            mask_red = cv2.inRange(hsv_img1, r1lower, r1upper)
            mask1 = mask1 + mask_red

            # For Pink Color
            lower2 = np.array([126, 78, 47])  # pink
            upper2 = np.array([145, 196, 100])
            lower2_p = np.array([0, 78, 47])
            upper2_p = np.array([0, 196, 100])
            mask2 = cv2.inRange(hsv_img2, lower2, upper2)
            mask2_pink = cv2.inRange(hsv_img2, lower2_p, upper2_p)
            mask2 = mask2 + mask2_pink
            
            # For white color

            d_img = camera_utilities.clean_mask(mask, kernel)  # green
            d_img1 = camera_utilities.clean_mask(mask1, kernel)  # red
            d_img2 = camera_utilities.clean_mask(mask2, kernel)  # pink

            # find the histogram
            cont = camera_utilities.get_largest_contour(d_img)
            cont1 = camera_utilities.get_largest_contour(d_img1)
            cont2 = camera_utilities.get_largest_contour(d_img2)

            # ----------------------------------------------
            green_present, max_cnt = camera_utilities.is_color_present(
                cont, min_area=2000)

            # ---------------------------------------

            red_present, max_cnt1 = camera_utilities.is_color_present(
                cont1, min_area=2000)

            # ----------------------------------------------------
            pink_present, max_cnt2 = camera_utilities.is_color_present(
                cont2, min_area=2000)

            # --------------------------------------------------------

            color_combo = (red_present, green_present, pink_present)
            # print(f"Color Combo: {color_combo}")

            if color_combo == (False, False, False):
                pass
            elif color_combo == (True, True, True):
                all_flag = True
            elif color_combo == (True, True, False):
                both_flag = True
            elif color_combo == (True, False, True):
                pink_red = True
            elif color_combo == (False, True, True):
                pink_green = True
            elif color_combo == (True, False, False):
                only_red = True
            elif color_combo == (False, True, False):
                only_green = True
            elif color_combo == (False, False, True):
                only_pink = True

            if all_flag:
                # print("ITS THE FIRST LOOP")
                # FOR GREEN BOX
                if cv2.contourArea(max_cnt) > cv2.contourArea(max_cnt1):
                    if cv2.contourArea(max_cnt) > 1000 and cv2.contourArea(max_cnt) < 306000:
                        # Draw a rectange on the contour
                        centroid_x, centroid_y = camera_utilities.process_box(
                            max_cnt, img)
                        centr_y.value = centroid_y
                        centr_x.value = centroid_x
                        green_b.value = True

                # FOR RED BOX
                elif cv2.contourArea(max_cnt1) > cv2.contourArea(max_cnt):
                    if (cv2.contourArea(max_cnt1) > 1000 and cv2.contourArea(max_cnt1) < 306000):
                        # Draw a rectange on the contour

                        centroid_x_red, centroid_y_red = camera_utilities.process_box(
                            max_cnt1, img)

                        centr_y.value = centroid_y_red
                        centr_x.value = centroid_x_red

                        red_b.value = True

                # FOR PINK BOX
                # elif cv2.contourArea(max_cnt2) > cv2.contourArea(max_cnt) and cv2.contourArea(max_cnt2) > cv2.contourArea(max_cnt1):
                if (cv2.contourArea(max_cnt2) > 2000 and cv2.contourArea(max_cnt2) < 306000):
                    # Draw a rectange on the contour
                    centroid_x_pink, centroid_y_pink = camera_utilities.process_box(
                        max_cnt2, img)

                    centr_x_pink.value = centroid_x
                    centr_y_pink.value = centroid_y
                    pink_b.value = True

            # FOR RED BOX
            elif only_red:
                # print(cv2.contourArea(max_cnt1))
                if cv2.contourArea(max_cnt1) > 1000 and cv2.contourArea(max_cnt1) < 306000:
                    # Draw a rectange on the contour
                    centroid_x_red, centroid_y_red = camera_utilities.process_box(
                        max_cnt1, img)
                    centr_y.value = centroid_y_red
                    centr_x.value = centroid_x_red

                    red_b.value = True

            # FOR GREEN BOX
            elif only_green:
                # print(cv2.contourArea(max_cnt))
                if cv2.contourArea(max_cnt) > 1000 and cv2.contourArea(max_cnt) < 306000:
                    # Draw a rectange on the contour
                    centroid_x, centroid_y = camera_utilities.process_box(
                        max_cnt, img)

                    centr_y.value = centroid_y
                    centr_x.value = centroid_x

                    # if(centroid_y > 100):
                    # if(counter_green >= max_count):
                    green_b.value = True

            # FOR PINK BOX
            elif only_pink:
                if (cv2.contourArea(max_cnt2) > 2000 and cv2.contourArea(max_cnt2) < 306000):
                    # Draw a rectange on the contour
                    centroid_x, centroid_y = camera_utilities.process_box(
                        max_cnt2, img)
                    centr_x_pink.value = centroid_x
                    centr_y_pink.value = centroid_y
                    pink_b.value = True

            elif both_flag:
                # print("BOTH ARE PRESENT...")
                # FOR GREEN BOX
                if cv2.contourArea(max_cnt) > cv2.contourArea(max_cnt1):
                    if cv2.contourArea(max_cnt) > 1000 and cv2.contourArea(max_cnt) < 306000:
                        # Draw a rectange on the contour
                        centroid_x, centroid_y = camera_utilities.process_box(
                            max_cnt, img)

                        centr_y.value = centroid_y
                        centr_x.value = centroid_x
                        green_b.value = True

                # FOR RED BOX
                elif cv2.contourArea(max_cnt1) > cv2.contourArea(max_cnt):
                    if (cv2.contourArea(max_cnt1) > 1000 and cv2.contourArea(max_cnt1) < 306000):
                        # Draw a rectange on the contour
                        centroid_x_red, centroid_y_red = camera_utilities.process_box(
                            max_cnt1, img)
                        centr_y.value = centroid_y_red
                        centr_x.value = centroid_x_red
                        red_b.value = True

            elif pink_red:
                # FOR RED BOX
                # if cv2.contourArea(max_cnt1) > cv2.contourArea(max_cnt2):
                if (cv2.contourArea(max_cnt1) > 1000 and cv2.contourArea(max_cnt1) < 306000):
                    # Draw a rectange on the contour
                    centroid_x_red, centroid_y_red = camera_utilities.process_box(
                        max_cnt1, img)

                    centr_y.value = centroid_y_red
                    centr_x.value = centroid_x_red

                    red_b.value = True
                # elif cv2.contourArea(max_cnt2) > cv2.contourArea(max_cnt1):
                if (cv2.contourArea(max_cnt2) > 2000 and cv2.contourArea(max_cnt2) < 306000):
                    # Draw a rectange on the contour
                    centroid_x, centroid_y = camera_utilities.process_box(
                        max_cnt2, img)

                    centr_x_pink.value = centroid_x
                    centr_y_pink.value = centroid_y

                    # if centroid_y > 500:
                    pink_b.value = True

            elif pink_green:
                # if cv2.contourArea(max_cnt) > cv2.contourArea(max_cnt2):
                if (cv2.contourArea(max_cnt) > 1000 and cv2.contourArea(max_cnt) < 306000):
                    # Draw a rectange on the contour
                    centroid_x, centroid_y = camera_utilities.process_box(
                        max_cnt, img)
                    centr_y.value = centroid_y
                    centr_x.value = centroid_x
                    green_b.value = True  # pink_b.value = False
                # elif cv2.contourArea(max_cnt2) > cv2.contourArea(max_cnt):
                if (cv2.contourArea(max_cnt2) > 2000 and cv2.contourArea(max_cnt2) < 306000):
                    # Draw a rectange on the contour
                    centroid_x, centroid_y = camera_utilities.process_box(
                        max_cnt2, img)
                    centr_x_pink.value = centroid_x
                    centr_y_pink.value = centroid_y
                    pink_b.value = True

            print(f"green:{green_b.value}  red:{red_b.value}, pink:{pink_b.value}")
            #print(f"green centr :{centr_x.value}, red_centr:{centr_x.value}, pink_centr:{centr_x_pink.value}")
            #cv2.imshow('Object Frame', img)
            '''if cv2.waitKey(1) & 0xFF == ord('q'):
                stop_b.value = True
                break'''
        #cv2.destroyAllWindows()
        #picam2.stop()

    except KeyboardInterrupt:
       #picam2.stop()
        # print(f"Exception: {e}")
        pass
    finally:
        #picam2.stop()
        pass


def servoDrive(color_b, stop_b, red_b, green_b, pink_b, counts, centr_y, centr_x, centr_y_red, centr_x_red, centr_x_pink, centr_y_pink, head, centr_y_b,  orange_o, centr_y_o, sp_angle, turn_trigger, specific_angle, imu_shared, lidar_f):
    pwm = pigpio.pi()
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

    trigger = reset_f = False
    blue_flag = False
    orange_flag = True

    change_path = False
    timer_started = False

    g_flag = r_flag = p_flag = False
    g_past = r_past = p_past = False

    red_stored = green_store = False

    red_turn = green_turn = False
    calc_time = False
    lap_finish = continue_parking = parking_head = parking_flag = False
    turn_flag = reset_flags = counter_reset = False
    finished = finish = stop_flag = False
    red_time = green_time = False
    pink_detected = False
    last_red = False
    cw = ccw = False
    reset_heading = previous_heading_stored = False
    pink_timer = pink_r = False
    back_bot = False
    green_timer = red_timer = False
    g_last_flag = r_last_flag = False
    reverse_complete = reverse = reverse_trigger = False
    blue_on = False
    finish_flag = False
    reset_servo = False
    trigger_reset = False
    ############ VARIABLES ##################
    color_n = ""
    setPointL = -70
    setPointR = 70
    setPointC = 0
    power = 100
    prev_power = 0
    last_counter = 12
    change_counter = 7  # 3
    rev_counter = 7
    heading_angle = counter = turn_t = current_time = gp_time = rp_time = buff = c_time = green_count = red_count = 0

    i = l = lap_finish_time = prev_distance = turn_trigger_distance = target_count = offset = button_state = past_time = 0

    correctAngle(heading_angle, head.value)
    previous_heading = -1
    stop_time = turn_cos_theta = parking_done = pink_d = g_time = r_time = u = avg_right = avg_head = avg_left = 0

    time_p = prev_time = prev_restore = finish_timer = prev_blue = prev_orange = avg_blue = avg_orange = 0

    c = c_time = fps_time2 = 0

    color_s = ""
    orange_c.value = True
    debounce_delay = 0.2
    last_time = 0
    try:

        while True:
            imu_shared.value = head.value
            # print(f"red:{red_b.value} green:{green_b.value}")
            # print(f"angles:{specific_angle}")
            # print(f"fps 2222:{1/(time.time() - fps_time2)}")
            # print(f"stop: {stop_b.value}")
            fps_time2 = time.time()
            # print(f"blue:{blue_c.value} orange:{orange_c.value}")
            # print(f"c_time:{c}")
            # color_sensor = imu.get_color()
            # color_sensor="None"
            # print(color_sensor)

            tfmini.getTFminiData()
            tf_h = lidar_f.value
            tf_l = tfmini.distance_left
            tf_r = tfmini.distance_right
            if (time.time() - last_time > debounce_delay):
                previous_state = button_state
                button_state = pwm.read(5)
                # time.sleep(0.03)

                if previous_state == 1 and button_state == 0:
                    button = not (button)
                    last_time = time.time()
                    print(
                        f"🔘 Button toggled! Drive {'started' if button else 'stopped'}")
                    power = 100
            ##### STOP CONDITION ######
            # print(f"rgb:{imu.color_rgb} color:{color_sensor}")
            # print(f"pink detected:{pink_detected}")
            if counter == last_counter and not lap_finish:
                print(
                    f"centr_y :{centr_y.value} centr_y_red:{centr_y_red.value}")
                if not finished:
                    target_count = counts.value + 35000
                    finished = True
                if counts.value >= target_count and not reverse_trigger:
                    power = 0
                    # Set duty cycle to 50% (128/255)
                    pwm.set_PWM_dutycycle(pwm_pin, power)
                    time.sleep(3)
                    power = 100
                    prev_power = 0
                    lap_finish = True
                    reverse_trigger = True
                    print(
                        f"Vehicle is stopped,..reverse_trigger: {reverse_trigger}")

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

            if continue_parking:  # THIS SETPOINT IS WHEN THE ROBOT IS IN THE PARKING MODE
                green_b.value = False
                red_b.value = False
                g_past = False
                r_past = False
                g_flag = False
                r_flag = False
                if orange_flag and (((centr_x_pink.value < centr_x.value) and (centr_x.value > 0 and centr_x_pink.value > 0)) or (centr_x_pink.value < centr_x.value and (centr_x.value > 0 and centr_x_pink.value > 0))):
                    setPointR = -35
                    setPointC = -35
                    finish = True
                elif blue_flag and (((centr_x_pink.value > centr_x.value) and (centr_x.value > 0 and centr_x_pink.value > 0)) or (centr_x_pink.value > centr_x.value and (centr_x.value > 0 and centr_x_pink.value > 0))):
                    setPointL = 35
                    setPointC = 35
                    finish = True

            if pink_b.value:  # DECIDES SETPOINT WHENEVER PINK IS IN THE FRAME
                print(f"PINK IS DETECTED...")
                if orange_flag:
                    if (centr_x_pink.value < centr_x.value) and (centr_x_pink.value > 0 and centr_x.value > 0) and not continue_parking:
                        setPointL = -35
                        setPointR = 70
                        print(f"setPointL : {setPointL}")
                        # print(f"setPointL: {setPointL}")
                elif blue_flag:
                    if (centr_x.value < centr_x_pink.value) and (centr_x_pink.value > 0 and centr_x.value > 0) and not continue_parking:
                        setPointR = 35
                        setPointL = -70
                        print(f"setPointR: {setPointR}")

                elif counter % 4 == 0 and not blue_flag and not orange_flag:
                    if ((centr_x_pink.value < 800 and centr_x_pink.value > 0) and ((centr_y.value ) <= centr_y_pink.value)) and not continue_parking:
                        setPointR = 35
                        setPointL = -70
                        print(f"at 0 counter orange:{setPointR} {setPointL}")

                    if ((centr_x_pink.value > 800) and ((centr_y.value) <= centr_y_pink.value)) and not continue_parking:
                        setPointL = -35
                        setPointR = 70
                        print(f"at 0 counter blue:{setPointR} {setPointL}")


                if lap_finish and not continue_parking:
                    pink_detected = False
                    print("Starting Parking...")
                    continue_parking = True
                pb_time = time.time()
            # IF DOES NOT SEE PINK, KEEP THE SAME SETPOINT FOR 1 SECOND AND THEN CHANGE
            elif not pink_b.value and time.time() - pb_time > 1 and not lap_finish:
                # print(f"Resetting setPoints...{pink_detected}")
                if g_flag and not continue_parking:
                    print(f"away from green {g_past}")
                    setPointL = setPointL - 1
                    setPointR = 70
                elif r_flag and not continue_parking:
                    print(f"away from red {r_past}")
                    setPointR = setPointR + 1
                    setPointL = -70
                else:
                    setPointR = 70
                    setPointL = -70


                avg_right_pass = (tf_r * 0.1) + (avg_right * 0.9)
                avg_left_pass = (tf_l * 0.1) + (avg_left * 0.9)
                avg_head = (tf_h * 0.10) + (avg_head*0.90)
                avg_left = (tf_l * 0.15) + (avg_left*0.85)
                avg_right = (tf_r * 0.15) + (avg_right*0.85)

                # print(f"average ::: r_pass:{avg_right_pass} l_pass:{avg_left_pass} h:{avg_head} l:{avg_left} r:{avg_right}")

            ##########
            if button:  # THIS BLOCK OF CODE WHEN BUTTON IS PRESSED
                # time.sleep(0.01)
                


                if not reset_servo:
                    time.sleep(0.5)
                    servo.setAngle(130)
                    time.sleep(0.5)
                    servo.setAngle(90)
                    reset_servo = True

                if not reverse:
                    imu_head = head.value
                else:
                    # print(f"Changing imu..{imu_head} {cw} {ccw}")
                    imu_head = head.value - 180

                x, y = enc.get_position(imu_head, counts.value)

                total_power = (power * 0.1) + (prev_power * 0.9)
                prev_power = total_power
                # Set duty cycle to 50% (128/255)
                pwm.set_PWM_dutycycle(pwm_pin, 2.55 * total_power)

                pwm.write(direction_pin, 1)  # Set pin 20 high

                if stop_b.value:
                    power = 0
                    prev_power = 0

                ######     DIRECTION DECISION (CLOCKWISE OR ANTICLOCKWISE)     #####
                if blue_c.value:
                    color_s = "Blue"
                elif orange_c.value:
                    color_s = "Orange"
                elif white_c.value:
                    color_s = "White"
                # print(f"Color Sensor: {color_s}")
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
                    print(f"PARKING ------> distance_head : {tf_h}")
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
                        # Set duty cycle to 50% (128/255)
                        pwm.set_PWM_dutycycle(pwm_pin, power)
                        pwm.write(direction_pin, 0)  # Set pin 20 hig '''
                        prev_time = time.time()
                    reverse_complete = True
                    while time.time() - prev_time < 0.5:
                        print("Robot is stopped")
                        power = 0
                        prev_power = 0
                        # Set duty cycle to 50% (128/255)
                        pwm.set_PWM_dutycycle(pwm_pin, power)

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
                    stop_flag = False
                    if (abs(corr) < 15) and (tf_h <= 5 and tf_h >= 0) and not finish_flag:
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
                        if blue_flag:  # BLUE RESET BLOCK

                            print(f"BLUE RESET...{reverse_trigger}")

                            if (red_b.value or red_turn) or reverse_trigger:  # red after trigger
                                if counter != rev_counter:
                                    green_count = 0
                                    red_count = 1
                                    pwm.write(red_led, 1)
                                    pwm.write(green_led, 0)
                                x, y = enc.get_position(imu_head, counts.value)
                                print(
                                    f"Red Detected after trigger...green: {g_flag} {g_past} red:{r_flag} {r_past} {setPointR} {setPointL}")
                                red_turn = True
                                if pink_b.value and not red_b.value:
                                    red_turn = False
                                    red_time = False
                                    reverse_trigger = False
                                elif (tf_h < 30 and time.time() - g_time > 0.8) and not red_b.value and not pink_b.value:
                                    red_turn = False
                                    reverse_trigger = False
                                    red_time = True
                                correctPosition(setPointC, heading_angle, x, y, counter, blue_flag, orange_flag,
                                                reset_f, reverse, head.value, centr_x_pink.value, centr_y.value, centr_y_red.value, centr_y_pink.value, finish, tf_h, tf_l, tf_r)

                            else:
                                if red_time:
                                    time_g = 0.8

                                else:
                                    time_g = 0.5

                                if not timer_started:
                                    current_time = time.time()
                                    timer_started = True

                                if not green_b.value and not red_b.value:
                                    print('reversing diection red')
                                    while (time.time() - current_time < time_g):
                                        servo.setAngle(100)
                                        x, y = enc.get_position(
                                            imu_head, counts.value)
                                        power = 100
                                        prev_power = 95
                                        # Set duty cycle to 50% (128/255)
                                        pwm.set_PWM_dutycycle(pwm_pin, power)
                                        # Set pin 20 hig
                                        pwm.write(direction_pin, 0)
                                    print('reversing diection red complete')
                                    print('Stopping Motor...')
                                    turn_cos_theta = math.cos(
                                        math.radians(abs(corr)))
                                    timer_started = False

                                elif (green_b.value or green_turn) or g_past:
                                    green_turn = True
                                    if counter != rev_counter:
                                        green_count = 1
                                        red_count = 0
                                        pwm.write(red_led, 0)
                                        pwm.write(green_led, 1)
                                    while 1 and green_b.value:
                                        tfmini.getTFminiData()
                                        correctAngle(heading_angle, head.value)
                                        if abs(corr) < 15:
                                            turn_cos_theta = math.cos(
                                                math.radians(abs(corr)))
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
                                        elif (time.time() - current_time > 0.5) and not green_b.value:
                                            print(
                                                "Green is not there breaking the loop...")
                                            break
                                        x, y = enc.get_position(
                                            imu_head, counts.value)
                                        power = 100
                                        prev_power = 0
                                        # Set duty cycle to 50% (128/255)
                                        pwm.set_PWM_dutycycle(pwm_pin, power)
                                        # Set pin 20 hig
                                        pwm.write(direction_pin, 0)
                                    print('Green reversing diection complete')
                                    print('Stopping Motor...')
                                    buff = 4
                                    time_started = False
                                power = 0
                                prev_power = 0
                                # Set duty cycle to 50% (128/255)
                                pwm.set_PWM_dutycycle(pwm_pin, power)

                                print(f"head: {tf_h}")
                                print(f"before update: {x} {y}")
                                time.sleep(0.8)
                                counter = counter + 1
                                c_time = time.time()
                                lane_reset = counter % 4
                                print(f"in Lane {lane_reset}")
                                if lane_reset == 1:
                                    enc.x = (
                                        150 - abs(turn_trigger_distance * turn_cos_theta)) - 10
                                if lane_reset == 2:
                                    enc.y = (
                                        abs(turn_trigger_distance * turn_cos_theta) - 250) + 10
                                if lane_reset == 3:
                                    enc.x = (
                                        abs(turn_trigger_distance * turn_cos_theta) - 150) + 10
                                if lane_reset == 0:
                                    enc.y = (
                                        50 - abs(turn_trigger_distance * turn_cos_theta)) - 10
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

                        if orange_flag:  # ORANGE RESET BLOCK
                            print(
                                f"ORANGE RESET...reverse_trigger:{reverse_trigger}")

                            if (green_b.value or green_turn) or (reverse_trigger):  # green after trigger
                                if counter != rev_counter:
                                    green_count = 1
                                    red_count = 0
                                    pwm.write(red_led, 0)
                                    pwm.write(green_led, 1)
                                x, y = enc.get_position(imu_head, counts.value)

                                print(f"Green Detected after trigger... ")
                                green_turn = True
                                if pink_b.value and not green_b.value:
                                    green_turn = False
                                    green_time = False
                                    reverse_trigger = False
                                elif lidar_f.value < 30 and not green_b.value and not pink_b.value:
                                    green_turn = False
                                    reverse_trigger = False
                                    green_time = True
                                correctPosition(setPointC, heading_angle, x, y, counter, blue_flag, orange_flag,
                                                reset_f, reverse, head.value, centr_x_pink.value, centr_y.value, centr_y_red.value, centr_y_pink.value, finish, tf_h, tf_l, tf_r)
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
                                    while time.time() - current_time < time_g:
                                        servo.setAngle(70)
                                        x, y = enc.get_position(
                                            imu_head, counts.value)
                                        power = 100
                                        prev_power = 65
                                        # Set duty cycle to 50% (128/255)
                                        pwm.set_PWM_dutycycle(pwm_pin, power)
                                        # Set pin 20 hig
                                        pwm.write(direction_pin, 0)

                                    print('reversing diection green complete')
                                    print('Stopping Motor...')
                                    timer_started = False
                                elif (red_b.value or red_turn) or r_past:
                                    red_turn = True
                                    if counter != rev_counter:
                                        red_count = 1
                                        green_count = 0
                                        pwm.write(red_led, 1)
                                        pwm.write(green_led, 0)
                                    '''while 1:
                                        #tfmini.getTFminiData()
                                        print(f"correct red heading..")
                                        correctAngle(heading_angle, head.value)
                                        if abs(corr) < 15:
                                            print(
                                                f"turn_trigger: {turn_trigger_distance}")
                                            break'''

                                    if not timer_started:
                                        current_time = time.time()
                                        timer_started = True
                                    print(
                                        f'reversing diection red pink color: {pink_b.value} pink flag: {p_flag} {p_past}  red color:{red_b.value} red flag: {r_past} {r_flag}')
                                    while 1:
                                        print("RED IS SEEN..")
                                        servo.setAngle(80)
                                        print(f"centr y: {centr_y_red.value}")
                                        if (red_b.value and (centr_y_red.value < 475 and centr_y_red.value > 0)):
                                            print(f"Breaking the loop...")
                                            break
                                        elif (time.time() - current_time > 0.5) and not red_b.value:
                                            print("Green is not there breaking the loop...")
                                            break
                                        # getTFminiData()
                                        x, y = enc.get_position(imu_head, counts.value)
                                        # print(f"x: {x}, y: {y},  count:{counts.value} distance_head : {distance_head}")
                                        power = 100
                                        prev_power = 0
                                        # Set duty cycle to 50% (128/255)
                                        pwm.set_PWM_dutycycle(pwm_pin, power)
                                        # Set pin 20 hig
                                        pwm.write(direction_pin, 0)
                                    print('red reversing diection complete')
                                    buff = 4
                                    timer_started = False
                                print('Stopping Motor...')

                                power = 0
                                prev_power = 0
                                # Set duty cycle to 50% (128/255)
                                pwm.set_PWM_dutycycle(pwm_pin, power)

                                # getTFminiData()
                                x, y = enc.get_position(imu_head, counts.value)

                                time.sleep(0.8)
                                counter = counter + 1
                                c_time = time.time()
                                lane_reset = counter % 4
                                turn_trigger_distance = lidar_f.value
                                print(
                                    f"head: {turn_trigger_distance}, corr: {turn_cos_theta}")
                                if lane_reset == 1:
                                    enc.x = (
                                        150 - (turn_trigger_distance ))
                                    print(f"x: {enc.x}")
                                if lane_reset == 2:
                                    enc.y = (
                                        250 - (turn_trigger_distance )) 
                                if lane_reset == 3:
                                    enc.x = ((turn_trigger_distance) - 150) 
                                if lane_reset == 0:
                                    enc.y = ((turn_trigger_distance) - 50) 
                                print(f'Resuming Motor...{offset}')

                                power = 100
                                if reverse == True:
                                    offset = -180
                                    heading_angle = (
                                        (90 * counter) % 360) + offset
                                    if abs(heading_angle) >= 360:
                                        heading_angle = (heading_angle % 360)
                                else:
                                    heading_angle = ((90 * counter) % 360)
                                sp_angle.value = heading_angle
                                red_turn = False
                                green_time = False
                                reset_f = False
                                g_flag = False
                                g_past = False
                                # centr_y_b.value = 0

                    else:
                        # print(f"power :{power} prev_power{prev_power}")

                        # avg_blue = (prev_b.value*0.1 + avg_blue*0.9)
                        # avg_orange = (prev_b.value*0.1 + avg_blue*0.9)

                        # and (time.time() - turn_t) > (4 + buff)):
                        if (turn_trigger.value and not trigger) and (time.time() - turn_t) > (4 + buff):
                            buff = 0
                            trigger = True
                            reset_f = True
                            timer_started = False
                            turn_t = time.time()
                        elif not turn_trigger.value:
                            trigger = False
                            pwm.write(blue_led, 0)
                            
                            # pwm.write(green_led, 0)
                        '''if color_s == color_n and not trigger and (time.time() - turn_t) > (4 + buff):
							buff = 0
							timer_started = False
							trigger = True
							reset_f = True
							GPIO.output(blue_led, GPIO.HIGH)
							turn_t = time.time()

						elif color_s == 'White':
							trigger = False
							GPIO.output(blue_led, GPIO.LOW)'''

                        prev_blue = centr_y_b.value
                        prev_orange = centr_y_o.value

                        if last_red:

                            print(
                                f"tf mini back: {tfmini.distance_back} front:{tf_h} sum:{tf_h + tfmini.distance_back} corr:{abs(corr)}")

                        if g_last_flag:
                            if ((imu_head < 10 or imu_head > 350) and heading_angle == 0) and ((tfmini.distance_back*math.cos(math.radians(abs(imu_head))) > 150 and imu_head > 350) or (tf_h*math.cos(math.radians(abs(imu_head))) < 160 and imu_head < 10)) and last_red:
                                u = u + 1
                                if u > 3:
                                    print(
                                        f"Change path is true after 4.2 seconds")
                                    reset_flags = True
                                    change_path = True
                                    reverse = True
                        elif r_last_flag:
                            if ((imu_head < 10 or imu_head > 350) and heading_angle == 0) and ((tfmini.distance_back*math.cos(math.radians(abs(imu_head)))) > 150 and imu_head < 10) or (tf_h*math.cos(math.radians(abs(imu_head))) < 160 and imu_head > 350) and last_red:
                                u = u + 1
                                if u > 3:
                                    print(
                                        f"Change path is true after 4.2 seconds")
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

                        if green_b.value and not r_flag and not continue_parking :
                            print(f"centr x: {centr_x.value} centr y: {centr_y.value}")
                            g_flag = True
                            # if (centr_x.value > 1500 or  centr_y.value > 900):
                            g_past = True
                            pwm.write(red_led, 0)
                            pwm.write(green_led, 0)
                            print('1')

                        elif (g_past or time.time() - gp_time < 0.5) and not continue_parking:
                            print("Avoiding green...")
                            #g_flag = True
                            # and ((avg_left_pass < 60 or avg_left_pass > 120) or counter!=rev_counter):
                            if tf_r <= 50:
                                print("Green Avoid Complete")
                                g_past = False
                                g_flag = False
                                red_count = 0
                                green_count = 1
                                pwm.write(red_led, 0)
                                pwm.write(green_led, 1)
                                buff = 0
                                gp_time = time.time()
                                
                            g_flag = True
                            print('2')

                        elif red_b.value and not g_flag and not continue_parking:
                            r_flag = True
                            print(f"centr x red: {centr_x_red.value} centr y red: {centr_y_red.value}")
                            r_past = True
                            pwm.write(red_led, 0)
                            pwm.write(green_led, 0)
                            print('3')

                        elif (r_past or time.time() - rp_time < 0.5) and not continue_parking:
                            print("Avoiding red...")
                            #r_flag = True
                            # and ((avg_right_pass < 60 or avg_right_pass > 120) or counter!=rev_counter):
                            if tf_l <= 50:
                                print(f"red Avoid complete")
                                r_past = False
                                r_flag = False
                                red_stored = False
                                red_count = 1
                                green_count = 0
                                pwm.write(red_led, 1)
                                pwm.write(green_led, 0)
                                buff = 0
                                rp_time = time.time()
                            r_flag = True
                            print('4')

                        elif pink_b.value and continue_parking :
                            # if (centr_x_pink.value < 800 and orange_flag) or (centr_x_pink.value > 800 and blue_flag):
                            p_flag = True
                            p_past = True
                            print('5')

                        elif p_past and continue_parking and not parking_flag:

                            if orange_flag:
                                print(
                                    f"prev_distance: {prev_distance}, distance_left: {tf_l} diff: {abs(prev_distance - tf_l)}")
                                p_flag = True
                                if tf_l <= 30 and (abs(prev_distance - tf_l) >= 7 and prev_distance > 0) and p_past:
                                    p_past = False
                                    p_flag = False
                                    parking_flag = True
                                    print("Pink Avoidance Complete...")
                                prev_distance = tf_l

                            elif blue_flag:
                                print(
                                    f"prev_distance: {prev_distance}, distance_right: {tf_r}  diff: {abs(prev_distance - tf_r)}")
                                if tf_r <= 30 and (abs(prev_distance - tf_r) >= 7 and prev_distance > 0) and p_past:
                                    p_past = False
                                    p_flag = False
                                    parking_flag = True
                                    print("Pink Avoidance Complete Blue...")
                                prev_distance = tf_r

                            print('6')

                        else:
                            g_flag = False
                            r_flag = False
                            p_flag = False
                            r_past = False
                            g_past = False
                            p_past = False
                            print("No flags set, moving forward")
                            print('7')

                            pwm.write(red_led, 0)
                            pwm.write(green_led, 0)

                        if not change_path:
                            if g_flag or g_last_flag:
                                if last_red:
                                    g_last_flag = True
                                    r_last_flag = False
                                print("avoiding green..")
                                correctPosition(setPointL, heading_angle, x, y, counter, blue_flag, orange_flag,
                                                reset_f, reverse, head.value, centr_x_pink.value, centr_y.value, centr_y_red.value, centr_y_pink.value, finish, tf_h, tf_l, tf_r)
                            elif r_flag or r_last_flag:
                                if last_red:
                                    r_last_flag = True
                                    g_last_flag = False
                                print("avoiding red...")
                                correctPosition(setPointR, heading_angle, x, y, counter, blue_flag, orange_flag,
                                                reset_f, reverse, head.value, centr_x_pink.value, centr_y.value, centr_y_red.value, centr_y_pink.value, finish, tf_h, tf_l, tf_r)
                            elif p_flag:
                                print("avoiding pink..")
                                if orange_flag:
                                    correctPosition(setPointR, heading_angle, x, y, counter, blue_flag, orange_flag,
                                                    reset_f, reverse, head.value, centr_x_pink.value, centr_y.value, centr_y_red.value, centr_y_pink.value, finish, tf_h, tf_l, tf_r)
                                elif blue_flag:
                                    correctPosition(setPointL, heading_angle, x, y, counter, blue_flag, orange_flag,
                                                    reset_f, reverse, head.value, centr_x_pink.value, centr_y.value, centr_y_red.value, centr_y_pink.value, finish, tf_h, tf_l, tf_r)
                            else:
                                print("Going straight")
                                correctPosition(setPointC, heading_angle, x, y, counter, blue_flag, orange_flag,
                                                reset_f, reverse, head.value, centr_x_pink.value, centr_y.value, centr_y_red.value, centr_y_pink.value, finish, tf_h, tf_l, tf_r)

                            print(f"g_last:{g_last_flag} r_last:{r_last_flag}")
                        else:

                            print(f"Turning 180...{abs(corr)} {i}")
                            print(
                                f"heading_angle:{heading_angle} prev_heading {previous_heading}")
                            if abs(heading_angle) == 180 and abs(corr) < 15:
                                power = 0
                                # Set duty cycle to 50% (128/255)
                                pwm.set_PWM_dutycycle(pwm_pin, power)
                                time.sleep(0.5)
                                power = 100
                                prev_power = 0
                                heading_angle = -180
                                change_path = False
                                last_red = False
                                print("Change path is false")
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
                                print(
                                    f"heading_angle before change {heading_angle}")
                                previous_heading = heading_angle
                                heading_angle = heading_angle + offset
                                print(
                                    f"heading_angle after change {heading_angle}")
                                previous_heading_stored = True
                                i = i + 1
                            correctAngle(heading_angle, head.value)
                # print(f"green:{green_count} r_count:{red_count}")
                print(
                    f"trigger:{trigger} turn_trigger: {turn_trigger.value} reset_f:{reset_f} red:{r_flag} green:{g_flag} pink:{pink_b.value} counter: {counter}, imu:{head.value}")
                print(f"r_past:{r_past} g_past:{g_past} p_past:{p_past}")
                print(f"red_b.value:{red_b.value} green_b.value:{green_b.value} pink_b.value:{pink_b.value}")
                print(f"x: {x}, y:{y} count:{counts.value} heading_angle:{heading_angle}")
                print(f"head:{tf_h} left:{tf_l} right: {tf_r}")
                print(f"L: {setPointL} R: {setPointR} setPointC: {setPointC}")
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
            # print(f"button:{button}")

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
    finally:
        pwm.set_PWM_dutycycle(12, 0)  # Stop motor
        pwm.write(20, 0)              # Set direction pin low (optional)
        print("Motors stopped safely.")
        pwm.stop()
        # pwm.close()


def runEncoder(counts, head, imu_shared, sp_angle):
    pwm = pigpio.pi()
    print("Encoder Process Started")

    try:
        while True:

            line = ser.readline().decode().strip()
            esp_data = line.split(" ")
            esp_data.append(1)
            if len(esp_data) >= 2:
                try:
                    head.value = float(esp_data[0])
                    imu_shared.value = head.value
                    #sp_angle.value = 0
                    counts.value = int(esp_data[1])
                    pwm.write(red_led, 1)
                except ValueError:
                    print(f"⚠️ Malformed ESP data: {esp_data}")
            else:
                print(f"⚠️ Incomplete ESP data: {esp_data}")
    except Exception as e:
        print(f"Exception Encoder:{e}")
    finally:
        ser.close()


def read_lidar(lidar_angle, lidar_distance, previous_angle, imu_shared, sp_angle, turn_trigger, specific_angle, lidar_f):
    # print("This is first line")
    global CalledProcessError
    pwm = pigpio.pi()
    trig_time = 0
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

    pwm.write(green_led, 1)

    # try:
    for line in process.stdout:
        line = line.strip()
        # print(line)
        if "theta" in line and "Dist" in line:
            try:
                angle_part = line.split()
                # print(angle_part)

                angle_index = angle_part.index("theta:") + 1
                dist_index = angle_part.index("Dist:") + 1

                angle = float(angle_part[angle_index])
                distance = float(angle_part[dist_index])
                angle = int(angle)

                imu_r = int(imu_shared.value)
                sp_angle.value = 360 - sp_angle.value
                # print(f"📍 Angle: {angle:.2f}°, Distance: {distance:.2f} mm")
            except Exception as e:
                print("⚠️ Parse error:", e)
        else:
            print("ℹ️", line)

        if previous_angle.value != angle:

            while (angle - previous_angle.value > 1):
                lidar_angle.value = previous_angle.value + 1
                lidar_distance.value = previous_distance
                previous_angle.value = previous_angle.value + 1
                rplidar[int(lidar_angle.value)] = lidar_distance.value
                if (int(lidar_angle.value) == (0 + imu_r + sp_angle.value) % 360):
                    specific_angle[0] = lidar_distance.value
                    lidar_front = lidar_distance.value
                    lidar_f.value = lidar_front
                if (int(lidar_angle.value) == (90 + imu_r + sp_angle.value) % 360):
                    specific_angle[1] = lidar_distance.value
                    lidar_left = lidar_distance.value

                if (int(lidar_angle.value) == (270 + imu_r + sp_angle.value) % 360):
                    specific_angle[2] = lidar_distance.value
                    lidar_right = lidar_distance.value
                '''if (lidar_front < 650 and lidar_right > 1800 and lidar_left < 1000):
                    turn_trigger.value = True
                else:
                    turn_trigger.value = False'''

                '''if (lidar_front < 750 and lidar_right > 1800 and lidar_left < 1000) and not turn_trigger.value:
                    turn_trigger.value = True
                    trig_time = time.time()
                elif time.time() - trig_time > 4:
                    turn_trigger.value = False
                print(f"front: {lidar_front}. right:{lidar_right} left:{lidar_left} sp_angle:{sp_angle.value}, turn_trigger:{turn_trigger.value} diff: {time.time() - trig_time} ")'''

                #print(f"angles: {specific_angle} imu: {imu_shared.value} total:{imu_r + lidar_angle.value} sp_angle:{sp_angle.value}")

            if (distance != 0):
                with lidar_angle.get_lock(), lidar_distance.get_lock(), previous_angle.get_lock(), imu_shared.get_lock():
                    lidar_angle.value = angle
                    lidar_distance.value = distance
                    previous_distance = distance
                    previous_angle.value = angle
                    rplidar[int(lidar_angle.value)] = lidar_distance.value
                    if (int(lidar_angle.value) == (0 + imu_r + sp_angle.value) % 360):
                        specific_angle[0] = lidar_distance.value
                        lidar_front = lidar_distance.value
                        lidar_f.value = lidar_front

                    if (int(lidar_angle.value) == (90 + imu_r + sp_angle.value) % 360):
                        specific_angle[1] = lidar_distance.value
                        lidar_left = lidar_distance.value
                    if (int(lidar_angle.value) == (270 + imu_r + sp_angle.value) % 360):
                        specific_angle[2] = lidar_distance.value
                        lidar_right = lidar_distance.value
                    # print(f"angles: {specific_angle}, imu: {imu_shared.value} total:{imu_r + lidar_angle.value}")
            if (lidar_front < 650 and lidar_right > 1800 and lidar_left < 1000):
                turn_trigger.value = True
            else:
                turn_trigger.value = False


            #print(f"front: {lidar_front}. right:{lidar_right} left:{lidar_left}  turn_trigger:{turn_trigger.value} diff:{time.time() - trig_time}  imu:{imu_r} sp_angle: {sp_angle.value}")
            # print(f"angle: {lidar_angle.value} distance:{rplidar[int(lidar_angle.value)]}")


if __name__ == '__main__':
    try:
        print("Starting process")

        P = multiprocessing.Process(target=Live_Feed, args=(color_b, stop_b, red_b, green_b, pink_b, centr_y,
                                    centr_x, centr_y_red, centr_x_red, centr_x_pink, centr_y_pink, centr_y_b, orange_o, centr_y_o))
        S = multiprocessing.Process(target=servoDrive, args=(color_b, stop_b, red_b, green_b, pink_b, counts, centr_y, centr_x, centr_y_red,
                                    centr_x_red, centr_x_pink, centr_y_pink, head, centr_y_b, orange_o, centr_y_o,  sp_angle, turn_trigger, specific_angle, imu_shared, lidar_f))
        E = multiprocessing.Process(target=runEncoder, args=(counts, head, imu_shared, sp_angle))
        lidar_proc = multiprocessing.Process(target=read_lidar, args=(
            lidar_angle, lidar_distance, previous_angle, imu_shared, sp_angle, turn_trigger, specific_angle, lidar_f))

        # Launch the lidar reader process

        # C = multiprocessing.Process(target=color_SP, args=(blue_c, orange_c, white_c))
        print("Starting lidar process")
        lidar_proc.start()
        print("lidar process startes")
        print("Image Process Start")
        P.start()
        print("Image Process Started")
        print("Servo Process Start")
        S.start()
        print("Servo Process Started")
        print("Encoder Process Start")

        E.start()
        print("Encoder Process Started")

    except KeyboardInterrupt:
        ser.close()
        E.terminate()
        S.terminate()
        P.terminate()
        lidar_proc.terminate()
        E.join()
        S.join()
        P.join()
        lidar_proc.join()
        pwm.hardware_PWM(12, 100, 0)
        pwm.bb_serial_read_close(RX_Head)
        pwm.bb_serial_read_close(RX_Left)
        pwm.bb_serial_read_close(RX_Right)
        pwm.stop()
        imu.close()
        tfmini.close()
