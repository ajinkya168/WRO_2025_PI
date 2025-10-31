import datetime
from itertools import combinations
from pycoral.utils.dataset import read_label_file
from pycoral.adapters import common, detect
from pycoral.utils.edgetpu import make_interpreter
import numpy as np
import cv2
import subprocess
import sys
import logging
from ctypes import c_float
import multiprocessing
import pigpio
import math
from Encoder import EncoderCounter
from Servo import Servo
import serial
import RPi.GPIO as GPIO
from TFmini import TFmini
import traceback
import time
import os

os.system('sudo pkill pigpiod')
os.system('sudo pigpiod')
time.sleep(5)

timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
log_dir = '/home/pi/WRO_2025_PI/logs'

log_file = open(f"{log_dir}/log_obstacle_2.txt", 'w')
sys.stdout = log_file

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
reset_pin = 19

# INITIALIZATION
process = None
ser = serial.Serial('/dev/UART_USB', 115200)
print('created uart')

pwm = pigpio.pi()
if not pwm.connected:
    print('Could not connect to pigpio daemon')
    exit(1)
#### INITIALIZATION ####

# Set pin modes for LEDs and resets
for pin in [reset_pin, blue_led, red_led, green_led]:
    pwm.set_mode(pin, pigpio.OUTPUT)
    pwm.write(pin, 0)  # Set LOW
# Set button pin as input with pull-up
pwm.set_mode(button_pin, pigpio.INPUT)
pwm.set_pull_up_down(button_pin, pigpio.PUD_UP)

#### RESETTING ARDUINO ####

print('Resetting....')

pwm.write(reset_pin, 0)  # Pull reset LOW
pwm.write(green_led, 1)  # Turn on green LED
time.sleep(1)

pwm.write(reset_pin, 1)  # Release reset (HIGH)
pwm.write(green_led, 0)  # Turn off green LED
time.sleep(1)

print('Reset Complete')

########### IMPORTING CLASSES ###############
servo = Servo(servo_pin)
# imu = IMUandColorSensor(board.SCL, board.SDA)
tfmini_lock = multiprocessing.Lock()

tfmini = TFmini(RX_Head, RX_Left, RX_Right, RX_Back)
# app = Flask(__name__)

rplidar = [None] * 360
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
red_b = multiprocessing.Value('b', False)
green_b = multiprocessing.Value('b', False)
pink_b = multiprocessing.Value('b', False)
centr_y = multiprocessing.Value('f', 0.0)
centr_x = multiprocessing.Value('f', 0.0)
centr_y_red = multiprocessing.Value('f', 0.0)
centr_x_red = multiprocessing.Value('f', 0.0)
centr_x_pink = multiprocessing.Value('f', 0.0)
centr_y_pink = multiprocessing.Value('f', 0.0)
head = multiprocessing.Value('f', 0.0)
sp_angle = multiprocessing.Value('i', 0)
turn_trigger = multiprocessing.Value('b', False)
# Shared memory for LIDAR and IMU
lidar_angle = multiprocessing.Value('d', 0.0)
lidar_distance = multiprocessing.Value('d', 0.0)
lidar_f = multiprocessing.Value('d', 0.0)
lidar_l = multiprocessing.Value('d', 0.0)
lidar_r = multiprocessing.Value('d', 0.0)
previous_angle = multiprocessing.Value('d', 0.0)
left_f = multiprocessing.Value('b', False)
right_f = multiprocessing.Value('b', False)
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


def correctPosition( setPoint, head, x, y, counter, blue, orange, heading, centr_x_p, centr_x_r, centr_x_g, centr_y_g, centr_y_r, centr_y_p, distance_h, distance_l, distance_r, red, green): # print("INSIDE CORRECT")
    # getTFminiData()
    global prevError, totalError, prevErrorGyro, totalErrorGyro, corr_pos

    error = 0
    correction = 0
    pTerm_e = 0
    dTerm_e = 0
    iTerm_e = 0
    lane = counter % 4
    n_head = 0
    # if(time.time() - last_time > 0.001):
    if lane == 0:
        error = setPoint - y
        print( f"lane: {lane}, error: {error:.2f} target:{(setPoint)}, x:{x} y:{y} not reverse" )
    elif lane == 1:
        if orange:
            error = x - (100 - setPoint)
            print( f"lane:{lane}, error:{error:.2f} target:{(100 - setPoint)}, setPoint:{setPoint} x:{x}, y:{y}" )
        elif blue:
            error = (100 + setPoint) - x
            print( f"lane:{lane}, error:{error} target:{(100 + setPoint)}, x:{x} y:{y} Bluee" )
    # print(f" trigger : {flag_t} setPoint: {setPoint} lane: {lane} correction:{correction}, error:{error} x:{x}, y:{y}, prevError :{prevError} angle:{head - correction}")
    elif lane == 2:
        if orange:
            error = y - (200 - setPoint)  # CHANGE 1
            print( f"lane:{lane} error:{error:.2f} target:{(200 - setPoint)},  x: {x} y: {y} setPoint:{setPoint}" )
        elif blue:
            error = y - (-200 - setPoint)
            print( f"lane:{lane} error:{error} target:{(-200 - setPoint)}, x: {x} y{y}")
    # print(f"setPoint: {flag_t} lane: {lane} correction:{correction}, error:{error} x:{x}, y:{y}, prevError :{prevError} angle:{head - correction}")
    elif lane == 3:
        if orange:
            error = (setPoint - 100) - x
            print( f"lane:{lane} error:{error:.2f} target:{(setPoint - 100)}, x: {x} y {y} setPoint:{setPoint}" )
        elif blue:
            error = x + (100 + setPoint)
            (f"lane:{lane} error:{error} target:{(55 + setPoint)}, x:{x} y {y}")

    corr_pos = error
    pTerm_e = kp_e * error
    dTerm_e = kd_e * (error - prevError)
    totalError += error
    iTerm_e = ki_e * totalError
    correction = pTerm_e + iTerm_e + dTerm_e

    print(f"Error: {error}")
 
    if (setPoint == -15 and orange):
        if distance_l <= 30:
            correction = 20
            print(f"Avoiding pink wall {correction}")
        elif distance_r < 50:
            if distance_r <= 35:
                correction = -45
                print(f"Avoiding pink wall {correction}")
            else:
                correction = -10
                print(f"Avoiding pink wall else{correction}")
        else:
            print('setPoint was not -35')
            pass
    if (setPoint == 15 and blue):
        if distance_r <= 30:
            correction = -20
            print(f"Avoiding pink wall {correction}")
        elif distance_l < 50:
            if distance_l <= 35:
                correction = 45
                print(f"Avoiding pink wall {correction}")
            else:
                correction = 10
                print(f"Avoiding pink wall else {correction}")
        else:
            print('setPoint was not 35')
            pass


    n_head = normalize_angle(heading, blue, orange, lane)

    if setPoint <= -40 and distance_l <= 25:
        print(f"Correcting Green Wall Orange")
        correction = 15
    elif setPoint >= 40 and distance_r <= 25:
        print( f"Correcting Red Wall... diff:{(n_head - head):.2f} heading:{heading:.2f} n_head:{n_head:.2f} head:{head} right {distance_r} head_d:{tfmini.distance_head}" )
        correction = -15
    else:
        print('No wall detected...')
        pass

    correction = max(-45, min(45, correction))

    print( f"diff:{(heading - head):.2f} heading:{heading:.2f} head:{head:.2f} right {distance_r} head_d:{tfmini.distance_head} correction:{correction}" )

    prevError = error

    if setPoint == 15:
        correctAngle(head + correction, heading, 1) #0.85
    else:
        correctAngle(head + correction, heading, 1.5)
        


def correctWall(setPoint_distance, dist, sp_h, imu_h):
    error_d = 0
    prevError_d = 0
    totalError_d = 0
    correction_d = 0
    totalError_d = 0
    prevError_d = 0

    error_d = dist - setPoint_distance

    # print("Error : ", error_gyro)
    pTerm = 0
    dTerm = 0
    iTerm = 0

    pTerm = 2 * error_d
    dTerm = 0 * (error_d - prevError_d)
    totalError_d += error_d
    iTerm = 0 * totalError_d
    correction = pTerm + iTerm + dTerm


    correction = max(-35, min(35, correction))

    prevError_d = error_d
    correctAngle(sp_h + correction, imu_h, 1.5)


def correctAngle(setPoint_gyro, heading, multiplier):
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

    pTerm = kp * error_gyro * multiplier
    dTerm = kd * (error_gyro - prevErrorGyro)
    totalErrorGyro += error_gyro
    iTerm = ki * totalErrorGyro
    correction = pTerm + iTerm + dTerm

    if multiplier == 3:
        correction = max(-60, min(60, correction))
    else:
        correction = max(-30, min(30, correction))

    prevErrorGyro = error_gyro
    servo.setAngle(90 - correction)


def correctReverseAngle(setPoint_gyro, heading, multiplier):
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

    pTerm = kp * error_gyro * multiplier
    dTerm = kd * (error_gyro - prevErrorGyro)
    totalErrorGyro += error_gyro
    iTerm = ki * totalErrorGyro
    correction = pTerm + iTerm + dTerm

    if multiplier == 3:
        correction = max(-50, min(50, correction))
    else:
        correction = max(-30, min(30, correction))

    prevErrorGyro = error_gyro
    servo.setAngle(90 + correction)


def Live_Feed( red_b, green_b, pink_b, centr_y, centr_x, centr_y_red, centr_x_red, centr_x_pink, centr_y_pink ):
    MODEL_PATH = '/home/pi/WRO_2025_PI/limelight_neural_detector_8bit_edgetpu.tflite'  # put your label file here (id -> name), or set to None
    LABELS = '/home/pi/WRO_2025_PI/label_map.txt'
    CONF_TH = 0.7
    CAM_INDEX = 0
    # Load model
    interpreter = make_interpreter(MODEL_PATH)
    interpreter.allocate_tensors()
    ih, iw = common.input_size(interpreter)

    # Labels (optional)
    labels = {}
    if LABELS:
        try:
            labels = read_label_file(LABELS)  # {id: "name"}
        except Exception as e:
            print('Label load warn:', e)
    FPS = 120
    cap = cv2.VideoCapture(CAM_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)
    cap.set(cv2.CAP_PROP_FPS, FPS)
    # 0.25 means "manual mode" on many drivers
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
    cap.set(cv2.CAP_PROP_EXPOSURE, -6)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # low-latency
    cls_name = None
    t_prev = time.time()
    pairs = []
    dets = []
    x1 = x2 = y1 = y2 = cx = cy = 0
    try:
        while True:
            ok, frame_bgr = cap.read()
            if not ok:
                break
            H, W = frame_bgr.shape[:2]
            # Preprocess

            rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
            inp = cv2.resize(rgb, (iw, ih))
            common.set_input(interpreter, inp)
            interpreter.invoke()
            scale_x, scale_y = W / float(iw), H / float(ih)

            # Decode detections
            # get_objects returns list of Obj with bbox in input space (iw, ih)
            objs = detect.get_objects(interpreter, score_threshold=CONF_TH)

            det = []
            for obj in objs:
                bbox = obj.bbox
                x1 = int(bbox.xmin * scale_x)
                y1 = int(bbox.ymin * scale_y)
                x2 = int(bbox.xmax * scale_x)
                y2 = int(bbox.ymax * scale_y)

                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2
                """cx = (bbox.xmax + bbox.xmin) //2

                cy = (bbox.ymax + bbox.ymin) // 2"""
                area = max(0, (x2 - x1)) * max(0, (y2 - y1))
                name = labels.get(obj.id, str(obj.id))
                # if area >= 2500:
                det.append((name, cx, cy, area))
            det.sort(key=lambda d: d[3], reverse=True)

            pair = []
            if len(det) >= 2:
                # Normal case: take only the first detected pair
                pair = (det[0], det[1])
            elif len(det) == 1:
                # Only one detection → second is None
                pair = (det[0], None)
            else:
                # No detections at all
                pair = (None, None)
            # Extract just names for checks
            n1 = pair[0][0] if pair[0] else None
            n2 = pair[1][0] if pair[1] else None
            if n1 == 'pink' and n2 is None:
                pink_b.value = True
                green_b.value = False
                red_b.value = False
                if n1 == 'pink':
                    centr_x_pink.value = pair[0][1]
                    centr_y_pink.value = pair[0][2]
                    centr_x.value = 0
                    centr_y.value = 0
                    centr_x_red.value = 0
                    centr_y_red.value = 0
            elif (n1, n2) in (('pink', 'red'), ('red', 'pink')):
                red_b.value = True
                green_b.value = False
                pink_b.value = True
                centr_x.value = 0
                centr_y.value = 0
                if n1 == 'red':
                    centr_x_red.value = pair[0][1]
                    centr_y_red.value = pair[0][2]
                elif n2 == 'red':
                    centr_x_red.value = pair[1][1]
                    centr_y_red.value = pair[1][2]
                if n1 == 'pink':
                    centr_x_pink.value = pair[0][1]
                    centr_y_pink.value = pair[0][2]
                elif n2 == 'pink':
                    centr_x_pink.value = pair[1][1]
                    centr_y_pink.value = pair[1][2]
            elif (n1, n2) in (('pink', 'green'), ('green', 'pink')):
                green_b.value = True
                red_b.value = False
                pink_b.value = True
                centr_x_red.value = 0
                centr_y_red.value = 0
                if n1 == 'green':
                    centr_x.value = pair[0][1]
                    centr_y.value = pair[0][2]
                elif n2 == 'green':
                    centr_x.value = pair[1][1]
                    centr_y.value = pair[1][2]
                if n1 == 'pink':
                    centr_x_pink.value = pair[0][1]
                    centr_y_pink.value = pair[0][2]
                elif n2 == 'pink':
                    centr_x_pink.value = pair[1][1]
                    centr_y_pink.value = pair[1][2]
            elif (n1, n2) in (('green', 'red'), ('green', None), ('green', 'green')):
                green_b.value = True
                red_b.value = False
                pink_b.value = False
                centr_x_red.value = 0
                centr_y_red.value = 0
                centr_x_pink.value = 0
                centr_y_pink.value = 0
                if n1 == 'green':
                    centr_x.value = pair[0][1]
                    centr_y.value = pair[0][2]
            elif (n1, n2) in (('red', 'green'), ('red', None), ('red', 'red')):
                red_b.value = True
                green_b.value = False
                pink_b.value = False
                centr_x.value = 0
                centr_y.value = 0
                centr_x_pink.value = 0
                centr_y_pink.value = 0
                if n1 == 'red':
                    centr_x_red.value = pair[0][1]
                    centr_y_red.value = pair[0][2]
            elif n1 is None and n2 is None:
                red_b.value = False
                green_b.value = False
                pink_b.value = False
                centr_x_pink.value = 0
                centr_y_pink.value = 0
                centr_x.value = 0
                centr_y.value = 0
                centr_x_red.value = 0
                centr_y_red.value = 0
            now = time.time()
            fps = 1.0 / max(1e-3, (now - t_prev))
            t_prev = now
            """print(f"pairs:{pair} red_b.value: {red_b.value} green_b.value:{green_b.value} pink_b:{pink_b.value}  fps:{fps}")
            print(f"centr g:{centr_x.value} centr g y:{centr_y.value}")
            print(f"centr r x:{centr_x_red.value} centr r y:{centr_y_red.value}")
            print(f"centr x pink:{centr_x_pink.value} centr y pink:{centr_y_pink.value}")"""
            # cv2.imshow("Coral SSD Live", frame_bgr)
            if cv2.waitKey(1) & 0xFF == ord('q'):  # ESC
                break
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        cv2.destroyAllWindows()

def servoDrive(red_b, green_b, pink_b, counts, centr_y, centr_x, centr_y_red, centr_x_red, centr_x_pink, centr_y_pink, head, sp_angle, turn_trigger, lidar_f, lidar_l, lidar_r, left_f, right_f):
    pwm = pigpio.pi()
    global imu, corr, corr_pos
    pb_time = 0
    pwm_pin = 12
    direction_pin = 20
    p_pass = 0
    pwm.set_mode(pwm_pin, pigpio.OUTPUT)  # Set pin 12 as an output
    pwm.set_mode(direction_pin, pigpio.OUTPUT)  # Set pin 20 as an output
    pwm.hardware_PWM(pwm_pin, 55, 0)

    pwm.set_PWM_dutycycle(pwm_pin, 0)  # Set duty cycle to 50% (128/255)

    enc = EncoderCounter()

    ############# FLAGS ###############
    button = False
    trigger = reset_f = False
    blue_flag = False
    orange_flag = False
    timer_v = 0
    g_flag = r_flag = p_flag = False
    g_past = r_past = p_past = False
    calc_time = False
    lap_finish = continue_parking = parking_heading = parking_flag = False
    turn_flag = reset_flags = counter_reset = False
    finished = False
    red_time = green_time = False
    ############ VARIABLES ##################
    setPointL = -40
    setPointR = 40
    setPointC = 0
    power = 95
    prev_power = 0
    last_counter = 4 #12
    counter = turn_t = current_time = gp_time = rp_time = buff = c_time = 0
    heading_angle = 0
    lap_finish_time = ( prev_distance ) = turn_trigger_distance = target_count = offset = button_STATE = 0
    time_p = prev_time = 0
    fps_time2 = 0
    debounce_delay = 0.2
    last_time = 0
    avoided = False
    avoided_time = time.time()
    reverse_until = 0
    encoder_counter_store = False
    encoder_counts_value = 0
    off = 6
    rev_count = 0
    reverse_true = False
    parking_flag = False
    parking_heading_reverse = False
    parking_rev_count = 0
    trigger_enc_flag = False
    trigger_enc = 0
    inParkingatStart = False
    startPark = 0
    exitPark = False
    init = False
    timer = 0
    norm_head = 0
    lane_reset = 0
    red_time = 0
    green_time = 0
    pink_time = 0
    finish_t = time.time()
    inside_pink = time.time()
    STATE = 1
    parking_count = 500
    parking_count_current = 0
    full_park = False
    servo.setAngle(40)#45
    timer_t = time.time()
    STATE_INIT = 1
    enc_count = 0
    parking_STATE = 1
    OBSTACLE_STATE = 1
    RESET_STATE = 1
    pink_thresh = 0
    blue_lap = False
    avoid_thres = 2
    buff_time = time.time()
    try:
        while True:
            imu_head = head.value
            # print(f"fps 2222:{1/(time.time() - fps_time2)}")
            fps_time2 = time.time()

            tfmini.getTFminiData()
            tf_h = lidar_f.value
            l_left = lidar_l.value
            l_right = lidar_r.value
            tf_l = tfmini.distance_left
            tf_r = tfmini.distance_right
            x, y = enc.get_position(head.value, counts.value)

            if not init:
                if tf_h > 0 and head.value > 0:
                    correctAngle(heading_angle, head.value, 1.5)
                    init = True
                else:
                    servo.setAngle(45)
                    
            if green_b.value:
                pwm.write(red_led, 0)
                pwm.write(green_led, 1)
            elif red_b.value:
                pwm.write(red_led, 1)
                pwm.write(green_led, 0)
            elif pink_b.value:
                pwm.write(blue_led, 1)
            else:
                pwm.write(red_led, 0)
                pwm.write(green_led, 0)
                pwm.write(blue_led, 0)
                
            if not inParkingatStart and not left_f.value and not right_f.value:
                print('Starting Parking at Start...')
                if tf_l < 25 and tf_h < 250 and tf_l > 0 and tf_h > 0 and pink_b.value:
                    print('Right side parking')
                    enc.x = 0
                    enc.y = 55 - tfmini.distance_right #-40
                    right_f.value = True
                    inParkingatStart = True
                elif ( tf_r < 25 and tf_h < 250 and tf_h > 0 and tf_r > 0 and pink_b.value ):
                    enc.x = 0
                    enc.y = tfmini.distance_left - 55#40
                    print('Left side parking')
                    left_f.value = True
                    inParkingatStart = True
            if right_f.value and not orange_flag:
                orange_flag = True
                blue_flag = False
            elif left_f.value and not blue_flag:
                blue_flag = True
                orange_flag = False
            ##### STOP CONDITION ######
            if counter == last_counter and not lap_finish:
                print( f"centr_y :{centr_y.value} centr_y_red:{centr_y_red.value}")
                print('REACHED MAXIMUM COUNTS')
                print(f"target:{target_count}")
                if not finished:
                    if orange_flag:
                        target_count = counts.value + 30000 #22000 - finish too late in the section
                    elif blue_flag:
                        target_count = counts.value + 23000
                    finished = True
                    
                if counts.value >= target_count:
                    power = 0
                    pink_b.value = False
                    # Set duty cycle to 50% (128/255)
                    runMotor(power, 1)
                    time.sleep(3)
                    pink_b.value = False
                    power = 70
                    prev_power = 0
                    if blue_flag or orange_flag:
                        lap_finish = True

                    print(f"Vehicle is stopped...")
                    
            if lap_finish and not continue_parking:
                if not counter_reset:
                    counter = counter % last_counter
                    counter_reset = True
                if orange_flag:
                    print('Correcting wall pid orange')
                    if parking_STATE == 1:
                        correctReverseAngle(heading_angle + 90, head.value, 3)
                        while abs(corr) > 5:
                            x, y = enc.get_position(head.value, counts.value)
                            tfmini.getTFminiData()
                            power = 70
                            prev_power = 0
                            pwm.set_PWM_dutycycle(pwm_pin, int(1.2 * power))
                            print ("mid turn ", abs(corr))
                            # 0 = reverse, 1 = forward (per your wiring)
                            pwm.write(direction_pin, 0)
                            correctReverseAngle( heading_angle + 90, head.value, 3)
                        while tfmini.distance_head < 50:
                            x, y = enc.get_position(head.value, counts.value)
                            tfmini.getTFminiData()
                            power = 70
                            prev_power = 0
                            pwm.set_PWM_dutycycle(pwm_pin, int(1.2 * power))
                            # 0 = reverse, 1 = forward (per your wiring)
                            pwm.write(direction_pin, 0)
                            correctReverseAngle(heading_angle + 90, head.value, 3)
                            print("p state 1 ")
                        parking_STATE = 2
                    if parking_STATE == 2:
                        heading_angle = heading_angle + 180
                        correctAngle(heading_angle, head.value, 3)
                        print("corr is", abs(corr))
                        while abs(corr) > 5:
                            x, y = enc.get_position(head.value, counts.value)
                            tfmini.getTFminiData()
                            power = 70
                            prev_power = 0
                            pwm.set_PWM_dutycycle(pwm_pin, int(1.2 * power))
                            # 0 = reverse, 1 = forward (per your wiring)
                            pwm.write(direction_pin, 1)
                            correctAngle(heading_angle, head.value, 3)
                            print("P state 2")
                        p_flag = True
                        continue_parking = True
                        parking_STATE = 3
                        pink_time = time.time()

                elif blue_flag:
                    print('Correcting wall pid blue')
                    if parking_STATE == 1:
                        correctReverseAngle(heading_angle - 90, head.value, 3)
                        while abs(corr) > 5:
                            x, y = enc.get_position(head.value, counts.value)
                            tfmini.getTFminiData()
                            power = 70
                            prev_power = 0
                            pwm.set_PWM_dutycycle(pwm_pin, int(1.2 * power))
                            # 0 = reverse, 1 = forward (per your wiring)
                            pwm.write(direction_pin, 0)
                            correctReverseAngle( heading_angle - 90, head.value, 3)
                        while tfmini.distance_head < 50:
                            x, y = enc.get_position(head.value, counts.value)
                            tfmini.getTFminiData()
                            power = 70
                            prev_power = 0
                            pwm.set_PWM_dutycycle(pwm_pin, int(1.2 * power))
                            # 0 = reverse, 1 = forward (per your wiring)
                            pwm.write(direction_pin, 0)
                            correctReverseAngle( heading_angle - 90, head.value, 3)
                        parking_STATE = 2
                    if parking_STATE == 2:
                        correctAngle(heading_angle, head.value, 3)
                        while abs(corr) > 5:
                            x, y = enc.get_position(head.value, counts.value)
                            tfmini.getTFminiData()
                            power = 70
                            prev_power = 0
                            pwm.set_PWM_dutycycle(pwm_pin, int(1.2 * power))
                            # 0 = reverse, 1 = forward (per your wiring)
                            pwm.write(direction_pin, 1)
                            correctAngle(heading_angle, head.value, 3)
                        p_flag = True
                        continue_parking = True
                        parking_STATE = 3
                        pink_time = time.time()
                        
            ################# LANE 0 DECISIONS #####################

            if counter % 4 == 0:  # DECIDES SETPOINT WHENEVER PINK IS IN THE FRAME
                # print(f"PINK IS DETECTED...")
                if orange_flag:
                    if centr_x_red.value < 200 and centr_x_red.value > 0:
                        red_b.value = False
                elif blue_flag:
                    if centr_x_red.value > 200:
                        red_b.value = False
                if orange_flag:
                    setPointL = -15
                    setPointR = 40
                    print(f"setPointL : {setPointL}")
                elif blue_flag:
                    setPointR = 15
                    setPointL = -40
                    print(f"setPointR: {setPointR}")
            else:
                if g_flag and not continue_parking:
                    print(f"away from green {g_past}")
                    setPointL = setPointL - 1
                    if orange_flag:
                        if setPointL < -100:
                            setPointL = -100
                    elif blue_flag:
                        if setPointL < -40:
                            setPointL = -40
                    setPointR = 40
                elif r_flag and not continue_parking:
                    print(f"away from red {r_past}")
                    setPointR = setPointR + 1
                    if orange_flag:
                        if setPointR > 40:
                            setPointR = 40
                    elif blue_flag:
                        if setPointR > 100:
                            setPointR = 100
                    setPointL = -40
                    
            if not button:
                print( f"red_b:{red_b.value}, green_b:{green_b.value}, pink_b:{pink_b.value} tf_h:{tf_h:.2f} diff:{(head.value - heading_angle):.2f} counts:{counts.value:.2f}" )
                print( f"centr_X:{centr_x_pink.value:.2f}   centr_x red:{centr_x_red.value:.2f} centr x green:{centr_x.value:.2f}" )
                print(f"x:{x} y:{y}")
                print(f"tfmini head:{tfmini.distance_head} left:{tfmini.distance_left} right:{tfmini.distance_right}")
            if time.time() - last_time > debounce_delay:
                previous_STATE = button_STATE
                button_STATE = pwm.read(5)
                # time.sleep(0.03)

                if previous_STATE == 1 and button_STATE == 0:
                    button = not (button)
                    last_time = time.time()
                    print( f"🔘 Button toggled! Drive {'started' if button else 'stopped'}" )
                    power = 95
            ##################### BUTTON STARTS THE CODE ##################
            if button:  # THIS BLOCK OF CODE WHEN BUTTON IS PRESSED
                print('-------------------------------------------------')

                x, y = enc.get_position(imu_head, counts.value)
                if inParkingatStart:
                    t_time = time.time()
                    print('in parking start')
                    if STATE_INIT == 1:
                        print('STATE init 1')
                        if orange_flag:
                            correctAngle(heading_angle + 90, head.value, 3)
                        elif blue_flag:
                            correctAngle(heading_angle - 90, head.value, 3)
                        while abs(corr) > 5:
                            x, y = enc.get_position(head.value, counts.value)
                            tfmini.getTFminiData()
                            runMotor(70, 1)
                            if orange_flag:
                                correctAngle(heading_angle + 90, head.value, 3)
                            elif blue_flag:
                                correctAngle(heading_angle - 90, head.value, 3)
                        while tfmini.distance_head > 30:
                            x, y = enc.get_position(head.value, counts.value)
                            if orange_flag:
                                correctAngle(heading_angle + 90, head.value, 3)
                            elif blue_flag:
                                correctAngle(heading_angle - 90, head.value, 3)
                            tfmini.getTFminiData()
                            runMotor(70, 1)
                        STATE_INIT = 2
                    if STATE_INIT == 2:
                        print('STATE init 2')
                        if orange_flag:
                            correctReverseAngle(heading_angle, head.value, 3)
                        elif blue_flag:
                            correctReverseAngle(heading_angle, head.value, 3)
                        enc_count = counts.value
                        while abs(corr) > 5 or counts.value > enc_count - 5000: #enc_COUNT-6000
                            x, y = enc.get_position(head.value, counts.value)
                            runMotor(70, 0)
                            # 0 = reverse, 1 = forward (per your wiring)
                            if orange_flag:
                                correctReverseAngle( heading_angle, head.value, 3)
                            elif blue_flag:
                                correctReverseAngle( heading_angle, head.value, 3)
                        STATE_INIT = 3
                        OBSTACLE_STATE = 1
                        RESET_STATE = 0
                        inParkingatStart = False
                if red_b.value or green_b.value:
                    power = 55
                elif not g_flag and not r_flag:
                    power = 85
                else:
                    power = 80
                if time.time() < avoided_time:
                    pwm.set_PWM_dutycycle(pwm_pin, 0)
                    pwm.write(direction_pin, 0)
                    print('block spotted')
                    continue  # skip the drive code below
                if avoided_time > 0 and time.time() < reverse_until:
                    # non-blocking reverse
                    power = 70
                    prev_power = 0
                    pwm.set_PWM_dutycycle(pwm_pin, int(1.3 * power))
                    # 0 = reverse, 1 = forward (per your wiring)
                    pwm.write(direction_pin, 0)
                    # keep the robot straight while reversing (or add bias if you want to angle out)
                    print('reverse after block spotted')
                    if red_b.value and not reset_f:
                        print('Red Detected, setting servo to 60 degrees')
                        servo.setAngle(70)
                    elif green_b.value and not reset_f:
                        print('Green Detected, setting servo to 120 degrees')
                        servo.setAngle(110)
                    continue  # still skip forward-drive code
                ################        PARKING         ################

                if parking_flag:
                    print( f"PARKING -|-----> distance_head : {tfmini.distance_head}")
                    print('Inside Parking Loop')
                    tfmini.getTFminiData()
                    if not calc_time:
                        c_time = time.time()
                        calc_time = True
                    time_p = 0 #1.3
                    full_park = True
                    if STATE == 1:
                        if blue_flag or orange_flag:
                            while time.time() - c_time <= time_p:
                                tfmini.getTFminiData()
                                parking_count_current = counts.value

                                '''if orange_flag or blue_flag:
                                    print(f"prev_distance: {prev_distance}")
                                    full_park = True
                                    if prev_distance - tfmini.distance_right >= 10:
                                        print('Breaking reverse loop..')
                                        full_park = True
                                        break
                                    prev_distance = tfmini.distance_right'''
                                full_park = True
                                print( f"Reversing backward... {counts.value} right:{tfmini.distance_right:.2f} left:{tfmini.distance_left:.2f} diff right: {prev_distance - tfmini.distance_right:.2f} diff left:{prev_distance - tfmini.distance_left:.2f}" )
                                power = 70
                                prev_power = 0
                                correctReverseAngle(heading_angle, head.value, 1)
                                # Set duty cycle to 50% (128/255)
                                pwm.set_PWM_dutycycle(pwm_pin, power)
                                pwm.write(direction_pin, 0)  # Set pin 20 hig
                                prev_time = time.time()

                        if full_park:
                            parking_count_current = counts.value
                            tfmini.getTFminiData()
                            if orange_flag:
                                parking_count = 1 #300
                            while counts.value > parking_count_current - parking_count:
                                if blue_flag:
                                    print( f"Reversing backward full park...{counts.value} {parking_count_current - parking_count}" )
                                    power = 70#70
                                    prev_power = 0
                                    correctReverseAngle( heading_angle, head.value, 1)
                                    # Set duty cycle to 50% (128/255)
                                    tfmini.getTFminiData()
                                    pwm.set_PWM_dutycycle(pwm_pin, power)
                                    pwm.write(direction_pin, 0)# Set pin 20 hig #0
                                    prev_time = time.time()
                                if orange_flag:
                                    print( f"Reversing backward full park...{counts.value} {parking_count_current - parking_count}" )
                                    power = 70#70
                                    prev_power = 0
                                    correctReverseAngle((heading_angle), (head.value), 1)
                                    print("state 1 turn is happening")
                                    # Set duty cycle to 50% (128/255)
                                    tfmini.getTFminiData()
                                    pwm.set_PWM_dutycycle(pwm_pin, power)
                                    pwm.write(direction_pin, 0)# Set pin 20 hig #0
                                    prev_time = time.time()
                        else:
                            print('Going for a partial park')
                            pass
                        STATE = 2
                        print('STATE 1')
                        
                    if STATE == 2:
                        if blue_flag:
                            heading_angle = heading_angle - 90
                            correctReverseAngle(heading_angle, head.value, 3)
                            while (tfmini.distance_left > 50) or abs(corr) > 15:
                                tfmini.getTFminiData()
                                print( f"corr:{abs(corr)} head:{tfmini.distance_head} left:{tfmini.distance_left}" )
                                print('Reversing backward...')
                                power = 85
                                prev_power = 0
                                correctReverseAngle( heading_angle, head.value, 3)
                                # Set duty cycle to 50% (128/255)
                                pwm.set_PWM_dutycycle(pwm_pin, power)
                                pwm.write(direction_pin, 0)  # Set pin 20 hig
                                
                        if orange_flag:
                            heading_angle = heading_angle - 90
                            correctReverseAngle(heading_angle, head.value, 3)
                            while (tfmini.distance_right > 50) or abs(corr) > 15:
                                tfmini.getTFminiData()
                                print( f"corr:{abs(corr)} head:{tfmini.distance_head} left:{tfmini.distance_left}" )
                                print('Reversing backward...')
                                power = 85
                                prev_power = 0
                                correctReverseAngle( heading_angle, head.value, 3)
                                # Set duty cycle to 50% (128/255)
                                pwm.set_PWM_dutycycle(pwm_pin, power)
                                pwm.write(direction_pin, 0)  # Set pin 20 hig
                        STATE = 3
                    if STATE == 3:
                        if full_park:
                            print(f"Doing the full park..")
                            if blue_flag:
                                heading_angle = heading_angle + 90
                                correctReverseAngle( heading_angle, head.value, 3)
                                while (tfmini.distance_right > 20) or abs(corr) > 5:
                                    tfmini.getTFminiData()
                                    print( f"corr:{abs(corr)} head:{tfmini.distance_head} left:{tfmini.distance_right}" )
                                    print('Reversing backward...')
                                    power = 85
                                    prev_power = 0
                                    correctReverseAngle( heading_angle, head.value, 3)
                                    # Set duty cycle to 50% (128/255)
                                    pwm.set_PWM_dutycycle(pwm_pin, power)
                                    # Set pin 20 hig
                                    pwm.write(direction_pin, 0)
                                    print("state 3 turn is happening")
                               
                            if orange_flag:
                                heading_angle = heading_angle + 90
                                correctReverseAngle( heading_angle, head.value, 3)
                                while (tfmini.distance_right > 20) or abs(corr) > 5:
                                    tfmini.getTFminiData()
                                    print( f"corr:{abs(corr)} head:{tfmini.distance_head} left:{tfmini.distance_right}" )
                                    print('Reversing forward...')
                                    power = 85
                                    prev_power = 0
                                    correctReverseAngle( heading_angle, head.value, 3)
                                        # Set duty cycle to 50% (128/255)
                                    pwm.set_PWM_dutycycle(pwm_pin, power)
                                        # Set pin 20 high
                                    pwm.write(direction_pin, 0)
                            STATE = 4
                        else:
                            print('Partial Park done')
                            STATE = 4
                            pass
                    if STATE == 4:
                        power = 0
                        prev_power = 0
                        pwm.set_PWM_dutycycle(pwm_pin, power)
                        sys.exit(0)
                else:
                    if reset_f:
                        print('RESETTING FLAGS...')
                        g_past = False
                        r_past = False
                        g_flag = False
                        r_flag = False
                        rev_count = 0
                        if RESET_STATE == 1:
                            timer_t = time.time()
                            norm_head = normalize_angle( head.value, blue_flag, orange_flag, lane_reset )
                            # abs((norm_head - heading_angle) - 360) > 8 or tfmini.distance_head > 60:
                            if blue_flag:
                                while ( abs((norm_head - heading_angle) - 360) > 8 or tfmini.distance_head > 60 ) and time.time() - timer_t < 1.5:
                                    norm_head = normalize_angle( head.value, blue_flag, orange_flag, lane_reset )
                                    print( f"correcting heading  blue time:{time.time() - timer_t} {x:.2f} {y:.2f} {head.value - heading_angle:.2f} {head.value} {tfmini.distance_head:.2f} distance_right:{tfmini.distance_right:.2f} distance_head:{tfmini.distance_head:.2f}" )
                                    x, y = enc.get_position( head.value, counts.value)
                                    tfmini.getTFminiData()
                                    correctAngle( heading_angle, head.value, 1.5)
                                    power = 60
                                    prev_power = 0
                                    runMotor(power, 1)
                            elif orange_flag:
                                while ( abs(norm_head - heading_angle) > 8 or tfmini.distance_head > 60 ) and (time.time() - timer_t < 1.5):
                                    norm_head = normalize_angle( head.value, blue_flag, orange_flag, lane_reset )
                                    print( f"correcting orange heading counter:{counter} imu:{head.value:.2f} diff:{abs(norm_head - heading_angle):.2f} tfmini head: {tfmini.distance_head:.2f} norm_head:{norm_head}" )
                                    x, y = enc.get_position( head.value, counts.value)
                                    tfmini.getTFminiData()
                                    correctAngle( heading_angle, head.value, 1.5)
                                    power = 70
                                    prev_power = 0
                                    runMotor(power, 1)
                            RESET_STATE = 2
                        if RESET_STATE == 2:
                            tfmini.getTFminiData()
                            if blue_flag:
                                thresh_distance = tfmini.distance_right
                            elif orange_flag:
                                thresh_distance = tfmini.distance_left
                            if thresh_distance > 30:
                                timer_t = time.time()
                                if blue_flag:
                                    correctAngle(heading_angle + 20, head.value, 1.5)
                                    while ( tfmini.distance_head > 20 or abs( corr) > 5 ) and time.time() - timer_t < 1.5:
                                        tfmini.getTFminiData()
                                        print( f"moving ahead blue to correct heading timer {time.time() - timer_t} x:{x:.2f} y:{y:.2f} lidar_f:{lidar_f.value:.2f} distance_right:{tfmini.distance_right:.2f} distance_head:{tfmini.distance_head:.2f} corr:{abs(corr)}" )
                                        correctAngle( heading_angle + 20, head.value, 1.5 )
                                        power = 90
                                        prev_power = 0
                                        runMotor(power, 1)
                                        x, y = enc.get_position( head.value, counts.value )

                                elif orange_flag:
                                    correctAngle(heading_angle - 20, head.value, 1.5)
                                    while (tfmini.distance_head > 20 or abs(corr)> 5) and (time.time() - timer_t < 1.5):
                                        tfmini.getTFminiData()
                                        print(f"moving ahead to correct heading x:{x:.2f} y:{y:.2f} lidar_f:{lidar_f.value:.2f} counter:{counter} imu:{head.value} {tfmini.distance_head:.2f} corr:{abs(corr)}")
                                        correctAngle(heading_angle - 20, head.value,1.5)
                                        power = 90
                                        prev_power = 0
                                        runMotor(power, 1)
                                        x, y = enc.get_position(head.value, counts.value)
                                if lap_finish:
                                    RESET_STATE = 4
                                else:
                                    RESET_STATE = 3
                            elif thresh_distance < 30:
                                timer_t = time.time()
                                if blue_flag:
                                    correctAngle(heading_angle - 20, head.value, 1.5)
                                    while ( tfmini.distance_head > 20 or abs( corr) > 5 ) and time.time() - timer_t < 1.5:
                                        tfmini.getTFminiData()
                                        print( f"moving ahead blue to correct heading timer {time.time() - timer_t} x:{x:.2f} y:{y:.2f} lidar_f:{lidar_f.value:.2f} distance_right:{tfmini.distance_right:.2f} distance_head:{tfmini.distance_head:.2f} corr:{abs(corr)}" )
                                        correctAngle( heading_angle - 20, head.value, 1.5 )
                                        power = 90
                                        prev_power = 0
                                        runMotor(power, 1)
                                        x, y = enc.get_position( head.value, counts.value )
                                elif orange_flag:
                                    correctAngle(heading_angle + 20, head.value, 1.5)
                                    while (tfmini.distance_head > 20 or abs(corr) > 5) and (time.time() - timer_t < 1.5):
                                        tfmini.getTFminiData()
                                        print(f"moving ahead to correct heading x:{x:.2f} y:{y:.2f} lidar_f:{lidar_f.value:.2f} counter:{counter} imu:{head.value} {tfmini.distance_head:.2f} corr:{abs(corr)}")
                                        correctAngle(heading_angle + 20, head.value, 1.5)
                                        power = 90
                                        prev_power = 0
                                        runMotor(power, 1)
                                        x, y = enc.get_position(head.value, counts.value)
                                if lap_finish:
                                    RESET_STATE = 4
                                else:
                                    RESET_STATE = 3
                        if RESET_STATE == 3:
                            x, y = enc.get_position(imu_head, counts.value)
                            buff = 4
                            counter = counter + 1
                            lane_reset = counter % 4
                            heading_angle = update_heading( counter, heading_angle, blue_flag, orange_flag)
                            sp_angle.value = heading_angle
                            timer_v = time.time()
                            norm_head = normalize_angle( head.value, blue_flag, orange_flag, lane_reset )
                            if blue_flag:
                                while ( abs((norm_head - heading_angle) - 360) > 8 and time.time() - timer_v < 2.5 ):
                                    norm_head = normalize_angle( head.value, blue_flag, orange_flag, lane_reset )
                                    print( f"reversing servo {head.value:.2f} {heading_angle} diff: {(heading_angle - head.value) + 360:.2f} rev_diff: {(head.value - heading_angle) - 360:.2f} {counts.value} x:{x:.2f} y:{y:.2f}  head_r :{tfmini.distance_right:.2f} head_d:{tfmini.distance_head:.2f}" )
                                    x, y = enc.get_position( head.value, counts.value)
                                    tfmini.getTFminiData()
                                    correctReverseAngle( heading_angle, head.value, 1)
                                    power = 90
                                    prev_power = 0
                                    runMotor(power, 0)
                            elif orange_flag:
                                while ( abs(norm_head - heading_angle) > 8 and time.time() - timer_v < 2.5):
                                    norm_head = normalize_angle( head.value, blue_flag, orange_flag, lane_reset )
                                    print( f"reversing servo counter:{counter} imu:{head.value:.2f} diff:{abs(norm_head - heading_angle):.2f} tfmini head: {tfmini.distance_head:.2f} tfmini left:{tfmini.distance_left}" )
                                    x, y = enc.get_position( head.value, counts.value)
                                    tfmini.getTFminiData()
                                    correctReverseAngle( heading_angle, head.value, 1)
                                    power = 90
                                    prev_power = 0
                                    runMotor(power, 0)
                            power = 0
                            prev_power = 0
                            runMotor(power, 0)
                            c_time = time.time()
                            time.sleep(0.5)
                            tfmini.getTFminiData()
                            if orange_flag:
                                turn_trigger_distance = tfmini.distance_left
                            elif blue_flag:
                                turn_trigger_distance = tfmini.distance_right
                            enc.x, enc.y = reset_coordinates( turn_trigger_distance, lane_reset, orange_flag, blue_flag, x, y )
                            print(f"enc.x: {enc.x:.2f} enc.y:{enc.y:.2f}")
                            power = 90
                            trigger_enc = counts.value
                            print('Encoder counts are stored for trigger')
                            reset_f = False
                            green_b.value = False
                            red_b.value = False
                            pink_b.value = False
                            RESET_STATE = 0
                            OBSTACLE_STATE = 1

                        if RESET_STATE == 4:
                            norm_head = normalize_angle(head.value, blue_flag, orange_flag, lane_reset )
                            if blue_flag:
                                heading_angle = heading_angle + 90
                                while abs((norm_head - heading_angle) - 360) > 8:
                                    norm_head = normalize_angle( head.value, blue_flag, orange_flag, lane_reset )
                                    print( f"reversing servo {head.value:.2f} {heading_angle} diff: {(heading_angle - head.value) + 360:.2f} rev_diff: {(head.value - heading_angle) - 360:.2f} {counts.value} x:{x:.2f} y:{y:.2f} lidar_f:{lidar_f.value:.2f}" )
                                    x, y = enc.get_position( head.value, counts.value)
                                    tfmini.getTFminiData()
                                    correctReverseAngle( heading_angle, head.value, 1)
                                    power = 90
                                    prev_power = 0
                                    runMotor(power, 0)
                                while tfmini.distance_head > 55:
                                    tfmini.getTFminiData()
                                    print( f"moving blue right ahead to correct heading x:{x} y:{y} lidar_f:{lidar_f.value:.2f} distance_right:{tfmini.distance_right:.2f} distance_head:{tfmini.distance_head:.2f}" )
                                    correctAngle( heading_angle, head.value, 1.5)
                                    power = 40
                                    prev_power = 0
                                    runMotor(power, 1)
                                    x, y = enc.get_position( head.value, counts.value)
                                heading_angle = heading_angle + 90
                            elif orange_flag:
                                heading_angle = heading_angle + 90
                                while abs((norm_head - heading_angle)) > 8:
                                    norm_head = normalize_angle(head.value, blue_flag, orange_flag, lane_reset)
                                    print(f"reversing servo {head.value:.2f} {heading_angle} diff: {(heading_angle - head.value) + 360:.2f} rev_diff: {(head.value - heading_angle) - 360:.2f} {counts.value} x:{x:.2f} y:{y:.2f} lidar_f:{lidar_f.value:.2f}")
                                    x, y = enc.get_position(head.value, counts.value)
                                    tfmini.getTFminiData()
                                    correctReverseAngle(heading_angle, head.value, 1)
                                    power = 90
                                    prev_power = 0
                                    runMotor(90, 0)
                                while tfmini.distance_head > 55:
                                    tfmini.getTFminiData()
                                    print(f"moving blue right ahead to correct heading x:{x} y:{y} lidar_f:{lidar_f.value} distance_right:{tfmini.distance_right:.2f} distance_head:{tfmini.distance_head:.2f}")
                                    correctAngle(heading_angle, head.value,1.5)
                                    power = 40
                                    prev_power = 0
                                    runMotor(power, 1)
                                    x, y = enc.get_position(head.value, counts.value)
                                heading_angle = heading_angle + 90
                            p_flag = True
                            p_past = True
                            continue_parking = True
                            reset_f = False
                            trigger = False
                            runMotor(0, 1)
                            power = 70
                            prev_power = 0
                            RESET_STATE = 0
                            OBSTACLE_STATE = 1
                            print('Calculating pink time...')
                            pink_time = time.time()
                            print(f"pink time {pink_time} ")

                    else:
                        # TRIGGGER CHECK VALUESSSS
                        if blue_flag and lap_finish:
                            blue_lap = True
                            
                        if ( (turn_trigger.value and not trigger) and not trigger_enc_flag and not blue_lap) and not continue_parking:
                            buff = 0
                            print('Trigger Detected...')
                            trigger = True
                            reset_f = True
                            RESET_STATE = 1
                            avoided_time = time.time() + 0.3
                            turn_t = time.time()
                        elif counts.value > trigger_enc + 24000 and not continue_parking: #22000
                            print( f"Trigger enc flag is set: {trigger_enc_flag} counts:{counts.value} trigger_enc:{trigger_enc + 22000}" )
                            trigger = False
                            trigger_enc_flag = False
                            print('Encoder counts done for trigger')
                        ################## PANDAV 3.0 ###################
      
                        if lap_finish:
                            if p_flag and continue_parking and not parking_flag:
                                power = 60
                                print('avoiding pink..')
                                if blue_flag:
                                    correctAngle( heading_angle, head.value, 1.5)
                                elif orange_flag:
                                    correctAngle(heading_angle,head.value,1.5)
                                    '''if time.time() - pink_time < 4: #4
                                        print('avoiding wall in p_flag')
                                        correctAngle( heading_angle, head.value, 1.5)
                                        #correctWall( 30, tfmini.distance_right, heading_angle, head.value )
                                    else:
                                        print('now correcting Angle')
                                        correctAngle( heading_angle, head.value, 1.5)
                                    print( f"time after reversing heading {time.time() - pink_time} distance_right:{tf_r} distance_left:{tf_l} prev_distance:{prev_distance}" )'''
                                if orange_flag:
                                    pink_thresh = 1.5 # 4
                                elif blue_flag:
                                    pink_thresh = 1.5
                                if time.time() - pink_time > pink_thresh:
                                    tfmini.getTFminiData()
                                    if orange_flag or blue_flag:
                                        print( f"prev_distance: {prev_distance}, distance_right: {tfmini.distance_right} diff: {prev_distance - tfmini.distance_right} diff_flag:{(prev_distance - tfmini.distance_right) >= 10}" )
                                        p_flag = True
                                        # if tfmini.distance_right <= 35 and ((prev_distance - tfmini.distance_right) >= 10 and prev_distance > 0):
                                        #if ( tfmini.distance_left > 150 and tfmini.distance_left < 500 ):
                                        if tfmini.distance_head < 90: 
                                            p_pass = 2
                                            if p_pass == 2:
                                                p_past = False
                                                parking_flag = True
                                                print( 'Pink Avoidance Complete...')
                                        prev_distance = tfmini.distance_right
                        else:
                            if OBSTACLE_STATE == 1:
                                if green_b.value and not g_flag and not r_flag and centr_y.value > 250 :
                                    if centr_x.value < 320 and centr_x.value > 0:
                                        avoided_time = time.time() + 0.3
                                        reverse_until = avoided_time + 0.7       
                                    g_flag = True
                                    OBSTACLE_STATE = 2
                                    print('Obstacle STATE changed to 2')
                                elif red_b.value and not g_flag and not r_flag and centr_y_red.value > 250 :
                                    if centr_x_red.value > 300:
                                        avoided_time = time.time() + 0.3
                                        reverse_until = avoided_time + 0.7
                                    r_flag = True

                                    OBSTACLE_STATE = 2
                                    print('Obstacle STATE changed to 2')
                                else:
                                    correctPosition( setPointC, heading_angle, x, y, counter, blue_flag, orange_flag, head.value, centr_x_pink.value, centr_x_red.value, centr_x.value, centr_y.value, centr_y_red.value, centr_y_pink.value, tf_h, tf_l, tf_r, red_b.value, green_b.value )
                                    p_flag = False
                                    encoder_counter_store = False
                                    g_flag = False
                                    r_flag = False
                                    rev_count = 0
                                    print('Obstacle STATE changed to 0')
                                    print('No flags set, moving forward')
                            if OBSTACLE_STATE == 2:
                                if g_flag:
                                    print(f'avoiding green..green avoid: {time.time() - green_time}')
                                    correctPosition(setPointL, heading_angle, x, y, counter, blue_flag, orange_flag, head.value, centr_x_pink.value, centr_x_red.value, centr_x.value, centr_y.value, centr_y_red.value, centr_y_pink.value, tf_h, tf_l, tf_r, red_b.value, green_b.value )
                                    if green_b.value:
                                        green_time = time.time()
                                    if ( tf_r <= 40 and tf_r > 0) or (time.time() - green_time > avoid_thres and not green_b.value): 
                                        g_flag = False
                                        OBSTACLE_STATE = 3
                                        buff_time = time.time()
                                        print('Obstacle STATE changed to 1')
                                elif r_flag:
                                    print(f'avoiding red... time avoid : {time.time() - red_time}')
                                    correctPosition(setPointR, heading_angle, x, y, counter, blue_flag, orange_flag, head.value, centr_x_pink.value, centr_x_red.value, centr_x.value, centr_y.value, centr_y_red.value, centr_y_pink.value, tf_h, tf_l, tf_r, red_b.value, green_b.value )
                                    if red_b.value:
                                        red_time = time.time() 
                                    if ( tf_l <= 40 and tf_l > 0 ) or (time.time() - red_time > avoid_thres and not red_b.value): 
                                        r_flag = False
                                        OBSTACLE_STATE = 3
                                        buff_time = time.time()
                                        print('Obstacle STATE changed to 1')
                            if OBSTACLE_STATE == 3:
                                r_flag = False
                                g_flag = False
                                correctPosition(setPointC, heading_angle, x, y, counter, blue_flag, orange_flag, head.value, centr_x_pink.value, centr_x_red.value, centr_x.value, centr_y.value, centr_y_red.value, centr_y_pink.value, tf_h, tf_l, tf_r, red_b.value, green_b.value )
                                if time.time() - buff_time > 1.2:
                                    OBSTACLE_STATE = 1

                                
                total_power = (power * 0.1) + (prev_power * 0.9)
                prev_power = total_power
                runMotor(total_power, 1)

                print( f"centr_x.value: {centr_x.value} centr_y.value: {centr_y.value} centr_red: {centr_x_red.value} centr_y_red:{centr_y_red.value} centr_pink: {centr_x_pink.value}" )
                print( f"left_b.value:{left_f.value} right_b.value:{right_f.value} orange_flag:{orange_flag} blue_flag:{blue_flag}" )
                print( f"trigger:{trigger} turn_trigger: {turn_trigger.value} reset_f:{reset_f} counter: {counter}, imu:{head.value:2f}" )
                print( f"red_b.value:{red_b.value} green_b.value:{green_b.value} pink_b.value:{pink_b.value}" )
                print( f"r_flag:{r_flag} g_flag:{g_flag} rev_count: {rev_count}")
                print( f"r_past:{r_past} g_past:{g_past} p_past:{p_past} pass_c:{p_pass}" )
                print( f"x: {x:.2f}, y:{y:.2f} count:{counts.value} heading_angle:{heading_angle}" )
                print( f"F: {tf_h:.2f}  L: {l_left:.2f} R: {l_right:.2f} left:{tf_l} " )
                print( f"OBSTACLE_STATE: {OBSTACLE_STATE} RESET_STATE: {RESET_STATE}" )
                print('---------------------------------------------------')
                # print(f"color_s:{color_s} color_n:{color_n} centr_y_b.value: {centr_y_b.value} centr_x:{centr_x.value} centr_red: {centr_x_red.value} centr_pink:{centr_x_pink.value} setPointL:{setPointL} setPointR:{setPointR} g_count:{green_count} r_count:{red_count} x: {x}, y: {y} counts: {counts.value}, prev_distance: {prev_distance}, head_d: {tfmini.distance_head} right_d: {tfmini.distance_right}, left_d: {tfmini.distance_left}, back_d:{tfmini.distance_back} imu: {imu_head}, heading: {heading_angle}, cp: {continue_parking}, counter: {counter}, pink_b: {pink_b.value} p_flag = {p_flag}, g_flag: {g_flag} r_flag: {r_flag} p_past: {p_past}, g_past: {g_past}, r_past: {r_past} , red_stored:{red_stored} green_stored:{green_stored}")
            else:
                power = 0
                pwm.hardware_PWM(12, 55, 0)
                counter = 0
    except Exception as e:
        print(f"Exception: {e}")
        tb = traceback.extract_tb(e.__traceback__)
        filename, lineno, func, text = tb[-1]
        print(f"⚠️ Exception in {filename}, line {lineno}, in {func}")
        if isinstance(e, KeyboardInterrupt):
            power = 0
            pwm.hardware_PWM(12, 55, 0)
            heading_angle = 0
            counter = 0
            correctAngle(heading_angle, head.value, 1.5)
            red_b.value = False
            green_b.value = False
    finally:
        pwm.set_PWM_dutycycle(12, 0)  # Stop motor
        pwm.write(20, 0)  # Set direction pin low (optional)
        print('Motors stopped safely.')
        pwm.stop()
        # pwm.close()


def update_heading(counter, heading_angle, blue, orange):
    if blue:
        return -((90 * counter) % 360)
    elif orange:
        return (90 * counter) % 360

def runMotor(speed, direction):  # direction 0 - reverse 1- forward
    pwm.set_PWM_dutycycle(12, int(speed * 2.55))  # Stop motor
    pwm.write(20, direction)  # Set direction pin low (optional)


def normalize_angle(angle, blue, orange, lane):
    """Normalize angle to be within the range of -180 to 180 degrees."""
    if blue:
        if angle < 180 and lane == 0:
            return angle + 360
        else:
            return angle
    elif orange:
        if angle > 180 and lane == 0:
            return angle - 360
        else:
            return angle


def reset_coordinates(distance, lane, orange, blue, x, y):
    if lane == 1:
        return (150 - distance) - 5, y
    if lane == 2:
        if orange:
            return x, (250 - distance) - 5
        elif blue:
            return x, (distance - 250) + 5
    if lane == 3:
        return (distance - 150) + 5, y
    if lane == 0:
        if orange:
            return x, ((distance) - 50) + 5
        elif blue:
            return x, (50 - distance) - 5


def runEncoder(counts, head):
    pwm = pigpio.pi()
    print('Encoder Process Started')
    time.sleep(2)
    try:
        while True:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            esp_data = line.split()
            # print(f"esp_data: {esp_data}")
            if len(esp_data) >= 2:
                try:
                    head.value = float(esp_data[0])
                    counts.value = int(esp_data[1])
                except ValueError:
                    print(f"⚠️ Malformed ESP data: {esp_data}")
            else:
                print(f"⚠️ Incomplete ESP data: {esp_data}")
    except Exception as e:
        print(f"Exception Encoder:{e}")
    finally:
        ser.close()


def read_lidar( lidar_angle, lidar_distance, sp_angle, turn_trigger, lidar_f, lidar_l, lidar_r, left_f, right_f, head ): # print("This is first line")
    global CalledProcessError
    pwm = pigpio.pi()
    trig_time = 0
    previous_angle = 0
    F = 0
    L = 0
    R = 0
    prev_sp = 0
    sp = 0
    lidar_binary_path = '/home/pi/rplidar_sdk/output/Linux/Release/ultra_simple'
    print('⏳ Waiting for LIDAR output...')

    global previous_distance, lidar_front, lidar_left, lidar_right, angle
    if not os.path.isfile(lidar_binary_path):
        print(f"❌ File not found: {lidar_binary_path}")
        return
    print('🚀 Launching ultra_simple...')

    process = subprocess.Popen( [lidar_binary_path, '--channel', '--serial', '/dev/LIDAR_USB', '460800'],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True )

    # try:
    for line in process.stdout:
        line = line.strip()
        # print(line)
        if 'theta' in line and 'Dist' in line:
            try:
                angle_part = line.split()
                # print(angle_part)

                angle_index = angle_part.index('theta:') + 1
                dist_index = angle_part.index('Dist:') + 1

                angle = float(angle_part[angle_index])
                distance = float(angle_part[dist_index])
                angle = int(angle)
                sp = -(sp_angle.value % 360)
                imu_r = int(head.value)
                if previous_angle is None:
                    previous_angle = angle
                # print(f"📍 Angle: {angle:.2f}°, Distance: {distance:.2f} mm")
            except Exception as e:
                print('⚠️ Parse error:', e)
        else:
            print('ℹ️', line)
        if previous_angle != angle:
            while abs(angle - previous_angle) > 1:
                lidar_angle.value = (previous_angle + 1) % 360
                lidar_distance.value = previous_distance
                previous_angle = lidar_angle.value
                rplidar[int(lidar_angle.value)] = lidar_distance.value
                if int(lidar_angle.value) == (0 + imu_r + sp) % 360:
                    lidar_front = lidar_distance.value
                    F = ( 0.2 * F + 0.8 * lidar_distance.value if F else lidar_distance.value )
                    lidar_f.value = F
                if int(lidar_angle.value) == (90 + imu_r + sp) % 360:
                    lidar_left = lidar_distance.value
                    L = ( 0.2 * L + 0.8 * lidar_distance.value if L else lidar_distance.value )
                    lidar_l.value = L
                if int(lidar_angle.value) == (270 + imu_r + sp) % 360:
                    lidar_right = lidar_distance.value
                    R = ( 0.2 * R + 0.8 * lidar_distance.value if R else lidar_distance.value )
                    lidar_r.value = R
                # print("in while loop...")
                if (F <= 950 and R >= 1500) and right_f.value and not left_f.value:
                    turn_trigger.value = True
                elif (F <= 950 and L >= 1500) and left_f.value and not right_f.value:
                    turn_trigger.value = True
                else:
                    turn_trigger.value = False
            if distance != 0:
                with lidar_angle.get_lock(), lidar_distance.get_lock(), head.get_lock():
                    lidar_angle.value = angle
                    lidar_distance.value = distance
                    previous_distance = distance
                    previous_angle = angle
                    rplidar[int(lidar_angle.value)] = lidar_distance.value
                    if int(lidar_angle.value) == (0 + imu_r + sp) % 360:
                        lidar_front = lidar_distance.value
                        F = ( 0.2 * F + 0.8 * lidar_distance.value if F else lidar_distance.value )
                        lidar_f.value = F
                    if int(lidar_angle.value) == (90 + imu_r + sp) % 360:
                        lidar_left = lidar_distance.value
                        L = ( 0.2 * L + 0.8 * lidar_distance.value if L else lidar_distance.value )
                        lidar_l.value = L
                    if int(lidar_angle.value) == (270 + imu_r + sp) % 360:
                        lidar_right = lidar_distance.value
                        R = ( 0.2 * R + 0.8 * lidar_distance.value if R else lidar_distance.value )
                        lidar_r.value = R

                    if (F <= 950 and R >= 1500) and right_f.value and not left_f.value:
                        turn_trigger.value = True
                    elif ( (F <= 950 and L >= 1500) and left_f.value and not right_f.value ):
                        turn_trigger.value = True
                    else: 
                        turn_trigger.value = False # print(f"front: {F:.2f}. right:{R:.2f} left:{L:.2f}  turn_trigger:{turn_trigger.value} imu:{imu_r} sp_angle: {sp} right_f.value:{right_f.value} left_f.value:{left_f.value}")
            # print(f"front: {lidar_front}. right:{lidar_right} left:{lidar_left}  turn_trigger:{turn_trigger.value} diff:{time.time() - trig_time}  imu:{imu_r} sp_angle: {sp_angle.value}")
            # print(f"angle: {lidar_angle.value} distance:{rplidar[int(lidar_angle.value)]}")


if __name__ == '__main__':
    try:
        print('Starting process')

        P = multiprocessing.Process( target=Live_Feed, args=( red_b, green_b, pink_b, centr_y, centr_x, centr_y_red, centr_x_red, centr_x_pink, centr_y_pink ))
        S = multiprocessing.Process( target=servoDrive, args=( red_b, green_b, pink_b, counts, centr_y, centr_x, centr_y_red, centr_x_red, centr_x_pink, centr_y_pink, head, sp_angle, turn_trigger, lidar_f, lidar_l, lidar_r, left_f, right_f ))
        E = multiprocessing.Process(target=runEncoder, args=(counts, head))
        lidar_proc = multiprocessing.Process( target=read_lidar, args=(lidar_angle, lidar_distance, sp_angle, turn_trigger, lidar_f, lidar_l, lidar_r, left_f, right_f, head))  # noqa  # Launch the lidar reader process  # C = multiprocessing.Process(target=color_SP, args=(blue_c, orange_c, white_c))

        P.start()
        E.start()
        lidar_proc.start()
        S.start()

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
        pwm.hardware_PWM(12, 55, 0)
        pwm.bb_serial_read_close(RX_Head)
        pwm.bb_serial_read_close(RX_Left)
        pwm.bb_serial_read_close(RX_Right)
        pwm.stop()
        tfmini.close()
