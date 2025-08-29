import os
os.system('sudo pkill pigpiod')
os.system('sudo pigpiod')
import time
time.sleep(5)
from TFmini import TFmini
import RPi.GPIO as GPIO
import serial
from Servo import Servo
from Encoder import EncoderCounter
import math
import pigpio
import multiprocessing
from ctypes import c_float
import logging
import sys
import subprocess
import cv2
import time
import numpy as np
from pycoral.utils.edgetpu import make_interpreter
from pycoral.adapters import common, detect
from pycoral.utils.dataset import read_label_file
from itertools import combinations
log_file = open('/home/pi/WRO_2025_PI/logs/log_9.txt', 'w')
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
# imu = IMUandColorSensor(board.SCL, board.SDA)
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
lidar_angle = multiprocessing.Value('d', 0.0)
lidar_distance = multiprocessing.Value('d', 0.0)
imu_shared = multiprocessing.Value('d', 0.0)
specific_angle = multiprocessing.Array(c_float, 3)  # shared array of 3 integers
lidar_f = multiprocessing.Value('d', 0.0)
lidar_l = multiprocessing.Value('d', 0.0)
lidar_r = multiprocessing.Value('d', 0.0)

previous_angle = multiprocessing.Value('d', 0.0)
shared_lock = multiprocessing.Lock()
left_f = multiprocessing.Value('b', False)
right_f = multiprocessing.Value('b', False)
stop_evt = multiprocessing.Event()
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


def correctPosition(setPoint, head, x, y, counter, blue, orange, reset, reverse, heading, centr_x_p, centr_x_r, centr_x_g, centr_y_g, centr_y_r,  centr_y_p, finish, distance_h, distance_l, distance_r, red, green):
    # print("INSIDE CORRECT")
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
        print(
            f"lane: {lane}, error: {error:.2f} target:{(setPoint)}, x:{x} y:{y} not reverse")
    elif lane == 1:
        if orange:
            error = x - (100 - setPoint)
            print(f"lane:{lane}, error:{error:.2f} target:{(100 - setPoint)}, setPoint:{setPoint} x:{x}, y:{y}")

        elif blue:
            error = (100 + setPoint) - x
            print(f"lane:{lane}, error:{error} target:{(100 + setPoint)}, x:{x} y:{y} Bluee")
    # print(f" trigger : {flag_t} setPoint: {setPoint} lane: {lane} correction:{correction}, error:{error} x:{x}, y:{y}, prevError :{prevError} angle:{head - correction}")
    elif lane == 2:
        if orange:
            error = y - (200 - setPoint)  # CHANGE 1
            print(
                f"lane:{lane} error:{error:.2f} target:{(200 - setPoint)},  x: {x} y: {y} setPoint:{setPoint}")
        elif blue:
            error = y - (-200 - setPoint)
            print(f"lane:{lane} error:{error} target:{(-200 - setPoint)}, x: {x} y{y}")
    # print(f"setPoint: {flag_t} lane: {lane} correction:{correction}, error:{error} x:{x}, y:{y}, prevError :{prevError} angle:{head - correction}")
    elif lane == 3:
        if orange:
            error = (setPoint - 100) - x
            print(
                f"lane:{lane} error:{error:.2f} target:{(setPoint - 100)}, x: {x} y {y} setPoint:{setPoint}")
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
    if setPoint == 0:
        if abs(error) < 15 and orange:
            print(f"absolute is 0")
            correction = 0
        elif abs(error) < 15 and blue:
            print(f"absolute is 0")
            correction = 0

    if not reset:
        print(f"In not reset...")
        tfmini.getTFminiData()
        if (((setPoint == -35) and orange) or (counter == 0 and (centr_x_p < 300 and centr_x_p > 0) and ((centr_x_g or centr_x_r) >  centr_x_p) and not blue and not orange) and not finish):
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
                correction = -20
                print("setPoint was not -35")
                pass

        if (((setPoint == 35) and blue) or (counter == 0 and (centr_x_p < 300 and centr_x_p > 0) and ((centr_x_g or centr_x_r) < centr_y_p) and not blue and not orange) and not finish):

            if distance_r <= 30:
                correction = -20
                print(f"Avoiding pink wall {correction}")

            elif distance_l < 50:
                if distance_l <= 35:
                    correction = 45
                    print(f"Avoiding pink wall {correction}")

                else:
                    correction = 20
                    print(f"Avoiding pink wall {correction}")
            else:
                correction = 20
                print("setPoint was not 35")
                pass

        if not blue:
            
            if heading > 180 and lane == 0:
                n_head = heading - 360
            else:
                n_head = heading

            if (setPoint <= -70 ) and distance_l <= 20:
                print(f"Correcting Green Wall Orange")
                correction = 15
            elif (setPoint >= 70) and ((tfmini.distance_head <= 25 and (n_head - head > 35)) or distance_r <= 20):
                print(f"Correcting Green Wall... diff:{(n_head - head):.2f} heading:{heading:.2f} n_head:{n_head:.2f} head:{head} right {distance_r} head_d:{tfmini.distance_head}")
                correction = -15
            else:
                print("No wall detected...")
                pass

        else:

            if heading < 180 and lane == 0:
                n_head = heading + 360
            else:
                n_head = heading
                 
                

            if (setPoint >= 70 ) and distance_r <= 20:
                print(f"correctng red wall in blue")
                correction = -15
            elif (setPoint <= -70 ) and ((tfmini.distance_head <=25 and abs((n_head - head) - 360) > 35) or distance_l <= 20):
                print(f"Correcting Green Wall... diff:{abs((n_head - head) - 360):.2f} heading:{heading:.2f} n_head:{n_head:.2f} head:{head} right {distance_r} head_d:{tfmini.distance_head}")
                correction = 15
            else:
                print("No wall detected...")
                pass


    

    if correction > 45:
        correction = 45
    elif correction < -45:
        correction = -45
            
    print(f"diff:{(heading - head):.2f} heading:{heading:.2f} head:{head:.2f} right {distance_r} head_d:{tfmini.distance_head} correction:{correction}") 

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


def correctAngle2(setPoint_gyro, heading):
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

    pTerm = kp * error_gyro * 3
    dTerm = kd * (error_gyro - prevErrorGyro)
    totalErrorGyro += error_gyro
    iTerm = ki * totalErrorGyro
    correction = pTerm + iTerm + dTerm

    if correction > 60:
        correction = 60
    elif correction < -60:
        correction = -60

    prevErrorGyro = error_gyro
    servo.setAngle(90 - correction)





def correctReverseAngle(setPoint_gyro, heading):
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
    servo.setAngle(90 + correction)


def correctReverseAngle2(setPoint_gyro, heading):
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

    pTerm = kp * error_gyro * 3
    dTerm = kd * (error_gyro - prevErrorGyro)
    totalErrorGyro += error_gyro
    iTerm = ki * totalErrorGyro
    correction = pTerm + iTerm + dTerm

    if correction > 50:
        correction = 50
    elif correction < -50:
        correction = -50

    prevErrorGyro = error_gyro
    servo.setAngle(90 + correction)



def Live_Feed(color_b, stop_evt, red_b, green_b, pink_b, centr_y, centr_x, centr_y_red, centr_x_red, centr_x_pink, centr_y_pink, centr_y_b, orange_o, centr_y_o, shared_lock):
    MODEL_PATH = "/home/pi/WRO_2025_PI/limelight_neural_detector_8bit_edgetpu.tflite"
    # put your label file here (id -> name), or set to None
    LABELS = "/home/pi/WRO_2025_PI/label_map.txt"
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
            print("Label load warn:", e)
    FPS = 120
    cap = cv2.VideoCapture(CAM_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)
    cap.set(cv2.CAP_PROP_FPS,          FPS)
    # 0.25 means "manual mode" on many drivers
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
    cap.set(cv2.CAP_PROP_EXPOSURE, -6)
    cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)   # low-latency
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
                
                cx = (x1 + x2)//2
                cy = (y1 + y2)//2
                '''cx = (bbox.xmax + bbox.xmin) //2
                cy = (bbox.ymax + bbox.ymin) // 2'''
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
            print(f"pairs:{pair} red_b.value: {red_b.value} green_b.value:{green_b.value} pink_b:{pink_b.value}  fps:{fps}")
            print(f"centr g:{centr_x.value} centr g y:{centr_y.value}")
            print(f"centr r x:{centr_x_red.value} centr r y:{centr_y_red.value}")
            print(f"centr x pink:{centr_x_pink.value} centr y pink:{centr_y_pink.value}")
            #cv2.imshow("Coral SSD Live", frame_bgr)
            if cv2.waitKey(1) & 0xFF == ord('q'):  # ESC
                break
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        cv2.destroyAllWindows()


def servoDrive(color_b, stop_evt, red_b, green_b, pink_b, counts, centr_y, centr_x, centr_y_red, centr_x_red, centr_x_pink, centr_y_pink, head, centr_y_b,  orange_o, centr_y_o, sp_angle, turn_trigger, specific_angle, imu_shared, lidar_f, lidar_l, lidar_r, shared_lock, left_f, right_f):
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

    change_path = False
    timer_started = False
    timer_v = 0
    g_flag = r_flag = p_flag = False
    g_past = r_past = p_past = False

    red_stored = green_store = False

    red_turn = green_turn = False
    calc_time = False
    lap_finish = continue_parking = parking_heading = parking_flag = False
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
    not_block = False
    ############ VARIABLES ##################
    color_n = ""
    setPointL = -70
    setPointR = 70
    setPointC = 0
    power = 95
    prev_power = 0
    last_counter = 12

    counter = turn_t = current_time = gp_time = rp_time = buff = c_time = green_count = red_count = 0
    heading_angle = 0
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
    avoided = False
    avoided_time = time.time()
    reverse_until = 0
    encoder_counter_store = False
    encoder_counts_value = 0
    l_left = 0
    off = 5000
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
    timer = 0
    norm_head = 0
    lane_reset = 0
    red_time = 0
    green_time = 0
    pink_time = 0
    finish_t = time.time()
    inside_pink = time.time()
    state = 1
    try:
        while True:
            imu_shared.value = head.value
            imu_head = head.value

            # print(f"angles:{specific_angle}")
            # print(f"fps 2222:{1/(time.time() - fps_time2)}")
            fps_time2 = time.time()

            tfmini.getTFminiData()
            tf_h = lidar_f.value
            l_left = lidar_l.value
            l_right = lidar_r.value
            tf_l = tfmini.distance_left
            tf_r = tfmini.distance_right
           

           
            
            if not button:
                print(f"red_b:{red_b.value}, green_b:{green_b.value}, pink_b:{pink_b.value} tf_h:{tf_h:.2f} diff:{(head.value - heading_angle):.2f} counts:{counts.value:.2f}")
                #print(f"centr_X:{centr_x_pink.value} centr_y:{centr_y_pink.value}")
                #print(f"corr:{corr}")
            
            if (time.time() - last_time > debounce_delay):
                previous_state = button_state
                button_state = pwm.read(5)
                # time.sleep(0.03)

                if previous_state == 1 and button_state == 0:
                    button = not (button)
                    last_time = time.time()
                    print(
                        f"🔘 Button toggled! Drive {'started' if button else 'stopped'}")
                    power = 95
            ##### STOP CONDITION ######
            # print(f"rgb:{imu.color_rgb} color:{color_sensor}")
            # print(f"pink detected:{pink_detected}")
            if counter == last_counter and not lap_finish:
                print(
                    f"centr_y :{centr_y.value} centr_y_red:{centr_y_red.value}")
                print("REACHED MAXIMUM COUNTS")
                print(f"target:{target_count}")
                if not finished:
                    target_count = counts.value + 22000
                    finished = True
                if counts.value >= target_count and not reverse_trigger:
                    power = 0
                    pink_b.value = False
                    # Set duty cycle to 50% (128/255)
                    pwm.set_PWM_dutycycle(pwm_pin, power)
                    time.sleep(3)
                    pink_b.value = False
                    power = 70
                    prev_power = 0
                    lap_finish = True
                    reverse_trigger = True
                    print(f"Vehicle is stopped...")
                    

            if lap_finish:
                if not counter_reset:
                    counter = counter % last_counter
                    counter_reset = True

            if lap_finish and not continue_parking:
                if orange_flag:
                    setPointR = -100
                    setPointC = -100
                elif blue_flag:
                    setPointL = 100
                    setPointC = 100


            if continue_parking:  # THIS SETPOINT IS WHEN THE ROBOT IS IN THE PARKING MODE
                print("Inside Continue parking")
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
                

            ################## IF PINK IS DETECTED THEN WHAT? ###############

            if pink_b.value:  # DECIDES SETPOINT WHENEVER PINK IS IN THE FRAME
                #print(f"PINK IS DETECTED...")
                if orange_flag:
                    if (centr_x_pink.value < centr_x.value) and (centr_x_pink.value > 0 and centr_x.value > 0) and not continue_parking:
                        setPointL = -35
                        setPointR = 70
                        print(f"setPointL : {setPointL}")
                elif blue_flag:
                    if (centr_x_red.value < centr_x_pink.value) and (centr_x_pink.value > 0 and centr_x_red.value > 0) and not continue_parking:
                        setPointR = 35
                        setPointL = -70
                        print(f"setPointR: {setPointR}")

                elif counter % 4 == 0 and not blue_flag and not orange_flag:
                    if ((centr_x_pink.value < 300 and centr_x_pink.value > 0) and ((centr_x_red.value) > centr_x_pink.value)) and not continue_parking:
                        setPointR = 35
                        setPointL = -70
                        print(f"at 0 counter orange:{setPointR} {setPointL}")

                    if ((centr_x_pink.value > 300) and ((centr_y.value or centr_y_red.value) <= centr_y_pink.value)) and not continue_parking:
                        setPointL = -35
                        setPointR = 70
                        print(f"at 0 counter blue:{setPointR} {setPointL}")

                '''if lap_finish and not continue_parking:
                    pink_detected = False
                    print("Starting Parking...")
                    continue_parking = True'''
                pb_time = time.time()
            # IF DOES NOT SEE PINK, KEEP THE SAME SETPOINT FOR 1 SECOND AND THEN CHANGE
            elif not pink_b.value and time.time() - pb_time > 1 and not lap_finish:
                # print(f"Resetting setPoints...{pink_detected}")
                if g_flag and not continue_parking:
                    print(f"away from green {g_past}")
                    setPointL = setPointL - 1
                    if setPointL < -150:
                        setPointL = -150
                    setPointR = 70
                elif r_flag and not continue_parking:
                    print(f"away from red {r_past}")
                    setPointR = setPointR + 1
                    if setPointR > 150:
                        setPointR = 150
                    setPointL = -70



            if not inParkingatStart and not left_f.value and not right_f.value:
                print("Starting Parking at Start...")
                if tf_l < 20 and tf_h < 200 and tf_l> 0 and tf_h> 0 and pink_b.value :
                    print("Right side parking")
                    enc.x = 0
                    enc.y = -40
                    right_f.value = True
                    inParkingatStart = True
                elif tf_r < 20 and tf_h < 200 and tf_h> 0 and tf_r > 0 and pink_b.value :
                    enc.x = 0
                    enc.y = 40
                    print("Left side parking")
                    left_f.value = True
                    inParkingatStart = True

            if right_f.value and not orange_flag:
                orange_flag = True
                blue_flag = False
            elif left_f.value and not blue_flag:
                blue_flag = True
                orange_flag = False

            ##################### BUTTON STARTS THE CODE ##################
            if button:  # THIS BLOCK OF CODE WHEN BUTTON IS PRESSED
                # time.sleep(0.01)
                print("-------------------------------------------------")

                x, y = enc.get_position(imu_head, counts.value)
                if time.time() < startPark and not parking_flag:
                    red_b.value = False
                    green_b.value = False
                    pwm.set_PWM_dutycycle(pwm_pin, int(2.0*power))
                    pwm.write(direction_pin, 1)
                    if orange_flag:
                        correctAngle(90, head.value)
                    elif blue_flag:
                        correctAngle(-90, head.value)
                    # correctAngle(heading_angle, imu_head)  # still steer if needed
                    print("coming out of the zone")
                    continue  # skip the drive code below

                if inParkingatStart and not parking_flag:
                    print("Coming out of the zone..")
                    if not exitPark:
                        startPark = time.time() + 2.3
                        exitPark = True
                    if abs(corr) < 10 and tf_h > 200:
                        inParkingatStart = False
                        
                if red_b.value or green_b.value:
                    power = 55
                elif not g_flag and not r_flag:
                    power = 85
                else:
                    power = 70

                if time.time() < avoided_time:
                    pwm.set_PWM_dutycycle(pwm_pin, 0)
                    pwm.write(direction_pin, 0)
                    # correctAngle(heading_angle, imu_head)  # still steer if needed
                    print("block spotted")
                    continue  # skip the drive code below

                if avoided_time > 0 and time.time() < reverse_until:
                    # non-blocking reverse
                    power = 70
                    prev_power = 0
                    pwm.set_PWM_dutycycle(pwm_pin, int(1.3 * power))
                    # 0 = reverse, 1 = forward (per your wiring)
                    pwm.write(direction_pin, 0)
                    # keep the robot straight while reversing (or add bias if you want to angle out)
                    # correctAngle(sp_angle.value, imu_head)
                    print("reverse after block spotted")
                    if red_b.value and not reset_f:
                        print("Red Detected, setting servo to 60 degrees")
                        servo.setAngle(70)
                    elif green_b.value and not reset_f:
                        print("Green Detected, setting servo to 120 degrees")
                        servo.setAngle(110)
                    continue  # still skip forward-drive code

                if not reset_servo:
                    time.sleep(0.5)
                    servo.setAngle(130)
                    time.sleep(0.5)
                    servo.setAngle(90)
                    reset_servo = True


                total_power = (power * 0.1) + (prev_power * 0.9)
                prev_power = total_power
                # Set duty cycle to 50% (128/255)
                pwm.set_PWM_dutycycle(pwm_pin, 2.55 * total_power)

                pwm.write(direction_pin, 1)  # Set pin 20 high

                ################        PARKING         ################

                if parking_flag and not stop_flag:
                    print(f"PARKING ------> distance_head : {tfmini.distance_head}")
                    print("Inside Parking Loop")
                    tfmini.getTFminiData()
                    if not calc_time:
                        c_time = time.time()
                        calc_time = True
                    '''if pink_b.value and not pink_r:
                        print("Time is same")
                        time_p = 0.7
                        pink_r = True
                    elif not pink_b.value and not pink_r:
                        print("increasing time..")
                        time_p = 2.5
                        pink_r = True'''
                    time_p = 0.7

                    
                    if state == 8:
                        if orange_flag:
                            heading_angle = heading_angle - 90
                            correctReverseAngle2(heading_angle, head.value)
                            
                            while (tfmini.distance_left > 50) or abs(corr) > 15:
                                tfmini.getTFminiData()
                                print(f"corr:{abs(corr)} head:{tfmini.distance_head} left:{tfmini.distance_left}")
                                print("Reversing backward...")
                                power = 85
                                prev_power = 0
                                correctReverseAngle2(heading_angle, head.value)
                                # Set duty cycle to 50% (128/255)
                                pwm.set_PWM_dutycycle(pwm_pin, power)
                                pwm.write(direction_pin, 0)  # Set pin 20 hig  
                        elif blue_flag:
                            heading_angle = heading_angle + 90
                            correctReverseAngle2(heading_angle, head.value)
                            
                            while (tfmini.distance_right > 50) or abs(corr) > 15:
                                tfmini.getTFminiData()
                                print(f"corr:{abs(corr)} head:{tfmini.distance_head} right:{tfmini.distance_right}")
                                print("Reversing backward...")
                                power = 85
                                prev_power = 0
                                correctReverseAngle2(heading_angle, head.value)
                                # Set duty cycle to 50% (128/255)
                                pwm.set_PWM_dutycycle(pwm_pin, power)
                                pwm.write(direction_pin, 0)  # Set pin 20 hig  
                        state = 9
                    if state == 9:
                        if orange_flag:
                            heading_angle = heading_angle + 90
                            correctReverseAngle2(heading_angle, head.value)
                            while (tfmini.distance_right > 20) or abs(corr) > 5:
                                tfmini.getTFminiData()
                                print(f"corr:{abs(corr)} head:{tfmini.distance_head} left:{tfmini.distance_right}")
                                print("Reversing backward...")
                                power = 85
                                prev_power = 0
                                correctReverseAngle2(heading_angle, head.value)
                                # Set duty cycle to 50% (128/255)
                                pwm.set_PWM_dutycycle(pwm_pin, power)
                                pwm.write(direction_pin, 0)  # Set pin 20 hig 
                        elif blue_flag:
                            heading_angle = heading_angle - 90
                            correctReverseAngle2(heading_angle, head.value)
                            while (tfmini.distance_left > 20) or abs(corr) > 5:
                                tfmini.getTFminiData()
                                print(f"corr:{abs(corr)} head:{tfmini.distance_head} right:{tfmini.distance_left}")
                                print("Reversing backward...")
                                power = 85
                                prev_power = 0
                                correctReverseAngle2(heading_angle, head.value)
                                # Set duty cycle to 50% (128/255)
                                pwm.set_PWM_dutycycle(pwm_pin, power)
                                pwm.write(direction_pin, 0)  # Set pin 20 hig                        
                        state = 6
                    if state == 1:  
                        while time.time() - c_time < time_p :
                            print("Reversing backward...")
                            power = 70
                            prev_power = 0
                            correctReverseAngle(heading_angle, head.value)
                            # Set duty cycle to 50% (128/255)
                            pwm.set_PWM_dutycycle(pwm_pin, power)
                            pwm.write(direction_pin, 0)  # Set pin 20 hig
                            prev_time = time.time()
                        reverse_complete = True
                        state = 8
                        print('state 1')
                    if state == 2:
                        while time.time() - prev_time < 0.5:
                            print("Robot is stopped")
                            power = 0
                            prev_power = 0
                            # Set duty cycle to 50% (128/255)
                            pwm.set_PWM_dutycycle(pwm_pin, power)

                        if orange_flag:
                            print("Changing heading angle")
                            heading_angle = heading_angle + 80
                            print("Heading angle changed")

                        elif blue_flag:
                            print("Changing heading angle Blue")
                            heading_angle = heading_angle - 80
                            print("Heading angle changed Blue")
                        
                        state = 3
                        print('state 2')

                    print(f"Correcting angle..{abs(corr)}")
                    if state == 3:
                        while tfmini.distance_head > 12 or abs(corr) > 15:
                            tfmini.getTFminiData() 
                            print(f"Moving slowly.. {abs(corr)}")
                            power = 85
                            prev_power = 84
                            pwm.write(direction_pin, 1)  # Set pin 20 hig

                            pwm.set_PWM_dutycycle(pwm_pin, power)
                            correctAngle2(heading_angle, head.value)
                            print(f"parking heading reverse: {parking_heading_reverse}")
                        
                        state = 4
                        prev_time = time.time()
                        print('state 3')
                    if state == 4:
                        if orange_flag:
                            heading_angle = heading_angle - 80
                        elif blue_flag:
                            heading_angle = heading_angle + 80
                        correctAngle2(heading_angle, head.value)
                        while tfmini.distance_head > 20 or abs(corr) > 15:
                            power = 70
                            prev_power = 0
                            tfmini.getTFminiData()
                            pwm.set_PWM_dutycycle(pwm_pin, power)
                            correctAngle2(heading_angle, head.value)
                            print(f"finish flag:{tf_h} corr:{abs(corr)}")
                            pwm.write(direction_pin, 1)  # Set pin 20 hig
                        state = 5
                        prev_time = time.time()
                        print('state 4')
                    
                    if state == 5:
                        while time.time() - prev_time < 1:
                            power = 50
                            prev_power = 0
                            pwm.set_PWM_dutycycle(pwm_pin, power)
                            correctReverseAngle(heading_angle, head.value)
                            print(f"finish flag:{tf_h} corr:{abs(corr)}")
                            pwm.write(direction_pin, 0)  # Set pin 20 hig 
                        state = 6
                        prev_time = time.time()
                        print('state 6')
                    
                    if state == 6:
                        power = 0
                        prev_power = 0
                        pwm.set_PWM_dutycycle(pwm_pin, power)  
                        sys.exit(0)                      
                else:
                    if reset_f:
                        print("RESETTING FLAGS...")
                        g_past = False
                        r_past = False
                        g_flag = False
                        r_flag = False
                        rev_count = 0
                        if blue_flag:
                            if head.value < 180 and lane_reset == 0:
                                norm_head = head.value + 360
                            else:
                                norm_head = head.value
                            while abs((norm_head - heading_angle) - 360) > 8 or tfmini.distance_head > 60:
                                if head.value < 180 and lane_reset == 0:
                                    norm_head = head.value + 360
                                else:
                                    norm_head = head.value
                                print(f"correcting heading  blue {head.value - heading_angle:.2f} {head.value} {tfmini.distance_head:.2f} distance_right:{tfmini.distance_right:.2f} distance_head:{tfmini.distance_head:.2f}")
                                x, y = enc.get_position(head.value, counts.value)
                                tfmini.getTFminiData()
                                correctAngle(heading_angle, head.value)
                                power = 70
                                prev_power = 0
                                pwm.set_PWM_dutycycle(pwm_pin, int(2.0 * power))
                                pwm.write(direction_pin, 1)          
                            if tfmini.distance_right > 25 :
                                while tfmini.distance_head > 20:
                                    tfmini.getTFminiData()
                                    print(f"moving ahead blue to correct heading x:{x:.2f} y:{y:.2f} lidar_f:{lidar_f.value:.2f} distance_right:{tfmini.distance_right:.2f} distance_head:{tfmini.distance_head:.2f}")
                                    correctAngle(heading_angle + 20, head.value)
                                    #correctReverseAngle(heading_angle, head.value)
                                    power = 90
                                    prev_power = 0
                                    pwm.set_PWM_dutycycle(pwm_pin, int(2.0 * power))
                                    pwm.write(direction_pin, 1)  # 0 = reverse, 1 = forward (per your wiring)
                                    x, y = enc.get_position(head.value, counts.value)

                                power = 0
                                prev_power = 0
                                # Set duty cycle to 50% (128/255)
                                pwm.set_PWM_dutycycle(pwm_pin, power)
                                x, y = enc.get_position(imu_head, counts.value)
                                buff = 4

                                counter = counter + 1
                                lane_reset = counter % 4
                                heading_angle = -((90 * counter) % 360)
                                sp_angle.value = heading_angle
                                
                                if not timer_started:
                                    timer_v = time.time()
                                    timer_started = True                                                  

                                if head.value < 180 and lane_reset == 0:
                                    norm_head = head.value + 360
                                else:
                                    norm_head = head.value
                                while abs((norm_head - heading_angle) - 360) > 8 and time.time() - timer_v < 2.5:
                                    if head.value < 180 and lane_reset == 0:
                                        norm_head = head.value + 360
                                    else:
                                        norm_head = head.value
                                    print(f"reversing servo {head.value:.2f} {heading_angle} diff: {(heading_angle - head.value) + 360:.2f} rev_diff: {(head.value - heading_angle) - 360:.2f} {counts.value} x:{x:.2f} y:{y:.2f} lidar_f:{lidar_f.value:.2f}")
                                    x, y = enc.get_position(head.value, counts.value)
                                    tfmini.getTFminiData()
                                    correctReverseAngle(heading_angle, head.value)
                                    power = 90
                                    prev_power = 0
                                    pwm.set_PWM_dutycycle(pwm_pin, int(2.0 * power))
                                    pwm.write(direction_pin, 0)  # 0 = reverse, 1 = forward (per your wiring)      
                                power = 0
                                prev_power = 0
                                pwm.set_PWM_dutycycle(pwm_pin, int(2.0 * power))

                                
                                c_time = time.time()

                                time.sleep(0.5)
                                tfmini.getTFminiData()

                                turn_trigger_distance = tfmini.distance_right

                                if lane_reset == 1:
                                    enc.x = (150 - turn_trigger_distance) - 5
                                if lane_reset == 2:
                                    enc.y = (turn_trigger_distance - 250) + 5
                                if lane_reset == 3:
                                    enc.x = (turn_trigger_distance - 150) + 5
                                if lane_reset == 0:
                                    enc.y = (50 - turn_trigger_distance) - 5

                                power = 90

                                if not trigger_enc_flag:
                                    print("Encoder counts are stored for trigger")
                                    trigger_enc = counts.value
                                    trigger_enc_flag = True
                                reset_f = False
                                green_b.value = False
                                red_b.value = False
                                pink_b.value = False
                                
                            else:
                                if lap_finish:
                                    print("Lap is finished and it is inside 30")
                                    while tfmini.distance_head > 20 :
                                        tfmini.getTFminiData()
                                        print(f"moving blue right ahead to correct heading x:{x} y:{y} lidar_f:{lidar_f.value} distance_right:{tfmini.distance_right:.2f} distance_head:{tfmini.distance_head:.2f}")
                                        correctAngle(heading_angle - 20, head.value)
                                        #correctReverseAngle(heading_angle, head.value)
                                        power = 80
                                        prev_power = 0
                                        pwm.set_PWM_dutycycle(pwm_pin, int(2.0 * power))
                                        pwm.write(direction_pin, 1)  # 0 = reverse, 1 = forward (per your wiring)
                                        x, y = enc.get_position(head.value, counts.value)   
                                         
                                    heading_angle = heading_angle + 90
                                    if head.value < 180 and lane_reset == 0:
                                        norm_head = head.value + 360
                                    else:
                                        norm_head = head.value
                                    while abs((norm_head - heading_angle) - 360) > 8 :
                                        if head.value < 180 and lane_reset == 0:
                                            norm_head = head.value + 360
                                        else:
                                            norm_head = head.value
                                        print(f"reversing servo {head.value:.2f} {heading_angle} diff: {(heading_angle - head.value) + 360:.2f} rev_diff: {(head.value - heading_angle) - 360:.2f} {counts.value} x:{x:.2f} y:{y:.2f} lidar_f:{lidar_f.value:.2f}")
                                        x, y = enc.get_position(head.value, counts.value)
                                        tfmini.getTFminiData()
                                        correctReverseAngle(heading_angle, head.value)
                                        power = 90
                                        prev_power = 0
                                        pwm.set_PWM_dutycycle(pwm_pin, int(2.0 * power))
                                        pwm.write(direction_pin, 0)  # 0 = reverse, 1 = forward (per your wiring)
                                                                            
                                                                          
                                    while tfmini.distance_head > 50 :
                                        tfmini.getTFminiData()
                                        print(f"moving blue right ahead to correct heading x:{x} y:{y} lidar_f:{lidar_f.value} distance_right:{tfmini.distance_right:.2f} distance_head:{tfmini.distance_head:.2f}")
                                        correctAngle(heading_angle, head.value)
                                        #correctReverseAngle(heading_angle, head.value)
                                        power = 40
                                        prev_power = 0
                                        pwm.set_PWM_dutycycle(pwm_pin, power)
                                        pwm.write(direction_pin, 1)  # 0 = reverse, 1 = forward (per your wiring)
                                        x, y = enc.get_position(head.value, counts.value) 
                                    
                                    heading_angle = heading_angle + 90
                                    p_flag = True
                                    p_past = True
                                    continue_parking = True
                                    reset_f = False
                                    power = 0
                                    prev_power = 0
                                    pwm.set_PWM_dutycycle(pwm_pin, power)
                                    power = 70
                                    prev_power = 0
                                    print("Calculating pink time...")
                                    pink_time = time.time()
                                    print(f"pink time {pink_time} ")
                                        
                                else:
                                    while tfmini.distance_head > 80 :
                                        tfmini.getTFminiData()
                                        print(f"moving blue right ahead to correct heading x:{x} y:{y} lidar_f:{lidar_f.value} distance_right:{tfmini.distance_right:.2f} distance_head:{tfmini.distance_head:.2f}")
                                        correctAngle(heading_angle, head.value)
                                        #correctReverseAngle(heading_angle, head.value)
                                        power = 80
                                        prev_power = 0
                                        pwm.set_PWM_dutycycle(pwm_pin, int(1.9 * power))
                                        pwm.write(direction_pin, 1)  # 0 = reverse, 1 = forward (per your wiring)
                                        x, y = enc.get_position(head.value, counts.value)
                                    while tfmini.distance_head < 70 :
                                        tfmini.getTFminiData()
                                        print(f"moving ahead to correct heading x:{x} y:{y} lidar_f:{lidar_f.value} head_d: {tfmini.distance_head:.2f} left:{tfmini.distance_left:.2f}")
                                        correctReverseAngle(heading_angle, head.value)
                                        #correctReverseAngle(heading_angle, head.value)
                                        power = 60
                                        prev_power = 0
                                        pwm.set_PWM_dutycycle(pwm_pin, int(1.9 * power))
                                        pwm.write(direction_pin, 0)  # 0 = reverse, 1 = forward (per your wiring)
                                        x, y = enc.get_position(head.value, counts.value)

                                    power = 0
                                    prev_power = 0
                                    # Set duty cycle to 50% (128/255)
                                    pwm.set_PWM_dutycycle(pwm_pin, power)
                
                                    x, y = enc.get_position(imu_head, counts.value)
                                    buff = 4
                                    time.sleep(0.5)
                                    
                                    c_time = time.time()
                                    counter = counter + 1                                
                                    lane_reset = counter % 4
                                    tfmini.getTFminiData()

                                    turn_trigger_distance = tfmini.distance_head
                                    
                                    if lane_reset == 1:
                                        enc.x = (150 - turn_trigger_distance) - 12
                                    if lane_reset == 2:
                                        enc.y = (turn_trigger_distance - 250) + 12
                                    if lane_reset == 3:
                                        enc.x = (turn_trigger_distance - 150) + 12
                                    if lane_reset == 0:
                                        enc.y = (50 - turn_trigger_distance) - 12

                                    heading_angle = -((90 * counter) % 360)
                                    sp_angle.value = heading_angle
                                    power = 70
                                    if not trigger_enc_flag:
                                        print("Encoder counts are stored for trigger")
                                        trigger_enc = counts.value
                                        trigger_enc_flag = True
                                    # print(f'Resuming Motor...{offset}')
                                    power = 70
                                    prev_power = 0
                                    reset_f = False
                                    green_b.value = False
                                    red_b.value = False
                                    pink_b.value = False                            
                        elif orange_flag:
                            if head.value > 180 and lane_reset == 0:
                                norm_head = head.value - 360
                            else:
                                norm_head = head.value
                            while abs(norm_head - heading_angle) > 8 or tfmini.distance_head > 60:
                                if head.value > 180 and lane_reset == 0:
                                    norm_head = head.value - 360
                                else:
                                    norm_head = head.value                                
                                print(f"correcting orange heading counter:{counter} imu:{head.value:.2f} diff:{abs(norm_head - heading_angle):.2f} tfmini head: {tfmini.distance_head:.2f} norm_head:{norm_head}")
                                x, y = enc.get_position(head.value, counts.value)
                                tfmini.getTFminiData()
                                correctAngle(heading_angle, head.value)
                                power = 70
                                prev_power = 0
                                pwm.set_PWM_dutycycle(pwm_pin, int(1.8 * power))
                                pwm.write(direction_pin, 1) 
                            print(f"distance_left:{tfmini.distance_left:.2f} distance_head:{tfmini.distance_head:.2f} after correction")
                            if tfmini.distance_left > 25:
                                while tfmini.distance_head > 20:
                                    tfmini.getTFminiData()
                                    print(f"moving ahead to correct heading x:{x:.2f} y:{y:.2f} lidar_f:{lidar_f.value:.2f} counter:{counter} imu:{head.value} {tfmini.distance_head:.2f}")
                                    correctAngle(heading_angle - 20, head.value)
                                    #correctReverseAngle(heading_angle, head.value)
                                    power = 90
                                    prev_power = 0
                                    pwm.set_PWM_dutycycle(pwm_pin, int(2.0 * power))
                                    pwm.write(direction_pin, 1)  # 0 = reverse, 1 = forward (per your wiring)
                                    x, y = enc.get_position(head.value, counts.value)

 
            
                                tfmini.getTFminiData()
                                x, y = enc.get_position(imu_head, counts.value)
                                buff = 4


                                c_time = time.time()
                                counter = counter + 1
                                lane_reset = counter % 4

                                                                
                                heading_angle = ((90 * counter) % 360)
                                sp_angle.value = heading_angle
                                
                                if not timer_started:
                                    timer_v = time.time()
                                    timer_started = True
                                    
                                if head.value > 180 and lane_reset == 0:
                                    norm_head = head.value - 360
                                else:
                                    norm_head = head.value  
                                while abs(norm_head - heading_angle) > 8 and time.time() - timer_v < 2.5:
                                    if head.value > 180 and lane_reset == 0:
                                        norm_head = head.value - 360
                                    else:
                                        norm_head = head.value 
                                    print(f"reversing servo counter:{counter} imu:{head.value:.2f} diff:{abs(norm_head - heading_angle):.2f} tfmini head: {tfmini.distance_head:.2f} norm_head:{norm_head}")
                                    x, y = enc.get_position(head.value, counts.value)
                                    tfmini.getTFminiData()
                                    correctReverseAngle(heading_angle, head.value)
                                    power = 90
                                    prev_power = 0
                                    pwm.set_PWM_dutycycle(pwm_pin, int(1.9 * power))
                                    pwm.write(direction_pin, 0)  # 0 = reverse, 1 = forward (per your wiring)

                                power = 0
                                prev_power = 0
                                # Set duty cycle to 50% (128/255)
                                pwm.set_PWM_dutycycle(pwm_pin, power)
                                time.sleep(0.5)
                                tfmini.getTFminiData()

                                turn_trigger_distance = tfmini.distance_left

                                if lane_reset == 1:
                                    enc.x = (150 - (turn_trigger_distance)) - 5
                                    print(f"x: {enc.x}")
                                if lane_reset == 2:
                                    enc.y = (250 - (turn_trigger_distance)) - 5
                                if lane_reset == 3:
                                    enc.x = ((turn_trigger_distance) - 150) + 5
                                if lane_reset == 0:
                                    enc.y = ((turn_trigger_distance) - 50) + 5
                                power = 90

                                if not trigger_enc_flag:
                                    print("Encoder counts are stored for trigger")
                                    trigger_enc = counts.value
                                    trigger_enc_flag = True
                                power = 70
                                green_b.value = False
                                red_b.value = False
                                pink_b.value = False
                                reset_f = False
                            else:
                                if lap_finish:
                                    print("Lap is finished and it is inside 30")
                                    while tfmini.distance_head > 20 or head.value > 180:
                                        tfmini.getTFminiData()
                                        print(f"moving blue right ahead to correct heading x:{x} y:{y} lidar_f:{lidar_f.value} distance_right:{tfmini.distance_right:.2f} distance_head:{tfmini.distance_head:.2f}")
                                        correctAngle(heading_angle + 20, head.value)
                                        #correctReverseAngle(heading_angle, head.value)
                                        power = 80
                                        prev_power = 0
                                        pwm.set_PWM_dutycycle(pwm_pin, int(2.0 * power))
                                        pwm.write(direction_pin, 1)  # 0 = reverse, 1 = forward (per your wiring)
                                        x, y = enc.get_position(head.value, counts.value)   
                                         
                                    heading_angle = heading_angle - 90
                                    if head.value > 180 and lane_reset == 0:
                                        norm_head = head.value - 360
                                    else:
                                        norm_head = head.value 
                                    while abs((norm_head - heading_angle)) > 8:
                                        if head.value > 180 and lane_reset == 0:
                                            norm_head = head.value - 360
                                        else:
                                            norm_head = head.value 
                                        print(f"reversing servo {head.value:.2f} {heading_angle} diff: {(heading_angle - head.value) + 360:.2f} rev_diff: {(head.value - heading_angle) - 360:.2f} {counts.value} x:{x:.2f} y:{y:.2f} lidar_f:{lidar_f.value:.2f}")
                                        x, y = enc.get_position(head.value, counts.value)
                                        tfmini.getTFminiData()
                                        correctReverseAngle(heading_angle, head.value)
                                        power = 90
                                        prev_power = 0
                                        pwm.set_PWM_dutycycle(pwm_pin, int(2.0 * power))
                                        pwm.write(direction_pin, 0)  # 0 = reverse, 1 = forward (per your wiring)
                                                                            
                                                                          
                                    while tfmini.distance_head > 50 :
                                        tfmini.getTFminiData()
                                        print(f"moving blue right ahead to correct heading x:{x} y:{y} lidar_f:{lidar_f.value} distance_right:{tfmini.distance_right:.2f} distance_head:{tfmini.distance_head:.2f}")
                                        correctAngle(heading_angle, head.value)
                                        #correctReverseAngle(heading_angle, head.value)
                                        power = 40
                                        prev_power = 0
                                        pwm.set_PWM_dutycycle(pwm_pin, power)
                                        pwm.write(direction_pin, 1)  # 0 = reverse, 1 = forward (per your wiring)
                                        x, y = enc.get_position(head.value, counts.value) 
                                    
                                    heading_angle = heading_angle - 90
                                    p_flag = True
                                    p_past = True
                                    continue_parking = True
                                    reset_f = False
                                    power = 0
                                    prev_power = 0
                                    pwm.set_PWM_dutycycle(pwm_pin, power)
                                    power = 70
                                    prev_power = 0
                                    print("Calculating pink time...")
                                    pink_time = time.time()
                                    print(f"pink time {pink_time} ")
                                else:
                                    while tfmini.distance_head > 80 :
                                        tfmini.getTFminiData()
                                        print(f"moving ahead to correct heading x:{x} y:{y} lidar_f:{lidar_f.value} head_d: {tfmini.distance_head:.2f} left:{tfmini.distance_left:.2f}")
                                        correctAngle(heading_angle, head.value)
                                        #correctReverseAngle(heading_angle, head.value)
                                        power = 90
                                        prev_power = 0
                                        pwm.set_PWM_dutycycle(pwm_pin, int(1.9 * power))
                                        pwm.write(direction_pin, 1)  # 0 = reverse, 1 = forward (per your wiring)
                                        x, y = enc.get_position(head.value, counts.value)
                                    while tfmini.distance_head < 70 :
                                        tfmini.getTFminiData()
                                        print(f"moving ahead to correct heading x:{x} y:{y} lidar_f:{lidar_f.value} head_d: {tfmini.distance_head:.2f} left:{tfmini.distance_left:.2f}")
                                        correctReverseAngle(heading_angle, head.value)
                                        #correctReverseAngle(heading_angle, head.value)
                                        power = 60
                                        prev_power = 0
                                        pwm.set_PWM_dutycycle(pwm_pin, int(1.9 * power))
                                        pwm.write(direction_pin, 0)  # 0 = reverse, 1 = forward (per your wiring)
                                        x, y = enc.get_position(head.value, counts.value)
                                    power = 0
                                    prev_power = 0
                                    # Set duty cycle to 50% (128/255)
                                    pwm.set_PWM_dutycycle(pwm_pin, power)
                
                                    x, y = enc.get_position(imu_head, counts.value)
                                    time.sleep(0.5)
                                    tfmini.getTFminiData()

                                    c_time = time.time()
                                    counter = counter + 1                                

                                    lane_reset = counter % 4
                                    turn_trigger_distance = tfmini.distance_head

                                    if lane_reset == 1:
                                        enc.x = (150 - (turn_trigger_distance)) - 12
                                        print(f"x: {enc.x}")
                                    if lane_reset == 2:
                                        enc.y = (250 - (turn_trigger_distance)) - 12
                                    if lane_reset == 3:
                                        enc.x = ((turn_trigger_distance) - 150) + 12
                                    if lane_reset == 0:
                                        enc.y = ((turn_trigger_distance) - 50) + 12

                                    heading_angle = ((90 * counter) % 360)
                                    sp_angle.value = heading_angle
                                    power = 70

                                    if not trigger_enc_flag:
                                        print("Encoder counts are stored for trigger")
                                        trigger_enc = counts.value
                                        trigger_enc_flag = True
                                    # print(f'Resuming Motor...{offset}')
                                    power = 70
                                    reset_f = False
                                    green_b.value = False
                                    red_b.value = False
                                    pink_b.value = False
                    else:
                        # TRIGGGER CHECK VALUESSSS
                                        
                        if (turn_trigger.value and not trigger) and not trigger_enc_flag:
                            buff = 0
                            print("Trigger Detected...")
                            trigger = True
                            reset_f = True
                            avoided_time = time.time() + 0.3
                            timer_started = False
                            turn_t = time.time()
                        elif trigger_enc_flag:
                            print(f"Trigger enc flag is set: {trigger_enc_flag} counts:{counts.value} trigger_enc:{trigger_enc + 22000}")
                            if counts.value > trigger_enc + 22000:
                                trigger = False
                                trigger_enc_flag = False
                                print("Encoder counts done for trigger")
                            pwm.write(blue_led, 0)

                        ################### PANDAV 2.0 ####################

                        if green_b.value and not r_flag and not continue_parking and not g_flag and not reset_f and centr_y.value > 240 and not lap_finish:
                            if rev_count < 1:
                                if centr_x.value < 320:
                                    avoided_time = time.time() + 0.3
                                    if orange_flag:
                                        reverse_until = avoided_time + 0.7
                                    else:
                                        reverse_until = avoided_time + 0.7
                                    rev_count += 1
                            g_flag = True
                            g_past = True
                            print("Green detected")
                            pwm.write(red_led, 0)
                            pwm.write(green_led, 0)
                            print('1')

                        elif (g_past) and not continue_parking and not reset_f and  not lap_finish:
                            g_flag = True
                            if green_b.value:
                                green_time = time.time()
                                
                            if tf_r <= 50 or time.time() - green_time > 2:
                            #if time.time() - green_time > 2.5:

                                if not green_b.value and not encoder_counter_store:
                                    encoder_counts_value = counts.value
                                    encoder_counter_store = True
                                    g_flag = False
                                    g_past = False

                                    print("encoder counts are stored for green")
                            print('2')

                        elif red_b.value and not g_flag and not continue_parking and not r_flag and not reset_f and centr_y_red.value > 240 and not lap_finish:
                            r_flag = True
                            r_past = True
                            if rev_count < 1:
                                if centr_x_red.value > 300:
                                    avoided_time = time.time() + 0.3
                                    if orange_flag:
                                        reverse_until = avoided_time + 0.7
                                    else:
                                        reverse_until = avoided_time + 0.7
                                rev_count += 1
                            # print(f"x cent:{centr_x_red.value} centr y:{centr_y_red.value}")
                            print("Red detected")
                            pwm.write(red_led, 0)
                            pwm.write(green_led, 0)

                            print('3')

                        elif (r_past) and not continue_parking and not reset_f and not lap_finish:
                            r_flag = True
                            if red_b.value:
                                red_time  = time.time()
                            if tf_l <= 50 or time.time() - red_time > 2:
                            #if time.time() - red_time > 2.5:

                                if not red_b.value and not encoder_counter_store:
                                    encoder_counts_value = counts.value
                                    encoder_counter_store = True
                                    r_flag = False
                                    r_past = False
                                    print("encoder counts stored for red")
                            
                                
                            print('4')

                        elif pink_b.value and continue_parking and not p_flag:
                            if (centr_x_pink.value < 200 and centr_x_pink.value > 0 and orange_flag) or (centr_x_pink.value > 500 and blue_flag):
                                p_flag = True
                                p_past = True
                            print('5')

                        elif p_past and continue_parking and not parking_flag:

                            print(f"time after reversing heading {time.time() - pink_time} distance_right:{tf_r} distance_left:{tf_l} prev_distance:{prev_distance}")
                            if time.time() - pink_time > 2:
                                tfmini.getTFminiData()
                                tf_r = tfmini.distance_right
                                tf_l = tfmini.distance_left
                                if orange_flag:
                                    print(f"prev_distance: {prev_distance}, distance_right: {tf_r} diff: {abs(prev_distance - tf_r)}")
                                    p_flag = True
                                    if tf_r <= 30 and (abs(prev_distance - tf_r) >= 10 and prev_distance > 0) and not pink_b.value:
                                        p_pass = 2
                                        if p_pass == 2:
                                            p_past = False
                                            p_flag = False
                                            parking_flag = True
                                            print("Pink Avoidance Complete...")
                                    prev_distance = tf_r

                                elif blue_flag:
                                    
                                    print(f"prev_distance: {prev_distance}, distance_left: {tf_l}  diff: {abs(prev_distance - tf_l)}")
                                    if tf_l <= 30 and (abs(prev_distance - tf_l) >= 10 and prev_distance > 0) and not pink_b.value:
                                        p_pass = 2
                                        if p_pass == 2:
                                            p_past = False
                                            p_flag = False
                                            parking_flag = True
                                            print("Pink Avoidance Complete Blue...")
                                    prev_distance = tf_l

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
                        print(f"time after reversing heading {time.time() - pink_time}")
                        print(f"counts {counts.value} flag :{encoder_counter_store} encoder: {encoder_counts_value} encoder updated: {encoder_counts_value + off} ")
                        if encoder_counter_store:
                            g_flag = False
                            r_flag = False
                            g_past = False
                            r_past = False
                            if counts.value > encoder_counts_value + off:
                                encoder_counter_store = False
                                rev_count = 0
                                print("Encoder counts done")

                        if g_flag:
                            print("avoiding green..")
                            correctPosition(setPointL, heading_angle, x, y, counter, blue_flag, orange_flag,
                                            reset_f, reverse, head.value, centr_x_pink.value, centr_x_red.value, centr_x.value, centr_y.value, centr_y_red.value, centr_y_pink.value, finish, tf_h, tf_l, tf_r, red_b.value, green_b.value)
                        elif r_flag:
                            print("avoiding red...")
                            correctPosition(setPointR, heading_angle, x, y, counter, blue_flag, orange_flag,
                                            reset_f, reverse, head.value, centr_x_pink.value, centr_x_red.value, centr_x.value, centr_y.value, centr_y_red.value, centr_y_pink.value, finish, tf_h, tf_l, tf_r, red_b.value, green_b.value)
                        elif p_flag:
                            print("avoiding pink..")
                            if blue_flag:
                                
                                if tf_l < 30:
                                    correctAngle(heading_angle + 10, head.value)
                                else:
                                    correctAngle(heading_angle, head.value)
                            elif orange_flag:
                                if tf_r < 30:
                                    correctAngle(heading_angle - 10, head.value)  
                                else:
                                    correctAngle(heading_angle, head.value)

                            '''if orange_flag:
                                correctPosition(setPointR, heading_angle, x, y, counter, blue_flag, orange_flag,
                                                reset_f, reverse, head.value, centr_x_pink.value, centr_x_red.value, centr_x.value, centr_y.value, centr_y_red.value, centr_y_pink.value, finish, tf_h, tf_l, tf_r, red_b.value, green_b.value)
                            elif blue_flag:
                                correctPosition(setPointL, heading_angle, x, y, counter, blue_flag, orange_flag,
                                                reset_f, reverse, head.value, centr_x_pink.value, centr_x_red.value, centr_x.value, centr_y.value, centr_y_red.value, centr_y_pink.value, finish, tf_h, tf_l, tf_r, red_b.value, green_b.value)'''
                        else:

                            print("Going straight")
                            correctPosition(setPointC, heading_angle, x, y, counter, blue_flag, orange_flag,
                                            reset_f, reverse, head.value, centr_x_pink.value, centr_x_red.value, centr_x.value, centr_y.value, centr_y_red.value, centr_y_pink.value, finish, tf_h, tf_l, tf_r, red_b.value, green_b.value)

                print(f"centr_x.value: {centr_x.value} centr_y.value: {centr_y.value} centr_red: {centr_x_red.value} centr_y_red:{centr_x_red.value} centr_pink: {centr_x_pink.value}")
                print(f"left_b.value:{left_f.value} right_b.value:{right_f.value} orange_flag:{orange_flag} blue_flag:{blue_flag}")
                print(f"trigger:{trigger} turn_trigger: {turn_trigger.value} reset_f:{reset_f} counter: {counter}, imu:{head.value:2f}")
                print(f"red_b.value:{red_b.value} green_b.value:{green_b.value} pink_b.value:{pink_b.value}")
                print(f"r_flag:{r_flag} g_flag:{g_flag} rev_count: {rev_count}")
                print(f"r_past:{r_past} g_past:{g_past} p_past:{p_past} pass_c:{p_pass}")
                print(f"x: {x:.2f}, y:{y:.2f} count:{counts.value} heading_angle:{heading_angle}")
                print(f"F: {tf_h:.2f}  L: {l_left:.2f} R: {l_right:.2f} left:{tf_l} head:{(math.cos(math.radians(abs(corr))) * tfmini.distance_head):.2f} right: {tf_r} POWER = {power} corr: {abs(corr)}")
                print(f"L: {setPointL} R: {setPointR} setPointC: {setPointC} off:{off}")
                print("---------------------------------------------------")
                '''print(f"lap_finish:{lap_finish} counter:{counter} continue_parking:{continue_parking}") 
                print(f"(p_flag:{p_flag} p_past:{p_past} ")
                print(f"parking_flag:{parking_flag}")
                print(f"x pink:{centr_x_pink.value} y pink:{centr_y_pink.value}")'''
                # print(f"color_s:{color_s} color_n:{color_n} centr_y_b.value: {centr_y_b.value} centr_x:{centr_x.value} centr_red: {centr_x_red.value} centr_pink:{centr_x_pink.value} setPointL:{setPointL} setPointR:{setPointR} g_count:{green_count} r_count:{red_count} x: {x}, y: {y} counts: {counts.value}, prev_distance: {prev_distance}, head_d: {tfmini.distance_head} right_d: {tfmini.distance_right}, left_d: {tfmini.distance_left}, back_d:{tfmini.distance_back} imu: {imu_head}, heading: {heading_angle}, cp: {continue_parking}, counter: {counter}, pink_b: {pink_b.value} p_flag = {p_flag}, g_flag: {g_flag} r_flag: {r_flag} p_past: {p_past}, g_past: {g_past}, r_past: {r_past} , red_stored:{red_stored} green_stored:{green_stored}")
            else:
                power = 0
                pwm.hardware_PWM(12, 55, 0)
                heading_angle = 0
                counter = 0
                correctAngle(heading_angle, head.value)
                # red_b.value = False
                # green_b.value = False
                # print("BUUTTTTON ELSE")
            # print(f"button:{button}")

    except Exception as e:
        print(f"Exception: {e}")
        if isinstance(e, KeyboardInterrupt):
            power = 0
            pwm.hardware_PWM(12, 55, 0)
            heading_angle = 0
            counter = 0
            correctAngle(heading_angle, head.value)
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
    time.sleep(2)
    pwm.write(green_led, 1)
    try:
        while True:
            line = ser.readline().decode('utf-8', errors = 'ignore').strip()
            esp_data = line.split()
            # print(f"esp_data: {esp_data}")
            if len(esp_data) >= 2:
                try:
                    head.value = float(esp_data[0])
                    imu_shared.value = head.value
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


def read_lidar(lidar_angle, lidar_distance, imu_shared, sp_angle, turn_trigger, specific_angle, lidar_f, lidar_l, lidar_r, stop_evt, left_f, right_f):
    # print("This is first line")
    global CalledProcessError
    pwm = pigpio.pi()
    trig_time = 0
    previous_angle = 0
    F = 0
    L = 0
    R = 0
    prev_sp = 0
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
                if previous_angle is None:
                    previous_angle = angle
                # print(f"📍 Angle: {angle:.2f}°, Distance: {distance:.2f} mm")
            except Exception as e:
                print("⚠️ Parse error:", e)
        else:
            print("ℹ️", line)
        '''sp_angle.value = sp_angle.value % 360
        if sp_angle.value > 180:
            sp_angle.value = sp_angle.value - 360''' 
        if previous_angle != angle:
            if prev_sp != sp_angle.value:
                sp_angle.value = 360 - sp_angle.value
            prev_sp = sp_angle.value
            while (abs(angle - previous_angle) > 1):
                lidar_angle.value = (previous_angle + 1) % 360
                lidar_distance.value = previous_distance
                previous_angle = lidar_angle.value
                rplidar[int(lidar_angle.value)] = lidar_distance.value
                if (int(lidar_angle.value) == (0 + imu_r + sp_angle.value) % 360):
                    lidar_front = lidar_distance.value
                    F = 0.2*F + 0.8*lidar_distance.value if F else lidar_distance.value
                    lidar_f.value = F
                if (int(lidar_angle.value) == (90 + imu_r + sp_angle.value) % 360):
                    lidar_left = lidar_distance.value
                    L = 0.2*L + 0.8*lidar_distance.value if F else lidar_distance.value
                    lidar_l.value = L

                if (int(lidar_angle.value) == (270 + imu_r + sp_angle.value) % 360):
                    lidar_right = lidar_distance.value
                    R = 0.2*R + 0.8*lidar_distance.value if F else lidar_distance.value
                    lidar_r.value = R
                
                #print("in while loop...")    
                if (F <= 850 and R >= 1500) and right_f.value and not left_f.value:
                    turn_trigger.value = True
                elif (F <= 850 and L >= 1500) and left_f.value and not right_f.value:
                    turn_trigger.value = True
                else:
                    turn_trigger.value = False                    
                #print(f"front: {F}. right:{R} left:{L}  turn_trigger:{turn_trigger.value} imu:{imu_r} sp_angle: {sp_angle.value} right_f.value:{right_f.value} left_f.value:{left_f.value}")
                
                # print(f"front: {lidar_front}. right:{lidar_right} left:{lidar_left} ")

            if (distance != 0):
                with lidar_angle.get_lock(), lidar_distance.get_lock(), imu_shared.get_lock():
                    lidar_angle.value = angle
                    lidar_distance.value = distance
                    previous_distance = distance
                    previous_angle = angle
                    rplidar[int(lidar_angle.value)] = lidar_distance.value
                    if (int(lidar_angle.value) == (0 + imu_r + sp_angle.value) % 360):
                        lidar_front = lidar_distance.value
                        F = 0.2*F + 0.8*lidar_distance.value if F else lidar_distance.value
                        lidar_f.value = F

                    if (int(lidar_angle.value) == (90 + imu_r + sp_angle.value) % 360):
                        lidar_left = lidar_distance.value
                        L = 0.2*L + 0.8*lidar_distance.value if F else lidar_distance.value
                        lidar_l.value = L
                    if (int(lidar_angle.value) == (270 + imu_r + sp_angle.value) % 360):
                        lidar_right = lidar_distance.value
                        R = 0.2*R + 0.8*lidar_distance.value if F else lidar_distance.value
                        lidar_r.value = R
                        
                    # print(f"angles: {specific_angle}, imu: {imu_shared.value} total:{imu_r + lidar_angle.value}")

                    if (F <= 850 and R >= 1500) and right_f.value and not left_f.value:
                        turn_trigger.value = True
                    elif (F <= 850 and L >= 1500) and left_f.value and not right_f.value:
                        turn_trigger.value = True
                    else:
                        turn_trigger.value = False  

                    #print(f"front: {F}. right:{R} left:{L}  turn_trigger:{turn_trigger.value} imu:{imu_r} sp_angle: {sp_angle.value} right_f.value:{right_f.value} left_f.value:{left_f.value}")
            # print(f"front: {lidar_front}. right:{lidar_right} left:{lidar_left}  turn_trigger:{turn_trigger.value} diff:{time.time() - trig_time}  imu:{imu_r} sp_angle: {sp_angle.value}")
            # print(f"angle: {lidar_angle.value} distance:{rplidar[int(lidar_angle.value)]}")


if __name__ == '__main__':
    try:
        print("Starting process")

        P = multiprocessing.Process(target=Live_Feed, args=(color_b, stop_evt, red_b, green_b, pink_b, centr_y,
                                    centr_x, centr_y_red, centr_x_red, centr_x_pink, centr_y_pink, centr_y_b, orange_o, centr_y_o, shared_lock))
        S = multiprocessing.Process(target=servoDrive, args=(color_b, stop_evt, red_b, green_b, pink_b, counts, centr_y, centr_x, centr_y_red,
                                    centr_x_red, centr_x_pink, centr_y_pink, head, centr_y_b, orange_o, centr_y_o,  sp_angle, turn_trigger, specific_angle, imu_shared, lidar_f, lidar_l, lidar_r, shared_lock, left_f, right_f))
        E = multiprocessing.Process(target=runEncoder, args=(counts, head, imu_shared, sp_angle))
        lidar_proc = multiprocessing.Process(target=read_lidar, args=(lidar_angle, lidar_distance, imu_shared, sp_angle, turn_trigger, specific_angle, lidar_f, lidar_l, lidar_r, stop_evt, left_f, right_f))

        # Launch the lidar reader process

        # C = multiprocessing.Process(target=color_SP, args=(blue_c, orange_c, white_c))

        print("Image Process Start")        
        P.start()
        print("Image Process Started")
        E.start()
        print("Starting lidar process")
        lidar_proc.start()
        print("lidar process startes")

        print("Servo Process Start")
        S.start()
        print("Servo Process Started")
        print("Encoder Process Start")

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
        pwm.hardware_PWM(12, 55, 0)
        pwm.bb_serial_read_close(RX_Head)
        pwm.bb_serial_read_close(RX_Left)
        pwm.bb_serial_read_close(RX_Right)
        pwm.stop()
        tfmini.close()