from TFmini import TFmini
import RPi.GPIO as GPIO
import serial
from Servo import Servo
#from BNO085 import IMUandColorSensor
from Encoder import EncoderCounter
import math
#import board
import pigpio
import multiprocessing
from ctypes import c_float
#from picamera2 import Picamera2
import logging
import sys
import os
import time
import subprocess
import cv2, time, numpy as np
from pycoral.utils.edgetpu import make_interpreter
from pycoral.adapters import common, detect
from pycoral.utils.dataset import read_label_file
from itertools import combinations
os.system('sudo pkill pigpiod')
os.system('sudo pigpiod')
time.sleep(2)


# import RPi.GPIO as GPIO
# import time
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
#imu = IMUandColorSensor(board.SCL, board.SDA)
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
previous_angle = multiprocessing.Value('d', 0.0)
shared_lock = multiprocessing.Lock()

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


def correctPosition(setPoint, head, x, y, counter, blue, orange, reset, reverse, heading, centr_x_p, centr_y_g, centr_y_r, centr_y_p, finish, distance_h, distance_l, distance_r):
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
        print(
            f"lane: {lane}, error: {error} target:{(setPoint)}, x:{x} y:{y} not reverse")
    elif lane == 1:
        if orange:
            error = x - (100 - setPoint)
            print(
                f"lane:{lane}, error:{error} target:{(100 - setPoint)}, x:{x}, y:{y}")

        elif blue:
            error = (100 + setPoint) - x
            # print(f"lane:{lane}, error:{error} target:{(55 + setPoint)}, x:{x} y:{y} Bluee")
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
            # print(f"lane:{lane} error:{error} target:{(55 + setPoint)}, x:{x} y {y}")

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
        tfmini.getTFminiData()
        if (((setPoint == -35 ) and orange) or (counter == 0 and (centr_x_p < 800 and centr_x_p > 0) and ((centr_y_g or centr_y_r) <= centr_y_p) and not blue and not orange) and not finish):
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

        elif (((setPoint == 35 ) and blue) or (counter == 0 and (centr_x_p < 800 and centr_x_p > 0) and ((centr_y_g or centr_y_r) <= centr_y_p) and not blue and not orange) and not finish):
            
            if distance_r <= 30:
                correction = -20
                print(f"Avoiding pink wall {correction}")

            elif distance_l < 50 or distance_l > 55:
                if distance_l <= 35:
                    correction = 45
                    print(f"Avoiding pink wall {correction}")

                else:
                    correction = 10
                    print(f"Avoiding pink wall {correction}")
            else:
                correction = 0

        if not blue:
            if (setPoint <= -70 ) and distance_l <= 20:
                print(f"Correcting Green Wall Orange")
                correction = 10
            else:
                pass

            if (setPoint >= 70 ) and (distance_r <= 20 or (tfmini.distance_head <= 20)):
                print(f"Correcting Red Wall...")
                correction = -10
            else:
                pass

        else:
            if (setPoint <= -70 or setPoint == 0) and (distance_l <= 20 or (tfmini.distance_head <= 20)) :
                print(f"Correcting Green Wall Blue")
                correction = 10
            else:
                pass

            if (setPoint >= 70 or setPoint == 0) and distance_r <= 20:
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

    if correction > 40:
        correction = 40
    elif correction < -40:
        correction = -40

    prevErrorGyro = error_gyro
    servo.setAngle(90 - correction)

_running = True
def _graceful_stop(signum, frame):
    global _running
    _running = False
    # optional: print so you see it fired
    print(f"\n🔔 Worker got signal {signum}, stopping...")

# loop to capture video frames
def Live_Feed(color_b, stop_evt, red_b, green_b, pink_b, centr_y, centr_x, centr_y_red, centr_x_red, centr_x_pink, centr_y_pink, centr_y_b, orange_o, centr_y_o, shared_lock):
    MODEL_PATH = "/home/pi/WRO_2025_PI/limelight_neural_detector_8bit.tflite"
    LABELS     = "/home/pi/WRO_2025_PI/label_map.txt"   # put your label file here (id -> name), or set to None
    CONF_TH    = 0.5
    CAM_INDEX  = 0

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

    cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)   # low-latency
    cls_name = None
    t_prev = time.time()
    pairs = []
    dets =[]
    x1 = x2 = y1 =  y2 = cx = cy = 0
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
                cx = x1 + x2//2
                cy = y1 + y2//2
                area = max(0, (x2 - x1)) * max(0, (y2 - y1))
                name = labels.get(obj.id, str(obj.id))
                if area >= 2500:
                    det.append((name, cx, cy, area))
            dets.sort(key=lambda d: d[3], reverse=True)

            pair=[]
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
                if n1 == 'pink':
                    centr_x_pink.value = pair[0][1]
                    centr_y_pink.value = pair[0][2]
            else:
                pink_b.value = False

            if (n1, n2) in (('pink', 'red'), ('red', 'pink')):
                red_b.value = True
                green_b.value = False
                pink_b.value = True
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
                    centr_x_red.value = pair[1][1]
                    centr_y_red.value = pair[1][2] 
            elif (n1, n2) in (('pink', 'green'), ('green', 'pink')):
                green_b.value = True
                red_b.value = False
                pink_b.value = True
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
            elif (n1, n2) in (('green', 'red'), ('green', None)):
                green_b.value = True
                red_b.value = False
                pink_b.value = False
                if n1 == 'green':
                    centr_x.value = pair[0][1]
                    centr_y.value = pair[0][2]
                elif n2 == 'green':
                    centr_x.value = pair[1][1]
                    centr_y.value = pair[1][2]
            elif (n1, n2) in (('red', 'green'), ('red', None)):
                red_b.value = True
                green_b.value = False
                pink_b.value = False
                if n1 == 'red':
                    centr_x_red.value = pair[0][1]
                    centr_y_red.value = pair[0][2]
                elif n2 == 'red':
                    centr_x_red.value = pair[1][1]
                    centr_y_red.value = pair[1][2]
                      
            elif n1 is None and n2 is None :
                red_b.value = False
                green_b.value = False
                pink_b.value = False
                centr_x_pink.value = 0
                centr_y_pink.value = 0
                centr_x.value = 0
                centr_y.value = 0
            now = time.time()
            fps = 1.0 / max(1e-3, (now - t_prev)); t_prev = now
            print(f"pairs:{pair} red_b.value: {red_b.value} green_b.value:{green_b.value} pink_b:{pink_b.value} cx:{cx} cy:{cy} fps:{fps}")
            #cv2.imshow("Coral SSD Live", frame_bgr)
            '''if cv2.waitKey(1) & 0xFF == 27:  # ESC
                break'''
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        cv2.destroyAllWindows()


def servoDrive(color_b, stop_evt, red_b, green_b, pink_b, counts, centr_y, centr_x, centr_y_red, centr_x_red, centr_x_pink, centr_y_pink, head, centr_y_b,  orange_o, centr_y_o, sp_angle, turn_trigger, specific_angle, imu_shared, lidar_f, lidar_l, shared_lock):
    pwm = pigpio.pi()
    global imu, corr, corr_pos

    pb_time = 0
    pwm_pin = 12
    direction_pin = 20

    pwm.set_mode(pwm_pin, pigpio.OUTPUT)  # Set pin 12 as an output
    pwm.set_mode(direction_pin, pigpio.OUTPUT)  # Set pin 20 as an output
    pwm.hardware_PWM(pwm_pin, 55, 0)

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
    power = 70
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
    avoided = False
    avoided_time = time.time()
    reverse_until = 0
    encoder_counter_store = False
    encoder_counts_value = 0
    l_left = 0
    off = 4000
    rev_count = 0
    try:
        while True:
            imu_shared.value = head.value
            imu_head = head.value
            if not button:
                print(f"red_b:{red_b.value}, green_b:{green_b.value}, pink_b:{pink_b.value}")
                print(f"centr_X:{centr_x_pink.value} centr_y:{centr_y_pink.value}")
            # print(f"angles:{specific_angle}")
            # print(f"fps 2222:{1/(time.time() - fps_time2)}")
            fps_time2 = time.time()


            tfmini.getTFminiData()
            tf_h = lidar_f.value
            l_left = lidar_l.value
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
                    power = 70
            ##### STOP CONDITION ######
            # print(f"rgb:{imu.color_rgb} color:{color_sensor}")
            # print(f"pink detected:{pink_detected}")
            if counter == last_counter and not lap_finish:
                print(
                    f"centr_y :{centr_y.value} centr_y_red:{centr_y_red.value}")
                print("REACHED MAXIMUM COUNTS")
                print(f"target:{target_count}")
                if not finished:
                    target_count = counts.value + 17500
                    finished = True
                if counts.value >= target_count and not reverse_trigger:
                    power = 0
            
                    # Set duty cycle to 50% (128/255)
                    pwm.set_PWM_dutycycle(pwm_pin, power)
                    time.sleep(3)
                    power = 70
                    prev_power = 0
                    lap_finish = True
                    reverse_trigger = True
                    print(
                        f"Vehicle is stopped...")

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
                print("Inside Continue parking")
                green_b.value = False
                red_b.value = False
                g_past = False
                r_past = False
                g_flag = False
                r_flag = False
                if orange_flag: # and (((centr_x_pink.value < centr_x.value) and (centr_x.value > 0 and centr_x_pink.value > 0)) or (centr_x_pink.value < centr_x_red.value and (centr_x_red.value > 0 and centr_x_pink.value > 0))):
                    setPointR = -35
                    setPointC = -35
                    finish = True
                elif blue_flag: # and (((centr_x_pink.value > centr_x.value) and (centr_x.value > 0 and centr_x_pink.value > 0)) or (centr_x_pink.value > centr_x_red.value and (centr_x_red.value > 0 and centr_x_pink.value > 0))):
                    setPointL = 35
                    setPointC = 35
                    finish = True

            if pink_b.value:  # DECIDES SETPOINT WHENEVER PINK IS IN THE FRAME
                print(f"PINK IS DETECTED...")
                if orange_flag:
                    if (centr_x_pink.value < centr_x.value) and (centr_x_pink.value > 0 and centr_x.value > 0) and not continue_parking:
                        setPointL = -40
                        setPointR = 70
                        print(f"setPointL : {setPointL}")
                        # print(f"setPointL: {setPointL}")
                elif blue_flag:
                    if (centr_x_red.value < centr_x_pink.value) and (centr_x_pink.value > 0 and centr_x_red.value > 0) and not continue_parking:
                        setPointR = 40
                        setPointL = -70
                        print(f"setPointR: {setPointR}")

                elif counter % 4 == 0 and not blue_flag and not orange_flag:
                    if ((centr_x_pink.value < 400 and centr_x_pink.value > 0) and ((centr_x_red.value) > centr_x_pink.value)) and not continue_parking:
                        setPointR = 40
                        setPointL = -70
                        print(f"at 0 counter orange:{setPointR} {setPointL}")

                    if ((centr_x_pink.value > 800) and ((centr_y.value or centr_y_red.value) <= centr_y_pink.value)) and not continue_parking:
                        setPointL = -40
                        setPointR = 70
                        print(f"at 0 counter blue:{setPointR} {setPointL}")


                if lap_finish and not continue_parking:
                    pink_detected = False
                    print("Starting Parking...")
                    continue_parking = True
                pb_time = time.time()
            # IF DOES NOT SEE PINK, KEEP THE SAME SETPOINT FOR 1 SECOND AND THEN CHANGE
            elif not pink_b.value and time.time() - pb_time > 1 and not lap_finish:
                #print(f"Resetting setPoints...{pink_detected}")
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
            else:
                setPointR = 70
                setPointL = -70
            ##########
            if button:  # THIS BLOCK OF CODE WHEN BUTTON IS PRESSED
                # time.sleep(0.01)

                if red_b.value or green_b.value:
                    power = 50
                else:
                    power = 65


                if time.time() < avoided_time  :
                    pwm.set_PWM_dutycycle(pwm_pin, 0)
                    pwm.write(direction_pin, 0)
                    #correctAngle(heading_angle, imu_head)  # still steer if needed
                    print("block spotted")
                    continue  # skip the drive code below   

                if avoided_time > 0 and time.time() < reverse_until :
                    # non-blocking reverse
                    power = 65
                    prev_power = 0
                    pwm.set_PWM_dutycycle(pwm_pin, int(1.3 * power))
                    pwm.write(direction_pin, 0)  # 0 = reverse, 1 = forward (per your wiring)
                    # keep the robot straight while reversing (or add bias if you want to angle out)
                    #correctAngle(sp_angle.value, imu_head)
                    print("reverse after block spotted")
                    servo.setAngle(90)
                    continue  # still skip forward-drive code

                                

                if not reset_servo:
                    time.sleep(0.5)
                    servo.setAngle(130)
                    time.sleep(0.5)
                    servo.setAngle(90)
                    reset_servo = True

                x, y = enc.get_position(imu_head, counts.value)

                total_power = (power * 0.1) + (prev_power * 0.9)
                prev_power = total_power
                # Set duty cycle to 50% (128/255)
                pwm.set_PWM_dutycycle(pwm_pin, 2.55 * total_power)

                pwm.write(direction_pin, 1)  # Set pin 20 high


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
                        power = 70
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
                        #correctAngle(heading_angle, head.value)
                    back_bot = False
                    stop_flag = False
                    if (abs(corr) < 15) and (tf_h <= 500 and tf_h >= 0) and not finish_flag:
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
                        print("RESET FLAG")


                        setPointL = -70
                        setPointR = 70
                        setPointC = 0
                        g_past = False
                        servo.setAngle(90)
                        if blue_flag:  # BLUE RESET BLOCK

                            print(f"BLUE RESET")

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
                                elif (tf_h < 300 and time.time() - g_time > 0.8) and not red_b.value and not pink_b.value:
                                    red_turn = False
                                    reverse_trigger = False
                                    red_time = True
                                correctPosition(setPointC, heading_angle, x, y, counter, blue_flag, orange_flag,
                                                reset_f, reverse, head.value, centr_x_pink.value, centr_y.value, centr_y_red.value, centr_y_pink.value, finish, tf_h, tf_l, tf_r)

                            else:
                                if red_time:
                                    time_g = 1

                                else:
                                    time_g = 0.5
                                print(f"time_g:{time_g}")
                                if not timer_started:
                                    current_time = time.time()
                                    timer_started = True

                                if not green_b.value and not red_b.value:
                                    print('reversing diection red')
                                    while (time.time() - current_time < time_g):
                                        servo.setAngle(70)
                                        x, y = enc.get_position(
                                            imu_head, counts.value)
                                        power = 70
                                        prev_power = 70
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
                                        #correctAngle(heading_angle, head.value)
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
                                        power = 70
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

                                print(f"front: {tf_h}")
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
                                power = 70
                                heading_angle = -((90 * counter) % 360)
                                green_turn = False
                                red_turn = False
                                reset_f = False
                                red_time = False
                                r_flag = False
                                r_past = False

                        if orange_flag:  # ORANGE RESET BLOCK
                            print(
                                f"ORANGE RESET...setPoint C:{tf_h}")
                            if  (not green_b.value) and not red_b.value and not not_block:
                                print("In first block")
                                not_block = False
                                if tf_h < 500 and (math.cos(math.radians(abs(corr))) * tfmini.distance_head) < 50:
                                    not_block = True
                            elif (green_b.value or green_turn) or (reverse_trigger):  # green after trigger
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
                                elif tf_h < 300 and (math.cos(math.radians(abs(corr))) * tfmini.distance_head) < 30 and not green_b.value and not pink_b.value:
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
                                    time_g = 0.8
                                print(f"time_g:{time_g}")

                                if not timer_started:
                                    current_time = time.time()
                                    timer_started = True

                                if not red_b.value and not r_past:
                                    if tf_h < 500 and (math.cos(math.radians(abs(corr))) * tfmini.distance_head) < 50:
                                        print('reversing diection green')
                                        while time.time() - current_time < time_g:
                                            servo.setAngle(70)
                                            x, y = enc.get_position(
                                                imu_head, counts.value)
                                            power = 70
                                            prev_power = 65
                                            # Set duty cycle to 50% (128/255)
                                            pwm.set_PWM_dutycycle(pwm_pin, power)
                                            # Set pin 20 hig
                                            pwm.write(direction_pin, 0)
                                        print('reversing diection green complete')
                                        print('Stopping Motor...')
                                        timer_started = False

                                elif (red_b.value or red_turn) and r_past :
                                    print('Stopping Motor... RED IS SEEN')
                                    red_turn = True
                                    print(f'reversing diection red pink color: {pink_b.value} pink flag: {p_flag} {p_past}  red color:{red_b.value} red flag: {r_past} {r_flag}')
                                    while 1:
                                        print("RED IS SEEN..")
                                        servo.setAngle(80)
                                        print(f"centr y: {centr_y_red.value}")
                                        if (centr_y_red.value < 500 and centr_y_red.value > 0):
                                            print(f"Breaking the loop...")
                                            break
                                        elif (time.time() - current_time > 1.4):
                                            print("Green is not there breaking the loop...")
                                            break
                                        # getTFminiData()
                                        x, y = enc.get_position(imu_head, counts.value)
                                        # print(f"x: {x}, y: {y},  count:{counts.value} distance_head : {distance_head}")
                                        power = 70
                                        prev_power = 0
                                        # Set duty cycle to 50% (128/255)
                                        pwm.set_PWM_dutycycle(pwm_pin, power)
                                        # Set pin 20 hig
                                        pwm.write(direction_pin, 0)

                                    print('red reversing diection complete')
                                    
                                    timer_started = False
                                print('Stopping Motor...')

                                power = 0
                                prev_power = 0
                                # Set duty cycle to 50% (128/255)
                                pwm.set_PWM_dutycycle(pwm_pin, power)

                                # getTFminiData()
                                x, y = enc.get_position(imu_head, counts.value)
                                buff = 4
                                time.sleep(0.8)
                                counter = counter + 1
                                c_time = time.time()
                                lane_reset = counter % 4
                                turn_trigger_distance = tf_h

                                if lane_reset == 1:
                                    enc.x = (
                                        1500 - (turn_trigger_distance ))/10
                                    print(f"x: {enc.x}")
                                if lane_reset == 2:
                                    enc.y = (
                                        2500 - (turn_trigger_distance))/10 
                                if lane_reset == 3:
                                    enc.x = ((turn_trigger_distance) - 1500)/10 
                                if lane_reset == 0:
                                    enc.y = ((turn_trigger_distance) - 500)/10 
                                #print(f'Resuming Motor...{offset}')

                                power = 70
                                heading_angle = ((90 * counter) % 360)
                                sp_angle.value = heading_angle
                                red_turn = False
                                green_time = False
                                reset_f = False
                                g_flag = False
                                g_past = False
                                avoided = False
                                # centr_y_b.value = 0

                    else:                      
                        if tfmini.distance_head < 10:
                            prev_restore = time.time()
                            print(f"counter: {counter} Trigger detected...")
                            power = 60
                            prev_power = 0
                            while time.time() - prev_restore < 2:
                                tfmini.getTFminiData()
                                servo.setAngle(70)
                                pwm.set_PWM_dutycycle(pwm_pin, power)  # Set duty cycle to 50% (128/255)
                                pwm.write(direction_pin, 0)  # Set pin 20 hig
                                any_color = True
                        else:
                            blue_on = False

                        # TRIGGGER CHECK VALUESSSS
                        if (turn_trigger.value and not trigger) and (time.time() - turn_t) > 4 + buff and tfmini.distance_head < 55:
                            buff = 0
                            trigger = True
                            reset_f = True
                            timer_started = False
                            turn_t = time.time()
                        elif not turn_trigger.value:
                            trigger = False
                            pwm.write(blue_led, 0)

                        prev_blue = centr_y_b.value
                        prev_orange = centr_y_o.value



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

                        '''if green_b.value and not r_flag and not continue_parking and not g_flag:
                            print(f"centr x: {centr_x.value} centr y: {centr_y.value}")
                            g_flag = True
                            g_past = True
                            avoided_time = time.time() + 0.5
                            reverse_until = avoided_time + 0.7
                            pwm.write(red_led, 0)
                            pwm.write(green_led, 0)
                            print('1')


                        elif (g_past or time.time() - gp_time < 0.5) and not continue_parking:
                            print("Avoiding green...")
                            g_flag = True
                            if (tf_r <= 50 and not green_b.value) and tf_l <= 25 and tf_h >= 1000:
                                print("Green Avoid Complete")
                                g_past = False
                                g_flag = False
                                pwm.write(red_led, 0)
                                pwm.write(green_led, 1)
                                gp_time = time.time()

                            print('2')

                        elif red_b.value and not g_flag and not continue_parking and not r_flag:
                            r_flag = True
                            r_past = True
                            avoided_time = time.time() + 0.5
                            reverse_until = avoided_time + 0.7
                            #print(f"x cent:{centr_x_red.value} centr y:{centr_y_red.value}")

                            pwm.write(red_led, 0)
                            pwm.write(green_led, 0)

                            print('3')

                        elif (r_past or time.time() - rp_time < 0.5) and not continue_parking:
                            print("Avoiding red...")
                            r_flag = True
                            # and ((avg_right_pass < 50 or avg_right_pass > 120) or counter!=rev_counter):
                            if tf_r <= 25:
                                if tf_l <= 50 and not red_b.value:
                                    print(f"red Avoid complete")
                                    r_past = False
                                    r_flag = False
                                    red_stored = False
                                    pwm.write(red_led, 1)
                                    pwm.write(green_led, 0)
                                    rp_time = time.time()
                                    print("red block saved")
                            else:
                                if tf_l <= 50 and not red_b.value and tf_h> 1500:
                                    print(f"red Avoid complete")
                                    r_past = False
                                    r_flag = False
                                    red_stored = False
                                    pwm.write(red_led, 1)
                                    pwm.write(green_led, 0)
                                    rp_time = time.time()
                                    print("red block saved")
                            print('4')

                        elif pink_b.value and continue_parking and not p_flag:
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
                            pwm.write(green_led, 0)'''

                        if green_b.value and not r_flag and not continue_parking and not g_flag :
                            print(f"centr x: {centr_x.value} centr y: {centr_y.value}")

                            if not trigger and rev_count <1:
                                avoided_time = time.time() + 0.5
                                reverse_until = avoided_time + 0.7
                                rev_count +=1
                            g_flag = True
                            g_past = True
                            pwm.write(red_led, 0)
                            pwm.write(green_led, 0)
                            print('1')


                        elif (g_past) and not continue_parking  :
                            print("Avoiding green...")
                            g_flag = True
                            if tf_r <= 50:
                                if not green_b.value and not encoder_counter_store:
                                    encoder_counts_value = counts.value
                                    encoder_counter_store = True
                                    print("encoder counts are stored for green")

                            print('2')

                        elif red_b.value and not g_flag and not continue_parking and not r_flag :
                            r_flag = True
                            r_past = True
                            if not trigger and rev_count < 1:
                                avoided_time = time.time() + 0.5
                                reverse_until = avoided_time + 0.7
                                rev_count += 1

                            #print(f"x cent:{centr_x_red.value} centr y:{centr_y_red.value}")
                            print("Red detected")
                            pwm.write(red_led, 0)
                            pwm.write(green_led, 0)

                            print('3')

                        elif (r_past) and not continue_parking :
                            print("Avoiding red...")
                            r_flag = True
                            if tf_l <= 50:
                                if not red_b.value and not encoder_counter_store:
                                    encoder_counts_value = counts.value
                                    encoder_counter_store = True
                                    print("encoder counts stored for red")
                            # and ((avg_right_pass < 50 or avg_right_pass > 120) or counter!=rev_counter):
                            print('4')

                        elif pink_b.value and continue_parking and not p_flag:
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
                        print(f"counts {counts.value} encoder: {encoder_counts_value} encoder updated: {encoder_counts_value + off} ")
                        if encoder_counter_store and (g_past or r_past):
                            if counts.value > encoder_counts_value + off:
                                g_past = False
                                r_past = False
                                g_flag = False
                                r_flag = False
                                rev_count = 0
                                encoder_counter_store = False
                                print("Encoder counts done")

                        if g_flag :
                            print("avoiding green..")
                            correctPosition(setPointL, heading_angle, x, y, counter, blue_flag, orange_flag,
                                            reset_f, reverse, head.value, centr_x_pink.value, centr_y.value, centr_y_red.value, centr_y_pink.value, finish, tf_h, tf_l, tf_r)
                        elif r_flag:
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


                print(f"trigger:{trigger} turn_trigger: {turn_trigger.value} reset_f:{reset_f} counter: {counter}, imu:{head.value}")
                print(f"red_b.value:{red_b.value} green_b.value:{green_b.value} pink_b.value:{pink_b.value}")
                print(f"r_flag:{r_flag} g_flag:{g_flag}")
                print(f"r_past:{r_past} g_past:{g_past} p_past:{p_past}")
                print(f"x: {x}, y:{y} count:{counts.value} heading_angle:{heading_angle}")
                print(f"tf_h :{tf_h} {l_left} left:{tf_l} head:{(math.cos(math.radians(abs(corr))) * tfmini.distance_head)} right: {tf_r} POWER = {power}")
                print(f"L: {setPointL} R: {setPointR} setPointC: {setPointC}")
                # print(f"color_s:{color_s} color_n:{color_n} centr_y_b.value: {centr_y_b.value} centr_x:{centr_x.value} centr_red: {centr_x_red.value} centr_pink:{centr_x_pink.value} setPointL:{setPointL} setPointR:{setPointR} g_count:{green_count} r_count:{red_count} x: {x}, y: {y} counts: {counts.value}, prev_distance: {prev_distance}, head_d: {tfmini.distance_head} right_d: {tfmini.distance_right}, left_d: {tfmini.distance_left}, back_d:{tfmini.distance_back} imu: {imu_head}, heading: {heading_angle}, cp: {continue_parking}, counter: {counter}, pink_b: {pink_b.value} p_flag = {p_flag}, g_flag: {g_flag} r_flag: {r_flag} p_past: {p_past}, g_past: {g_past}, r_past: {r_past} , red_stored:{red_stored} green_stored:{green_stored}")
            else:
                power = 0
                pwm.hardware_PWM(12, 55, 0)
                heading_angle = 0
                counter = 0
                correctAngle(heading_angle, head.value)
                #red_b.value = False
                #green_b.value = False
                #print("BUUTTTTON ELSE")
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

    try:
        while True:

            line = ser.readline().decode().strip()
            esp_data = line.split(" ")
            #print(f"esp_data: {esp_data}")
            esp_data.append(1)
            if abs(esp_data[-1]) >= 1:
                try:

                    head.value = float(esp_data[0])
                    imu_shared.value = head.value
                    #print(f"head value:{head.value}")
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


def read_lidar(lidar_angle, lidar_distance, imu_shared, sp_angle, turn_trigger, specific_angle, lidar_f, lidar_l, stop_evt):
    # print("This is first line")
    global CalledProcessError
    pwm = pigpio.pi()
    trig_time = 0
    previous_angle = 0
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
                
                # print(f"📍 Angle: {angle:.2f}°, Distance: {distance:.2f} mm")
            except Exception as e:
                print("⚠️ Parse error:", e)
        else:
            print("ℹ️", line)
        sp_angle.value = 360 - sp_angle.value
        if previous_angle != angle:
            while (abs(angle - previous_angle) > 1):
                lidar_angle.value = (previous_angle + 1) % 360
                lidar_distance.value = previous_distance
                #previous_angle = previous_angle + 1
                previous_angle = lidar_angle.value
                rplidar[int(lidar_angle.value)] = lidar_distance.value
                if (int(lidar_angle.value) == (0 + imu_r + sp_angle.value) % 360):
                    specific_angle[0] = lidar_distance.value
                    lidar_front = lidar_distance.value
                    lidar_f.value = lidar_front
                if (int(lidar_angle.value) == (90 + imu_r + sp_angle.value) % 360):
                    specific_angle[1] = lidar_distance.value
                    lidar_left = lidar_distance.value
                    lidar_l.value = lidar_left

                if (int(lidar_angle.value) == (270 + imu_r + sp_angle.value) % 360):
                    specific_angle[2] = lidar_distance.value
                    lidar_right = lidar_distance.value
                    
                if (lidar_front < 650 and lidar_right > 1800 and lidar_left < 1000) :
                    turn_trigger.value = True                    
                else:
                    turn_trigger.value = False
                '''if (lidar_front < 750 and lidar_right > 1800 and lidar_left < 1000) and not turn_trigger.value:
                    turn_trigger.value = True
                    trig_time = time.time()
                elif time.time() - trig_time > 4:
                    turn_trigger.value = False
                print(f"front: {lidar_front}. right:{lidar_right} left:{lidar_left} sp_angle:{sp_angle.value}, turn_trigger:{turn_trigger.value} diff: {time.time() - trig_time} ")'''
                #print(f"front: {lidar_front}. right:{lidar_right} left:{lidar_left} ")

                #print(f"angles: {specific_angle} imu: {imu_shared.value} total:{imu_r + lidar_angle.value} sp_angle:{sp_angle.value}")

            if (distance != 0):
                with lidar_angle.get_lock(), lidar_distance.get_lock(), imu_shared.get_lock():
                    lidar_angle.value = angle
                    lidar_distance.value = distance
                    previous_distance = distance
                    previous_angle = angle
                    rplidar[int(lidar_angle.value)] = lidar_distance.value
                    if (int(lidar_angle.value) == (0 + imu_r + sp_angle.value) % 360):
                        specific_angle[0] = lidar_distance.value
                        lidar_front = lidar_distance.value
                        lidar_f.value = lidar_front

                    if (int(lidar_angle.value) == (90 + imu_r + sp_angle.value) % 360):
                        specific_angle[1] = lidar_distance.value
                        lidar_left = lidar_distance.value
                        lidar_l.value = lidar_left
                    if (int(lidar_angle.value) == (270 + imu_r + sp_angle.value) % 360):
                        specific_angle[2] = lidar_distance.value
                        lidar_right = lidar_distance.value
                    # print(f"angles: {specific_angle}, imu: {imu_shared.value} total:{imu_r + lidar_angle.value}")
 
                    if (lidar_front < 650 and lidar_right > 1800 and lidar_left < 1000) :
                        turn_trigger.value = True      
                    else:
                        turn_trigger.value = False
                        

            #print(f"front: {lidar_front}. right:{lidar_right} left:{lidar_left}  turn_trigger:{turn_trigger.value} diff:{time.time() - trig_time}  imu:{imu_r} sp_angle: {sp_angle.value}")
            # print(f"angle: {lidar_angle.value} distance:{rplidar[int(lidar_angle.value)]}")


if __name__ == '__main__':
    try:
        print("Starting process")

        P = multiprocessing.Process(target=Live_Feed, args=(color_b, stop_evt, red_b, green_b, pink_b, centr_y,
                                    centr_x, centr_y_red, centr_x_red, centr_x_pink, centr_y_pink, centr_y_b, orange_o, centr_y_o, shared_lock))
        S = multiprocessing.Process(target=servoDrive, args=(color_b, stop_evt, red_b, green_b, pink_b, counts, centr_y, centr_x, centr_y_red,
                                    centr_x_red, centr_x_pink, centr_y_pink, head, centr_y_b, orange_o, centr_y_o,  sp_angle, turn_trigger, specific_angle, imu_shared, lidar_f, lidar_l, shared_lock))
        E = multiprocessing.Process(target=runEncoder, args=(counts, head, imu_shared, sp_angle))
        lidar_proc = multiprocessing.Process(target=read_lidar, args=(
            lidar_angle, lidar_distance, imu_shared, sp_angle, turn_trigger, specific_angle, lidar_f, lidar_l, stop_evt))

        # Launch the lidar reader process

        # C = multiprocessing.Process(target=color_SP, args=(blue_c, orange_c, white_c))

        print("Image Process Start")
        #time.sleep(3)
        P.start()
        #time.sleep(3)
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