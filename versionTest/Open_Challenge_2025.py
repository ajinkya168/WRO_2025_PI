import os

os.system("sudo pkill pigpiod")
os.system("sudo pigpiod")

import RPi.GPIO as GPIO
import time
import multiprocessing
import pigpio
#import board
from ctypes import c_float
import subprocess
from Encoder import EncoderCounter
import serial
import sys
from TFmini import TFmini
log_file = open('/home/pi/WRO_2025_PI/logs/log_current.txt', 'w')
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
reset_pin = 8

tfmini = TFmini(RX_Head, RX_Left, RX_Right, RX_Back)


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
lidar_angle = multiprocessing.Value('d', 0.0)
lidar_distance = multiprocessing.Value('d', 0.0)
imu_shared = multiprocessing.Value('d', 0.0)
specific_angle = multiprocessing.Array(c_float, 3)  # shared array of 3 integers
lidar_f = multiprocessing.Value('d', 0.0)
sp_angle = multiprocessing.Value('i', 0)
left_f = multiprocessing.Value('b', False)
right_f = multiprocessing.Value('b', False)



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


corr = 0
prev_time = 0
reset_imu = False


def correctAngle(setPoint_gyro, left, right, trigger, heading, distance_h, distance_l, distance_r):
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

    if (distance_l < 15 and distance_l >= 0):
        print("Inside Left")
        # corr -=1
        # setPoint_gyro -= corr
        correction = correction - 20

    # distance_right = -1

    elif (distance_r < 15 and distance_r >= 0):
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


def servoDrive(distance, block, pwm, counts, head, lidar_f, sp_angle, turn_trigger, left_f, right_f):
    print("ServoProcess started")
    global heading
    global distance_right, distance_head, distance_left

    pwm.set_mode(servo, pigpio.OUTPUT)

    pwm.set_PWM_frequency(servo, 50)


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
    correctAngle(0, left_flag, right_flag, trigger, head.value, 0,0,0)
    target_count = 0
    turn_t = 0

    try:
        while True:
            tfmini.getTFminiData()
            tf_h = lidar_f.value
            tf_l = tfmini.distance_left
            tf_r = tfmini.distance_right
            init_flag = False
            previous_state = button_state
            button_state = pwm.read(5)
            if previous_state == 1 and button_state == 0:
                button = not (button)
                init_flag = True
                power = 100

                print("Button is pressed")

            if not button:
                print(f"tf_h: {tf_h}, tf_l: {tf_l}, tf_r: {tf_r}")

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
                
                if counter == 12:
                    counter = -1
                    print(f"Counters are over")
                
                if counter == -1:    
                    if tfmini.distance_head < 150 and heading_angle == 0 and (head.value < 5 or head.value > 355) and not trigger:
                        print(f"Open CHallenge Finished")
                        power = 0
                        pwm.set_PWM_dutycycle(12, power)  # Set duty cycle to 50% (128/255)
                        sys.exit()


                if not right_flag and not left_flag:
                    if tf_r > 180:
                        right_flag = True
                        right_f.value = True
                    elif tf_l > 180:
                        left_flag = True
                        left_f.value = True

                correctAngle(heading_angle, left_flag, right_flag, trigger, head.value, tf_h, tf_l, tf_r)



                if right_flag:
                    print("Right Flag is set")
                    if (tfmini.distance_right > 150 and tfmini.distance_head < 100) and not trigger:
                        # time.sleep(0.5)
                        counter = counter + 1
                        heading_angle = ((90 * counter) % 360)
                        trigger = True

                    elif tfmini.distance_right < 100 and tfmini.distance_head > 150:
                        trigger = False

                elif left_flag:
                    if (tfmini.distance_left > 150 and tfmini.distance_head < 100) and not trigger:
                        # time.sleep(0.5)
                        counter = counter + 1
                        heading_angle = -(90 * counter) % 360
                        trigger = True
                    elif tfmini.distance_left < 100 and tfmini.distance_head > 150:
                        trigger = False
                


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
                correctAngle(heading_angle, left_flag, right_flag, trigger, head.value, tf_h, tf_l, tf_r)
            print(f"counts:{counts.value} counter:{counter} trigger:{trigger}, trigger_b:{turn_trigger.value} right_flag:{right_flag} left_flag:{left_flag} heading_angle:{heading_angle}, head:{head.value} tf_h: {lidar_f.value}, tf_l: {tf_l}, tf_r: {tf_r}")
            #print(f"heading:{head.value} {heading_angle}  counter:{counter} {trigger},  target_count:{target_count}, encoder_c:{counts.value}, L C R:{distance_left} {distance_head} {distance_right}")
    except KeyboardInterrupt:
        power = 0
        pwm.set_PWM_dutycycle(12, 0)
    except Exception as e:
        print(f"Exception: {e}")

def runEncoder(counts, head):
    print("Encoder Process Started")

    try:
        while True:
            line = ser.readline().decode('utf-8', errors = 'ignore').strip()
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


def read_lidar(lidar_angle, lidar_distance, sp_angle, turn_trigger, specific_angle, lidar_f, head, left_f, right_f):
    #print("This is first line")
    global CalledProcessError
    trig_time = 0
    previous_angle = 0
    lidar_binary_path = '/home/pi/rplidar_sdk/output/Linux/Release/ultra_simple'
    print("⏳ Waiting for LIDAR output...")
    prev_sp = 0
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
                imu_r = int(head.value)
                sp_angle.value = 360 - sp_angle.value

                #print(f"📍 Angle: {angle:.2f}°, Distance: {distance:.2f} mm")
            except Exception as e:
                print("⚠️ Parse error:", e)
        else:
            print("ℹ️", line)

        if sp_angle.value > 180:
            sp_angle.value = sp_angle.value - 360            
        if previous_angle != angle:
            if prev_sp != sp_angle.value:
                sp_angle.value = 360 - sp_angle.value
            prev_sp = sp_angle.value    
                
            while(angle - previous_angle > 1):
                lidar_angle.value = (previous_angle + 1) % 360
                lidar_distance.value = previous_distance
                previous_angle = lidar_angle.value
                if(int(lidar_angle.value) == (0 + imu_r + sp_angle.value) % 360):
                    lidar_front = lidar_distance.value
                    lidar_f.value = lidar_front
                if(int(lidar_angle.value) == (90 + imu_r + sp_angle.value) % 360):
                    lidar_left = lidar_distance.value

                if(int(lidar_angle.value) == (270 + imu_r + sp_angle.value) % 360):
                    lidar_right = lidar_distance.value

                if(lidar_front < 900 and lidar_right > 1500) and right_f.value and not left_f.value:
                    turn_trigger.value = True
                elif(lidar_front < 900 and lidar_left > 1500) and left_f.value and not right_f.value:
                    turn_trigger.value = True
                else:
                    turn_trigger.value = False
                print(f"turn_trigger: {turn_trigger.value} lidar_front: {lidar_front}, lidar_left: {lidar_left}, lidar_right: {lidar_right} sp_angle:{sp_angle.value} head:{head.value}")

                #print(f"angles: {specific_angle} imu: {imu_shared.value} total:{imu_r + lidar_angle.value} sp_angle:{sp_angle.value}")
                
            if(distance != 0): 
                with lidar_angle.get_lock(), lidar_distance.get_lock(), imu_shared.get_lock():
                    lidar_angle.value = angle
                    lidar_distance.value = distance
                    previous_distance = distance
                    previous_angle = angle
                    if(int(lidar_angle.value) == (0 + imu_r  + sp_angle.value) % 360):
                        lidar_front = lidar_distance.value
                        lidar_f.value = lidar_front
                    if(int(lidar_angle.value) == (90 + imu_r + sp_angle.value) % 360):
                        specific_angle[1] = lidar_distance.value
                        lidar_left = lidar_distance.value
                    if(int(lidar_angle.value) == (270 + imu_r + sp_angle.value) % 360 ):
                        lidar_right = lidar_distance.value                                      
                    if(lidar_front < 900 and lidar_right > 1500) and right_f.value and not left_f.value:
                        turn_trigger.value = True
                    elif(lidar_front < 900 and lidar_left > 1500) and left_f.value and not right_f.value:
                        turn_trigger.value = True
                    else:
                        turn_trigger.value = False
                    print(f"turn_trigger: {turn_trigger.value} lidar_front: {lidar_front}, lidar_left: {lidar_left}, lidar_right: {lidar_right} sp_angle:{sp_angle.value} head:{head.value}")
                            




if __name__ == "__main__":
    try:

        
        S = multiprocessing.Process(target=servoDrive, args=(distance, block, pwm, counts, head, lidar_f, sp_angle, turn_trigger, left_f, right_f))
        E = multiprocessing.Process(target=runEncoder, args=(counts, head,))
        lidar_proc = multiprocessing.Process(target=read_lidar, args=(lidar_angle, lidar_distance, sp_angle, turn_trigger, specific_angle, lidar_f, head, left_f, right_f))

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
        GPIO.cleanup()