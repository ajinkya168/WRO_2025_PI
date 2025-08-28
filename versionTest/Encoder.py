import pigpio
import RPi.GPIO as GPIO
import time
import math
#import board
from math import atan2, sqrt
#import matplotlib.pyplot as plt
import multiprocessing
from Globals import *
from initHardware import *

class EncoderCounter:
    channelA = 9
    channelB = 11
    const = (2 * math.pi * 2)

    def __init__(self):

        self.pi = pigpio.pi()

        self.pi.set_mode(self.channelA, pigpio.INPUT)
        self.pi.set_pull_up_down(self.channelA, pigpio.PUD_UP)  # channel A
        self.pi.set_mode(self.channelB, pigpio.INPUT)
        self.pi.set_pull_up_down(self.channelB, pigpio.PUD_UP)  # channel B

        self.prev_state_A = self.pi.read(self.channelA)
        self.prev_state_B = self.pi.read(self.channelB)
        self.count = 0
        self.x = 0
        self.y = 0
        self.dx = 0
        self.dy = 0
        self.error_x = 0
        self.error_y = 0
        self.x_history = [self.x]
        self.y_history = [self.y]
        self.prev_distance = 0
        self.hardware = initHardware()



    def get_position(self, heading, counter):
        # with self.lock:
        # revolution = (self.prev_count - self.count)/1040

        revolution = (counter/2015)  #168
        distance_cm = revolution * self.const
        change = distance_cm - self.prev_distance
        self.dx = math.cos(math.radians(heading)) * change
        self.dy = math.sin(math.radians(heading)) * change
        # print(f"Change : {change}")

        self.x += self.dx
        self.y += self.dy

        self.prev_distance = distance_cm

        # self.prev_count = self.count
        return self.x, self.y


    def correctPosition(self, setPoint, head, x, y, counter, blue, orange, reset, reverse, heading, centr_x_p, centr_x_r, centr_x_g, centr_y_g, centr_y_r,  centr_y_p, finish, distance_h, distance_l, distance_r, red, green):
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
                print(
                f"lane:{lane}, error:{error:.2f} target:{(100 - setPoint)}, setPoint:{setPoint} x:{x}, y:{y}")

            elif blue:
                error = (100 + setPoint) - x
                print(
                    f"lane:{lane}, error:{error} target:{(100 + setPoint)}, x:{x} y:{y} Bluee")
        # print(f" trigger : {flag_t} setPoint: {setPoint} lane: {lane} correction:{correction}, error:{error} x:{x}, y:{y}, prevError :{prevError} angle:{head - correction}")
        elif lane == 2:
            if orange:
                error = y - (200 - setPoint)  # CHANGE 1
                print(
                f"lane:{lane} error:{error:.2f} target:{(200 - setPoint)},  x: {x} y: {y} setPoint:{setPoint}")
            elif blue:
                error = y - (-200 - setPoint)
                print(
                f"lane:{lane} error:{error} target:{(-200 - setPoint)}, x: {x} y{y}")
        # print(f"setPoint: {flag_t} lane: {lane} correction:{correction}, error:{error} x:{x}, y:{y}, prevError :{prevError} angle:{head - correction}")
        elif lane == 3:
            if orange:
                error = (setPoint - 100) - x
                print(
                f"lane:{lane} error:{error:.2f} target:{(setPoint - 100)}, x: {x} y {y} setPoint:{setPoint}")
            elif blue:
                error = x + (100 + setPoint)
                print(f"lane:{lane} error:{error} target:{(55 + setPoint)}, x:{x} y {y}")

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
            self.hardware.tfmini.getTFminiData()
            if (((setPoint == -35) and orange) or (counter == 0 and (centr_x_p < 300 and centr_x_p > 0) and ((centr_x_g or centr_x_r) > centr_x_p) and not blue and not orange) and not finish):
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

                if (setPoint <= -70) and distance_l <= 20:
                    print(f"Correcting Green Wall Orange")
                    correction = 15
                elif (setPoint >= 70) and ((self.hardware.tfmini.distance_head <= 25 and (n_head - head > 35)) or distance_r <= 20):
                    print(
                    f"Correcting Green Wall... diff:{(n_head - head):.2f} heading:{heading:.2f} n_head:{n_head:.2f} head:{head} right {distance_r} head_d:{self.hardware.tfmini.distance_head}")
                    correction = -15
                else:
                    print("No wall detected...")
                    pass

            else:

                if heading < 180 and lane == 0:
                    n_head = heading + 360
                else:
                    n_head = heading

                if (setPoint >= 70) and distance_r <= 20:
                    print(f"correctng red wall in blue")
                    correction = -15
                elif (setPoint <= -70) and ((self.hardware.tfmini.distance_head <= 25 and abs((n_head - head) - 360) > 35) or distance_l <= 20):
                    print(
                    f"Correcting Green Wall... diff:{abs((n_head - head) - 360):.2f} heading:{heading:.2f} n_head:{n_head:.2f} head:{head} right {distance_r} head_d:{self.hardware.tfmini.distance_head}")
                    correction = 15
                else:
                    print("No wall detected...")
                    pass

        if correction > 45:
            correction = 45
        elif correction < -45:
            correction = -45

        print(f"diff:{(heading - head):.2f} heading:{heading:.2f} head:{head:.2f} right {distance_r} head_d:{self.hardware.tfmini.distance_head} correction:{correction}")
        prevError = error
        self.hardware.servo.correctAngle(head + correction, heading)
