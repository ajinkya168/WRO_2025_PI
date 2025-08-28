import pigpio
from Globals import *
from initHardware import *
class Servo:
    def __init__(self, pin, frequency=50):
        self.pin = pin
        self.frequency = frequency
        self.pwm = pigpio.pi()
        self.pwm.set_mode(self.pin, pigpio.OUTPUT)
        self.pwm.set_PWM_frequency(self.pin, self.frequency)

    def setAngle(self, angle):
        pulse_width = 500 + round(angle * 11.11)
        self.pwm.set_servo_pulsewidth(self.pin, pulse_width)

    def set_PWM(self, PWM):
        self.pwm.set_PWM_dutycycle(self.pin, PWM)
        

    def correctAngle(self, setPoint_gyro, heading):
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
        self.setAngle(90 - correction)


    def correctReverseAngle(self, setPoint_gyro, heading):
        global corr
        error_gyro = 0
        prevErrorGyro = 0
        totalErrorGyro = 0
        correction = 0


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
        self.setAngle(90 + correction)
