from Globals import *
import pigpio
import time
from Servo import Servo
from TFmini import TFmini
from Encoder import EncoderCounter
import serial
class initHardware:
    pi = pigpio.pi()
    servo = Servo(servo_pin)
    tfmini = TFmini(RX_Head, RX_Left, RX_Right, RX_Back)
    ser = serial.Serial('/dev/UART_USB', 115200)

    def resetLeds(self):
        for pin in [blue_led, red_led, green_led]:
            self.pi.set_mode(pin, pigpio.OUTPUT)
            self.pi.write(pin, 0)  # Set LOW
                
    def resetArduino(self):
        print("Resetting....")
        self.pi.write(self.reset_pin, 0)          # Pull reset LOW
        self.pi.write(self.green_led, 1)          # Turn on green LED
        time.sleep(1)

        self.pi.write(self.reset_pin, 1)          # Release reset (HIGH)
        self.pi.write(self.green_led, 0)          # Turn off green LED
        time.sleep(1)
        print("Reset Complete")
        
    def setButtonMode(self):
        self.pi.set_mode(button_pin, pigpio.INPUT)
        self.pi.set_pull_up_down(button_pin, pigpio.PUD_UP) 
