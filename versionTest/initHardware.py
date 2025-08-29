import Globals as g
import pigpio
import time
from Servo import Servo
from TFmini import TFmini
from Encoder import EncoderCounter
import serial


class initHardware:
    pi = pigpio.pi()
    servo = Servo(g.servo_pin)
    tfmini = TFmini(g.RX_Head, g.RX_Left, g.RX_Right, g.RX_Back)
    ser = serial.Serial('/dev/UART_USB', 115200)
    # Define hardware pins from Globals

    def resetLeds(self):
        for pin in [g.blue_led, g.red_led, g.green_led]:
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
        self.pi.set_mode(g.button_pin, pigpio.INPUT)
        self.pi.set_pull_up_down(g.button_pin, pigpio.PUD_UP) 
