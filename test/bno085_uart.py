# SPDX-FileCopyrightText: 2020 Bryan Siepert, written for Adafruit Industries
#
# SPDX-License-Identifier: Unlicense
import time
import board
#import busio
import RPi.GPIO as GPIO
#from adafruit_extended_bus import ExtendedI2C as I2C
GPIO.setmode(GPIO.BCM)
GPIO.setup(14, GPIO.OUT)
GPIO.output(14, GPIO.LOW)
time.sleep(1)  
GPIO.output(14, GPIO.HIGH)
#uart = busio.UART(board.TX, board.RX, baudrate=115200, receiver_buffer_size=2048)

# uncomment and comment out the above for use with Raspberry Pi
import serial
uart = serial.Serial("/dev/ttyS0", 115200)
import time
# for a USB Serial cable:
# import serial
# uart = serial.Serial("/dev/ttyUSB0", baudrate=115200)

from adafruit_bno08x_rvc import BNO08x_RVC  # pylint:disable=wrong-import-position

rvc = BNO08x_RVC(uart, timeout = 1)
prev_time = 0
while True:
    try:
        if time.time() - prev_time > 0.01:
            yaw, pitch, roll, x_accel, y_accel, z_accel = rvc.heading
            prev_time = time.time()
    except:
        pass
    print("Yaw: %2.2f Pitch: %2.2f Roll: %2.2f Degrees" % (yaw, pitch, roll))
    print("Acceleration X: %2.2f Y: %2.2f Z: %2.2f m/s^2" % (x_accel, y_accel, z_accel))
    print("")
    time.sleep(0.01)
