import os
os.system('sudo pkill pigpiod')
os.system('sudo pigpiod')
import pigpio
import serial
import math
import time
import RPi.GPIO as GPIO
import board
from Encoder import EncoderCounter
from BNO085 import IMUandColorSensor
GPIO.setmode(GPIO.BCM)
GPIO.setup(14, GPIO.OUT)

print("Resetting....")
GPIO.output(14, GPIO.LOW)
time.sleep(1)  
GPIO.output(14, GPIO.HIGH)

time.sleep(1)

print("Reset COmplete")

pwm = pigpio.pi()
pwm.set_mode(12, pigpio.OUTPUT)  # Set pin 12 as an output
pwm.set_mode(20, pigpio.OUTPUT)  # Set pin 20 as an output
pwm.hardware_PWM(12, 100, 0)
pwm.set_PWM_dutycycle(12, 0)  # Set duty cycle to 50% (128/255)

total_power = 0
prev_power = 0
power = int(input("Enter power: "))

ser = serial.Serial('/dev/UART_USB', 115200)#uart_init()
imu = IMUandColorSensor(board.SCL, board.SDA)
enc = EncoderCounter()
print("creatyed uart")
#ser.flush()
head =0
count = 0
if __name__ == '__main__':
	try:
		while True:
			total_power = (power * 0.1) + (prev_power * 0.9)
			prev_power = total_power
			pwm.set_PWM_dutycycle(12, 2.55 * total_power) 
			esp_data1= ser.readline().decode().strip()
			data = esp_data1.split(" ")
			#print(data)
			if data[0].isdigit() or data[1].isdigit():
				head = float(data[0])
				count = int(data[1])
				print(f"Received X: {x}, Y: {y}")
				print(f" ESP1: {head}, {int(data[1])} type head:{type(head)} type count:{type(count)}")
			else:
				pass
				
			x , y = enc.get_position(head, count)
			print(f"x:{x} y:{y}")
			if x > 100:
				power = 0
				prev = 0
				pwm.set_PWM_dutycycle(12, power) 
				break

	except Exception as e:
		print(f"exception: {e}")
		pass

	

	
