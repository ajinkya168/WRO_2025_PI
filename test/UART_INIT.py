import serial
import math
import time
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
GPIO.setup(8, GPIO.OUT)
GPIO.setup(26, GPIO.OUT)  # blue
GPIO.setup(10, GPIO.OUT)  # red
GPIO.setup(6, GPIO.OUT)  # green
GPIO.output(26, GPIO.LOW)
GPIO.output(6, GPIO.LOW)
GPIO.output(10, GPIO.LOW)
print("Resetting....")
GPIO.output(8, GPIO.LOW)
GPIO.output(6, GPIO.HIGH)
time.sleep(1)  
GPIO.output(8, GPIO.HIGH)
GPIO.output(6, GPIO.LOW)
time.sleep(2)
print("Reset COmplete")

# Open the serial port
'''def uart_init():
	ser = serial.Serial('/dev/UART_USB', 115200)
	#data = input()
	#rad = str(math.radians(float(data)))
	ser.flush()
	ser.write(b'1')
	while True:
		response = ser.readline().decode().strip()
		start = response.split(" ")
		print(f"response {response}")
		if start[1] == '0':
			print("ESP32 is ready.")
			break
		#time.sleep(1)
	return ser'''



reset = False
esp_data1 = 0
ser = serial.Serial('/dev/UART_USB', 115200)#uart_init()
print("creatyed uart")
#ser.flush()
if __name__ == '__main__':
	try:
		while True:
			#time.sleep(0.01)
			print("ESP while loop")
			esp_data1= ser.readline().decode().strip()

			data = esp_data1.split(" ")

			print(data)
			
			if data[0].isdigit() or data[1].isdigit():
				GPIO.output(10, GPIO.LOW)
				head = float(data[0])
				count = int(data[1])
				#print(f"Received X: {x}, Y: {y}")
				print(f" ESP1: {head}, {int(data[1])} type head:{type(head)} type count:{type(count)}")
			else:
				GPIO.output(10, GPIO.HIGH)
	except Exception as e:
		print(f"exception: {e}")
		pass

	

	
