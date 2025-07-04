import serial
import math
# Open the serial port
ser = serial.Serial('/dev/UART_USB', 115200)
ser.flush()
#data = input()
#rad = str(math.radians(float(data)))
ser.write(b"1")
print("Command sent: b'1'")
#ser.flush()
while True:
    # Rea	d data from ESP32
	
	#ser.write(rad.encode())
	esp_data = ser.readline().decode().strip()
	#esp_data = esp_data + 1   
	# if esp_data.startswith("X: "):
	#x, y = esp_data.split(" ")
	#x = float(x.split("")[1])
	#y = float(y)

	#print(f"Received X: {x}, Y: {y}")
	print(f" ESP: {esp_data}, type:{type(esp_data)}")
