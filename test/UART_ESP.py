import serial
import math
# Open the serial port

ser = serial.Serial('/dev/UART_USB', 115200)
#ser.flush()
#data = input()
#rad = str(math.radians(float(data)))
ser.write(b"1")
print("Command sent: b'1'")
#ser.flush()
head = 0.0
counts = 0
while True:
    # Rea	d data from ESP32
	
	#ser.write(rad.encode(
	esp_data = ser.readline().decode('utf-8', errors='ignore').strip()
	esp_data = esp_data.split()
	if len(esp_data) >= 2:
		try:
			head = float(esp_data[0])
			counts = int(esp_data[1])
		except ValueError:
			print(f"⚠️ Malformed numbers: {esp_data}")
	else:
		print(f"⚠️ Incomplete frame: {esp_data!r}")
	#esp_data = esp_data + 1   
	# if esp_data.startswith("X: "):
	#x, y = esp_data.split(" ")
	#x = float(x.split("")[1])
	#y = float(y)

	#print(f"Received X: {x}, Y: {y}")
	print(f" head: {head - (-90):.2f} rev diff:{-90 - head:.2f}, counts: {counts}")
	#print(f" ESP: {esp_data}, type:{type(esp_data)}")
