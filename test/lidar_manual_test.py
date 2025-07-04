import subprocess
import threading
import os


scan_data = []
previous_angle = 0
def read_lidar():
    # Full path to the ultra_simple binary
    lidar_binary_path = '/home/pi/rplidar_sdk/sdk/ultra_simple/ultra_simple'

    # Ensure it is executable
    if not os.path.isfile(lidar_binary_path):
        print(f"❌ File not found: {lidar_binary_path}")
        return

    print("🚀 Launching ultra_simple...")
#    angle_input = float(input("Enter angle: "))
    # Start the binary
    process = subprocess.Popen(
        [lidar_binary_path, '/dev/ttyUSB0', '460800'],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True  # decode bytes to string
    )

    try:
        global previous_angle
        for line in process.stdout:
            line = line.strip()
            if "°" in line:
                try:
                    angle_part, distance_part = line.split("°")
                    angle = float(angle_part.strip())
                    distance = float(distance_part.strip().replace("mm", ""))
                    #scan_data.append((angle, distance))
                    #print(f"📍 Angle: {angle:.2f}°, Distance: {distance:.2f} mm")
                except Exception as e:
                    print("⚠️ Parse error:", e)
            else:
                print("ℹ️", line)
            if distance != 0.0 and previous_angle != angle:
                print(f" ^=^s^m Angle: {angle:.2f} , Distance: {distance:.2f} mm")
            previous_angle = angle
    except KeyboardInterrupt:
        print("🛑 Ctrl+C received. Stopping LIDAR.")
        process.terminate()
    finally:
        print("🔌 Lidar process ended.")

# Run lidar in a background thread
if __name__ == '__main__':
	#angle = float(input("Enter angle: "))
	read_lidar()
