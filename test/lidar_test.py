import subprocess
import threading
import os

def read_lidar():
    # Full path to the ultra_simple binary
    lidar_binary_path = '/home/pi/rplidar_sdk/output/Linux/Release/ultra_simple'

    # Ensure it is executable
    if not os.path.isfile(lidar_binary_path):
        print(f"❌ File not found: {lidar_binary_path}")
        return

    print("🚀 Launching ultra_simple...")

    # Start the binary
    process = subprocess.Popen(
        [lidar_binary_path, '--channel', '--serial', '/dev/LIDAR_USB', '460800'],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True  # decode bytes to string
    )

    try:
        for line in process.stdout:
            #print(f"this is raw: {line}")
            line = line.strip()
            
            if "theta" in line and "Dist" in line:
                try:
                    angle_part = line.split()
                    #print(angle_part)
                    angle = float(angle_part[1])
                    distance = float(angle_part[3])
                    print(f"📍 Angle: {angle:.2f}°, Distance: {distance:.2f} mm")
                except Exception as e:
                    print("⚠️ Parse error:", e)
            else:
                print("ℹ️", line)
    except KeyboardInterrupt:
        print("🛑 Ctrl+C received. Stopping LIDAR.")
        process.terminate()
    finally:
        print("🔌 Lidar process ended.")

# Run lidar in a background thread
#lidar_thread = threading.Thread(target=read_lidar)
#lidar_thread.start()

# 🔄 Meanwhile your main Python program can continue
if __name__ == '__main__':
	read_lidar()
