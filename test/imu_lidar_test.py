import serial
import subprocess
import os
import multiprocessing
from multiprocessing import Array

rplidar = [None]*360

previous_distance = 0
dist_0 = 0
dist_90 = 0
dist_270 = 0
angle = 0
specific_angle = [None]*3
def read_lidar(lidar_angle, lidar_distance, previous_angle, imu):
    lidar_binary_path = '/home/pi/rplidar_sdk/output/Linux/Release/ultra_simple'
    global previous_distance, dist_270, dist_90, dist_0, angle  
    if not os.path.isfile(lidar_binary_path):
        print(f"❌ File not found: {lidar_binary_path}")
        return

    print("🚀 Launching ultra_simple...")

    process = subprocess.Popen(
        [lidar_binary_path, '--channel', '--serial', '/dev/LIDAR_USB', '460800'],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True
    )

    try:
        for line in process.stdout:
            line = line.strip()
            #print(line)
            if "theta" in line and "Dist" in line:
                try:
                    angle_part = line.split()
                    #print(angle_part)
                    angle = float(angle_part[1])
                    distance = float(angle_part[3])
                    #print(f"📍 Angle: {angle:.2f}°, Distance: {distance:.2f} mm")
                except Exception as e:
                    print("⚠️ Parse error:", e)
            else:
                print("ℹ️", line)

            angle = int(angle)
            imu_r = int(imu.value)
            if previous_angle.value != angle:
        
                while(angle - previous_angle.value > 1):
                    lidar_angle.value = previous_angle.value + 1
                    lidar_distance.value = previous_distance
                    previous_angle.value = previous_angle.value + 1
                    rplidar[int(lidar_angle.value)] = lidar_distance.value
                    if(int(lidar_angle.value) == (0 + imu_r) % 360):
                        specific_angle[0] = lidar_distance.value
                    if(int(lidar_angle.value) == (90 + imu_r) % 360):
                        specific_angle[1] = lidar_distance.value
                    if(int(lidar_angle.value) == (270 + imu_r) % 360):
                        specific_angle[2] = lidar_distance.value                                    
                    print(f"angles: {specific_angle} imu: {imu.value} total:{imu.value + lidar_angle.value}")
                   
                if(distance != 0): 
                    with lidar_angle.get_lock(), lidar_distance.get_lock(), previous_angle.get_lock(), imu.get_lock():
                        lidar_angle.value = angle
                        lidar_distance.value = distance
                        previous_distance = distance
                        previous_angle.value = angle
                        rplidar[int(lidar_angle.value)] = lidar_distance.value
                        if(int(lidar_angle.value) == (0 + imu_r) % 360):
                            specific_angle[0] = lidar_distance.value
                        if(int(lidar_angle.value) == (90 + imu_r) % 360):
                            specific_angle[1] = lidar_distance.value
                        if(int(lidar_angle.value) == (270 + imu_r) % 360  ):
                            specific_angle[2] = lidar_distance.value                                       
                        print(f"angles: {specific_angle}, imu: {imu.value} total:{imu_r + lidar_angle.value}")
                        #print(f"angle: {lidar_angle.value} distance:{rplidar[int(lidar_angle.value)]}")

    except KeyboardInterrupt:
        print("🛑 Ctrl+C received. Stopping LIDAR.")
        process.terminate()
    finally:
        print("🔌 Lidar process ended.")

if __name__ == '__main__':
    # Start LIDAR reader in a separate process
    # Shared memory values
    previous_angle = multiprocessing.Value('d', 0.0)
    lidar_angle = multiprocessing.Value('d', 0.0)
    lidar_distance = multiprocessing.Value('d', 0.0)
    imu = multiprocessing.Value('d', 0.0)
    
    lidar_proc = multiprocessing.Process(
        target=read_lidar,
        args=(lidar_angle, lidar_distance, previous_angle, imu)) 

    lidar_proc.start()

    # Open serial connection to ESP32
    ser = serial.Serial('/dev/UART_USB', 115200)
    ser.flush()
    ser.write(b"1")
    print("✅ Command sent to ESP32: b'1'")

    # Fixed-size list for 360 degrees
    lidar_data_list = [None] * 360

    try:
        while True:
            line = ser.readline().decode().strip()
            esp_data = line.split(" ")
            
            esp_data.append(1)
            #print(esp_data)
           
            esp_angle = float(esp_data[0])
            count = int(esp_data[1])
            imu.value = esp_angle
            #print(f"imu: {imu.value}")



    except KeyboardInterrupt:
        print("👋 Shutting down...")
        lidar_proc.terminate()
        lidar_proc.join()

        # Final printout (optional)
        print("\n📦 Final LIDAR Data at 0°, 90°, 270°:")
        for deg in [0, 90, 270]:
            dist = lidar_data_list[deg]
            if dist is not None:
                print(f"  {deg}° → {dist:.2f} mm")
            else:
                print(f"  {deg}° → ❌ No data")
