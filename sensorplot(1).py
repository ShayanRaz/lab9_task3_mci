import serial
import matplotlib.pyplot as plt
from drawnow import *
import time

# === Setup Serial ===
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

# Wait and clear buffer to avoid initial garbage
time.sleep(2)
ser.reset_input_buffer()

plt.ion()
accAngle_vals = []
gyroY_vals = []
fusedAngle_vals = []
time_ms = []
cnt = 0

# === Plotting Function ===
def makeFig():
    plt.clf()
    plt.title('Live Accelerometer Angle, Gyro Rate & Fused Angle')
    plt.grid(True)
    plt.xlabel('Sample Number')
    plt.ylabel('Degree / deg/s')
    plt.plot(time_ms, accAngle_vals, 'r.-', label='Accel Angle (deg)')
    plt.plot(time_ms, gyroY_vals, 'g.-', label='Gyro Rate (deg/s)')
    plt.plot(time_ms, fusedAngle_vals, 'b.-', label='Fused Angle (deg)')
    plt.legend(loc='upper left')

# === Main Loop ===
print("Waiting for data from STM32...")
print("Skip initialization messages...")

while True:
    try:
        if ser.in_waiting > 0:
            # Read line with error handling
            raw_line = ser.readline()
            
            try:
                line = raw_line.decode('utf-8', errors='ignore').strip()
            except:
                continue
            
            # Skip empty lines
            if not line:
                continue
            
            # Skip initialization messages
            if any(word in line for word in ["WHO_AM_I", "Initialized", "Calibrating", "Offset", "OK", "Error", "STILL"]):
                print(f"Info: {line}")
                continue
            
            # Split by tab (your STM32 uses \t separator)
            values = line.split('\t')
            
            # Filter empty strings
            values = [v.strip() for v in values if v.strip()]
            
            # Check if we have exactly 3 values
            if len(values) == 3:
                try:
                    # Your STM32 sends: accAngleY, c.gy, angleX
                    accAngle = float(values[0])    # Accelerometer angle in degrees
                    gyroY = float(values[1])       # Gyro Y rate in deg/s
                    fusedAngle = float(values[2])  # Fused angle in degrees
                    
                    accAngle_vals.append(accAngle)
                    gyroY_vals.append(gyroY)
                    fusedAngle_vals.append(fusedAngle)
                    time_ms.append(cnt)
                    cnt += 1
                    
                    drawnow(makeFig)
                    plt.pause(0.0001)
                    
                    # Keep only last 100 samples for better visualization
                    if len(accAngle_vals) > 100:
                        accAngle_vals.pop(0)
                        gyroY_vals.pop(0)
                        fusedAngle_vals.pop(0)
                        time_ms.pop(0)
                        
                except ValueError as e:
                    print(f"Could not convert values: {values}")
                    continue
            else:
                # Only print if it looks like data
                if any(c.isdigit() or c == '.' or c == '-' for c in line):
                    print(f"Unexpected format ({len(values)} values): {line}")
                    
    except KeyboardInterrupt:
        print("\nStopping...")
        ser.close()
        break
    except Exception as e:
        print(f"Error: {e}")
        continue
