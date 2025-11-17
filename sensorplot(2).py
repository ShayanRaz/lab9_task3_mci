import serial
import matplotlib.pyplot as plt
from drawnow import *

# === Setup Serial ===
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
ser.flush()

plt.ion()
acc_vals = []
gyro_vals = []
fused_vals = []
time_vals = []
cnt = 0

# === Plotting Function ===
def makeFig():
    plt.clf()
    plt.title('Accelerometer Angle, Gyro Rate & Fused Angle')
    plt.grid(True)
    plt.xlabel('Sample Number')
    plt.ylabel('Degree')
    plt.plot(time_vals, acc_vals, 'r.-', label='Accel Angle')
    plt.plot(time_vals, gyro_vals, 'g.-', label='Gyro Rate')
    plt.plot(time_vals, fused_vals, 'b.-', label='Fused Angle')
    plt.legend(loc='upper left')

# === Main Loop ===
print("Waiting for data from STM32...")
while True:
    try:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').strip()
            
            # Skip initialization messages
            if "System Ready" in line or "Initialized" in line or "Calibrating" in line or "WHO_AM_I" in line or "Offset" in line:
                print(f"Info: {line}")
                continue
            
            # Split by tab
            values = line.split('\t')
            
            # Filter empty strings and strip whitespace
            values = [v.strip() for v in values if v.strip()]
            
            if len(values) == 3:
                # Convert to integers (your STM32 sends integers)
                acc_angle = int(values[0])
                gyro_rate = int(values[1])
                fused_angle = int(values[2])
                
                acc_vals.append(acc_angle)
                gyro_vals.append(gyro_rate)
                fused_vals.append(fused_angle)
                time_vals.append(cnt)
                cnt += 1
                
                drawnow(makeFig)
                plt.pause(0.0001)
                
                # Keep only last 100 samples for better visualization
                if len(acc_vals) > 100:
                    acc_vals.pop(0)
                    gyro_vals.pop(0)
                    fused_vals.pop(0)
                    time_vals.pop(0)
            else:
                print(f"Skipping: {line}")
                
    except ValueError as e:
        print(f"Value Error: {e}, Line: {line}")
    except KeyboardInterrupt:
        print("\nStopping...")
        ser.close()
        break
    except Exception as e:
        print(f"Error: {e}")

