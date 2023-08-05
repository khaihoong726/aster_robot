import serial
import numpy as np
from time import sleep

# ----------- Helper Functions ----------
def readData(board, num):
    board.reset_input_buffer()
    discard = board.readline()
    sensor = []
    for _ in range(num):
        data = board.readline()
        data = data.decode('utf-8').replace('\x00',"").replace('\r\n',"")
        sensor.append(data)
    return sensor

# --------- Connect to Board ---------------
print("Connecting to STM32...")
sleep(1)

board = serial.Serial("/dev/ttyACM0", 115200)
print("Connected to STM32\n")

# --------- Calibrate Gyroscope -------------
print("Calibrating Gyro...")
sleep(1)

sensor = readData(board, 10)

gx_offset = 0
gy_offset = 0
gz_offset = 0
for i in range(len(sensor)):
    data = sensor[i].split(',')

    gx = float(data[5])
    gy = float(data[6])
    gz = float(data[7])

    gx_offset += gx
    gy_offset += gy
    gz_offset += gz

gx_offset = -gx_offset / len(sensor)
gy_offset = -gy_offset / len(sensor)
gz_offset = -gz_offset / len(sensor)

print("Calibrated offset gx, gy, gz:", gx_offset, gy_offset, gz_offset, "\n")

# ---------- Calibrate Accelerometer -------------
def parseAccelData(sensor, num):
    accel = np.array([0.0, 0.0, 0.0])
    for i in range(num):
        data = sensor[i].split(',')

        accel[0] += float(data[2])
        accel[1] += float(data[3])
        accel[2] += float(data[4])

    accel = accel / num
    return accel

print("Calibrating Accelerometer...\n")
sleep(1)

g = 9.81
S = np.zeros((3, 6))
L = np.array([[-g, g, 0, 0, 0, 0],
              [0, 0, -g, g, 0, 0],
              [0, 0, 0, 0, -g, g]])

input("Align IMU along the +Z axis. Press Enter to continue...")
sensor = readData(board, 10)
accel = parseAccelData(sensor, 10)
S.T[5] = accel

input("Align IMU along the -Z axis. Press Enter to continue...")
sensor = readData(board, 10)
accel = parseAccelData(sensor, 10)
S.T[4] = accel

input("Align IMU along the +Y axis. Press Enter to continue...")
sensor = readData(board, 10)
accel = parseAccelData(sensor, 10)
S.T[3] = accel

input("Align IMU along the -Y axis. Press Enter to continue...")
sensor = readData(board, 10)
accel = parseAccelData(sensor, 10)
S.T[2] = accel

input("Align IMU along the +X axis. Press Enter to continue...")
sensor = readData(board, 10)
accel = parseAccelData(sensor, 10)
S.T[1] = accel

input("Align IMU along the -X axis. Press Enter to continue...")
sensor = readData(board, 10)
accel = parseAccelData(sensor, 10)
S.T[0] = accel

C = L @ S.T @ np.linalg.inv(S @ S.T)

print("Acceleration Calibration Matrix:")
print("\t", C, "\n")

# ---------- Close Serial Port ---------------
print("Closing connection to the STM32")
board.close()
