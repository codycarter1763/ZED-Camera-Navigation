#RELATIVE ANGLES

# import serial
# import struct
# import time

# def send_cmd(ser, cmd, data=bytes([])):
#     size = len(data)
#     header_checksum = (cmd + size) & 0xFF
#     body_checksum = 0
#     for b in data:
#         body_checksum = (body_checksum + b) & 0xFF
#     packet = bytes([0x3E, cmd, size, header_checksum]) + data + bytes([body_checksum])
#     ser.write(packet)
#     time.sleep(0.1)
#     response = ser.read(100)
#     return response

# def set_pitch(ser, angle):
#     mode = 2
#     pitch = int(angle / 0.02197265625)
#     data = struct.pack('<Bhhhhhh', mode, 0, 0, 500, pitch, 0, 0)
#     send_cmd(ser, 67, data)

# ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
# time.sleep(2)

# print("Turning motors on...")
# send_cmd(ser, 77)
# time.sleep(1)

# current_angle = 0
# set_pitch(ser, current_angle)
# print(f"Gimbal at {current_angle} degrees - ready")

# while True:
#     cmd = input("Enter relative angle change (e.g. -45, 90) or 'q' to quit: ")
#     if cmd == 'q':
#         break
#     try:
#         delta = float(cmd)
#         current_angle += delta
#         set_pitch(ser, current_angle)
#         print(f"Moved to {current_angle} degrees")
#     except:
#         print("Invalid input")

# ser.close()


#ABSOLUTE ANGLES
# import serial
# import struct
# import time
# import threading

# def send_cmd(ser, cmd, data=bytes([])):
#     size = len(data)
#     header_checksum = (cmd + size) & 0xFF
#     body_checksum = 0
#     for b in data:
#         body_checksum = (body_checksum + b) & 0xFF
#     packet = bytes([0x3E, cmd, size, header_checksum]) + data + bytes([body_checksum])
#     ser.write(packet)
#     time.sleep(0.1)
#     response = ser.read(100)
#     return response

# def set_pitch(ser, angle):
#     mode = 2
#     pitch = int(angle / 0.02197265625)
#     data = struct.pack('<Bhhhhhh', mode, 0, 0, 300, pitch, 0, 0)
#     send_cmd(ser, 67, data)

# current_angle = 0
# running = True

# def hold_loop(ser):
#     while running:
#         set_pitch(ser, current_angle)
#         time.sleep(0.5)

# ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
# time.sleep(2)

# print("Turning motors on...")
# send_cmd(ser, 77)
# time.sleep(1)

# current_angle = 0
# set_pitch(ser, current_angle)
# print("Gimbal at 0 degrees - ready")

# t = threading.Thread(target=hold_loop, args=(ser,))
# t.daemon = True
# t.start()

# while True:
#     cmd = input("Enter angle (0, 30, -30, 45, -45, 60, -60, 90, -90) or 'q' to quit: ")
#     if cmd == 'q':
#         running = False
#         break
#     try:
#         current_angle = float(cmd)
#         set_pitch(ser, current_angle)
#         print(f"Set pitch to {current_angle} degrees")
#     except:
#         print("Invalid input")

# ser.close()


#ABSOLUTE ANGLES WITH DRIFT CORRECTION
import serial
import struct
import time

def send_cmd(ser, cmd, data=bytes([])):
    size = len(data)
    header_checksum = (cmd + size) & 0xFF
    body_checksum = 0
    for b in data:
        body_checksum = (body_checksum + b) & 0xFF
    packet = bytes([0x3E, cmd, size, header_checksum]) + data + bytes([body_checksum])
    ser.write(packet)
    time.sleep(0.1)
    response = ser.read(100)
    return response

def set_pitch(ser, angle):
    mode = 2
    pitch = int(angle / 0.02197265625)
    data = struct.pack('<Bhhhhhh', mode, 0, 0, 300, pitch, 0, 0)
    send_cmd(ser, 67, data)
    time.sleep(0.5)
    mode = 0
    data = struct.pack('<Bhhhhhh', mode, 0, 0, 0, 0, 0, 0)
    send_cmd(ser, 67, data)
    print(f"Set pitch to {angle} degrees")

ser = serial.Serial('/dev/ttyACM2', 115200, timeout=1)
time.sleep(2)

print("Turning motors on...")
send_cmd(ser, 77)
time.sleep(1)

set_pitch(ser, 0)
print("Gimbal at 0 degrees - ready")

while True:
    cmd = input("Enter angle (0, 5, -5, 10, -10, 15, -15, 20, -20, 25, -25, 30, -30, 35, -35, 40, -40, 45, -45, 50, -50, 55, -55,  60, -60, 65, -65, 70, -70, 75, -75, 80, -80, 85, -85, 90, -90, 95, -95, 100, -100, 105, -105, 110, -110, 115, -115, 120, -120, 125, -125, 130, -130, 135, -135, 140, -140, 145, -145, 150, -150, 155, -155, 160, -160, 165, -165, 170, -170, 175, -175, 180, -180, 360, -360) or 'q' to quit: ")
    if cmd == 'q':
        break
    try:
        angle = float(cmd)
        set_pitch(ser, angle)
    except:
        print("Invalid input")

ser.close()