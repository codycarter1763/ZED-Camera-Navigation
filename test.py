import serial, time
s = serial.Serial('COM9', 115200, timeout=5)
time.sleep(3)
deadline = time.time() + 10
while time.time() < deadline:
    if s.in_waiting:
        line = s.readline().decode('utf-8', errors='ignore').strip()
        print(repr(line))
s.close()