# find_baud.py — run this on the Jetson
import serial, time

PORT = "/dev/ttyTHS1"
BAUDS = [9600, 115200, 57600, 38400, 19200]

for baud in BAUDS:
    try:
        s = serial.Serial(PORT, baud, timeout=1)
        time.sleep(0.3)
        s.reset_input_buffer()
        s.write(b"AT\r\n")
        time.sleep(0.8)
        resp = s.read(s.in_waiting)
        s.close()
        print(f"{baud:>7}: {repr(resp)}")
    except Exception as e:
        print(f"{baud:>7}: ERROR {e}")