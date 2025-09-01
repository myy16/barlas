import serial
import time

# Arduino portunu kontrol et!
arduino_port = "/dev/ttyACM0"
baud_rate = 9600

try:
    ser = serial.Serial(arduino_port, baud_rate, timeout=1)
    time.sleep(2)  # Arduino reset bekle
except Exception as e:
    print("Bağlantı hatası:", e)
    exit()

while True:
    # LASER ON
    ser.write(b"LASER,ON\n")
    print("Sent: LASER,ON")
    time.sleep(2)  # 2 saniye bekle

    # LASER OFF
    ser.write(b"LASER,OFF\n")
    print("Sent: LASER,OFF")
    time.sleep(2)
