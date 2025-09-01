import serial
import time

arduino = serial.Serial('/dev/ttyACM0', 9600)
time.sleep(2)  # Arduino resetlenmesi için

def move_servo(pan, tilt):
    cmd = f"MOVE,{pan},{tilt}\n"
    arduino.write(cmd.encode())
    print(arduino.readline().decode().strip())

def laser_on():
    arduino.write(b"LASER,ON\n")
    print(arduino.readline().decode().strip())

def laser_off():
    arduino.write(b"LASER,OFF\n")
    print(arduino.readline().decode().strip())

# Örnek kullanım
laser_off()
move_servo(180, 180)
time.sleep(1)
laser_on()
