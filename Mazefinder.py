#!/usr/bin/env python3

from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
from ev3dev2.sensor.lego import ColorSensor, GyroSensor, UltrasonicSensor
from ev3dev2.led import Leds
from ev3dev2.sound import Sound
from ev3dev2 import DeviceNotFound
from ev3dev2.motor import LargeMotor, MediumMotor, OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D, SpeedRPS
from ev3dev2.button import Button

# import paho.mqtt.client as mqtt
# from ev3dev2.auto import *

from Constants import *
from Drive import *
from UltrasonicArm import *
from Data import *

import datetime, time, sys

start_time = time.time()

# Start Robot Init
sound = Sound()
leds = Leds()
btn = Button()

# client = mqtt.Client()
# client.connect("172.20.10.2",1883,60)

# client.on_connect = on_connect
# client.on_message = on_message

# client.loop_forever()

# try:
#     leftMotor = LargeMotor(OUTPUT_A)
#     rightMotor = LargeMotor(OUTPUT_B)
# #    ultrasound_arm = MediumMotor(OUTPUT_C)
# #    color = ColorSensor(INPUT_2)
#     gyroSensor = GyroSensor(INPUT_1)
#     ultrasonicSensor = UltrasonicSensor(INPUT_2)

# except DeviceNotFound as connection_error:
#     print(connection_error, file = sys.stderr)

leftMotor = LargeMotor(OUTPUT_A)
rightMotor = LargeMotor(OUTPUT_B)
gyroSensor = GyroSensor(INPUT_1)

drive = Drive(leftMotor, rightMotor, gyroSensor, start_time)
# us_arm = UltrasonicArm(ultrasound_arm, us_sensor)

def elapsed_time():
    return str(time.time() - start_time)

def init_robot():
    print(elapsed_time() + ': INIT ROBOT', file = sys.stderr, flush = True)
    drive.drive_init()
    #ultrasound_arm_calibrate()

def main():
<<<<<<< HEAD
    # drive.drive_dist(100) #in mm
    spot_turn()
=======
    spot_turn()
    #drive.drive_indef()
    # drive.drive_dist(150) #in mm
>>>>>>> cb02dd917ad98d9bbe37a5d5f757c9209b9e4aa4

def terrain():
    drive.drive_indef()

def spot_turn():
<<<<<<< HEAD
    drive.drive_spot_turn(90)
=======
    while(True):    
        drive.drive_turn_update(drive.gyro_pid_update(90))
>>>>>>> cb02dd917ad98d9bbe37a5d5f757c9209b9e4aa4

if __name__ == '__main__':
    init_robot()
    main()

# MQTT PID TUNING
# This is the Subscriber

# def on_connect(client, userdata, flags, rc):
#     print("CONNECTED, RC: "+str(rc))
#     client.subscribe("topic/gyro_p")

# def on_message(client, userdata, msg):
#     if (msg.payload == 'Q'):
#       client.disconnect()
#     elif (0 <= int(msg.payload) <= 10):
#       temp_tuning_GYRO_P = msg.payload
#       print(temp_tuning_GYRO_P, file = sys.stderr, flush = True)