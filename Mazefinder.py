#!/usr/bin/env python3

from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
from ev3dev2.sensor.lego import ColorSensor, GyroSensor, UltrasonicSensor
from ev3dev2.led import Leds
from ev3dev2.sound import Sound
from ev3dev2 import DeviceNotFound
from ev3dev2.motor import LargeMotor, MediumMotor, OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D, SpeedRPS
from ev3dev2.button import Button

from threading import Thread

from Constants import *
from Drive import *
from UltrasonicArm import *
from Data import *
from MazeAlgorithm import *

import datetime, time, sys, csv

start_time = time.time()
data = Data()

# Start Robot Init
sound = Sound()
leds = Leds()
btn = Button()


try:
    leftMotor = LargeMotor(OUTPUT_A)
    rightMotor = LargeMotor(OUTPUT_B)
    ultrasonicMotor = MediumMotor(OUTPUT_C)
    gyroSensor = GyroSensor(INPUT_1)
    ultrasonicSensor = UltrasonicSensor(INPUT_2)

except DeviceNotFound as connection_error:
    print(connection_error, file = sys.stderr)

drive = Drive(leftMotor, rightMotor, gyroSensor, ultrasonicSensor , start_time)
us_arm = UltrasonicArm(ultrasonicMotor, gyroSensor, ultrasonicSensor)

# us_thread = Thread(target=us_arm.us_maze_detect())
mazealg = MazeAlgorithm()

def elapsed_time():
    return str(time.time() - start_time)

def init_robot():
    print(elapsed_time() + ': INIT ROBOT', file = sys.stderr, flush = True)
    drive.drive_init()
    # us_thread.setDaemon(True)
    # us_thread.start()

def main():    
    complianceTest()
    # maze()

def terrain():
    drive.drive_indef()

def complianceTest():
    print("INIT: COMPLIANCE TURN READY", file = sys.stderr)
    complianceTurn()
    print("END: COMPLIANCE TURN COMPLETE", file = sys.stderr)
    btn.wait_for_bump(btn.enter, 500)
    time.sleep(2)
    print("INIT: COMPLIANCE DRIVE READY", file = sys.stderr)
    complianceDrive()
    print("INIT: COMPLIANCE DRIVE COMPLETE", file = sys.stderr)

def complianceTurn():
    drive.drive_spot_turn(data.gyroSetpoint(-90))
    drive.drive_spot_turn(data.gyroSetpoint(90))

def complianceDrive():
    drive.drive_dist(data.distSetpoint(600))

def maze():
    while True:
        if ultrasonicSensor.value() > US_SAFE_DIST:
            drive.drive_ultrasonic(US_SAFE_DIST)
        else:
            valueSet = us_arm.us_maze_detect()
            drive.cycle_search(valueSet)

if __name__ == '__main__':
    init_robot()
    main()
    drive.create_gyro_csv()
    # drive.create_position_csv()