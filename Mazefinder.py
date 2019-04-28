#!/usr/bin/env python3

from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
from ev3dev2.sensor.lego import ColorSensor, GyroSensor, UltrasonicSensor
from ev3dev2.led import Leds
from ev3dev2.sound import Sound
from ev3dev2 import DeviceNotFound
from ev3dev2.motor import LargeMotor, MediumMotor, OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D, SpeedRPS
from ev3dev2.button import Button

from Constants import *
from Drive import *
from UltrasonicArm import *
from Data import *
from MazeAlgorithm import *

import datetime, time, sys, csv

start_time = time.time()
data = Data()

leftMotor = LargeMotor(OUTPUT_A)
rightMotor = LargeMotor(OUTPUT_B)
ultrasonicMotor = MediumMotor(OUTPUT_C)
clawMotor = MediumMotor(OUTPUT_D)
colorSensor = ColorSensor(INPUT_2)
gyroSensor = GyroSensor(INPUT_3)
ultrasonicSensor = UltrasonicSensor(INPUT_4)
try:
    victimUltrasonicSensor = UltrasonicSensor(INPUT_1)
except:
    victimUltrasonicSensor = 1
    data.cprint('FATAL: VICTIM ULTRASONIC SENSOR IS NOT PLUGGED IN')

drive = Drive(leftMotor, rightMotor, gyroSensor, ultrasonicSensor, colorSensor, victimUltrasonicSensor, clawMotor, start_time)
us_arm = UltrasonicArm(ultrasonicMotor, gyroSensor, ultrasonicSensor)

mazealg = MazeAlgorithm()

def elapsed_time():
    return str(time.time() - start_time)

def init_robot():
    print('WARNING: INIT ROBOT', file = sys.stderr, flush = True)
    drive.drive_init()

def main():
    # maze()
    incline()
    # drive.retract_claw()
    # drive.zero_claw()
    # identify()
    # rescue()
    # terrain()

def terrain():
    drive.drive_indef()

def obstacles():
    drive.drive_indef()

def incline():
    drive.drive_indef()

def identify():
    drive.find_target(1)                                                                                                                                                       
    drive.id()
    drive.find_target(2)                                                                                                                                   
    drive.id()

def rescue():
    drive.find_target(1)
    drive.grasp_target()                                                                                                                                                     

def maze():
    while True:
        if ultrasonicSensor.value() > K_US_SAFE_DIST:
            drive.drive_ultrasonic(K_US_SAFE_DIST)
        else:
            valueSet = us_arm.us_maze_detect()
            drive.cycle_search(valueSet)

if __name__ == '__main__':
    init_robot()
    main()
    drive.create_gyro_csv()