#!/usr/bin/env python3

# from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
# from ev3dev2.sensor.lego import ColorSensor, GyroSensor, UltrasonicSensor
# from ev3dev2.led import Leds
# from ev3dev2.sound import Sound
# from ev3dev2 import DeviceNotFound
# from ev3dev2.motor import LargeMotor, MediumMotor, OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D, SpeedRPS
# from ev3dev2.button import Button
# from Constants import *
# from Drive import *
#from UltrasonicArm import *

# import datetime, time

# Start Robot Init
# sound = Sound()
# leds = Leds()
# btn = Button()

# try:
#     leftMotor = LargeMotor(OUTPUT_A)
#     rightMotor = LargeMotor(OUTPUT_B)
# #    ultrasound_arm = MediumMotor(OUTPUT_C)
# #    color = ColorSensor(INPUT_2)
#     gyroSensor = GyroSensor(INPUT_1)
#     ultrasonicSensor = UltrasonicSensor(INPUT_2)

# except DeviceNotFound as connection_error:
#     print(connection_error, file = sys.stderr)

# leftMotor = LargeMotor(OUTPUT_A)
# rightMotor = LargeMotor(OUTPUT_B)
# gyroSensor = GyroSensor(INPUT_1)

# drive = Drive(leftMotor, rightMotor, gyroSensor)
#us_arm = UltrasonicArm(ultrasound_arm, us_sensor)

# def elapsed_time():
#     return int(time.time() - start_time)

# def init_robot():
#     print(': Initialising robot', file = sys.stderr)
#     drive.gyro_calibrate()
#     drive.drive_zero_position()
#     #ultrasound_arm_calibrate()

def main():
    # drive.drive_dist(300) #in mm
    while True:
        print('hello')

# def terrain():
#     drive.drive_indef()

# def spot_turn():
#     drive.drive_spot_turn(Direction.LEFT)
#     drive.drive_spot_turn(Direction.RIGHT)


if __name__ == '__main__':
    # init_robot()
    # selection = input(": Robot initialised", file = sys.stderr)
    # spot_turn()
    main()

