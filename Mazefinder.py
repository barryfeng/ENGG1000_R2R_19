from ev3dev2.sensor import INPUT_2, INPUT_3, INPUT_4
from ev3dev2.sensor.lego import ColorSensor, GyroSensor, UltrasonicSensor
from ev3dev2.led import Leds
from ev3dev2.sound import Sound
from ev3dev2 import DeviceNotFound
from ev3dev2.motor import LargeMotor, MediumMotor, OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D, SpeedRPS
from ev3dev2.button import Button
from Constants import *
from Drive import *
from UltrasonicArm import *

import datetime, time, threading

def eprint(self, *args, **kwargs):
    print(*args, file=sys.stderr, **kwargs)

# Start Robot Init
sound = Sound()
leds = Leds()
btn = Button()

leftMotor = LargeMotor(OUTPUT_A)
rightMotor = LargeMotor(OUTPUT_B)

start_time = int(time.time())
# try:
#     leftMotor = LargeMotor(OUTPUT_A)
#     rightMotor = LargeMotor(OUTPUT_B)
#     #ultrasound_arm = MediumMotor(OUTPUT_C)
#     gyroSensor = GyroSensor(INPUT_1)
#     #color = ColorSensor(INPUT_2)
#     #us_sensor = UltrasonicSensor(INPUT_3)

# except DeviceNotFound as connection_error:
#     print(connection_error)

drive = Drive(leftMotor, rightMotor, gyroSensor)

#us_arm = UltrasonicArm(ultrasound_arm, us_sensor)

def init_robot():
    drive.gyro_calibrate()
    #ultrasound_arm_calibrate()

# def ultrasound_arm_calibrate():
#     sound.speak('Move the arm to right hardstop. Calibrating arm in two seconds!')
#     us_sensor.mode = 'US-DIST-CM'
#     ultrasound_arm.stop_action = 'HOLD'
#     time.sleep(2)
#     ultrasound_arm.position = 0
#     ultrasound_arm.on_to_position(100, 90, True)
#     ultrasound_arm.position = 0
#     sound.speak('Arm calibration complete.')

# End Robot Init

# if drive_logging_enabled:
#     drive_log_row = [iterated_time, speed_left + compensate, speed_right + compensate]
#     with open(datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S").csv, 'a') as f:
#         w = csv.writer(f)
#         w.writerow(drive_log_row)

# # Detect Target
# def detect_target():
#     gyro_zero()
#     target_list = []
#     ultrasound_arm.on_to_position(100, -90, True)
#     ultrasound_arm.on_to_position(10, 90, True)
#     if us.value() < 300:
#         print("Object Found")
#         target_list.append(ultrasound_arm.position)
#     return target_list

# Go to target
# def rescue(target_angle):
#   Drive.drive_start(update_gyro_pid(target_angle))

def main():
    drive.drive_dist(300) #in mm
    drive.drive_spot_turn(Direction.LEFT)

if __name__ == '__main__':
    init_robot()
    selection = input("Robot initialised, type function to run to run program (Drive indefinitely [0], drive then turn [1])")
    eprint("Test message to computer")
    main()
