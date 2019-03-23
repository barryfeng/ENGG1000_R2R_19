from Constants import *
from enum import Enum

from ev3dev2.wheel import EV3Tire
from ev3dev2.sensor.lego import GyroSensor
from ev3dev2.sensor import INPUT_1
from __future__ import print_function

import time, sys, syslog

tire = EV3Tire()
gyroSensor = GyroSensor(INPUT_1)

class Drive:
    def __init__(self, drive_left, drive_right, gyro):
        self.drive_left = drive_left
        self.drive_right = drive_right
        self.gyro = gyro

        self.dist_left = 0
        self.dist_right = 0

        self.integral = 0
        self.last_error = 0

    def gyro_calibrate(self):
        time.sleep(1)
        self.gyro_zero()
        time.sleep(2)

    def gyro_zero(self):
        gyroSensor.mode = 'GYRO-RATE'
        gyroSensor.mode = 'GYRO-ANG'

    def drive_speed_update(self, compensate):
        self.drive_left.on(DRIVE_SPEED - compensate, brake= True)
        self.drive_right.on(DRIVE_SPEED + compensate, brake=True)
        self.drive_dist_update()
        #print(compensate)

    def drive_turn_update(self, compensate):
        self.drive_left.on(DRIVE_SPEED - compensate, brake= True)
        self.drive_right.on(-DRIVE_SPEED + compensate, brake=True)

    def drive_pid_update(self, setpoint):
        error = self.gyro.value() - setpoint
        self.integral += (error * CYCLE_TIME)
        derivative = (error - self.last_error) / CYCLE_TIME

        compensate = error * GYRO_P + self.integral * GYRO_I + derivative * GYRO_D

        self.last_error = error
        self.eprint(compensate)
        return compensate

    def drive_stop(self):
        self.drive_left.stop()
        self.drive_right.stop()

    def drive_dist_update(self):
        self.dist_left += (self.drive_left.speed/self.drive_left.count_per_rot) * CYCLE_TIME #Rotations per cycle time
        self.dist_right += (self.drive_right.speed/self.drive_right.count_per_rot) * CYCLE_TIME
        #print('Left: ' + str(self.dist_left))
        #print('Right: ' + str(self.dist_right))

    def eprint(self, *args, **kwargs):
        print(*args, file=sys.stderr, **kwargs)

    # DRIVE FUNCTIONS

    def drive_dist(self, desired_dist):
        total_rotations = desired_dist / tire.circumference_mm
        while self.drive_left.position / self.drive_left.count_per_rot and self.drive_right.position / self.drive_right.count_per_rot < total_rotations:
            self.drive_speed_update(self.drive_pid_update(Direction.STRAIGHT))
            print(str(self.drive_left.position / self.drive_left.count_per_rot) + ' out of ' + str(total_rotations))
            time.sleep(CYCLE_TIME)
    
    def drive_indef(self):
        self.drive_speed_update(self.drive_pid_update(Direction.STRAIGHT))
        time.sleep(CYCLE_TIME)

    def drive_slow_turn(self, direction):
        current_dir = gyroSensor.value()
        setpoint = current_dir + direction        
        while True:
            self.drive_speed_update(self.drive_pid_update(direction))
            if setpoint - TURNING_ACC <= self.gyro.value() <= setpoint + TURNING_ACC/2:
                break
            time.sleep(CYCLE_TIME)

    def drive_spot_turn(self, direction):
        current_dir = gyroSensor.value()
        setpoint = current_dir + direction
        while True:
            self.drive_turn_update(self.drive_pid_update(setpoint))
            if setpoint - TURNING_ACC/2 <= gyroSensor.value() <= setpoint + TURNING_ACC/2:
                break
            time.sleep(CYCLE_TIME)




    # def basic_straight(self):
    #     self.drive_left.on_for_rotations(10)
    #     self.drive_right.on_for_rotations(10)

    # def basic_turn_left(self):
    #     self.drive_left.stop()
    #     self.drive_right.on_for_degrees(360)

class Direction(Enum):
    LEFT = -90
    RIGHT = 90
    STRAIGHT = 0 