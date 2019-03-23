from Constants import *
from enum import Enum

from ev3dev2.wheel import EV3Tire
from ev3dev2.sensor.lego import GyroSensor, UltrasonicSensor
from __future__ import print_function

import time, sys, syslog

tire = EV3Tire()

class Drive:
    def __init__(self, drive_left, drive_right, gyro, ultrasonic):
        self.drive_left = drive_left
        self.drive_right = drive_right
        self.gyro = gyro
        self.ultrasonic = ultrasonic

        self.dist_left = 0
        self.dist_right = 0

        self.integral = 0
        self.last_error = 0

    def gyro_calibrate(self):
        print('INIT: GYRO CALIBRATION', file = sys.stderr)
        time.sleep(1)
        self.gyro_zero()
        time.sleep(2)
        print('END: GYRO CALIBRATION', file = sys.stderr, flush = True)

    def gyro_zero(self):
        self.gyro.mode = 'GYRO-RATE'
        self.gyro.mode = 'GYRO-ANG'

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
        print(compensate, file=sys.stderr)
        return compensate

    def drive_stop(self):
        self.drive_left.off(brake=True)
        self.drive_right.off(brake=True)

    def drive_dist_update(self):
        self.dist_left += (self.drive_left.speed/self.drive_left.count_per_rot) * CYCLE_TIME #Rotations per cycle time
        self.dist_right += (self.drive_right.speed/self.drive_right.count_per_rot) * CYCLE_TIME
        #print('Left: ' + str(self.dist_left))
        #print('Right: ' + str(self.dist_right))

    # DRIVE FUNCTIONS
        
    def drive_indef(self):
        while True:
            self.drive_speed_update(self.drive_pid_update(Direction.STRAIGHT))
            time.sleep(CYCLE_TIME)

    def drive_dist(self, desired_dist):
        total_rotations = desired_dist / tire.circumference_mm
        while True:
            self.drive_speed_update(self.drive_pid_update(Direction.STRAIGHT))
            print(str(self.drive_left.position / self.drive_left.count_per_rot) + ' out of ' + str(total_rotations), file = sys.stderr)
            if (total_rotations - ROTATION_ACC <= self.drive_left.position + self.drive_right.position) / 360 <= total_rotations + ROTATION_ACC:
                break
            time.sleep(CYCLE_TIME)
        self.drive_stop()

    def drive_ultrasonic (self, safe_dist):
        while True:
            self.drive_speed_update(self.drive_pid_update(Direction.STRAIGHT))
            if self.ultrasonic <= safe_dist:
                break
            time.sleep(CYCLE_TIME)
        self.drive_stop

    # TURNING FUNCTIONS

    def drive_slow_turn(self, direction):
        current_dir = self.gyro.value()
        setpoint = current_dir + direction        
        while True:
            self.drive_speed_update(self.drive_pid_update(direction))
            if setpoint - TURNING_ACC <= self.gyro.value() <= setpoint + TURNING_ACC/2:
                break
            time.sleep(CYCLE_TIME)
        self.drive_stop()

    def drive_spot_turn(self, direction):
        current_dir = self.gyro.value()
        setpoint = current_dir + direction
        while True:
            self.drive_turn_update(self.drive_pid_update(setpoint))
            if setpoint - TURNING_ACC/2 <= self.gyro.value() <= setpoint + TURNING_ACC/2:
                break
            time.sleep(CYCLE_TIME)
        self.drive_stop()

    def drive_rescue_logged_turn(self):
        while True:
            self.drive_turn_update(self.drive_pid_update(Direction.RIGHT))



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