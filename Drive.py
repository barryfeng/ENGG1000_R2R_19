#!/usr/bin/env python3

from Constants import *
from Data import *
from enum import Enum
# import matplotlib.pyplot as plt

from ev3dev2.sensor.lego import GyroSensor, UltrasonicSensor
from ev3dev2.motor import MoveTank

import time, sys, syslog, csv

data = Data()

class Drive:
    def __init__(self, drive_left, drive_right, gyro, ultrasonic, start_time):
        self.drive_left = drive_left
        self.drive_right = drive_right
        self.gyro = gyro
        self.ultrasonic = ultrasonic
        self.start_time = start_time

        self.dist_left = 0
        self.dist_right = 0

        self.gyro_integral = 0
        self.gyro_last_error = 0

        self.distance_integral = 0
        self.distance_last_error = 0

        self.dist_travelled = 0

        self.logged_gyro = [[0,0]]
        self.logged_position = [[0,0]]

        self.drive_cycle_start = 0
        self.drive_cycle_end = 0

    def elapsed_time(self):
        return str(time.time() - self.start_time)

    def gyro_calibrate(self):
        print('INIT: GYRO CALIBRATION', file = sys.stderr, flush = True)
        self.gyro_zero()
        print('END: GYRO CALIBRATION', file = sys.stderr, flush = True)

    def gyro_zero(self):
        time.sleep(1)
        self.gyro.mode = 'GYRO-RATE'
        self.gyro.mode = 'GYRO-ANG'
        time.sleep(1)

    def drive_init(self):
        if K_DRIVING_DIRECTION == 1:
            self.drive_left.polarity = self.drive_left.POLARITY_INVERSED
            self.drive_right.polarity = self.drive_right.POLARITY_INVERSED
        elif K_DRIVING_DIRECTION == -1:
            self.drive_left.polarity = self.drive_left.POLARITY_NORMAL
            self.drive_right.polarity = self.drive_right.POLARITY_NORMAL
        else:
            print("ERROR: INVALID DRIVING DIRECTION", file = sys.stderr)
        self.drive_zero_position()
        self.drive_left.stop_action = self.drive_left.STOP_ACTION_HOLD
        self.drive_right.stop_action = self.drive_right.STOP_ACTION_HOLD
        self.gyro_calibrate()

    def drive_zero_position(self):
        self.drive_left.position = 0
        self.drive_right.position = 0

    def drive_speed_update(self, heading_compensate):
        self.drive_cycle_start = time.time()
        self.drive_left.on(DRIVE_SPEED - K_DRIVING_DIRECTION * heading_compensate, brake= True)
        self.drive_right.on(DRIVE_SPEED + K_DRIVING_DIRECTION * heading_compensate, brake=True)
        self.drive_dist_update()
        self.drive_cycle_end = time.time()

    def drive_turn_update(self, heading_compensate):
        self.drive_left.on(K_DRIVING_DIRECTION * K_TURNING_SPEED * - heading_compensate, brake= True)
        self.drive_right.on(K_DRIVING_DIRECTION * K_TURNING_SPEED * heading_compensate, brake=True)

    def gyro_pid_update(self, setpoint):
        error = self.gyro.value() - setpoint
        self.gyro_integral += (error * (self.drive_cycle_end-self.drive_cycle_start))

        heading_compensate = error * GYRO_P + self.gyro_integral * GYRO_I

        self.gyro_last_error = error
        # print(self.elapsed_time() + ' GYRO_PID: ' + str(error), file=sys.stderr, flush = True)
        self.logged_gyro.append([self.elapsed_time(), int(error)])

        if (50 <= heading_compensate):
            return 49.9
        elif (heading_compensate <= -50):
            return -49.9
        else:
            return heading_compensate

    def drive_stop(self):
        self.drive_left.stop()
        self.drive_right.stop()

    def drive_dist_update(self):
        self.dist_left = (self.drive_left.position /360 * EV3_RIM)
        self.dist_right = (self.drive_right.position / 360 * EV3_RIM)
        self.dist_travelled = ((self.dist_left + self.dist_right) / 2)
        # self.logged_position.append([self.elapsed_time(), self.dist_travelled*math.cos(math.radians(self.gyro_setpoint)), self.dist_travelled*math.sin(math.radians(self.gyro_setpoint))])
   
    # DRIVE FUNCTIONS
        
    def drive_indef(self):
        while True:
            self.drive_speed_update(self.gyro_pid_update(0))
            time.sleep(CYCLE_TIME)

    def drive_dist(self, desired_dist):
        self.drive_zero_position()
        while True:
            self.drive_speed_update(self.gyro_pid_update(0))
            self.drive_dist_update()
            if desired_dist - DIST_ACC <= self.dist_travelled:
                self.drive_stop()
                self.drive_dist_update()
                self.gyro_zero()
                break

    def drive_ultrasonic(self, safe_dist):
        while True:
            self.drive_speed_update(self.gyro_pid_update(0))
            if self.ultrasonic.value() <= safe_dist:
                self.drive_stop()
                self.drive_dist_update()
                self.gyro_zero()
                break

    # TURNING FUNCTIONS

    # def drive_slow_turn(self, direction):
    #     current_dir = self.gyro.value()
    #     setpoint = current_dir + direction        
    #     while True:
    #         self.drive_speed_update(self.gyro_pid_update(direction))
    #         if setpoint - TURNING_ACC < self.gyro.value() < setpoint + TURNING_ACC:
    #             break
    #         time.sleep(CYCLE_TIME)
    #     self.drive_stop()

    def drive_spot_turn(self, setpoint):
        while True:
            self.drive_turn_update(self.gyro_pid_update(setpoint))
            if setpoint - TURNING_ACC <= int(self.gyro.value()) <= setpoint + TURNING_ACC:
                self.drive_stop()
                self.gyro_zero()
                break
            
    # DETECTION FUNCTIONS

    def cycle_search(self, valueSet):
        left = valueSet[0]
        right = valueSet[1]
        if left <= K_WALL_SAFE_DIST and right <= K_WALL_SAFE_DIST:
            print("ACTION: DEAD END " + valueSet, file = sys.stderr)
            self.drive_spot_turn(data.gyroSetpoint(180))
        elif K_WALL_SAFE_DIST <= left and right <= K_WALL_SAFE_DIST:
            print("ACTION: CORNER ON LEFT " + valueSet, file = sys.stderr)
            self.drive_spot_turn(data.gyroSetpoint(90))
        elif K_WALL_SAFE_DIST <= right and left <= K_WALL_SAFE_DIST:
            print("ACTION: CORNER ON RIGHT " + valueSet, file = sys.stderr)
            self.drive_spot_turn(data.gyroSetpoint(-90))
        elif K_WALL_SAFE_DIST <= left and K_WALL_SAFE_DIST <= right:
            print("ACTION: TEE JUNCTION " + valueSet, file = sys.stderr)
            self.drive_spot_turn(data.gyroSetpoint(90))

    def drive_rescue_logged_turn(self):
        while True:
            self.drive_turn_update(self.gyro_pid_update(0))

    # LOGGING

    def create_gyro_csv(self):
        csv.register_dialect('dialect', delimiter=',', quoting=csv.QUOTE_NONE)
        gyro_file = open('gyro_error.csv', 'w')  
        with gyro_file:  
            writer = csv.writer(gyro_file, dialect='dialect')
            writer.writerows(self.logged_gyro)

    # def create_position_csv(self):
    #     csv.register_dialect('dialect', delimiter=',', quoting=csv.QUOTE_NONE)
    #     position_file = open('position_setpoint.csv', 'w')  
    #     with position_file:  
    #         writer = csv.writer(position_file, dialect='dialect')
    #         writer.writerows(self.logged_position)