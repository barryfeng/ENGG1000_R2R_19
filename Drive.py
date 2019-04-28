#!/usr/bin/env python3

from Constants import *
from Data import *
from enum import Enum
# import matplotlib.pyplot as plt

from ev3dev2.sensor.lego import GyroSensor, UltrasonicSensor
from ev3dev2.motor import MoveTank
from threading import Thread
from enum import Enum

import time, sys, syslog, csv

data = Data()

class Drive:
    def __init__(self, drive_left, drive_right, gyro, ultrasonic, colourSensor, victimUltrasonic, clawMotor, start_time):
        self.drive_left = drive_left
        self.drive_right = drive_right
        self.gyro = gyro
        self.ultrasonic = ultrasonic
        self.victimUltrasonic = victimUltrasonic
        self.color = colourSensor
        self.start_time = start_time
        self.claw = clawMotor

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

        self.driving_direction = Direction.WHEELSIDE

    def elapsed_time(self):
        return str(time.time() - self.start_time)

    def gyro_calibrate(self):
        print('INIT: GYRO CALIBRATION', file = sys.stderr, flush = True)
        self.gyro_zero()
        print('END: GYRO CALIBRATION', file = sys.stderr, flush = True)

    def gyro_zero(self):
        time.sleep(1)
        self.gyro.mode = 'GYRO-CAL'
        self.gyro.mode = 'GYRO-ANG'
        time.sleep(1)

    def drive_init(self):
        self.set_drive_direction(Direction.WHEELSIDE)
        self.color.mode = 'COL-COLOR'
        self.drive_zero_position()
        self.drive_left.stop_action = self.drive_left.STOP_ACTION_HOLD
        self.drive_right.stop_action = self.drive_right.STOP_ACTION_HOLD
        self.gyro_calibrate()

    def set_drive_direction(self, direction):
        self.driving_direction = direction
        print('ALERT: DRIVING DIRECTION CHANGED - ' + str(self.driving_direction), file =sys.stderr)
        if self.driving_direction == Direction.WHEELSIDE:
            self.drive_left.polarity = self.drive_left.POLARITY_INVERSED
            self.drive_right.polarity = self.drive_right.POLARITY_INVERSED
            self.driving_direction == Direction.WHEELSIDE
        elif self.driving_direction == Direction.CLAWSIDE:
            self.drive_left.polarity = self.drive_left.POLARITY_NORMAL
            self.drive_right.polarity = self.drive_right.POLARITY_NORMAL
            self.driving_direction == Direction.CLAWSIDE
        else:
            print("ERROR: INVALID DRIVING DIRECTION, USING WHEELSIDE", file = sys.stderr)
            self.drive_left.polarity = self.drive_left.POLARITY_NORMAL
            self.drive_right.polarity = self.drive_right.POLARITY_NORMAL
            self.driving_direction == Direction.WHEELSIDE.value

    def drive_zero_position(self):
        self.drive_left.position = 0
        self.drive_right.position = 0

    def drive_speed_update(self, heading_compensate):
        self.drive_cycle_start = time.time()
        self.drive_left.on(K_DRIVE_SPEED - self.driving_direction.value * heading_compensate, brake= True)
        self.drive_right.on(K_DRIVE_SPEED + self.driving_direction.value * heading_compensate, brake=True)
        self.drive_dist_update()
        self.drive_cycle_end = time.time()

    def drive_turn_update(self, heading_compensate):
        self.drive_left.on(self.driving_direction.value * K_TURNING_SPEED * - heading_compensate, brake= True)
        self.drive_right.on(self.driving_direction.value * K_TURNING_SPEED * heading_compensate, brake=True)

    def gyro_pid_update(self, setpoint):
        error = self.gyro.value() - setpoint
        self.gyro_integral += (error * (self.drive_cycle_end-self.drive_cycle_start))

        heading_compensate = error * K_GYRO_P + self.gyro_integral * K_GYRO_I

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
        self.dist_left = (self.drive_left.position /360 * K_EV3_RIM)
        self.dist_right = (self.drive_right.position / 360 * K_EV3_RIM)
        self.dist_travelled = ((self.dist_left + self.dist_right) / 2)
        # self.logged_position.append([self.elapsed_time(), self.dist_travelled*math.cos(math.radians(self.gyro_setpoint)), self.dist_travelled*math.sin(math.radians(self.gyro_setpoint))])
   
    # DRIVE FUNCTIONS
        
    def drive_indef(self):
        while True:
            self.drive_speed_update(self.gyro_pid_update(0))
            time.sleep(K_CYCLE_TIME)

    def drive_dist(self, desired_dist):
        self.drive_zero_position()
        while True:
            self.drive_speed_update(self.gyro_pid_update(0))
            self.drive_dist_update()
            if desired_dist - K_DIST_ACC <= self.dist_travelled:
                self.drive_stop()
                self.drive_dist_update()
                break

    def drive_ultrasonic(self, safe_dist):
        while True:
            self.drive_speed_update(self.gyro_pid_update(0))
            if self.ultrasonic.value() <= safe_dist:
                self.drive_stop()
                self.drive_dist_update()
                break

    def drive_victim_ultrasonic(self, safe_dist):
        while True:
            self.drive_speed_update(self.gyro_pid_update(0))
            if self.victimUltrasonic.value() <= safe_dist:
                self.drive_stop()
                break

    # TURNING FUNCTIONS
    def drive_spot_turn(self, setpoint):
        while True:
            self.drive_turn_update(self.gyro_pid_update(setpoint))
            if setpoint - K_TURNING_ACC <= int(self.gyro.value()) <= setpoint + K_TURNING_ACC:
                self.drive_stop()
                self.gyro_zero()
                break
            
    # DETECTION FUNCTIONS

    def cycle_search(self, valueSet):
        left = valueSet[0]
        right = valueSet[1]
        if left <= K_WALL_SAFE_DIST and right <= K_WALL_SAFE_DIST:
            print("ACTION: DEAD END ", file = sys.stderr)
            self.drive_spot_turn(data.gyroSetpoint(180))
        elif K_WALL_SAFE_DIST <= left and right <= K_WALL_SAFE_DIST:
            print("ACTION: CORNER ON LEFT ", file = sys.stderr)
            self.drive_spot_turn(data.gyroSetpoint(90))
        elif K_WALL_SAFE_DIST <= right and left <= K_WALL_SAFE_DIST:
            print("ACTION: CORNER ON RIGHT ", file = sys.stderr)
            self.drive_spot_turn(data.gyroSetpoint(-90))
        elif K_WALL_SAFE_DIST <= left and K_WALL_SAFE_DIST <= right:
            print("ACTION: TEE JUNCTION ", file = sys.stderr)
            self.drive_spot_turn(data.gyroSetpoint(90))

    def find_target(self):
        data.setLedOff()
        while True:                
            self.drive_left.on(10)
            self.drive_right.on(-10)
            if self.victimUltrasonic.value() < 400:
                self.drive_stop()
                self.find_target_center()
                break

    def find_target_center(self):
        edge_initial = self.gyro.value()
        edge_final = 0
        while True:
            self.drive_left.on(3)
            self.drive_right.on(-3)
            if self.victimUltrasonic.value() > 400:
                self.drive_stop()
                edge_final = self.gyro.value()
                break
        self.drive_spot_turn(data.average(edge_initial, edge_final))
        self.approach_target()

    def approach_target(self):
        self.set_drive_direction(Direction.CLAWSIDE)
        self.drive_victim_ultrasonic(40)

    def id(self):
        while True:
            if self.color.value() == 2:
                print('BLUE', file = sys.stderr)
                data.setLed('GREEN')
                time.sleep(300)
                break
            elif self.color.value() == 5:
                print('RED', file = sys.stderr)
                data.setLed('RED')
                time.sleep(300)
                break
            else:
                print('INVALID COLOR', file = sys.stderr)
                data.setLedOff()
        

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

    # CLAW
    def retract_claw(self):
        self.claw.on_to_position(100, 2100)
    
    def extend_claw(self):
        self.claw.on_to_position(100,0)

class Direction(Enum):
    CLAWSIDE = -1
    WHEELSIDE = 1