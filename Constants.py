#!/usr/bin/env python3

import math

CYCLE_TIME = 0.1 # SECONDS

DRIVE_LEFT_P = 0
DRIVE_LEFT_I = 0
DRIVE_LEFT_D = 0

DRIVE_RIGHT_P = 0
DRIVE_RIGHT_I = 0
DRIVE_RIGHT_D = 0

DRIVE_SPEED = 100 # PERCENT

USARM_P = 1
USARM_I = 1
USARM_D = 1

<<<<<<< HEAD
GYRO_P = 0.85
GYRO_I = 0  
=======
GYRO_P = 3
GYRO_I = 0
GYRO_D = 0
>>>>>>> cb02dd917ad98d9bbe37a5d5f757c9209b9e4aa4

US_P = 0.0
US_I = 0
US_D = 0

#Enablers
DRIVE_PID_LOGGING = True
DRIVE_POS_LOGGING = True

#Sensor Values
COLOR_BLUE = 1
COLOR_RED = 2

#Errors
TURNING_ACC = 0.25 # +- DEG
DIST_ACC = 5 # +- MM

EV3_RIM = 56 * math.pi

