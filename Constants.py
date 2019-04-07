#!/usr/bin/env python3

import math

CYCLE_TIME = 0.05 # SECONDS

DRIVE_SPEED = 50 # PERCENT

K_TURNING_SPEED = 0.75

K_USARM_P = 0.8

GYRO_P = 0.95
GYRO_I = 0

#Sensor Values
COLOR_BLUE = 1
COLOR_RED = 2

#Errors
TURNING_ACC = 0.5 # +- DEG
DIST_ACC = 5 # +- MM

EV3_RIM = 56 * math.pi

US_SAFE_DIST = 300

K_WALL_SAFE_DIST = 300
