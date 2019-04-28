#!/usr/bin/env python3

import math

K_CYCLE_TIME = 0.05 # SECONDS

K_DRIVE_SPEED = 50 # PERCENT
K_TURNING_SPEED = 0.75

K_USARM_P = 0.0
K_GYRO_P = 0.95
K_GYRO_I = 0

#Errors
K_TURNING_ACC = 0.5 # +- DEG
K_DIST_ACC = 5 # +- MM

K_EV3_RIM = 60 * math.pi

K_US_SAFE_DIST = 220
K_WALL_SAFE_DIST = 300
