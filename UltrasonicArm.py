from Constants import *
from ev3dev2.sensor.lego import GyroSensor, UltrasonicSensor

import time,sys

class UltrasonicArm:
    def __init__(self, arm_rot, gyro, ultrasonic):
        self.arm_rot = arm_rot
        self.gyro = gyro
        self.ultrasonic = ultrasonic

    def us_arm_pid_update(self, setpoint):
        error = self.ultrasonic.value() - setpoint
        compensate = error * USARM_P
        return compensate

    def us_locate(self):
        # Start drive locate function
        target_dict = {}
        i = 0
        while i < 200:
            target_dict[self.gyro.value()] = self.ultrasonic.value()
            i += 1
            time.sleep(CYCLE_TIME)
        print(target_dict, file = sys.stderr)
