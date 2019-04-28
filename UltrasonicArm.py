from Constants import *
from Data import *
from ev3dev2.sensor.lego import GyroSensor, UltrasonicSensor

import time,sys

data = Data()

class UltrasonicArm:
    def __init__(self, arm_motor, gyro, ultrasonic):
        self.arm_motor = arm_motor
        self.gyro = gyro
        self.ultrasonic = ultrasonic

    def us_arm_init(self):
        self.arm_motor.stop_action = self.arm_motor.STOP_ACTION_HOLD
        self.arm_motor.position_p = K_USARM_P

    def us_motor_zero(self):
        self.arm_motor.position = 0

    def us_maze_detect(self):
        valueSet = []
        self.arm_motor.on_for_degrees(25,90)
        valueSet.append(self.ultrasonic.value())
        time.sleep(0.5)
        self.arm_motor.on_for_degrees(100,-180)
        valueSet.append(self.ultrasonic.value())
        time.sleep(0.5)
        self.arm_motor.on_for_degrees(100,90)
        return valueSet

    def exit_check(self):
        valueSet = []
        self.us_motor_zero()
        self.arm_motor.run_to_abs_pos(90)
        valueSet.append(self.ultrasonic.value())
        self.arm_motor.run_to_abs_pos(-90)
        valueSet.append(self.ultrasonic.value())
        self.arm_motor.run_to_abs_pos(0)
        return valueSet