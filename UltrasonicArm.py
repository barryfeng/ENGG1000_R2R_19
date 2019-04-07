from Constants import *
from ev3dev2.sensor.lego import GyroSensor, UltrasonicSensor

import time,sys

class UltrasonicArm:
    def __init__(self, arm_motor, gyro, ultrasonic):
        self.arm_motor = arm_motor
        self.gyro = gyro
        self.ultrasonic = ultrasonic

    def us_arm_init(self):
        print("Init")
        self.arm_motor.stop_action = self.arm_motor.STOP_ACTION_HOLD
        self.arm_motor.position_p = K_USARM_P

    def us_locate(self):
        # Start drive locate function
        target_dict = {}
        i = 0
        while i < 200:
            target_dict[self.gyro.value()] = self.ultrasonic.value()
            i += 1
            time.sleep(CYCLE_TIME)
        print(target_dict, file = sys.stderr)

    def us_maze_detect(self):
        valueSet = []
        self.arm_motor.on_for_degrees(25,90)
        valueSet.append(self.ultrasonic.value())
        time.sleep(0.1)
        self.arm_motor.on_for_degrees(100,-180)
        valueSet.append(self.ultrasonic.value())
        time.sleep(0.1)
        self.arm_motor.on_for_degrees(100,90)
        self.arm_motor.stop()
        print(valueSet,file=sys.stderr)
        return valueSet

    # # Detect Target
    # def detect_target(self):
    #     gyro_zero()
    #     target_list = []
    #     ultrasound_arm.on_to_position(100, -90, True)
    #     ultrasound_arm.on_to_position(10, 90, True)
    #     if us.value() < 300:
    #         print("Object Found")
    #         target_list.append(ultrasound_arm.position)
    #     return target_list

    # Go to target
    # def rescue(self,target_angle):
    #   Drive.drive_start(update_gyro_pid(target_angle))
