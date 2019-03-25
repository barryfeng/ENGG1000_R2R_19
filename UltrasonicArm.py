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

    # def ultrasound_arm_calibrate(self):
#     sound.speak('Move the arm to right hardstop. Calibrating arm in two seconds!')
#     us_sensor.mode = 'US-DIST-CM'
#     ultrasound_arm.stop_action = 'HOLD'
#     time.sleep(2)
#     ultrasound_arm.position = 0
#     ultrasound_arm.on_to_position(100, 90, True)
#     ultrasound_arm.position = 0
#     sound.speak('Arm calibration complete.')

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
