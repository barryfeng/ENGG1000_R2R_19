from Constants import *

class UltrasonicArm:
    def __init__(self, arm_rot, us_sensor):
        self.arm_rot = arm_rot
        self.us_sensor = us_sensor

    def us_arm_pid_update(self, setpoint):
        error = self.us_sensor.value() - setpoint
        compensate = error * USARM_P
        return compensate
