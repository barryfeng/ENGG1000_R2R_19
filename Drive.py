from Constants import *

class Drive:
    def __init__(self, drive_left, drive_right, gyro):
        self.drive_left = drive_left
        self.drive_right = drive_right
        self.gyro = gyro

        self.dist_left = 0
        self.dist_right = 0

        self.integral = 0
        self.last_error = 0

    def drive_update(self, compensate):
        speed_left =  DRIVE_SPEED
        speed_right = DRIVE_SPEED
        self.drive_left.on(speed_left - compensate, brake= True)
        self.drive_right.on(speed_right + compensate, brake=True)
        self.drive_dist_update()
        print(compensate)

    def drive_stop(self):
        self.drive_left.stop()
        self.drive_right.stop()

    def drive_dist_update(self):
        self.dist_left += (self.drive_left.speed/self.drive_left.count_per_rot) * CYCLE_TIME
        self.dist_right += (self.drive_right.speed/self.drive_right.count_per_rot) * CYCLE_TIME
        #print('Left: ' + str(self.dist_left))
        #print('Right: ' + str(self.dist_right))

    def drive_pid_update(self, setpoint):
        error = self.gyro.value() - setpoint
        self.integral += (error * CYCLE_TIME)
        derivative = (error - self.last_error) / CYCLE_TIME

        compensate = error * GYRO_P + self.integral * GYRO_I + derivative * GYRO_D

        self.last_error = error
        return compensate

    # def basic_straight(self):
    #     self.drive_left.on_for_rotations(10)
    #     self.drive_right.on_for_rotations(10)

    # def basic_turn_left(self):
    #     self.drive_left.stop()
    #     self.drive_right.on_for_degrees(360)
