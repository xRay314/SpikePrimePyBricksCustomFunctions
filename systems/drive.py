from pybricks.hubs import PrimeHub
from pybricks.parameters import Direction, Port, Axis, Color
from pybricks.pupdevices import ColorSensor, Motor

from controls.pid import PID_control
from controls.constants import gyro_move_values, line_follow_values, mmPerDegree

class moveSys:
    def __init__(self, hub, right_drive_motor, left_drive_motor, right_line_sensor, left_line_sensor):
        #Setup Hub
        self.hub = hub

        #Setup Right Motor
        self.right = right_drive_motor
        self.right.reset_angle()

        #Setup Left Motor
        self.left = left_drive_motor
        self.left.reset_angle()

        #Setup Line Sensors
        self.rightSensor = right_line_sensor
        self.leftSennsor = left_line_sensor

        #Setup Gyro for PID
        self.gyro_pid = PID_control(
            gyro_move_values.kp,
            gyro_move_values.ki,
            gyro_move_values.kd,
            gyro_move_values.iLimit)
        

        #Setup Line Follower for PID
        self.line_follow_pid = PID_control(
            line_follow_values.kp,
            line_follow_values.ki,
            line_follow_values.kd,
            line_follow_values.iLimit)
    def tDistance(self, MM):
        if MM == False:
            return (abs(self.right.angle()) + abs(self.left.angle())) / 2
        elif MM == True:
            return self.calcMMDistance(abs(self.right.angle()) + abs(self.left.angle()) /2)
    
    def calcMMDistance(self, distance):
        return distance * mmPerDegree
    
    def stop_moving(self):
        self.right.brake()
        self.left.brake()

    def reset_angles(self):
        self.right.reset_angle()
        self.left.reset_angle()

    def gyro_move(self, distance, targetAngle, lowSpeed, highSpeed, ratio, accel=False, decel=False):
        self.reset_angles()
        self.gyro_pid.reset()

        self.total_distance_MM = abs(self.calcMMDistance(distance))
        self.travelled_distance_MM = 0
        self.distance_remaining_MM = self.total_distance_MM - self.travelled_distance_MM
        
        self.speed = lowSpeed
        # ratio =0.3 d
        self.accelDistance = self.total_distance_MM * ratio # eg 30
        self.decelDistance = self.total_distance_MM * (1 - ratio)

        while self.distance_remaining > 0:
            self.travelled_distance_MM = self.tDistance(True)
            self.distance_remaining_MM = self.total_distance_MM - self.travelled_distance_MM

            if accel == True and self.travelled_distance <= self.accelDistance:
                self.speed = lowSpeed+(highSpeed-lowSpeed)*(self.travelled_distance/self.accelDistance) 
            elif decel == True and self.travelled_distance >= self.decelDistance:
                self.speed = lowSpeed-(highSpeed-lowSpeed)*(self.remaining_distance/self.accelDistance)
            else :
                self.speed = lowSpeed

            self.current_yaw=self.hub.imu.heading()
            self.correction=self.gyro_pid.compute(targetAngle, self.current_yaw)
            self.right.dc(self.speed+self.correction)
            self.left.dc(self.speed-self.correction)

    def gyro_turn(self, angle, motorSplit, side, lowSpeed, highSpeed, ratio, accel=False, decel=False, tolerance=5):
        self.current_yaw= self.hub.imu.heading()
        self.start_angle = self.current_yaw
        self.error = angle - self.current_yaw

        self.total_angle = self.error
        self.turned_angle = 0
        self.angle_remaining = self.total_angle - self.turned_angle
        
        self.speed = lowSpeed
        # ratio =0.3 d
        self.accel_angle = abs(self.total_angle * ratio) # eg 30
        self.decel_angle = abs(self.total_angle * (1 - ratio))

        while abs(self.error) > tolerance:
            self.currentr_yaw = self.hub.imu.heading()
            self.error = angle - self.current_yaw
            
            self.turned_angle = abs(self.start_angle - self.current_yaw)

            if accel == True and self.turned_angle <= self.accelDistance:
                self.speed = lowSpeed+(highSpeed-lowSpeed)*(self.turned_angle/self.accel_angle) 
            elif decel == True and self.turned_angle >= self.decelDistance:
                self.speed = lowSpeed-(highSpeed-lowSpeed)*(self.error/self.decel_angle)
            else :
                self.speed = lowSpeed
            
            if side == 'R':
                self.right.dc(self.speed*motorSplit)
                self.left.dc(self.speed)
            elif side == 'L':
                self.right.dc(self.speed)
                self.left.dc(self.speed*motorSplit)
            else:
                if lowSpeed*angle > 0:
                    self.right.dc(self.speed*motorSplit)
                    self.left.dc(self.speed)
                else:
                    self.right.dc(self.speed)
                    self.left.dc(self.speed*motorSplit)


    def line_follow(self, distance, lowSpeed, highSpeed, ratio, accel=False, decel=False):
        self.reset_angles()
        self.line_follow_pid.reset()

        self.total_distance_MM = abs(self.calcMMDistance(distance))
        self.travelled_distance_MM = 0
        self.distance_remaining_MM = self.total_distance_MM - self.travelled_distance_MM
        
        self.speed = lowSpeed
        # ratio =0.3 d
        self.accelDistance = self.total_distance_MM * ratio # eg 30
        self.decelDistance = self.total_distance_MM * (1 - ratio)

        while self.distance_remaining > 0 :
            self.travelled_distance_MM = self.tDistance(True)
            self.distance_remaining_MM = self.total_distance_MM - self.travelled_distance_MM

            if accel == True and self.travelled_distance <= self.accelDistance:
                self.speed = lowSpeed+(highSpeed-lowSpeed)*(self.travelled_distance/self.accelDistance) 
            elif decel == True and self.travelled_distance >= self.decelDistance:
                self.speed = lowSpeed-(highSpeed-lowSpeed)*(self.remaining_distance/self.accelDistance)
            else :
                self.speed = lowSpeed

            
            self.correction=self.gyro_pid.compute(self.leftSennsor.reflection(), self.rightSennsor.reflection())
            self.right.dc(self.speed+self.correction)
            self.left.dc(self.speed-self.correction)