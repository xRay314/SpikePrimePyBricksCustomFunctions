from systems.robot import Bot
from controls.constants import *

robot = Bot()

def intialize():
    robot.hub.imu.reset_heading(0) 
    robot.move.reset_angles()
    print(robot.hub.battery.voltage(), "mv")

def test_gyro():
    print("Yaw:", robot.imu.heading())
    print("Drive Motor Right:", robot.right_drive_motor.angle())
    print("Drive Motor Left:", robot.left_drive_motor.angle())
    print("Line Sensor Right:", robot.right_line_sensor.reflection())
    print("Line Sensor Left:", robot.left_line_sensor.reflection())

def section1():
    robot.move.gyro_move(500,0,30,100,0.3,True,True)
    robot.move.stop_moving()