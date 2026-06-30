from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, GyroSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

hub = PrimeHub()
hub.imu.reset_heading(0)

motorL = Motor(Port.A)
motorR = Motor(Port.B)

kP=2
checkNegative = 0

def findYaw():
    yaw = hub.imu.heading()
    if checkNegative > 0 and yaw < 0:
        return yaw + 360

    elif checkNegative < 0 and yaw > 0:
        return yaw - 360
    
    else:
        return yaw

def tDegrees():
    return (abs(motorR)+abs(motorL))/2

def gyroMove(degrees, speed, targetAngle):
    motorR.reset_angle()
    motorL.reset_angle()

    while tDegrees() < abs(degrees):
        error = targetAngle-findYaw()
        correction = kP*error
        motorR.run(speed+correction)
        motorL.run(speed-correction)

def yawTurn(angle, speed1, speed2, motorSplit, direction, tolerance):
    yawStart=findYaw()
    error=angle-findYaw()
    halfAngle=error/2
    speed=speed1

    while not abs(error) < tolerance :
        error=angle-findYaw()
        speed=speed2+((speed1-speed2)*(error/angle))
        if direction = 'R' :
            motorL.run(speed)
            motorR.run(speed*motorSplit)
        elif direction = 'L' :
            motorR.run(speed)
            motorL.run(speed*motorSplit)
        else :
            if speed1*angle > 0 :
                motorL.run(speed)
                motorR.run(speed*motorSplit)
            else:
                motorR.run(speed)
                motorL.run(speed*motorSplit)

def stopMoving() :
    motorL.stop()
    motorR.stop()
'''
gyroMove(400,50,0)
yawTurn(45,40,100,0.5,'R',5)
yawTurn(90,100,40,0.5,'R',5)
gyroMove(400,50,90)
checkNegative=1
yawTurn(135,40,100,0.5,'R',5)
yawTurn(180,100,40,0.5,'R',5)
gyroMove(400,50,180)
yawTurn(225,40,100,0.5,'R',5)
yawTurn(270,100,40,0.5,'R',5)
checkNegative=0
gyroMove(400,50,-90)
yawTurn(-45,40,100,0.5,'R',5)
yawTurn(0,100,40,0.5,'R',5)
gyroMove(400,50)
'''