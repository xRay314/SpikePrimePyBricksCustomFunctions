from .pid import PID_values
from math import pi

gyro_move_values = PID_values(
    kp=5, 
    ki=0, 
    kd=0,
    iLimit=100)

line_follow_values = PID_values(
    kp=0.5, 
    ki=0, 
    kd=0,
    iLimit=100)

line_follow_reflection = 60

wheel_diameter = 62.4
mmPerDegree = (pi * wheel_diameter) / 360
