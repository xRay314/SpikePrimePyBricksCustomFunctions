from pybricks.hubs import PrimeHub
from pybricks.parameters import Direction, Port, Axis
from pybricks.pupdevices import ColorSensor, Motor

from systems.drive import moveSys
#from systems.arm import armSys

class Bot():
    def __init__(self):
        self.hub = PrimeHub(top_side=Axis.X,front_side=Axis.Y)
        self.right_drive_motor = Motor(Port.A, Direction.COUNTERCLOCKWISE)
        self.left_drive_motor = Motor(Port.E, Direction.CLOCKWISE)

        self.right_arm_motor = Motor(Port.B, Direction.CLOCKWISE)
        self.left_arm_motor = Motor(Port.F, Direction.COUNTERCLOCKWISE)

        self.right_line_sensor = ColorSensor(Port.B)
        self.left_line_sensor = ColorSensor(Port.A)
        
        self.move = moveSys(
            self.hub,
            self.right_drive_motor,
            self.left_drive_motor,
            self.right_line_sensor,
            self.left_line_sensor
        )

        '''
        self.arms = armSys(
            self.right_arm_motor,
            self.left_arm_motor
        )
        '''
