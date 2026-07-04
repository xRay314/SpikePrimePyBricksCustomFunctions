from missions.test1 import *
from pybricks.tools import StopWatch

print("Successful Start")

runTimer = StopWatch()



intialize()
test_gyro()

print("time:", runTimer.time() / 1000, "s")