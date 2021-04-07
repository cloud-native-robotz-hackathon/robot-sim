import time
import math
import threading
from tornado import httpserver
from tornado import gen
from tornado.ioloop import IOLoop
import tornado.web
#from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor, DistanceSensor

# create the Robot instance.
robot = Robot()
lock = threading.Lock()

#TIME_STEP = 64
TIME_STEP = int(robot.getBasicTimeStep())
MAX_SPEED = float(6)
WHEEL_DIAMETER = 0.096
axle_length = 0.33
radius_wheel = 0.048
DEFAULT_SPEED = 6.28
DRIVE_FACTOR = 12.2 # positions to meters
TURN_FACTOR = 30 

port = robot.getCustomData()
print("Port: ", port)

# init inertialunit
#inertialUnit = robot.getDevice('inertial unit')
#inertialUnit.enable(TIME_STEP)

# initialize and set motor devices
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftSensor = robot.getDevice("left wheel sensor")
leftSensor.enable(TIME_STEP)
rightSensor = robot.getDevice("right wheel sensor")
rightSensor.enable(TIME_STEP)
# Set to endless rotational motion
#leftMotor.setPosition(float('+inf'))
#rightMotor.setPosition(float('+inf'))
#leftMotor.setVelocity(0.0)
#rightMotor.setVelocity(0.0)
#leftMotor.setVelocity(float('inf'))
#rightMotor.setVelocity(float('inf'))

class MainHandler(tornado.web.RequestHandler):
    def get(self):
        self.write("Robot Ready!")
class StopHandler(tornado.web.RequestHandler):
    def get(self):
        leftMotor.setVelocity(0)
        rightMotor.setVelocity(0)
        self.write('OK')
class RightHandler(tornado.web.RequestHandler):
    def get(self, deg):
        lock.acquire()
        #leftMotor.setVelocity(MAX_SPEED)
        #rightMotor.setVelocity(MAX_SPEED)
        leftPos = leftSensor.getValue()
        rightPos = rightSensor.getValue()
        # convert degree angle into radian
        angle = float(deg) * 3.1415926535 / 180
        # calculate how much more the left wheel has to rotate vs the right, to turn that angle
        delta_motor_pos = angle * axle_length / radius_wheel
        # add and substract half of delta_motor_pos to current motor positions 
        lefttargetPos = leftPos + delta_motor_pos / 2
        righttargetPos = rightPos - delta_motor_pos / 2
        print("left: " + str(lefttargetPos) + " right: " + str(righttargetPos))
        leftMotor.setPosition(lefttargetPos)
        rightMotor.setPosition(righttargetPos)
        lock.release()
        self.write('OK')
class LeftHandler(tornado.web.RequestHandler):
    def get(self, deg):
        lock.acquire()
        #leftMotor.setVelocity(MAX_SPEED)
        #rightMotor.setVelocity(MAX_SPEED)
        leftPos = leftSensor.getValue()
        rightPos = rightSensor.getValue()
        # convert degree angle into radian
        angle = float(deg) * 3.1415926535 / 180
        # calculate how much more the left wheel has to rotate vs the right, to turn that angle
        delta_motor_pos = angle * axle_length / radius_wheel
        # add and substract half of delta_motor_pos to current motor positions 
        lefttargetPos = leftPos - delta_motor_pos / 2
        righttargetPos = rightPos + delta_motor_pos / 2
        print("left: " + str(lefttargetPos) + " right: " + str(righttargetPos))
        leftMotor.setPosition(lefttargetPos)
        rightMotor.setPosition(righttargetPos)
        lock.release()
        self.write('OK')
class ForwardHandler(tornado.web.RequestHandler):
    def get(self, length):
        lock.acquire()
        leftMotor.setVelocity(MAX_SPEED)
        rightMotor.setVelocity(MAX_SPEED)
        leftOffset = leftSensor.getValue()
        rightOffset = rightSensor.getValue()
        print("left: " + str(leftOffset) + " right: " + str(rightOffset))
        leftMotor.setPosition(leftOffset + float(length))
        rightMotor.setPosition(rightOffset + float(length))
        lock.release()
        self.write('OK')
class DistanceHandler(tornado.web.RequestHandler):
    def get(self):
        # read sensors outputs
        psValues = []
        for i in range(8):
            psValues.append(ps[i].getValue())
        sensor0 = str(psValues[0])
        sensor7 = str(psValues[7])
        self.write(sensor0 + "\n")
        self.write(sensor7 + "\n")
class Application(tornado.web.Application):
    def __init__(self):
        handlers = [
            (r"/?", MainHandler),
            (r"/stop/?", StopHandler),
            (r"/left/(\w+)", LeftHandler),
            (r"/right/(\w+)", RightHandler),
            (r"/forward/(\w+)", ForwardHandler),
            (r"/distance/?", DistanceHandler)
        ]
        tornado.web.Application.__init__(self, handlers)

def robotstep():
    robot.step(TIME_STEP)
    #print("robotstep")

if __name__ == '__main__':
    app = Application()
    app.listen(port)
    tornado.ioloop.PeriodicCallback(robotstep, 1).start()
    tornado.ioloop.IOLoop.instance().start()
