import time
import math
from tornado import httpserver
from tornado import gen
from tornado.ioloop import IOLoop
import tornado.web
#from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor, DistanceSensor

# create the Robot instance.
robot = Robot()

#TIME_STEP = 64
TIME_STEP = int(robot.getBasicTimeStep())
MAX_SPEED = float(6)

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

class ForwardHandler(tornado.web.RequestHandler):
    def get(self):
        leftMotor.setPosition(float('+inf'))
        rightMotor.setPosition(float('+inf'))
        leftMotor.setVelocity(3)
        rightMotor.setVelocity(3)
        self.write('OK')

class ForwardTimeHandler(tornado.web.RequestHandler):
    def get(self, value):
        leftMotor.setPosition(float('+inf'))
        rightMotor.setPosition(float('+inf'))
        leftMotor.setVelocity(MAX_SPEED)
        rightMotor.setVelocity(MAX_SPEED)
        start_time = float(robot.getTime())
        end_time = start_time + float(value)
        while float(robot.getTime()) < float(end_time):
            robot.step(TIME_STEP)
        leftMotor.setVelocity(0)
        rightMotor.setVelocity(0)
        self.write('OK')

class StopHandler(tornado.web.RequestHandler):
    def get(self):
        leftMotor.setVelocity(0)
        rightMotor.setVelocity(0)
        self.write('OK')

class TurndegreesHandler(tornado.web.RequestHandler):
    def get(self):
        deg = 90
        axle_length = 0.32
        radius_wheel = 0.0825
        #leftMotor.setVelocity(MAX_SPEED)
        #rightMotor.setVelocity(MAX_SPEED)
        leftPos = leftSensor.getValue()
        rightPos = rightSensor.getValue()
        # convert degree angle into radian
        angle = deg * 3.1415926535 / 180
        # calculate how much more the left wheel has to rotate vs the right, to turn that angle
        detla_motor_pos = angle * axle_length / radius_wheel
        # add and substract half of detla_motor_pos to current motor positions 
        lefttargetPos = leftPos + detla_motor_pos / 2
        righttargetPos = rightPos - detla_motor_pos / 2
        print("left: " + str(lefttargetPos) + " right: " + str(righttargetPos))
        leftMotor.setPosition(lefttargetPos)
        rightMotor.setPosition(righttargetPos)
        self.write('OK')

class LefttimeHandler(tornado.web.RequestHandler):
    def get(self):
        leftMotor.setPosition(float('+inf'))
        rightMotor.setPosition(float('+inf'))
        start_time = float(robot.getTime())
        duration = 0.57
        end_time = start_time + duration
        print("START " + str(start_time))
        print("END " + str(end_time))
        leftMotor.setVelocity(-6)
        rightMotor.setVelocity(6)
        while float(robot.getTime()) < float(end_time):
            #leftMotor.setVelocity(-6)
            #rightMotor.setVelocity(6)
            robot.step(TIME_STEP)
        leftMotor.setVelocity(0)
        rightMotor.setVelocity(0)
        self.write('OK')

class LeftposHandler(tornado.web.RequestHandler):
    def get(self):
        leftMotor.setVelocity(-MAX_SPEED)
        rightMotor.setVelocity(MAX_SPEED)
        leftPos = leftSensor.getValue()
        rightPos = rightSensor.getValue()
        print("left: " + str(leftPos) + " right: " + str(rightPos))
        lefttargetPos = leftPos + 10
        righttargetPos = rightPos + 10
        print("left: " + str(lefttargetPos) + " right: " + str(righttargetPos))
        #while float(leftSensor.getValue()) < targetPos:
        leftMotor.setPosition(lefttargetPos)
        rightMotor.setPosition(righttargetPos)
        self.write('OK')

class ForwardpHandler(tornado.web.RequestHandler):
    def get(self):
        leftMotor.setVelocity(MAX_SPEED)
        rightMotor.setVelocity(MAX_SPEED)
        leftOffset = leftSensor.getValue()
        rightOffset = rightSensor.getValue()
        print("left: " + str(leftOffset) + " right: " + str(rightOffset))
        leftMotor.setPosition(leftOffset + 10)
        rightMotor.setPosition(rightOffset + 10)
        self.write('OK')

class PosHandler(tornado.web.RequestHandler):
    def get(self):
        leftOffset = leftSensor.getValue()
        rightOffset = rightSensor.getValue()
        print("left: " + str(leftOffset) + " right: " + str(rightOffset))
        self.write('OK')
        

class RightturnHandler(tornado.web.RequestHandler):
    def get(self):
        #duration = 90 / ROBOT_ANGULAR_SPEED_IN_DEGREES
        leftMotor.setVelocity(-MAX_SPEED)
        rightMotor.setVelocity(MAX_SPEED)
        leftMotor.setPosition(-10)
        rightMotor.setPosition(10)
        #start_time = float(robot.getTime())
       # end_time = start_time + duration
        #print("START " + str(start_time))
        #print("START " + str(end_time))
        #while float(robot.getTime()) < float(end_time):
            #print("turning")
            #print(float(robot.getTime()))
        #    robot.step(32)
        #leftMotor.setVelocity(0)
        #rightMotor.setVelocity(0)
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

class IuHandler(tornado.web.RequestHandler):
    def get(self):
        # read sensors outputs
        iu = inertialUnit.getRollPitchYaw()
        print(iu)
        #iudeg = iu[2] * 180.0 / 3.14
        #if iudeg < 0:
        #  iudeg += 360.0
        #print(int(iudeg))
        self.write(str(iu[2]))
        #self.write(str(iudeg))
        
class Application(tornado.web.Application):
    def __init__(self):
        handlers = [
            (r"/?", MainHandler),
            (r"/forward/?", ForwardHandler),
            (r"/stop/?", StopHandler),
            (r"/leftt/?", LefttimeHandler),
            (r"/turn/?", TurndegreesHandler),
            (r"/right/?", RightturnHandler),
            (r"/forwardp/?", ForwardpHandler),
            (r"/distance/?", DistanceHandler),
            (r"/getiu/?", IuHandler),
            (r"/pos/?", PosHandler),
            (r"/forward/(.*)", ForwardTimeHandler),
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
