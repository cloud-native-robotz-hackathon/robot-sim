import time
from tornado import httpserver
from tornado import gen
from tornado.ioloop import IOLoop
import tornado.web
#from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor, DistanceSensor

# create the Robot instance.
robot = Robot()

# max_wheel_speed = 1000
# num_dist_sensors = 8
# encoder_resolution = 159.23 # for wheel encoders
# tempo = 0.5  # Upper velocity bound = Fraction of the robot's maximum velocity = 1000 = 1 wheel revolution/sec  
# wheel_diameter = 4.1 # centimeters
# axle_length = 5.3 # centimeters

#TIME_STEP = 64
TIME_STEP = int(robot.getBasicTimeStep())
MAX_SPEED = float(6.28)
#ROBOT_ANGULAR_SPEED_IN_DEGREES = float(360)
ROBOT_ANGULAR_SPEED_IN_DEGREES = float(283.588111888)

port = robot.getCustomData()
print("Port: ", port)

# initialize sensors
# ps = []
# psNames = [
#     'ps0', 'ps1', 'ps2', 'ps3',
#     'ps4', 'ps5', 'ps6', 'ps7'
# ]

# for i in range(8):
#     ps.append(robot.getDevice(psNames[i]))
#     ps[i].enable(TIME_STEP)

# initialize and set motor devices
leftMotor = robot.getDevice('left wheel')
rightMotor = robot.getDevice('right wheel')
leftSensor = robot.getDevice("left wheel sensor")
leftSensor.enable(TIME_STEP)
rightSensor = robot.getDevice("right wheel sensor")
rightSensor.enable(TIME_STEP)
# Set to endless rotational motion
leftMotor.setPosition(float('+inf'))
rightMotor.setPosition(float('+inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

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

class LeftturnHandler(tornado.web.RequestHandler):
    def get(self):
        duration = 90 / ROBOT_ANGULAR_SPEED_IN_DEGREES
        leftMotor.setVelocity(-MAX_SPEED)
        rightMotor.setVelocity(MAX_SPEED)
        start_time = float(robot.getTime())
        end_time = start_time + duration
        #print("START " + str(start_time))
        #print("START " + str(end_time))
        while float(robot.getTime()) < float(end_time):
            #print("turning")
            #print(float(robot.getTime()))
            robot.step(TIME_STEP)
        leftMotor.setVelocity(0)
        rightMotor.setVelocity(0)
        self.write('OK')

class Left90Handler(tornado.web.RequestHandler):
    def get(self):
        leftMotor.setVelocity(MAX_SPEED)
        rightMotor.setVelocity(MAX_SPEED)
        leftOffset = leftSensor.getValue()
        rightOffset = rightSensor.getValue()
        leftMotor.setPosition(leftOffset + 2.25)
        rightMotor.setPosition(rightOffset - 2.25)
        #leftMotor.setPosition(leftOffset + 3.14)
        #rightMotor.setPosition(rightOffset - 3.14)
        self.write('OK')

class RightturnHandler(tornado.web.RequestHandler):
    def get(self):
        duration = 90 / ROBOT_ANGULAR_SPEED_IN_DEGREES
        leftMotor.setVelocity(MAX_SPEED)
        rightMotor.setVelocity(-MAX_SPEED)
        start_time = float(robot.getTime())
        end_time = start_time + duration
        #print("START " + str(start_time))
        #print("START " + str(end_time))
        while float(robot.getTime()) < float(end_time):
            #print("turning")
            #print(float(robot.getTime()))
            robot.step(32)
        leftMotor.setVelocity(0)
        rightMotor.setVelocity(0)
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
            (r"/forward/?", ForwardHandler),
            (r"/stop/?", StopHandler),
            (r"/left/?", LeftturnHandler),
            (r"/right/?", RightturnHandler),
            (r"/90/?", Left90Handler),
            (r"/distance/?", DistanceHandler),
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
