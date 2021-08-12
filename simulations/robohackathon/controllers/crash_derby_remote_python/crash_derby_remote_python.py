import time
import math
import threading
from tornado import httpserver
from tornado import gen
from tornado.ioloop import IOLoop
import tornado.web
#from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor, DistanceSensor, InertialUnit
from controller import LED

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
robot_name = robot.getName()
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

# get and enable camera device
camera = robot.getDevice('camera')
camera.enable(TIME_STEP)

# get and intialize imu 
iu = robot.getDevice('imu')
iu.enable(TIME_STEP)

# get and intialize LED 
led = robot.getDevice('led')

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
class RightIuHandler(tornado.web.RequestHandler):
    def get(self, deg):
        # Set to endless rotational motion
        leftMotor.setPosition(float('+inf'))
        rightMotor.setPosition(float('+inf'))
        leftMotor.setVelocity(0)
        rightMotor.setVelocity(0)
        # convert API requested degree value to radian
        angle = float(deg) * (3.1415926535 / 180)
        print("radians requested: " + str(round(angle, 2)))
        # get roll pitch yaw from IMU
        rpy = iu.getRollPitchYaw()
        # webots radians are 0->pi cw and 0->-pi ccw
        # convert negative (ccw) radians to 0->2pi to make calculation easier
        if rpy[2] < 0:
            curyaw = rpy[2] + 2 * math.pi
        else:
            curyaw = rpy[2]
        print("current orientation: " + str(round(curyaw, 2)))
        targetyaw = curyaw - angle
        # in case 0/2pi value was passed
        if targetyaw < 0:
            targetyaw = targetyaw + 6.28 
        print("target orientation (" + str(round(curyaw, 2)) + " - " + str(round(angle, 2)) + "): " + str(round(targetyaw, 2)))
        
        leftMotor.setVelocity(2)
        rightMotor.setVelocity(-2)
        print("starting to turn..........")
        while not math.isclose(curyaw, targetyaw, abs_tol = 0.02):
            robot.step(32)
            rpy = iu.getRollPitchYaw()
            if rpy[2] < 0:
                curyaw = rpy[2] + 2 * math.pi
            else:
                curyaw = rpy[2]
            #print("current_loop: " + str(round(curyaw, 2)))
            
        leftMotor.setVelocity(0)
        rightMotor.setVelocity(0)

        rpy = iu.getRollPitchYaw()
        if rpy[2] < 0:
            curyaw = rpy[2] + 2 * math.pi
        else:
            curyaw = rpy[2]
        print("final orientation: " + str(round(curyaw, 2)))
        print("OK")
        self.write('OK')

class LeftIuHandler(tornado.web.RequestHandler):
    def get(self, deg):
        # Set to endless rotational motion
        leftMotor.setPosition(float('+inf'))
        rightMotor.setPosition(float('+inf'))
        leftMotor.setVelocity(0)
        rightMotor.setVelocity(0)
        # convert API requested degree value to radian
        angle = float(deg) * (3.1415926535 / 180)
        print("radians requested: " + str(round(angle, 2)))
        # get roll pitch yaw from IMU
        rpy = iu.getRollPitchYaw()
        # webots radians are 0->pi cw and 0->-pi ccw
        # convert negative (ccw) radians to 0->2pi to make calculation easier
        if rpy[2] < 0:
            curyaw = rpy[2] + 2 * math.pi
        else:
            curyaw = rpy[2]
        print("current orientation: " + str(round(curyaw, 2)))
        targetyaw = curyaw + angle
        # in case 0/2pi value was passed
        if targetyaw > 2 * math.pi:
            targetyaw = targetyaw - 6.28 
        print("target orientation (" + str(round(curyaw, 2)) + " + " + str(round(angle, 2)) + "): " + str(round(targetyaw, 2)))
        
        leftMotor.setVelocity(-2)
        rightMotor.setVelocity(2)
        print("starting to turn..........")
        while not math.isclose(curyaw, targetyaw, abs_tol = 0.02):
            robot.step(32)
            rpy = iu.getRollPitchYaw()
            if rpy[2] < 0:
                curyaw = rpy[2] + 2 * math.pi
            else:
                curyaw = rpy[2]
            #print("current_loop: " + str(round(curyaw, 2)))
            
        leftMotor.setVelocity(0)
        rightMotor.setVelocity(0)

        rpy = iu.getRollPitchYaw()
        if rpy[2] < 0:
            curyaw = rpy[2] + 2 * math.pi
        else:
            curyaw = rpy[2]
        print("final orientation: " + str(round(curyaw, 2)))
        print("OK")
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
        
class GetImageHandler(tornado.web.RequestHandler):
    def get(self):
        image = camera.getImage()
        camera.saveImage(robot_name + ".jpg", 100)
        #self.write(image)
        
class GetIuHandler(tornado.web.RequestHandler):
    def get(self):
        rpy = iu.getRollPitchYaw()
        if rpy[2] < 0:
            curyaw = rpy[2] + 2 * math.pi
        else:
            curyaw = rpy[2]
        #print("roll: " + str(round(curyaw, 2)))
        #print("pitch: " + str(round(curyaw, 2)))
        print("yaw raw: " + str(round(rpy[2], 2)))
        print("yaw: " + str(round(curyaw, 2)))
        #self.write(image)

class LedHandler(tornado.web.RequestHandler):
    def get(self, state):
        if state == "on":
            led.set(1)
            self.write("LED on")
        elif state == "off":
            led.set(0)
            self.write("LED off")    
        else:
            self.write("on or off")
        

class Application(tornado.web.Application):
    def __init__(self):
        handlers = [
            (r"/?", MainHandler),
            (r"/stop/?", StopHandler),
            (r"/left/(\w+)", LeftHandler),
            (r"/leftiu/(\w+)", LeftIuHandler),
            (r"/right/(\w+)", RightHandler),
            (r"/rightiu/(\w+)", RightIuHandler),
            (r"/forward/(\w+)", ForwardHandler),
            (r"/image/?", GetImageHandler),
            (r"/iu/?", GetIuHandler),
            (r"/distance/?", DistanceHandler),
            (r"/led/(\w+)", LedHandler)
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
