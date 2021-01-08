from controller import Robot, DistanceSensor, Motor

# time in [ms] of a simulation step
TIME_STEP = 64

MAX_SPEED = 6.28

# create the Robot instance.
robot = Robot()

# initialize devices
ds = []
dsNames = [
    'ds0'
]

for i in range(1):
    ds.append(robot.getDevice(dsNames[i]))
    ds[i].enable(TIME_STEP)

leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# feedback loop: step simulation until receiving an exit event
while robot.step(TIME_STEP) != -1:
    # read sensors outputs
    dsValues = []
    for i in range(1):
        dsValues.append(ds[i].getValue())
    print(dsValues[0])
    # detect obstacles
    obstacle = dsValues[0] > 80.0

    # initialize motor speeds at 50% of MAX_SPEED.
    #leftSpeed  = 0.5 * MAX_SPEED
    #rightSpeed = 0.5 * MAX_SPEED
    leftSpeed  = 1.0
    rightSpeed = 1.0
    # modify speeds according to obstacles
    #if obstacle:
     #   # turn right
      #  leftSpeed  = 0.5 * MAX_SPEED
       # rightSpeed = -0.5 * MAX_SPEED
    # write actuators inputs
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)