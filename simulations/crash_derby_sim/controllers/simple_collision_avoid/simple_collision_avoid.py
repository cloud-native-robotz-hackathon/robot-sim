from controller import Robot, DistanceSensor, Motor, InertialUnit

# time in [ms] of a simulation step
MAX_SPEED = 6.28

# create the Robot instance.
robot = Robot()
TIME_STEP = int(robot.getBasicTimeStep())

# initialize devices
# ps = []
# psNames = [
#     'ps0', 'ps1', 'ps2', 'ps3',
#     'ps4', 'ps5', 'ps6', 'ps7'
# ]

# for i in range(8):
#     ps.append(robot.getDevice(psNames[i]))
#     ps[i].enable(TIME_STEP)
inertialUnit = robot.getDevice('inertial unit')
inertialUnit.enable(TIME_STEP)
leftDs = robot.getDevice('Left DS')
rightDs = robot.getDevice('Right DS')
backLeftDs = robot.getDevice('Back Left DS')
backRightDs = robot.getDevice('Back Right DS')
leftDs.enable(TIME_STEP)
rightDs.enable(TIME_STEP)
backLeftDs.enable(TIME_STEP)
backRightDs.enable(TIME_STEP)
leftMotor = robot.getDevice('left wheel')
rightMotor = robot.getDevice('right wheel')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# feedback loop: step simulation until receiving an exit event
while robot.step(TIME_STEP) != -1:
    # read sensors outputs
    leftDsValue = leftDs.getValue()
    rightDsValue = rightDs.getValue()
    backLeftDsValue = backLeftDs.getValue()
    backRightDsValue = backRightDs.getValue()
    iu = inertialUnit.getRollPitchYaw()
    print(iu)
    #print(leftDsValue)
    #print(rightDsValue)
    # detect obstacles
    front_obstacle = leftDsValue < 1000.0 or rightDsValue < 1000.0
    back_obstacle = backLeftDsValue > 80.0 or backRightDsValue > 80.0

    # initialize motor speeds at 50% of MAX_SPEED.
    leftSpeed  = 1 * MAX_SPEED
    rightSpeed = 1 * MAX_SPEED
    # modify speeds according to obstacles
    if front_obstacle:
        #print("front obstacle ")
        # turn right
        leftSpeed  = 1 * MAX_SPEED
        rightSpeed = -1 * MAX_SPEED
    # elif back_obstacle:
    #     # turn left
    #     leftSpeed  = -1 * MAX_SPEED
    #     rightSpeed = 1 * MAX_SPEED
    # write actuators inputs
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)