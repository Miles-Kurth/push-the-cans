#!/usr/bin/env pybricks-micropython

"""
Example LEGO® MINDSTORMS® EV3 Robot Educator Ultrasonic Sensor Driving Base Program
-----------------------------------------------------------------------------------

This program requires LEGO® EV3 MicroPython v2.0.
Download: https://education.lego.com/en-us/support/mindstorms-ev3/python-for-ev3

Building instructions can be found at:
https://education.lego.com/en-us/support/mindstorms-ev3/building-instructions#robot
"""

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, GyroSensor, ColorSensor
from pybricks.parameters import Port,Stop
from pybricks.tools import wait
from pybricks.robotics import DriveBase
from pybricks.iodevices import I2CDevice
import time


class LaserSensor:
    def __init__(self, port):
        self.i2c = I2CDevice(port, 0x02 >> 1)
        self.last_time = 0
        self.last_dist = 0

    def distance(self):
        now = time.time()
        if now - self.last_time > 0.05:
            self.last_time = now
            results = self.i2c.read(0x42, 2)
            self.last_dist = results[0] + (results[1] << 8)
        return self.last_dist


# Initialize the EV3 Brick.
ev3 = EV3Brick()

# Initialize motors
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
arm_motor = Motor(Port.A)

# Initialize sensors
laser_sensor = LaserSensor(Port.S1)
color_sensor = ColorSensor(Port.S3)
gyro_sensor = GyroSensor(Port.S4)

# Declare variables
armDirection = "not set"
targetAngle = 0 # degrees
speed = 200 # mm/s

# The DriveBase is composed of two motors, with a wheel on each motor.
# The wheel_diameter and axle_track values are used to make the motors
# move at the correct speed when you give a motor command.
# The axle track is the distance between the points where the wheels
# touch the ground.
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104) #104 -> 119?

# At start
ev3.speaker.set_volume(20); #ev3.speaker.beep(660,200)
#print(arm_motor.angle())


# Functions

def armTurnRight():
    global armDirection; armDirection = "Right"
    arm_motor.run_until_stalled(-500,then=Stop.HOLD,duty_limit=40)
    wait(100)
    
def armTurnLeft():
    global armDirection; armDirection = "Left"
    arm_motor.run_until_stalled(500,then=Stop.HOLD,duty_limit=40)
    wait(100)

def armTurnForwardRight():
    armTurnRight()
    global armDirection; armDirection = "Forward-Right"
    wait(400)
    arm_motor.reset_angle(0)
    arm_motor.run_target(-100,60,then=Stop.HOLD,wait=True)

def armTurnForwardLeft():
    armTurnLeft()
    global armDirection; armDirection = "Forward-Left"
    wait(400)
    arm_motor.reset_angle(0)
    arm_motor.run_target(100,-60,then=Stop.HOLD,wait=True)


def printLaserDistance():
    print("Laser distance: " + str(laser_sensor.distance()) + "mm")

def printGyroAngle():
    print("Gyro: " + str(gyro_sensor.angle()) + "°")

def turn90Degrees(degrees):
    gyro_sensor.reset_angle(0)
    print(degrees)
    if (degrees == 0):
        printGyroAngle()
    elif (degrees > 0):
        robot.drive(0,90)
        while (gyro_sensor.angle() < 80):
            # printGyroAngle()
            wait(1)
    elif (degrees < 0):
        robot.drive(0,-90)
        while (gyro_sensor.angle() > -80):
            # printGyroAngle()
            wait(1)
    robot.stop()
    wait(300)
    printGyroAngle()


def start():
    robot.brake()
    gyro_sensor.reset_angle(0)
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    wait(10)

def startTurnDynamic(speed):
    if (speed > 0):
        left_motor.run(1 * speed)
        right_motor.run(-1 * speed)
    else:
        left_motor.run(1 * speed)
        right_motor.run(-1 * speed)

def turnToAngle(angle):
    if (gyro_sensor.angle() % 360 > angle % 360):
        startTurnDynamic(-80)
        while (gyro_sensor.angle() % 360 > angle % 360):
            # printGyroAngle()
            wait(1)
    elif (gyro_sensor.angle() % 360 < angle % 360):
        startTurnDynamic(80)
        while (gyro_sensor.angle() % 360 < angle % 360):
            # printGyroAngle()
            wait(1)
    else:
        print("Already at specified angle!")

def driveToCan():
    printLaserDistance()
    robot.drive(1000,0)
    while (laser_sensor.distance() > 110):
        printLaserDistance()
    wait(100)
    robot.brake()

# End methods


obstacleNumber = 0

start()
startTurnDynamic(100)
wait(10)

while (gyro_sensor.angle() < 360):
    #print(obstacleNumber)
    while (laser_sensor.distance()) > 500:
        if (gyro_sensor.angle() >= 360): break
        wait(1)
    if (gyro_sensor.angle() >= 360): break

    obstacleNumber += 1
    print("Adding")

    while (laser_sensor.distance()) < 500:
        if (gyro_sensor.angle() >= 360): break
        wait(1)
    print(obstacleNumber)
    if (gyro_sensor.angle() >= 360): break
    

print("Loop is over")

printGyroAngle()
# gyro_sensor.reset_angle(0)
print(obstacleNumber)

if (obstacleNumber >= 5):
    turn90Degrees(-1)
else:
    turn90Degrees(1)

robot.brake()
robot.drive(200,0)




# Code below

# start()

# canWidth = 50
# canAngle = 0
# canDistance = 0

# startTurnDynamic(400)
# while True:
#     printLaserDistance()
#     # printGyroAngle()
#     # print(left_motor.angle())
#     # print(time.time())
#     if (laser_sensor.distance() < 400):
#         printLaserDistance()
#         canAngle = gyro_sensor.angle()
#         canDistance = laser_sensor.distance()
#         robot.brake()
#         break
# wait(300)
# print(str(canAngle) + " " + str(gyro_sensor.angle()))

# startTurnDynamic(-100)
# while (laser_sensor.distance() > canDistance + 25):
#     printGyroAngle()
#     printLaserDistance()
#     if (abs(canDistance - laser_sensor.distance()) < 5):
#         robot.brake()
#         break

# driveToCan()


# Gyro and laser test
# while True:
#     printGyroAngle()
#     printLaserDistance()


wait(1500)