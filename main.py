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
from pybricks.ev3devices import Motor, GyroSensor
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
        if now - self.last_time > 0.1:
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

# Initialize sensors
laser_sensor = LaserSensor(Port.S1)
gyro_sensor = GyroSensor(Port.S4)

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
    print(armDirection + ": " + str(laser_sensor.distance()) + "mm")

def printGyroAngle():
    print("Gyro: ",end='')
    print(gyro_sensor.angle())

def turn90Degrees(degrees):
    gyro_sensor.reset_angle(0)
    print(degrees)
    if (degrees == 0):
        printGyroAngle()
    elif (degrees > 0):
        robot.drive(0,90)
        while (gyro_sensor.angle() < 80):
            printGyroAngle()
    elif (degrees < 0):
        robot.drive(0,-90)
        while (gyro_sensor.angle() > -80):
            printGyroAngle()
    robot.stop()
    wait(300)
    printGyroAngle()


