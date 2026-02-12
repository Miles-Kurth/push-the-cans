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
        if now - self.last_time > 0.03:
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

def printLaserDistance():
    print("Laser distance: " + str(laser_sensor.distance()) + "mm")

def printGyroAngle():
    print("Gyro: " + str(gyro_sensor.angle()) + "°")


def start():
    robot.brake()
    gyro_sensor.reset_angle(0)
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    ev3.speaker.set_volume(20)
    playNote("A")
    playNote("C#")
    playNote("E")

    wait(10)

def resetWheelAngles():
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)

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

def waitUntilNextCan():
    while (laser_sensor.distance() > 500):
        print("Turning...")

def waitUntilCanEnds():
    while (laser_sensor.distance() < 500):
        print("Turning...")

def pushCanOut():
    printLaserDistance()
    resetWheelAngles()
    robot.drive(1000,-10)
    while (left_motor.angle() < 800):
        wait(1)
    robot.straight(100)
    print("Left angle: " + str(left_motor.angle()))
    print("Right angle: " + str(right_motor.angle()))
    global cansPushed; cansPushed += 1
    robot.brake()
    playNote("B")
    playNote("E")
    wait(10)

def returnToCenter():
    robot.drive(-800,0)
    while (left_motor.angle() > 200):
        wait(1)
    robot.drive(-200,0)
    while (left_motor.angle() > 50):
        wait(1)
    print("Left angle: " + str(left_motor.angle()))
    print("Right angle: " + str(right_motor.angle()))
    robot.brake()
    playNote("F")
    playNote("G")
    wait(10)


def playNote(note):
    if (note == "A"):
        ev3.speaker.beep(440)
    if (note == "A#"):
        ev3.speaker.beep(466.1637615)
    if (note == "B"):
        ev3.speaker.beep(493.8833013)
    if (note == "C"):
        ev3.speaker.beep(523.2511306)
    if (note == "C#"):
        ev3.speaker.beep(554.365262)
    if (note == "D"):
        ev3.speaker.beep(587.3295358)
    if (note == "D#"):
        ev3.speaker.beep(622.2539674)
    if (note == "E"):
        ev3.speaker.beep(659.2551138)
    if (note == "F"):
        ev3.speaker.beep(698.4564629)
    if (note == "F#"):
        ev3.speaker.beep(739.9888454)
    if (note == "G"):
        ev3.speaker.beep(783.990872)
    if (note == "G#"):
        ev3.speaker.beep(830.6093952)
    if (note == "A5"):
        ev3.speaker.beep(880)

# End methods


# Code below

# Push the cans #1 is DONE

start()

cansPushed = 0

while (cansPushed <= 4):
    startTurnDynamic(130)
    waitUntilCanEnds()
    waitUntilNextCan()
    pushCanOut()
    returnToCenter()
robot.brake()
for (i in range(3)){
    playNote("A5")
    wait(10)
}



# Gyro and laser test
# while True:
#     printGyroAngle()
#     printLaserDistance()


wait(1500)
