#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()


# Write your program here.

left_motor = Motor(Port.A)
right_motor = Motor(Port.D)
left_color = ColorSensor(Port.S1)
right_color = ColorSensor(Port.S4)
robot = DriveBase(left_motor, right_motor, 55.5, 104)

threshold = 50
kp = 1.2

ev3.speaker.beep()



def TURN(now_dir,target_dir):
        robot.straight(20)
        direction = (target_dir - now_dir) % 4
        turn_table = [0, 90, 180, -90]
        angle = turn_table[direction]
        robot.turn(angle)
# 임계값 작은게 검은색
    
def Rmove(n):
    i = 0
    for i in range(n):
        while True:
            left_reflection = left_color.reflection()
            right_reflection = right_color.reflection()
           
            if left_reflection <30:
                robot.stop()
                break
            else:
                error = right_reflection - threshold
                turn_rate = kp * error
                robot.drive(100, -turn_rate)
            wait(10)


        while True:
            left_reflection = left_color.reflection()
            right_reflection = right_color.reflection()
            if left_reflection > 30:
                robot.stop()
                break
            else:
                error = right_reflection - threshold
                turn_rate = kp * error
                robot.drive(100, -turn_rate)
            wait(10)

def Lmove(n):
    i = 0
    for i in range(n):
        while True:
            left_reflection = left_color.reflection()
            right_reflection = right_color.reflection()
           
            if right_reflection <30:
                robot.stop()
                break
            else:
                error = left_reflection - threshold
                turn_rate = kp * error
                robot.drive(100, turn_rate)
            wait(10)


        while True:
            left_reflection = left_color.reflection()
            right_reflection = right_color.reflection()
            if right_reflection > 30:
                robot.stop()
                break
            else:
                error = left_reflection - threshold
                turn_rate = kp * error
                robot.drive(100, turn_rate)
            wait(10)

Rmove(1)
TURN(1,4)
Rmove(2)
TURN(4,1)



Lmove(2)
TURN(1,2)
Lmove(1)
ev3.speaker.beep()


Lmove(1)
TURN(2,1)
Rmove(1)