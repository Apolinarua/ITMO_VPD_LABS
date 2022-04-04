#!/usr/bin/env python3

import time
import math
from ev3dev2.motor import *
from ev3dev2.sound import Sound
from ev3dev2.sensor import *
from ev3dev2.sensor.lego import *
from ev3dev2.sensor.virtual import *

Lmotor = LargeMotor(OUTPUT_A)
Rmotor = LargeMotor(OUTPUT_B)
tank_drive = MoveTank(OUTPUT_A, OUTPUT_B)

x0=x=0
y0=y=0

ks=118
kr=30

Wr=0.028
B=0.152
azimut=alpha=thetta=p=0
Ustr = 0
Urot = 0
posR0 = 0
posL0 = 0

a = 1
b = 1
c = 0.05
planed_time = 0

target = [[0.5, 0], [0,0],
[0,0.8], [0,0],
[-1.2, -0], [0,0],
[0, -1], [0,0]]

def idea_trajectory(a, b, c, t):
    x = a + b*math.sin(2*c*t)
    y = b*math.sin(c*t)
    return x, y

while (True):
    xg, yg = idea_trajectory(a, b, c, planed_time)
    planed_time+=0.2

    while (True):

        posR = Rmotor.position * (math.pi/180) - posR0
        posL = Lmotor.position * (math.pi/180) - posL0

        posR0 = Rmotor.position * (math.pi/180)
        posL0 = Lmotor.position * (math.pi/180)

        thetta = (posR0 - posL0) * Wr/B
        x = x0 + math.cos(thetta) * Wr/2 * (posR + posL)
        y = y0 + math.sin(thetta) * Wr/2 * (posR + posL)
        p = ((xg - x) ** 2 + (yg - y) ** 2) ** 0.5
        azimut = math.atan2(yg - y, xg - x)
        alpha = azimut - thetta

        if alpha > math.pi:
            alpha -= 2 * math.pi
        elif alpha < -math.pi:
            alpha += 2 * math.pi

        Ustr = ks * p
        Urot = kr * alpha

        if Ustr > 60:
            Ustr = 60
        elif Ustr < -60:
            Ustr = -60
        if Urot > 40:
            Urot = 40
        elif Urot < -40:
            Urot = -40

        Ul = (Ustr - Urot)
        Ur = (Ustr + Urot)

        if Ul > 100:
            Ul = 100
        elif Ul < -100:
            Ul = -100
        if Ur > 100:
            Ur = 100
        elif Ur < -100:
            Ur = -100

        tank_drive.on(Ul, Ur)
        print(str(x) + " " + str(y) + " " + str(p) + "\n")
        x0=x
        y0=y
        if p<0.5:
        #tank_drive.on(0,0)
            break