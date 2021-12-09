#!/usr/bin/env python3
from ev3dev.ev3 import *
import math


def sign(number):
    if number > 0.0:
        return 1.0
    if number < 0.0:
        return -1.0
    return 0.0


targets = [[30, 0], [30, 30], [0, 30], [0, 0]]
kpD = 0.3
kpA = 9.0
minError = 1.0
maxPower = 80
xPos = 0.0
yPos = 0.0
hPos = 0.0
eLlast = 0.0
eRlast = 0.0
rWheel = 2.8
trackWidth = 17.0
distanceError = 0.0
angleError = 0.0
mB = LargeMotor('outB')
mC = LargeMotor('outC')
fh = open('dif.txt', 'w')
fh.write('0 ' + '0' + '\n')
mC.position = 0
mB.position = 0
try:
    for target in targets:
        while True:
            currentL = mB.position
            currentR = mC.position
            deltaL = (currentL - eLlast) * rWheel * math.pi / 180.0
            eLlast = currentL
            deltaR = (currentR - eRlast) * rWheel * math.pi / 180.0
            eRlast = currentR
            deltaHeading = (currentR - currentL) * rWheel * (math.pi / 180.0) / trackWidth - hPos
            xPos += math.cos(hPos + deltaHeading / 2.0) * (deltaL + deltaR) * 0.5
            yPos += math.sin(hPos + deltaHeading / 2.0) * (deltaL + deltaR) * 0.5

            hPos += deltaHeading
            xError = xPos - target[0]
            yError = yPos - target[1]
            distanceError = math.sqrt(xError * xError + yError * yError)
            angleError = math.atan2(yError, xError) + math.pi - hPos
            while abs(angleError) > math.pi:
                angleError -= math.pi * 2 * sign(angleError)
            uD = distanceError * kpD
            uA = angleError * kpA
            if distanceError < minError:
                break
            powerB = (uD - uA) * (100 / 6)
            powerC = (uD + uA) * (100 / 6)
            powerC = min(abs(powerC), maxPower) * sign(powerC)
            powerB = min(abs(powerB), maxPower) * sign(powerB)
            if powerB < 10:
                powerB = 10
            if powerC < 10:
                powerC = 10
            mB.run_direct(duty_cycle_sp=int(powerB))
            mC.run_direct(duty_cycle_sp=int(powerC))
            fh.write(str(xPos) + ' ' + str(yPos) + '\n')


finally:
    mB.stop(stop_action='brake')
    mC.stop(stop_action='brake')
    fh.close()