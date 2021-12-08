from ev3dev.ev3 import *
import math


def sign(number):
    if number > 0.0:
        return 1.0
    if number < 0.0:
        return -1.0
    return 0.0


targets = [[0, 0], [10, 0]]
kpD = 0.0
kpA = 0.0
minError = 3.0
xPos = 0.0
yPos = 0.0
hPos = 0.0
eLlast = 0.0
eRlast = 0.0
rWheel = 2.5
trackWidth = 15.0
distanceError = 0.0
angleError = 0.0
mB = LargeMotor('outB')
mC = LargeMotor('outC')
fh = open('dif.txt', 'w')
fh.write('0 ' + '0' + '\n')
try:
    for target in targets:
        while True:
            currentL = mB.position()
            currentR = mC.position()
            deltaL = (currentL - eLlast) * rWheel * math.pi / 180.0
            eLlast = currentL
            deltaR = (currentR - eRlast) * rWheel * math.pi / 180.0
            eRlast = currentR
            deltaHeading = (currentR - currentL) * rWheel * (math.pi / 180.0) / trackWidth - hPos
            xPos += math.cos(hPos + deltaHeading / 2.0) * (deltaL + deltaR) * 0.5
            yPos += math.sin(hPos + deltaHeading / 2.0) * (deltaL + deltaR) * 0.5
            fh.write(str(xPos) + ' ' + str(yPos) + '\n')
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
            mB.run_direct(duty_cycle_sp=(uD - uA) * (100 / 6))
            mC.run_direct(duty_cycle_sp=(uD + uA) * (100 / 6))

finally:
    mB.stop(stop_action='brake')
    mC.stop(stop_action='brake')
    fh.close()