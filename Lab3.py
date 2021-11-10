#!/usr/bin/env python3
from ev3dev.ev3 import *
import time

mA = LargeMotor('outA')
mA.position = current_time = start_time = 0

fh = open("data.txt", "w")
fh.write('0 ' + '0 ' + '0 ' + '0' + '\n')

start_time = time.time()
kP = 0.0
kI = 0.0
kD = 0.0
kV = 0.0
kA = 0.0
kS = 0.0
maxI = 32767
P = 0.0
I = 0.0
D = 0.0
V = 0.0
A = 0.0
S = 0.0
power = 0.0
timeOld = 0.0
timeDelta = 0.0
currentTime = 0.0
currentVelocity = 0.0
velocityError = 0.0
velocityErrorOld = 0.0
velocityTargetOld = 0.0


def sign(number):
    if number > 0.0:
        return 1.0
    if number < 0.0:
        return -1.0
    return 0.0


def update(target):
    global timeDelta, timeOld, currentTime, currentVelocity, velocityErrorOld, velocityError, velocityTargetOld, power, P, I, D, V, A, S
    timeDelta = currentTime - timeOld
    timeOld = currentTime
    currentVelocity = mA.speed
    if target != 0.0:
        velocityError = target - currentVelocity
        P = velocityError * kP
        D = (velocityError - velocityErrorOld) / timeDelta
        I += (kI * velocityError) * timeDelta
        if target == 0.0:
            I = .0
        if abs(I) > maxI:
            I = sign(I) * maxI
        V = kV * target
        A = kA * (target - velocityTargetOld) / timeDelta
        S = kS * sign(target)
        power = P + I + D + V + A + S
        velocityErrorOld = velocityError
        velocityTargetOld = target
    else:
        velocityError = 0.0
        power = 0.0
        velocityErrorOld = 0.0
        I = 0.0
        velocityTargetOld = 0.0
    return power


goto = int(100)
try:
    while True:
        currentTime = time.time() - start_time
        if current_time > 5:
            break
        else:
            mA.run_direct(duty_cycle_sp=round(update(goto)))
            fh.write(str(current_time) + ' ' + str(mA.position) + ' ' + str(goto) + ' ' + str(mA.speed) + '\n')
finally:
    mA.stop(stop_action='brake')
    fh.close()
