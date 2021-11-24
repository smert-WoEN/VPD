#!/usr/bin/env python3
from ev3dev.ev3 import *
import time

mA = LargeMotor('outA')
mA.position = current_time = start_time = 0

fh = open("data.txt", "w")
fh.write('0 ' + '0 ' + '0' + '\n')

start_time = time.time()


def sign(number):
    if number > 0.0:
        return 1.0
    if number < 0.0:
        return -1.0
    return 0.0


kP = 1.0
kI = 0.0
kD = 0.0
kS = 0.0

maxI = float(50.0)
I = 0.0
maxSpeed = 100.0
errorOld = 0.0
timeOld = 0.0


def reg(target):
    global I, errorOld, timeOld
    timeDelta = (time.time() - start_time) - timeOld
    timeOld = time.time() - start_time
    pos = mA.position
    error = target - pos
    power = 0.0
    P = 0.0
    D = 0.0
    S = 0.0
    if target != pos:
        P = error * kP / timeDelta
        I += (error * kI) * timeDelta
        if I > maxI:
            I = sign(I) * maxI
        D = (error - errorOld) * kD
        S = kS * sign(error)
        power = P + I + D + S
        errorOld = error
    else:
        power = 0.0
        errorOld = 0.0
        I = 0.0
    power = min(abs(power), maxSpeed) * sign(power)
    return power


goto = int(200)
try:
    while True:
        current_time = time.time() - start_time
        mA.run_direct(duty_cycle_sp = round(reg(goto)))
        fh.write(str(current_time) + ' ' + str(mA.position) + ' ' + str(mA.speed) + ' ' + str(goto) + '\n')
finally:
    mA.stop(stop_action = 'brake')
    fh.close()
