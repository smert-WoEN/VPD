#!/usr/bin/env python3
from ev3dev.ev3 import *
from ev3dev2.power import PowerSupply
import time
import math


def sign(number):
    if number > 0.0:
        return 1.0
    if number < 0.0:
        return -1.0
    return 0.0


mB = LargeMotor(OUTPUT_B)
mC = LargeMotor(OUTPUT_C)
usSensor1 = UltrasonicSensor(INPUT_1)
usSensor1.mode = 'US-DIST-CM'
usSensor2 = UltrasonicSensor(INPUT_2)
usSensor2.mode = 'US-DIST-CM'
volts = PowerSupply()
fh = open("data.txt", "w")
fh.write('0 ' + '0 ' + '\n')
startTime = time.time()
timeOld = 0


h = float(22.5)
target = float(75.0)
kP = 1.7
kI = 0.35
kD = 0.09
maxI = 20.0
I = 0.0
errorOld = 0.0
speed = 60
voltageReference = 9.0
try:
    while True:
        currentTime = time.time() - startTime
        d1 = usSensor1.value()
        d2 = usSensor2.value()
        d = 0.5 * h * (d1 + d2) * math.sqrt(1.0 / (h * h + (d1 - d2) * (d1 - d2)))
        deltaTime = currentTime - timeOld
        timeOld = currentTime
        fh.write(str(currentTime) + ' ' + str(d) + ' \n')
        erorr = target - d
        P = erorr * kP
        I += erorr * kI * deltaTime
        if abs(I) > maxI:
            I = sign(I) * maxI
        D = (erorr - errorOld) * kD / deltaTime
        errorOld = erorr
        power = P + I + D
        power = power * voltageReference / volts.measured_volts
        if abs(power) > 100 - speed:
            power = sign(power) * (100 - speed)
        mB.run_direct(duty_cycle_sp = speed + power)
        mC.run_direct(duty_cycle_sp = speed - power)

finally:
    mB.stop(stop_action = 'brake')
    mC.stop(stop_actiom = 'brake')
    fh.close()