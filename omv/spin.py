# Untitled - By: vicru - 周二 10月 22 2019

import sensor, image, time
from pyb import Pin
import utime

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)

clock = time.clock()

pos = 0

pin0 = Pin('P0',Pin.OUT_PP,Pin.PULL_NONE)
pin1 = Pin('P1',Pin.OUT_PP,Pin.PULL_NONE)


def safe_spin(direction,angle):
    if angle<=150 and angle>=-150:
        if direction == 'left':
            if angle<=149:
                pin1.value(0)
                pin0.value(1)
                utime.sleep_us(100)
                pin0.value(0)
                utime.sleep_us(100)
                angle += 1
        elif direction == 'right':
            if angle>=-149:
                pin1.value(1)
                pin0.value(1)
                utime.sleep_us(100)
                pin0.value(0)
                utime.sleep_us(100)
                angle -= 1
        return angle
    elif angle<-150:
        return -150
    elif angle>150:
        return 150

while(True):
    clock.tick()
    img = sensor.snapshot()
    pos = safe_spin('left',pos)
    print(pos)
