# Edge detection with Canny:
#
# This example demonstrates the Canny edge detector.
import sensor, image, time, pyb, utime
from pyb import Pin

sensor.reset() # Initialize the camera sensor.
sensor.set_pixformat(sensor.RGB565) # or sensor.RGB565
sensor.set_framesize(sensor.QVGA) # or sensor.QVGA (or others)
sensor.skip_frames(time = 2000) # Let new settings take affect.
sensor.set_vflip(True)
sensor.set_hmirror(True)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

green_t = [81, 100, -75, -34, -41, 0]
#[94, 97, -82, -11, -58, 54]
#[86, 100, -71, 0, 14, 58]
#[86, 100, -71, 0, 14, 58]
#[66, 100, -105, -64, 32, 126]
clock = time.clock() # Tracks FPS.
LightHouseLock = 0
era = [0,0,320,240]
pos = 0

pin0 = Pin('P0',Pin.OUT_PP,Pin.PULL_NONE)
pin1 = Pin('P1',Pin.OUT_PP,Pin.PULL_NONE)

#my_stepper = Stepper(pin0,pin1)

def safe_spin(direction,angle):
    if angle<=150 and angle>=-150:
        if direction == 'left':
            if angle<=149:
                pin1.value(0)
                pin0.value(1)
                utime.sleep_ms(1)
                pin0.value(0)
                utime.sleep_ms(1)
                angle += 1
        elif direction == 'right':
            if angle>=-149:
                pin1.value(1)
                pin0.value(1)
                utime.sleep_ms(1)
                pin0.value(0)
                utime.sleep_ms(1)
                angle -= 1
        return angle
    elif angle<-150:
        return -150
    elif angle>150:
        return 150



while(True):
    clock.tick() # Track elapsed milliseconds between snapshots().
    img = sensor.snapshot() # Take a picture and return the image.
    blobs = img.find_blobs([green_t],roi=era)
    if len(blobs)>=1:
        #print('Lock')
        for i in range(1):
            target = blobs[i]
            era = [target.cx()-25,target.cy()-25,50,50]
            img.draw_cross(target.cx(),target.cy(),color=(255,0,0),size=10)
            img.draw_rectangle(era)
            if target.cx()<160:
                #my_stepper.rel_angle(1)
                pos = safe_spin('left',pos)
                print(target.cx())
                print('left')
            else:
                pos = safe_spin('right',pos)
                print(target.cx())
                print('right')
                #my_stepper.rel_angle(-1)
            print('pos = ')
            print(pos)
    else:
        era = [0,0,320,240]
        #print('Lost')
    #print(clock.fps()) # Note: Your OpenMV Cam runs about half as fast while
