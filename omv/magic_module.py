# 目的：检测灯塔/检测小车/通信

import sensor, image, time

#Initialization Begin

sensor.reset() # Initialize the camera sensor.
sensor.set_pixformat(sensor.RGB565) # or sensor.RGB565
sensor.set_framesize(sensor.QVGA) # or sensor.QVGA (or others)
sensor.skip_frames(time = 2000) # Let new settings take affect.
sensor.set_vflip(True)
sensor.set_hmirror(True)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

clock = time.clock()

#Initialization End


#My Function Begin
def find_lighthouse(img,[lighthouse_threshold],roi=[0,0,320,240]):
    blobs = img.find_blobs([lighthouse_threshold],roi=era)
    if len(blobs)!=1:
        target = blobs[i]
        era = [target.cx()-25,target.cy()-25,50,50]
        img.draw_cross(target.cx(),target.cy(),color=(255,0,0),size=10)
        img.draw_rectangle(era)
        if target.cx()<160:
            print(target.cx())
            print('left')
        else:
            print(target.cx())
            print('right')
    else:
        era = [0,0,320,240]
    return []

#My Function End



while(True):
    clock.tick()
    img = sensor.snapshot()
    print(clock.fps())
