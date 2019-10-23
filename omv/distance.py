# Untitled - By: vicru - 周六 10月 19 2019

import sensor, image, time

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)
sensor.set_auto_whitebal(False)

dark = [100, 25, -128, 127, -128, 127]
red = [0, 100, -52, 20, -93, 34]

clock = time.clock()

#k = 705.0
k = 0
target = [0,0,0,0,0,0,0,0,0]


while(True):
    clock.tick()
    img = sensor.snapshot()
    blobs = img.find_blobs([red],invert=True)
    if(len(blobs)>=1):
        target = blobs[0]
        #print("branch1")
    if(k==0):
        k = 15*(target[2]+target[3])/2
        print(k)
    else:
    #print("branch2")
        if(len(blobs)==1):
            distance = 2*k/(target[2]+target[3])
        print(distance)
    #print(clock.fps())
