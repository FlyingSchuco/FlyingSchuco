# Hello World Example
#
# Welcome to the OpenMV IDE! Click on the green run arrow button below to run the script!

import sensor, image, time

sensor.reset()                      # Reset and initialize the sensor.
sensor.set_pixformat(sensor.RGB565) # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.QVGA)   # Set frame size to QVGA (320x240)
sensor.set_hmirror(True)
sensor.set_vflip(True)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.skip_frames(time = 300)     # Wait for settings take effect.
clock = time.clock()                # Create a clock object to track the FPS.

#绿光检测
green = [35, 100, -128, -35, -7, 40]

while(True):
    clock.tick()                    # Update the FPS clock.
    img = sensor.snapshot()         # Take a picture and return the image.
    img.draw_rectangle((80,60,160,120))
    img.draw_rectangle((40,30,240,180))
    green_blobs = img.find_blobs([green])
    if green_blobs :
        for i in green_blobs:
            img.draw_cross(i[5],i[6])
            #角点检测
            roi = (0,i[2],320,240)

            #哈弗检测直线

            #边缘检测

    print(clock.fps())              # Note: OpenMV Cam runs about half as fast when connected
                                    # to the IDE. The FPS should increase once disconnected.
