import sensor, image, time
from pyb import UART, Timer
import ustruct
import cpufreq

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_hmirror(True)
sensor.set_vflip(True)
sensor.set_auto_gain(False)

sensor.set_auto_exposure(True)

#sensor.set_auto_exposure(False,20000)
sensor.skip_frames(time = 20)
clock = time.clock() # Tracks FPS.
cpufreq.set_frequency(400)


blue_threshold = (0, 72, 14, 62, -128, -53)
#green_threshold = (76, 100, -68,-20, -50, 127)
green_threshold = (51, 99, -128, -7, -128, 127)
red_threshold = (7, 67, 14, 127, -20, 127)

red_card = (10, 100, 21, 127, -128, 127)
green_card = (12, 100, -127, -18, -128, 127)

#串口初始化
uart = UART(3,19200)
uart.init(19200, bits=8, parity=None, stop=1)

RED = 1
GREEN = 2
BLUE = 3
LOST = 4

team = RED
state = 0
count = 0

ROI = (0,80,320,80)

#发送数据
def send_data(dx,dy,color):
    global uart
    if color == RED:
        ID = 0xFF
    elif color == GREEN:
        ID = 0x3F
    elif color == BLUE:
        ID = 0x0F
    elif color == LOST:
        ID = 0x03
    else:
        ID = 0x00
    data = ustruct.pack(">BBBhhB",
                    0x1D,
                    0xD1,
                    ID,
                    dx,
                    dy,
                    0xED)
    uart.write(data)

while(True):
    clock.tick();
    img = sensor.snapshot()
    #img.draw_rectangle(ROI)
    if team == RED:
        #blobs = img.find_blobs([red_card],roi = ROI)
        blobs = img.find_blobs([red_card],merge=True,roi = ROI,pixels_threshold=20)
        if blobs:
            i = blobs[0];
            #img.draw_rectangle(i[0:4])
            #img.draw_cross(i[5], i[6])
            dx = 160-i.cx()
            dy = 120-i.cy()
            #send_data(dx,dy,RED)
        #else:
            #send_data(0,0,LOST)
    elif team == GREEN:
        blobs = img.find_blobs([green_card],merge=True,roi = ROI,pixels_threshold=20)
        if blobs:
            i = blobs[0];
            #img.draw_rectangle(i[0:4])
            #img.draw_cross(i[5], i[6])
            dx = 160-i.cx()
            dy = 120-i.cy()
            #send_data(dx,dy,GREEN)
        #else:
            #send_data(0,0,LOST)
    print(clock.fps())
    print(cpufreq.get_current_frequencies())
