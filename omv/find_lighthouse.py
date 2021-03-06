import sensor, image, time
from pyb import UART, Timer
import ustruct

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_hmirror(True)
sensor.set_vflip(True)
sensor.set_auto_gain(False)

sensor.set_auto_exposure(False,500)
sensor.skip_frames(time = 2000)

BlueThreshold = (0, 72, 14, 62, -128, -53)
#GreenThreshold = (76, 100, -68,-20, -50, 127)
GreenThreshold = (51, 99, -128, -7, -128, 127)
RedThreshold = (7, 67, 14, 127, -20, 127)

RedCard =  (10, 100, 21, 127, -128, 127)
GreenCard = (12, 100, -127, -18, -128, 127)

#串口初始化
uart = UART(3,19200)
uart.init(19200, bits=8, parity=None, stop=1)

RED = 1
GREEN = 2
BLUE = 3
LOST = 4

team = GREEN
state = 0
count = 0

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

#定时器回调函数
def tick(timer):
    count = count + 1
    print(timer)
    if(count >= 10):
        count = 10
        state = 1
        sensor.set_auto_exposure(False,20000)
    else:
        state = 0

tim = Timer(4,freq=1)
tim.callback(tick)

while(True):
    img = sensor.snapshot()
    if team == RED:
        if state == 0:
            blobs = img.find_blobs([RedThreshold],merge=True,pixels_threshold=20)
        elif state == 1:
            blobs = img.find_blobs([RedCard],merge=True,pixels_threshold=20)
        if blobs:
            i = blobs[0];
            img.draw_rectangle(i[0:4])
            img.draw_cross(i[5], i[6])
            dx = 160-i.cx()
            dy = 120-i.cy()
            send_data(dx,dy,RED)
        else:
            send_data(0,0,LOST)
    elif team == GREEN:
        if state == 0:
            blobs = img.find_blobs([GreenThreshold],merge=True,pixels_threshold=10)
        elif state == 1:
            blobs = img.find_blobs([GreenCard],merge=True,pixels_threshold=10)
        if blobs:
            i = blobs[0];
            img.draw_rectangle(i[0:4])
            img.draw_cross(i[5], i[6])
            dx = 160-i.cx()
            dy = 120-i.cy()
            send_data(dx,dy,GREEN)
        else:
            send_data(0,0,LOST)
