import sensor, image, time
from pyb import UART
import ustruct

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_hmirror(True)
sensor.set_vflip(True)
sensor.set_auto_gain(False)

sensor.set_auto_exposure(False,75)
sensor.skip_frames(time = 2000)

BlueThreshold = (0, 72, 14, 62, -128, -53)
GreenThreshold = (51, 99, -128, -7, -128, 127)
RedThreshold = (7, 67, 14, 127, -20, 127)

#串口初始化
uart = UART(3,115200)
uart.init(19200, bits=8, parity=None, stop=1)

RED = 1
GREEN = 2
BLUE = 3
LOST = 4

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
    img = sensor.snapshot()
    blobs = img.find_blobs([RedThreshold])
    if blobs:
        for i in blobs:
            img.draw_rectangle(i[0:4])
            img.draw_cross(i[5], i[6])
            dx = 160-i.cx()
            dy = 120-i.cy()
            send_data(dx,dy,RED)
    else:
        send_data(0,0,LOST)
