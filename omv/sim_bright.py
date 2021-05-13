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
green_threshold = (20, 100, -40, -5, -40, 0)
red_threshold = (7, 67, 14, 127, -20, 127)

light_threshold =(220, 255)

red_card = (10, 100, 21, 127, -128, 127)
green_card = (88, 99, -70, -3, -5, 9)

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

ROI = (0,90,320,60)

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

stream = image.ImageIO("/G4.bin", "r")

while(True):
    clock.tick();
    img = stream.read(copy_to_fb=True, loop=True, pause=True)
    #img.copy(roi=ROI).to_grayscale().binary([light_threshold]).copy(copy_to_fb=True)
    gray_img = img.copy(roi=ROI).to_grayscale()
    blobs = gray_img.find_blobs([light_threshold], pixels_threshold=4)
    if(len(blobs) == 0):
        print('no')
    for the_blob in blobs:
        if(the_blob.roundness()>0.3):
            g_x = the_blob.x()-3
            g_y = the_blob.y()+90-3
            g_w = the_blob.w()+6
            g_h = the_blob.h()+6
            g_roi = (g_x, g_y, g_w, g_h)
            img.draw_rectangle(g_roi)
            print('yes')
    #print(clock.fps())
