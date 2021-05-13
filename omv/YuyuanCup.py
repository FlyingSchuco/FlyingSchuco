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

#sensor.set_auto_exposure(True)

sensor.set_auto_exposure(False,6000)
sensor.skip_frames(time = 20)
clock = time.clock() # Tracks FPS.
cpufreq.set_frequency(400)


blue_threshold = (0, 72, 14, 62, -128, -53)
green_threshold = (20, 100, -40, -5, -40, 0)
red_threshold = (7, 67, 14, 127, -20, 127)

light_threshold =(220, 255)

red_card = (50, 100, 21, 127, -128, 127)
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

ROI = (0,0,320,40)

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
    gray_img = img.copy(roi=ROI).to_grayscale()
    blobs = gray_img.find_blobs([light_threshold], pixels_threshold=4)
    #print(blobs)
    flag = 0
    if team == RED:
        # old version
        #blobs = img.find_blobs([red_card],roi = ROI,pixels_threshold=4)
        #if blobs:
            #i = blobs[0]
            #img.draw_cross(i[5], i[6])

        # new version
        for the_blob in blobs:
            if(the_blob.roundness()>0.3):
                #print(the_blob)
                r_x = the_blob.x()-5
                r_y = the_blob.y()+ROI[1]-5
                r_w = the_blob.w()+10
                r_h = the_blob.h()+10
                r_cx = the_blob.cx()
                r_cy = the_blob.cy()+ROI[1]
                r_roi = (r_x, r_y, r_w, r_h)
                #print(r_roi)
                #b_ch = img.copy(roi=r_roi).to_grayscale(rgb_channel=2)
                g_ch = img.copy(roi=r_roi).to_grayscale(rgb_channel=1)
                r_ch = img.copy(roi=r_roi).to_grayscale(rgb_channel=0)
                r_img = r_ch.sub(g_ch).binary([(35,255)]).erode(1).dilate(3)
                r_blobs = r_img.find_blobs([(128,255)])
                #print(r_blobs)
                if len(r_blobs) >= 1:
                    i = [0, 0]
                    i[0]  = r_blobs[0].cx()+r_x
                    i[1]  = r_blobs[0].cy()+r_y
                    img.draw_cross(i[0], i[1], color=[255, 0, 0])
                    flag = 1
                    img.draw_rectangle(r_roi)
                    #dx = 160-i.cx()
                    #dy = 120-i.cy()
                    #send_data(dx,dy,RED)
                #else:
                    #send_data(0,0,LOST)
    elif team == GREEN:
        for the_blob in blobs:
            if(the_blob.roundness()>0.3):
                #print(the_blob)
                g_x = the_blob.x()-5
                g_y = the_blob.y()+ROI[1]-5
                g_w = the_blob.w()+10
                g_h = the_blob.h()+10
                g_cx = the_blob.cx()
                g_cy = the_blob.cy()+ROI[1]
                g_roi = (g_x, g_y, g_w, g_h)
                #print(g_roi)
                #b_ch = img.copy(roi=g_roi).to_grayscale(rgb_channel=2)
                g_ch = img.copy(roi=g_roi).to_grayscale(rgb_channel=1)
                r_ch = img.copy(roi=g_roi).to_grayscale(rgb_channel=0)
                g_img = g_ch.sub(r_ch).binary([(30,255)]).dilate(2)
                g_blobs = g_img.find_blobs([(128,255)])
                #print(g_blobs)
                if len(g_blobs) >= 1:
                    i = [0, 0]
                    i[0]  = g_blobs[0].cx()+g_x
                    i[1]  = g_blobs[0].cy()+g_y
                    img.draw_cross(i[0], i[1], color=[255, 0, 0])
                    flag = 1
                    img.draw_rectangle(g_roi)
                    #dx = 160-i.cx()
                    #dy = 120-i.cy()
                    #send_data(dx,dy,GREEN)
                #else:
                    #send_data(0,0,LOST)
    #if(flag == 0):
        #print('no')
    #else :
        #print('yes')
    img.draw_rectangle(ROI)
    #print(clock.fps())
