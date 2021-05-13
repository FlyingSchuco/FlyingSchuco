
import sensor, image, time
from pyb import Pin, Timer,LED

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_hmirror(True)
sensor.set_vflip(True)
sensor.set_auto_gain(False)

sensor.set_auto_exposure(False,500)
sensor.skip_frames(time = 2000)

blue_led  = LED(3)

# 当被调用时，我们将返回timer对象
# 注意:在回调中不允许分配内存的函数
def tick(timer):
    count = count + 1
    sensor.set_auto_exposure(False,50000)

tim = Timer(2, freq=1)      # 使用定时器2创建定时器对象-以1Hz触发
tim.callback(tick)          # 将回调设置为tick函数

while (True):
    img = sensor.snapshot()
