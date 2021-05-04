
import sensor, image, time

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)
sensor.set_contrast(3)
sensor.set_gainceiling(16)

clock = time.clock()
def draw_keypoints(img, kpts):
    if kpts:
        print(kpts)
        img.draw_keypoints(kpts)
        img = sensor.snapshot()
        time.sleep(1)

center_rec = (100,160)
keyptM = image.load_descriptor("/keyptM.orb")
green = [35, 100, -128, -35, -7, 40]

while(True):
    clock.tick()
    img = sensor.snapshot()
    blobs = img.find_blobs([green])
    for i in blobs:
        print("find blobs!!\n")
        area = (i[1]-75,i[2],150,300)
        imgG = img.to_grayscale()
        keypt = imgG.find_keypoints(roi=area,max_keypoints=50, threshold=10, scale_factor=1.2)
        sim = image.match_descriptor(keyptM,keypt,threshold = 85)
        if (sim.count()>5):
            img.draw_rectangle(sim.rect())
            img.draw_cross(sim.cx(), sim.cy(), size=10)
            print("find the model\n")
    print(clock.fps())
