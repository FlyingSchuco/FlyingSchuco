# Edge detection with Canny:
#
# This example demonstrates the Canny edge detector.
import sensor, image, time

sensor.reset() # Initialize the camera sensor.
sensor.set_pixformat(sensor.GRAYSCALE) # or sensor.RGB565
sensor.set_framesize(sensor.QVGA) # or sensor.QVGA (or others)
sensor.skip_frames(time = 2000) # Let new settings take affect.
sensor.set_vflip(True)
sensor.set_hmirror(True)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

green_t = [94, 97, -82, -11, -58, 54]
#[86, 100, -71, 0, 14, 58]
#[86, 100, -71, 0, 14, 58]
#[66, 100, -105, -64, 32, 126]
clock = time.clock() # Tracks FPS.

while(True):
    clock.tick() # Track elapsed milliseconds between snapshots().
    img = sensor.snapshot() # Take a picture and return the image.
    img.find_edges(image.EDGE_CANNY, threshold=(50, 80))
    # Use Canny edge detector
    print(clock.fps()) # Note: Your OpenMV Cam runs about half as fast while
