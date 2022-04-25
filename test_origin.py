import numpy as np
import cv2
import time
import math
import serial

from Image import *
from Utils import *

from picamera import PiCamera
from picamera.array import PiRGBArray

N_SLICES = 5
IMAGE_WIDTH = 1088
IMAGE_HEIGHT = 608
#IMAGE_WIDTH = 720
#IMAGE_HEIGHT = 544

CROP_HEIGHT = int(IMAGE_HEIGHT/N_SLICES)

num = 1 # image number

IMAGE_LOG_DIR = "images_log/"
TMP_IMAGE_NAME = "./tmp_image.jpg"
DO_SAVE_LOG = False

WEIGHT_ORIGIN = 0.6

ser = serial.Serial('/dev/ttyUSB0', 9600)
if ser is None:
    ser = serial.Serial('/dev/ttyUSB1', 9600)

def cal(c):
    return c*55

def left(c):
    v = int(cal(c))
    cmd = ("L%d\n" % v).encode("ascii")
    print("[SEND] %s" % cmd)  
    ser.write(cmd)
    try:
        read_serial = ser.readline()
        print("[READ] %s" % read_serial)
    except:
        print("readline exception")
    

def right(c):
    v = int(cal(c))
    cmd = ("R%d\n" % v).encode("ascii")
    print("[SEND] %s" % cmd)
    ser.write(cmd)
    try:
        read_serial = ser.readline()
        print("[READ] %s" % read_serial)
    except:
        print("readline exception")
    

def main():
    
    ser = serial.Serial('/dev/ttyUSB0', 9600)
    if ser is None:
        ser = serial.Serial('/dev/ttyUSB1', 9600)
    
    camera = PiCamera()
    camera.resolution = (IMAGE_WIDTH, IMAGE_HEIGHT)
    rawCapture = PiRGBArray(camera, size=(IMAGE_WIDTH, IMAGE_HEIGHT))
   # camera.start_preview(fullscreen = False, window = (100, 20, 640, 480))
    camera.start_preview(fullscreen = False, window = (600, 200, IMAGE_WIDTH, IMAGE_HEIGHT))
    time.sleep(0.1)


    while True:
        try:
            camera.capture(rawCapture, format="bgr", use_video_port=True)
            Images = []
            for _ in range(N_SLICES):
                Images.append(Image())
            frame = rawCapture.array

            points = SlicePart(frame, Images, N_SLICES)
        
            topPoint = points[2][0]
            botPoint = points[-1][0]
            dx = topPoint - botPoint
            dy = 2*CROP_HEIGHT
        
            roadAngle = math.atan2(dy, dx)
            dAngle = roadAngle - math.pi/2
            max_dangle = math.atan(IMAGE_WIDTH/CROP_HEIGHT/2)
            curveRatio = dAngle/max_dangle
        
            #baseSpeed = MAX_BASE_SPEED * (1 - abs(curveRatio))
            diffRatio = Images[-1].diff/IMAGE_WIDTH/2
            #diffSpeed = MAX_DIFF_SPEED*((1-WEIGHT_ORIGIN)*curveRatio + WEIGHT_ORIGIN*diffRatio)
            v_change = (1-WEIGHT_ORIGIN)*curveRatio + WEIGHT_ORIGIN*diffRatio
            print(v_change)
            if v_change > 0: # turn left
                left(v_change)
            
            else:             # turn right
                right(-v_change)
        
        
            frame = RepackImages(Images)
            #frame = cv2.resize(fm, (IMAGE_WIDTH, IMAGE_HEIGHT), interpolation = cv2.INTER_AREA)
            #frame = cv2.line(frame, (topPoint, points[2][1] + 2*CROP_HEIGHT), (points[-1][0], points[-1][1] + 4*CROP_HEIGHT), (0, 0, 255), 3)
            #frame = cv2.putText(frame, "v_change: %f" % v_change, (10, IMAGE_HEIGHT - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0))
            #cv2.imshow("Camera image", frame)
            #cv2.imwrite('/home/pi/source/test/image%s.jpg' % num, frame)
            #num = num + 1
        
            
            rawCapture.truncate(0)
            
        except KeyboardInterrupt:
            print("Keyboard Interrupt")
            camera.close()
            cv2.destroyAllWindows()
            break;


main()
