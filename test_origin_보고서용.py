import numpy as np
import cv2
import time
import math
import serial

from Image import *
from Utils import *

from picamera import PiCamera
from picamera.array import PiRGBArray

N_SLICES = 5  # number of image slices
IMAGE_WIDTH = 1088  #image width
IMAGE_HEIGHT = 608  #image height

CROP_HEIGHT = int(IMAGE_HEIGHT/N_SLICES) # crop height

#num = 1 # image number

WEIGHT_CONSTANT = 0.6  # ratio of weight

ser = serial.Serial('/dev/ttyUSB0', 9600)   # serial 
if ser is None:
    ser = serial.Serial('/dev/ttyUSB1', 9600)

def cal(c): # coefficient for speed
    return c*55

def left(c): # turn left
    v = int(cal(c))
    cmd = ("L%d\n" % v).encode("ascii")
    print("[SEND] %s" % cmd)  
    ser.write(cmd)  # transmit to Arduino
    try:
        read_serial = ser.readline()
        print("[READ] %s" % read_serial) # debug for serial
    except:
        print("readline exception")
    

def right(c): # turn right
    v = int(cal(c))
    cmd = ("R%d\n" % v).encode("ascii")
    print("[SEND] %s" % cmd)
    ser.write(cmd)  # transmit to Arduino
    try:
        read_serial = ser.readline()
        print("[READ] %s" % read_serial) # debug for serial
    except:
        print("readline exception")
    

def main():
    
    ser = serial.Serial('/dev/ttyUSB0', 9600)
    if ser is None:
        ser = serial.Serial('/dev/ttyUSB1', 9600)
    
    camera = PiCamera()
    camera.resolution = (IMAGE_WIDTH, IMAGE_HEIGHT)
    rawCapture = PiRGBArray(camera, size=(IMAGE_WIDTH, IMAGE_HEIGHT))

    camera.start_preview(fullscreen = False, window = (600, 200, IMAGE_WIDTH, IMAGE_HEIGHT)) # show current camera
    time.sleep(0.1) # sleep for loading camera


    while True:
        try:
            camera.capture(rawCapture, format="bgr", use_video_port=True)   # capture
            Images = []
            for _ in range(N_SLICES):
                Images.append(Image())
            frame = rawCapture.array

            points = SlicePart(frame, Images, N_SLICES) # crop image and do PROCESS
        
            topPoint = points[2][0]     # x of third crop_image
            botPoint = points[-1][0]    # x of fifth crop_image
            dx = topPoint - botPoint    
            dy = 2*CROP_HEIGHT
                                        # LINE : line for 3rd_image center and 5th_image center
            theta = math.atan2(dy, dx)  # angle for vertical line and LINE
            angle = theta - math.pi/2 
            max_angle = math.atan(IMAGE_WIDTH/CROP_HEIGHT/2)
            curveRatio = angle/max_angle    # ratio of curve
    
            diffRatio = Images[-1].diff/IMAGE_WIDTH/2 # difference from center
            v_change = (1-WEIGHT_CONSTANT)*curveRatio + WEIGHT_CONSTANT*diffRatio # velocity change

            print(v_change)
            if v_change > 0:  # turn left
                left(v_change)
            
            else:             # turn right
                right(-v_change)
        
            rawCapture.truncate(0) # image clear
            
        except KeyboardInterrupt:
            print("Keyboard Interrupt")
            camera.close()
            cv2.destroyAllWindows()
            break;


main()
