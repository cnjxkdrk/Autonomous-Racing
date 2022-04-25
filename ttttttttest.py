import cv2
import time
from picamera import PiCamera
from picamera.array import PiRGBArray
from Image import *
from Utils import *

IMAGE_WIDTH = 1088
IMAGE_HEIGHT = 608

font = cv2.FONT_HERSHEY_SIMPLEX
direction = 0

camera = PiCamera()
camera.resolution = (IMAGE_WIDTH, IMAGE_HEIGHT)

camera.start_preview(fullscreen = False, window = (600, 200, IMAGE_WIDTH, IMAGE_HEIGHT))

time.sleep(100);
camera.stop_preview()
"""
rawCapture = PiRGBArray(camera, size=(IMAGE_WIDTH, IMAGE_HEIGHT))

time.sleep(0.1)

camera.capture(rawCapture, format="bgr", use_video_port=True)
frame = rawCapture.array
points = SlicePart(frame, Images, N_SLICES)

fm = RepackImages(Images)

cv2.imshow("Vision Race", fm)
"""