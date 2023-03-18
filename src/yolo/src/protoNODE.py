import cv2
from subprocess import call

def open_file_detect():
    call(['python3','./yolov5/detect.py' ])


open_file_detect()