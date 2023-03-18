#!/usr/bin/env python3
import rospy

import cv2
import numpy as np

from sensor_msgs.msg import Image
from utils.msg import lanes
from cv_bridge import CvBridge
import time

#####Helper_functions###################

def canny(img):
    if img is None:
        cap.release()
        cv2.destroyAllWindows()
        exit()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    kernel = 5
    blur = cv2.GaussianBlur(gray,(kernel, kernel),0)
    canny = cv2.Canny(gray, 50, 150)
    return canny

def region_of_interest(canny):
    height = canny.shape[0]
    width = canny.shape[1]
    mask = np.zeros_like(canny)
    triangle = np.array([[
    (0,height),
    (0, int(height*2.8/5)),
    (width, int(height*1.7/5)),
    (width, height)]], np.int32)
    cv2.fillPoly(mask, triangle, 255)
    masked_image = cv2.bitwise_and(canny, mask)
    return masked_image

def houghLines(cropped_canny):
    return cv2.HoughLinesP(cropped_canny, 2, np.pi/180, 75, 
        np.array([]), minLineLength=10, maxLineGap=40)

def addWeighted(frame, line_image):
    return cv2.addWeighted(frame, 0.8, line_image, 1, 1)
 
def display_lines(img,lines):
    line_image = np.zeros_like(img)
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_image,(x1,y1),(x2,y2),(0,0,255),10)
    return line_image
 
def make_points(image, line):
   
    try:
        slope, intercept = line
    except TypeError:
        slope, intercept = 0.001, 0 
    if slope==0:
        slope=0.001
        
    y1 = int(image.shape[0])
    y2 = int(y1*3/5)      
    x1 = int((y1 - intercept)/slope)
    x2 = int((y2 - intercept)/slope)
    return [[x1, y1, x2, y2]]

def average_slope_intercept(image, lines):
    left_fit = []
    right_fit = []
    left_line=[[int((image.shape[0])/0.001),image.shape[0],int((image.shape[0])*3/5/0.001),int(image.shape[0]*3/5)]]
    right_line=[[int((image.shape[0])/0.001),image.shape[0],int((image.shape[0])*3/5/0.001),int(image.shape[0]*3/5)]]
    
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line.reshape(4)
            parameters = np.polyfit((x1, x2), (y1, y2), 1)
            slope = parameters[0]
            intercept = parameters[1]
            
            if slope < 0:
                left_fit.append((slope, intercept))
            else:
                right_fit.append((slope, intercept))
                
            
            left_fit_average = np.average(left_fit, axis=0)
            if left_fit_average.size==2:
                left_line = make_points(image, left_fit_average)
            else:
                left_line=left_line
                
            right_fit_average = np.average(right_fit, axis=0)
            if right_fit_average.size==2: 
                right_line = make_points(image, right_fit_average)
            else:
                right_line=right_line
    return np.array([left_line, right_line],dtype=object)

def camera_to_vehicle_coords(x_cam, y_cam, z_cam, yaw_cam, pitch_cam, roll_cam, x_offset, y_offset, z_offset, focal_length):
    # Izračunaj matricu rotacije za kameru
    R_cam = np.array([[np.cos(yaw_cam)*np.cos(pitch_cam), -np.sin(yaw_cam)*np.cos(roll_cam)+np.cos(yaw_cam)*np.sin(pitch_cam)*np.sin(roll_cam), np.sin(yaw_cam)*np.sin(roll_cam)+np.cos(yaw_cam)*np.sin(pitch_cam)*np.cos(roll_cam)],
                      [np.sin(yaw_cam)*np.cos(pitch_cam), np.cos(yaw_cam)*np.cos(roll_cam)+np.sin(yaw_cam)*np.sin(pitch_cam)*np.sin(roll_cam), -np.cos(yaw_cam)*np.sin(roll_cam)+np.sin(yaw_cam)*np.sin(pitch_cam)*np.cos(roll_cam)],
                      [-np.sin(pitch_cam), np.cos(pitch_cam)*np.sin(roll_cam), np.cos(pitch_cam)*np.cos(roll_cam)]])

     # Prebaci koordinate sa kamere u koordinate vozila
    x_vehicle = (x_cam - x_offset) * z_cam / focal_length
    y_vehicle = (y_cam - y_offset) * z_cam / focal_length
    z_vehicle = z_cam
    coords_vehicle = np.dot(R_cam, np.array([x_vehicle, y_vehicle, z_vehicle]))
    
    return coords_vehicle

def distance_to_vehicle(x_vehicle, y_vehicle, z_vehicle):
    # Izračunaj euklidsku udaljenost između tačke i ishodišta koordinatnog sistema
    distance = np.sqrt(x_vehicle**2 + y_vehicle**2 + z_vehicle**2)
    return distance

#############################################


class lane_dealerNODE():
    def __init__(self):
        
        rospy.init_node('lane_dealerNODE', anonymous=False)

        rospy.Subscriber("/automobile/image_raw", Image, callback = self._streams)
        
        self.pub = rospy.Publisher("/lane/pos_raw", lanes, queue_size=1)

        self.bridge = CvBridge()
        self.alpha = 3
        self.H = 25
        self.teta = 15
        self.lane_sleep_s = 0.5

    def run(self):

        rospy.loginfo("starting lane_dealerNODE")
        
        rospy.spin()
        
    def _streams(self, msg):
        lane_data = lanes()

        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        canny_image = canny(image)
        cropped_canny = region_of_interest(canny_image)
    
        lines = houghLines(cropped_canny)
        averaged_lines = average_slope_intercept(image, lines)

        camera_to_vehicle_right=camera_to_vehicle_coords(averaged_lines[0][0][0], averaged_lines[0][0][1],10,10, self.teta, 10, 30, 30, self.H, self.alpha)
        camera_to_vehicle_left=camera_to_vehicle_coords(averaged_lines[0][0][2], averaged_lines[0][0][3],10,10, self.teta, 10, 30, 30, self.H, self.alpha)
    
        lane_data.rightX = camera_to_vehicle_right[0]
        lane_data.rightY = camera_to_vehicle_right[1]
        lane_data.rightZ = camera_to_vehicle_right[2]

        lane_data.leftX = camera_to_vehicle_left[0]
        lane_data.leftY = camera_to_vehicle_left[1]
        lane_data.leftZ = camera_to_vehicle_left[2]

        self.pub.publish(lane_data)
        time.sleep(self.lane_sleep_s)


if __name__ == "__main__":
    lane_dealer_node = lane_dealerNODE()
    lane_dealer_node.run()