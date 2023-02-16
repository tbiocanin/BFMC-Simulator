#!/usr/bin/env python3

# Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE


import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import math

class CameraHandler():
    # ===================================== INIT==========================================
    def __init__(self):
        """
        Creates a bridge for converting the image from Gazebo image intro OpenCv image
        """
        self.bridge = CvBridge()
        self.cv_image = np.zeros((640, 480))
        rospy.init_node('CAMnod', anonymous=True)
        self.image_sub = rospy.Subscriber("/automobile/image_raw", Image, self.callback)
        rospy.spin()

    def canny(self, img):
        if img is None:
            cap.release()
            cv2.destroyAllWindows()
            exit()
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        kernel = 5
        blur = cv2.GaussianBlur(gray,(kernel, kernel),0)
        canny = cv2.Canny(gray, 50, 150)
        return canny

    def region_of_interest(self,canny):
        height = canny.shape[0]
        width = canny.shape[1]
        mask = np.zeros_like(canny)
        triangle = np.array([[
        (0,height * 7/8),
        (0, int(height*2.5/5)),
        (width, int(height*2.5/5)),
        (width, height* 7/8)]], np.int32)
        cv2.fillPoly(mask, triangle, 255)
        masked_image = cv2.bitwise_and(canny, mask)
        return masked_image

    def houghLines(self, cropped_canny):
        return cv2.HoughLinesP(cropped_canny, 2, np.pi/180, 75, 
            np.array([]), minLineLength=10, maxLineGap=40)

    
    def addWeighted(self, frame, line_image):
        return cv2.addWeighted(frame, 0.8, line_image, 1, 1)

    def display_lines(self, img, lines):
        line_image = np.zeros_like(img)
        if lines is not None:
            for line in lines:
                for x1, y1, x2, y2 in line:
                    cv2.line(line_image,(x1,y1),(x2,y2),(0,0,255),10)
        return line_image

    def make_points(self, image, line):
   
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

    def average_slope_intercept(self, image, lines):
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
                
                #testirati logiku - ako je priblizno jednako nuli onda da zaobidje

                if slope < 0:
                    left_fit.append((slope, intercept))
                else:
                    right_fit.append((slope, intercept))
                    
                
                left_fit_average = np.average(left_fit, axis=0)
                if left_fit_average.size==2:
                    left_line = self.make_points(image, left_fit_average)
                else:
                    left_line=left_line
                    
                right_fit_average = np.average(right_fit, axis=0)
                if right_fit_average.size==2: 
                    right_line = self.make_points(image, right_fit_average)
                else:
                    right_line=right_line
        return np.array([left_line, right_line],dtype=object)

    def camera_to_vehicle_coords(self, x_cam, y_cam, z_cam, yaw_cam, pitch_cam, roll_cam, x_offset, y_offset, z_offset, focal_length):
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

    def distance_to_vehicle(self, x_vehicle, y_vehicle, z_vehicle):
        # Izračunaj euklidsku udaljenost između tačke i ishodišta koordinatnog sistema
        distance = np.sqrt(x_vehicle**2 + y_vehicle**2 + z_vehicle**2)
        return distance


    def callback(self,data):
        self.cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        print(np.info(self.cv_image))
        alpha=3
        H=25
        teta=15
        # _, frame = cap.read()
        canny_image = self.canny(self.cv_image) #potencijalno jos jedno mesto za fino tuniranje
        cropped_canny = self.region_of_interest(canny_image)
        # cv2.imshow("cropped_canny",cropped_canny)

        lines = self.houghLines(cropped_canny)
        averaged_lines = self.average_slope_intercept(self.cv_image, lines)
            
        #print(averaged_lines[0][0][3],averaged_lines[0][0][4],averaged_lines[1][0][1],averaged_lines[1][0][2])
        camera_to_vehicle_right=self.camera_to_vehicle_coords(averaged_lines[0][0][0], averaged_lines[0][0][1],10,10, teta, 10, 30, 30, H,alpha)
        camera_to_vehicle_left=self.camera_to_vehicle_coords(averaged_lines[0][0][2], averaged_lines[0][0][3],10,10, teta, 10, 30, 30, H,alpha)
        print(camera_to_vehicle_right)
        print(camera_to_vehicle_left)
        line_image = self.display_lines(self.cv_image, averaged_lines)
        combo_image = self.addWeighted(self.cv_image, line_image)

        cv2.imshow("Frame preview", combo_image)
        key = cv2.waitKey(1)
    
            
if __name__ == '__main__':
    try:
        nod = CameraHandler()
    except rospy.ROSInterruptException:
        pass