#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2
import matplotlib

class tlNODE():

    def __init__(self):

        rospy.init_node("TrafficLI")
        self.imageSub = rospy.Subscriber("/automobile/image_raw",Image,self.ImCall)
        self.windowSub = rospy.Subscriber("/yolo/window", String, self.callback)
        self.TLPub = rospy.Publisher("/trafficL/triggerL",Float32, queue_size = 10) 

        self.bridge = CvBridge()
        self.cv_image = np.zeros((640, 480))

    def run(self):
        rospy.loginfo("Starting SimToCarNODE")
        rospy.spin()

    def callback(self, data):
        cords = data.data
        #float(pom[1:len(pom)-1].split("     ")[3])
        cords = cords.split("     ")
        if len(cords)!=1:
            print(cords)
            sol = self.traffic_light_classification(self.cv_image, float(cords[1]),float(cords[2]),float(cords[3]),float(cords[4]))
            # print(sol)
            self.TLPub.publish(sol)

    def ImCall(self,data):
        self.cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')


    def traffic_light_classification(self, image,x1,y1,x2,y2):
    
        # image = cv2.imread(image)
        image = image[round(y1):round(y2), round(x1):round(x2)]
        red_light = False
        yellow_light = False
        green_light = False

        # if image == None:
        #     print("ticulica")

        # hsv = matplotlib.colors.rgb_to_hsv(image)
        # print(type(hsv))
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        
        lower_red = np.array([0,30,30])
        upper_red = np.array([20,255,255])

        
        lower_yellow = np.array([20,80,80])
        upper_yellow = np.array([40,255,255])

        # zapamtiti da je negde oko trideset granica - prim.prev. JOZE VODNIK
        # 
        lower_green = np.array([40,50,50])
        upper_green = np.array([100,255,255])

        # hsv = np.asmatrix(hsv)
        mask1 = cv2.inRange(hsv, lower_red, upper_red)
        
        mask2 = cv2.inRange(hsv, lower_yellow, upper_yellow)
        
        mask3 = cv2.inRange(hsv, lower_green, upper_green)
        

        res1 = cv2.bitwise_and(image,image, mask = mask1)
        res2 = cv2.bitwise_and(image,image, mask = mask2)
        res3 = cv2.bitwise_and(image,image, mask = mask3)

        if np.any(res1 == 255):
            red_light = True
        elif np.any(res2 == 255):
            yellow_light = True
        elif np.any(res3 == 255):
            green_light = True

        if red_light:
            return 0
        elif yellow_light:
            return 1
        elif green_light:
            return 2
        else:
            return -1

        #0 R, 1 Y, 2 G. -1 greska

    

if __name__ == '__main__':
    tl = tlNODE()
    tl.run()