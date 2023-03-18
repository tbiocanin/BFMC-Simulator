#!/usr/bin/env python3

# import modules
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import math
from tensorflow import keras
import tensorflow as tf

class signDetection():

    def __init__(self):

        # rospu.Publisher/Subscriber("", , callback = self.callback)

        self.bridge = CvBridge()
        self.cv_image = np.zeros((640, 480))
        rospy.init_node('Kita', anonymous = True)
        self.image_sub = rospy.Subscriber("/automobile/image_raw", Image, self.callback)
        self.model = keras.models.load_model('pesakModel')

        print(self.model.summary())
        print('model')

        rospy.spin()

    def run(self):

        rospy.loginfo("Starting signDetection node")
        rospy.spin()

    def callback(self, data):

        self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")


        # imageResized = np.reshape(self.cv_image, (-1, 256, 240, 3))
        imageResized = cv2.resize(self.cv_image, (128, 128))
        imageResized = imageResized.reshape(-1, 128, 128, 3)
        klasa = self.model.predict(imageResized, verbose = 0)

        # klasa = np.argmax(klasa)
        # class 0 -> crosswalk, class 1 -> stop sign, class 2 -> stop light !!

        print(klasa)

        cv2.imshow("Frame preview", self.cv_image)
        key = cv2.waitKey(1)

if __name__ == '__main__':

    try:
        node = signDetection()
        node.run()
    except rospy.ROSInterruptException:
        pass