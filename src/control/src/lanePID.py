#!/usr/bin/env python3

import rospy
import numpy

from utils.msg import teta
from utils.msg import u_raw

import time

class lanePID():
    def __init__(self):

        rospy.init_node("lanePID Node")
        self.sub_angle = rospy.Subscriber("/angle/angle_raw", teta, self.tetaError_stream)
        self.pub_lane_control = rospy.Publisher("/control/u_lane",u_raw,queue_size=5)

        self.u_lane_sleep_s=0.001

        #Za sad je PI kontroler, ukoliko ima potrebe prosiricemo

        #A class that implements a PI(D) controler 

        self.Kp = 4
        self.Ti = 0.01
        self.Ki = 1
        self.Ts = 0.2 
        self.e_old = 0
        self.ui = 0
        self.brojac = 0
        
    def run(self):
        rospy.loginfo("Starting lanePID Node")

        rospy.spin()

    def tetaError_stream(self, msg):
        data_u = u_raw()
        e = msg.tetaE
        up = self.Kp * e

        if self.brojac == 20:
            self.ui += self.Kp*(self.Ts/self.Ti)*e
            self.brojac = 0

        # u = up + self.ui
        u = up

        if u >= 5.5:
            u = 5.5
            self.ui -= self.Ki*(self.Ts/self.Ti)*e
        elif u <= -5.5:
            u = -5.5
            self.ui -= self.Ki*(self.Ts/self.Ti)*e

        data_u.u_lane = u

        self.pub_lane_control.publish(data_u)
        self.brojac += 1
        time.sleep(self.u_lane_sleep_s)

        
if __name__ == '__main__':
    laneControl = lanePID()
    laneControl.run()



        
