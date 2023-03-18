#!/usr/bin/env python3

import rospy
import numpy as np

from utils.msg import u_raw
from std_msgs.msg import String,Bool
import socket
import time


class SteeringNODE():

    def __init__(self):
        
        rospy.init_node("Steering NODE")
        # time.sleep(1)
        self.sub_control = rospy.Subscriber("/control/u_lane", u_raw, self.control_stream)
        self.pubAgent = rospy.Publisher("/automobile/command", String)
        self.signSub = rospy.Subscriber("/laneloc/enable",Bool, callback=self.getEnable)
        self.enable = 1
        # time.sleep(1)
        self.commandMap = {
            "4" : '{"action" : "4", "activate" : ' ,
            "2" : '{"action" : "2", "steerAngle" : ' 
        }

        #connection params
        self.HOST = '192.168.1.105'
        self.PORT = 12243

        #actuation params
        self.tetaMAX = 23.00
        self.tetaMIN = -23.00

        self.limit = 46

        #treba naci sta je okvirno pravo(nije 0)
        self.servo_angle_lane = 0

        self.servo_sleep_s = 0.001

        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.s.connect((self.HOST,self.PORT))
        #Init the PID params
        self.s.send((self.commandMap["4"] + "True" + "}").encode('utf-8'))
        time.sleep(1)

    def run(self):
        rospy.loginfo("Starting SteeringNODE")

        #TODO:
        # Ovde je za sad samo ova funckija(connection)
        # Po meni bi trebali napisati generalniju funkciju koja se
        # ovde poziva samo jednom, a sluzi samo kao switch 
        # za sve razlicite slucaje upravljanja
        # ostane connection fja koja je tu za cisto slanje parametra

        rospy.spin()

    def control_stream(self, msg):

        self.servo_angle_lane = 1.0*self.actuation(msg.u_lane)
        self.connection(self.servo_angle_lane)
        # print(self.servo_angle_lane)

    def getEnable(self,data):
        self.enable = data.data

    def actuation(self, value):

        return self.limit/(1.0 + np.exp(-value)) - self.limit/2
        

    def connection(self,steerValue):
        #class used to send a command to Serial Handler
        if steerValue == 0 : 
            steerValue = 0.0
        komanda = self.commandMap["2"] + str(steerValue) + " }"
        # print(komanda)
        # self.s.send(komanda.encode('utf-8'))
        if self.enable:
            self.pubAgent.publish(komanda)
        time.sleep(self.servo_sleep_s)

if __name__ == '__main__':
    steer = SteeringNODE()
    steer.run()

