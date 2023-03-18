#!/usr/bin/env python3

import rospy
import numpy as np
import time

from utils.msg import lanes
from utils.msg import teta
from finite_state_machine import CarState, States
from std_msgs.msg import Float32MultiArray, String, Bool

class DecisionMakingNODE():
    def __init__(self):

        rospy.init_node('Decision_NODE', anonymous=False)

        #CAR_STATE init
        self.car = CarState()

        #PUB/SUB init
        
        #tetaError params
        self.sub_lane = rospy.Subscriber("/camera/pos_raw", lanes, callback=self.cord_stream)
        self.pub_angle = rospy.Publisher("/angle/angle_raw",teta, queue_size=5 )
        self.signSub = rospy.Subscriber("/yolo/loc", String, callback=self.locCall)
        self.pubCommander = rospy.Publisher("/automobile/command", String)

        self.pubLaneEnable = rospy.Publisher("/laneloc/enable",Bool)

        self.teta_sleep_s = 0.01
        self.teta = teta()
        self.signCords = []
        self.commandMap = {
            "4" : '{"action" : "4", "activate" : ' ,
            "2" : '{"action" : "2", "steerAngle" : ',
            "1" : '{"action" : "1", "speed" : ' 
        }
        self.num_parking = 0
       
    def locCall(self,data):
        cords = data.data
        cords = [float(i) for i in cords.split(",")]
        print(cords) 


        if cords[2] != -1: self.num_parking += 1 

        if self.num_parking >= 23:
            self.switch(States.PARKING)
        else:    
            for i in range(0,len(cords)-1,2):
                if len(self.signCords)==0:
                    break
                if cords[i]==-1 and self.signCords[i]!=-1:
                    self.switch(States.STOP)
            self.signCords = cords

    def run(self):
        # while(not rospy.is_shutdown):
        self.State()
        time.sleep(self.teta_sleep_s)

        # rospy.spin()

    def cord_stream(self, msg):
        # Gets the coordinate from the lane dealer node, and computes
        # the angle between the reference vector and the target vector
        
        pomProm = self.teta.tetaE
        self.teta.tetaE = (np.arctan(msg.rightX/ msg.rightZ) - np.arctan(np.absolute(msg.leftX)/msg.leftZ))

        self.State()
        time.sleep(self.teta_sleep_s)
    
    def State(self):
        
        self.car.set_state(States.DRIVE)
        self.switch(self.car.get_state())
            


    def switch(self, state):
        if state == States.IDLE:
            pass
       
        elif state == States.DRIVE:
            self.pub_angle.publish(self.teta)
        
        elif state == States.STOP:
            print("STOOOOOOOOOOP")
            self.pubCommander.publish(self.commandMap["1"] + "0.0" + "}")
            time.sleep(1)
            print("JAJAJJAJ")
            self.pubCommander.publish(self.commandMap["1"] + "0.10" + "}")
            time.sleep(1)
            print("akakakak")
            self.pubCommander.publish(self.commandMap["1"] + "0.20" + "}")

        elif state == States.INTERSECTION:
            pass

        elif state == States.PARKING:
            print("PARKING")
            self.pubCommander.publish(self.commandMap["1"] + "0.0" + "}")

            self.pubLaneEnable.publish(False)
            # publish za iskljucenje ugla od camera.py NODEa
            # ovde ide niz komandi za uparkiravanje
            # self.pubCommander.publish(self.commandMap["1"] + "0.10" + "}")
            time.sleep(2)
            self.pubCommander.publish(self.commandMap["2"] + "23.0" + "}")
            
            time.sleep(2)
            self.pubCommander.publish(self.commandMap["1"] + "0.09" + "}")
            time.sleep(10)
            self.pubCommander.publish(self.commandMap["1"] + "-0.09" + "}")
            time.sleep(10)

            # ovde ide niz komandi za isparkiravanje

            self.pubLaneEnable.publish(True)
            self.pubCommander.publish(self.commandMap["1"] + "0.16" + "}")

            # publish za ukljucenje ugla od camera.py NODE-a.
            self.num_parking = -10000
            

        elif state == States.OVERTAKING:
            pass


if __name__ == '__main__' :

    rospy.loginfo("starting DecisionMakingNode")
    
    decNode = DecisionMakingNODE()
    while(not rospy.is_shutdown()):
        decNode.run()