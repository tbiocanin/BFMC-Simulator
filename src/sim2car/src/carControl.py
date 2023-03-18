#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import socket

class SimToCar():

    def __init__(self) -> None:
        
        rospy.init_node("SimToCar")

        self.HOST = '192.168.1.105'    # The remote host
        self.PORT = 12243 # The same port as used by the server
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.s.connect((self.HOST,self.PORT))

        self.sharedCommands = rospy.Subscriber("/automobile/command", String, self.callback)
        
    def run(self):
        rospy.loginfo("Starting SimToCarNODE")
        rospy.spin()

    def callback(self, data):
        a = str(data.data)
        self.s.send(a.encode('utf-8'))

if __name__ == '__main__':
    steer = SimToCar()
    steer.run()