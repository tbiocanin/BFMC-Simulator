# test script for sending commands over the server to the serial handler so that the car can move - it works
# data format -> {"action" : "number_Of_Action", "action_attribute" : "float/bool value"}
# for example -> {"action" : "1", "speed" : "0.3"} -> activate the motor and set its speed value to 0.3
#brake (steerAngle)
#{"action" : "2", "steerAngle" : -10.0} 

import socket
import time
# promeniti ovu skriptu za potrebe simulatora 
HOST = '192.168.1.106'    # The remote host
PORT = 12243 # The same port as used by the server
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

s.connect((HOST, PORT))
steerStr = "{" + "action" + ":" +"2 , " + "steerAngle:"
while True:

    print("Enter a desired command")
    inputVal = input()
    s.send(inputVal.encode('utf-8'))
