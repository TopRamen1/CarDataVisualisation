#!/usr/bin/env python

import rospy
from core2.msg import Core2PubMsg
import datetime as dt
import numpy as np
import csv
import os
import threading

class Odometry:
    def __init__(self):
        pass
    circuit_12V = 0 # other value in the future. now circuit it's just for example
    temperature = 0


def state_callback(msg):
   lock.acquire()
   Odometry.circuit_12V = msg.circuit_12V
   Odometry.temperature = msg.temperature
   lock.release()
   
def write_data(filename):
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), filename)
    with open(path,'a') as file:

        writer = csv.writer(file)
        writer.writerow(["time","circuit_12V", "temperature"])
        while not rospy.is_shutdown():
            row = [dt.datetime.now().strftime('%H:%M:%S'),Odometry.circuit_12V, Odometry.temperature]
            writer.writerow(row) 
            print(row)
            rospy.sleep(1)
    

if __name__ == "__main__":  
    rospy.Subscriber('/external_devices_output', Core2PubMsg, state_callback)
    rospy.init_node('analize', anonymous=True)
    lock = threading.Lock()
    write_data("odometry_data.csv")


    
    