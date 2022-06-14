#!/usr/bin/env python

import rospy
from core2.msg import Core2PubMsg
from math import sin, cos

def publish_mission_number(number):
    msg = Core2PubMsg()
    msg.circuit_12V = number
    indicators_publisher.publish(msg)


if __name__ == "__main__":
    indicators_publisher = rospy.Publisher('/external_devices_output', Core2PubMsg, queue_size=10)
    rospy.init_node('generator1', anonymous=True)
    number = 1
    while True:
        msg = Core2PubMsg()
        msg.circuit_12V = sin(number)
        msg.temperature = cos(number)
        indicators_publisher.publish(msg)
        number += 0.2
        rospy.sleep(1)

