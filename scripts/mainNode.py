#!/usr/bin/env python3
import rospy
import numpy as np
from mcity_msg.msg import Control
import vehicleControl.getData as getData

def run():
    rospy.init_node('Laptop', anonymous=True)
    data = getData.Data()
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        print(data)
        rate.sleep()

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
