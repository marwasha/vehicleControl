#!/usr/local/bin/python3.8
import rospy
import numpy as np
import threading
import vehicleControl.getData as getData                # Subs to GPS data
import vehicleControl.steeringWheel as steeringWheel    # Gets steering wheel info
from mcity_msg.msg import Control

def run():
    # Setup
    rospy.init_node('Laptop', anonymous=True)
    data = getData.Data()
    wheel = steeringWheel.Wheel()
    rate = rospy.Rate(50)
    pub = rospy.Publisher("/mkz_bywire_intf/control", Control, queue_size = 10)

    # Make sure wheel is intialized
    while not wheel.setup:
        rate.sleep()

    # Main loop
    while not rospy.is_shutdown():
        user = wheel.getControl()
        pub.publish(user)
        rate.sleep()

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
