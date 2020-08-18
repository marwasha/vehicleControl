#!/usr/local/bin/python3.8
import rospy
import numpy as np
import threading
import vehicleControl.getData as getData                # Subs to GPS data
import vehicleControl.steeringWheel as steeringWheel    # Gets steering wheel info
import vehicleControl.supervisor as supervisor
from mcity_msg.msg import Control

def run():
    # Setup
    rospy.init_node('Laptop', anonymous=True)
    data = getData.gpsData()
    wheel = steeringWheel.Wheel()
    rate = rospy.Rate(50)
    pub = rospy.Publisher("/mkz_bywire_intf/control", Control, queue_size = 10)
    #ACC = supervisor.accPID(set = 4, P = .01, I = .001)

    # Make sure wheel is intialized
    while not wheel.setup:
        rate.sleep()

    # Main loop
    while not rospy.is_shutdown():
        user = wheel.getControl()
        #acc = ACC.controlSig(data.Vx)
        #user.throttle_cmd = acc.throttle_cmd
        #user.brake_cmd = acc.brake_cmd
        print(data)
        pub.publish(user)
        rate.sleep()

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
