#!/usr/local/bin/python3.8
import rospy
import numpy as np
import threading
import vehicleControl.getData as getData                # Subs to GPS data
import vehicleControl.steeringWheel as steeringWheel    # Gets steering wheel info
import vehicleControl.supervisor as supervisor
import vehicleControl.road as road
import json
from mcity_msg.msg import Control

# Must do
# TODO: 1. PCIS mkz
# TODO: 2. Setup supervisor
# TODO: 3. Setup anti windup PID
# TODO: 4. Get the wheel to control stuff
# Bonus
# TODO: 1* get some of the modules running in cython for speed
# TODO: 2* get a pygame display to show live data

# ### is for the wheel, can just ^f replace them for when we want the wheel, should add a flag or something

def run():
    # Setup
    rospy.init_node('LaptopCtrl', anonymous=True)
    data = getData.gpsData()
    ### wheel = steeringWheel.Wheel()
    roadS = road.road()
    rate = rospy.Rate(50)
    pub = rospy.Publisher("/mkz_bywire_intf/control", Control, queue_size = 10)
    #3 ACC = supervisor.accPID(set = 4, P = .01, I = .001)

    # Make sure wheel is intialized
    ###while not wheel.setup:
    ###    rate.sleep()
    print("Starting Main Loop")
    # Main loop
    while not rospy.is_shutdown():
        ### user = wheel.getControl()
        #3 acc = ACC.controlSig(data.Vx)
        #3 user.throttle_cmd = acc.throttle_cmd
        #3 user.brake_cmd = acc.brake_cmd
        x, r, p = roadS.step(data.getDataClean())
        print(json.dumps(x, indent = 1))
        ### pub.publish(user)
        rate.sleep()

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
