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
# TODO: 1. PCIS mkz Ok we have one for 5
# TODO: 2. Setup supervisor Done!
# TODO: 3. Setup anti windup PID Done!
# TODO: 4. Get the wheel to control stuff']
# TODO: 5. Record Desired Data Function
# Bonus
# TODO: 1* get some of the modules running in cython for speed
# TODO: 2* get a pygame display to show live data

# ### is for the wheel, can just ^f replace them for when we want the wheel, should add a flag or something

ST_RATIO = 12
MPH2MPS = 0.44704
RAD2DEG = 180/np.pi
DEG2RAD = np.pi/180

def run():
    # Setup
    rospy.init_node('LaptopCtrl', anonymous=True)
    data = getData.gpsData() # Set up module to get GPS data
    wheel = steeringWheel.Wheel()
    roadS = road.road(pathfile="data/route/ParkingLotStraight.csv") # Set uo class for RTK to states, make sure to set route
    rate = rospy.Rate(50)
    pub = rospy.Publisher("/mkz_bywire_intf/control", Control, queue_size = 10)
    speedMPH = 5
    ACC = supervisor.CC(set = speedMPH*MPH2MPS, P = .1, I = .01)
    LK = supervisor.LK(speedMPH)

    # Make sure wheel is intialized
    while not wheel.setup:
        rate.sleep()
    print("Starting Main Loop")
    # Main loop
    while not rospy.is_shutdown():
        dataNow = data.getDataClean() # have the proper locks to get the data
        ctrl = Control(); # Init Blank Message
        ctrl = wheel.getControl()
        acc = CC.controlSig(dataNow['Vx'])
        states, r, p = roadS.step(dataNow)
        x = np.array([[states['y']], [states['nu']], [states['dPsi']], [states['r']]])
        u = LK.supervise(x, ctrl.steering_cmd*DEG2RAD/12, p)
        ctrl.steering_cmd = u*RAD2DEG*12
        ctrl.throttle_cmd = acc.throttle_cmd
        ctrl.brake_cmd = acc.brake_cmd
        pub.publish(ctrl)
        rate.sleep()

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
