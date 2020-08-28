#!/usr/local/bin/python3.8
import rospy
import numpy as np
import threading
import vehicleControl.getData as getData                # Subs to GPS data
import vehicleControl.steeringWheel as steeringWheel    # Gets steering wheel info
import vehicleControl.supervisor as supervisor
import vehicleControl.road as road
import vehicleControl.controlPublisher as controlPublisher
import vehicleControl.recordTestData as recordTestData
import json
import csv
from mcity_msg.msg import Control

# Must do
# TODO: 1. PCIS mkz Ok we have one for 5
# TODO: 5. Record Desired Data Function
# Bonus
# TODO: 1* get some of the modules running in cython for speed
# TODO: 2* get a pygame display to show live data

# ### is for the wheel, can just ^f replace them for when we want the wheel, should add a flag or something

ST_RATIO = 14.8
MPH2MPS = 0.44704
RAD2DEG = 180/np.pi
DEG2RAD = np.pi/180
CONTROLRATE = 10 # Rate we are running control loop at
VEHICLERATE = 50 # Vehicle runs at 50hz
SPEEDMPH = 5 # Speed setpoint

def run():
    '''The main control loop to control the vehicle

    This first initalizes the needed modules then runs a control loop at the set
    frequency.
    '''

    rospy.init_node('LaptopCtrl', anonymous=True)
    data = getData.gpsData() # Set up module to get GPS data
    control = controlPublisher.controlPublisher(VEHICLERATE) # Sets up module to publish control at 50hz
    wheel = steeringWheel.Wheel(CONTROLRATE) # Sets up module to get steering inputs
    roadS = road.road(file_name="ParkingLotStraight.csv") # Set uo class for RTK to states, make sure to set route
    rate = rospy.Rate(CONTROLRATE)
    CC = supervisor.CC(set = SPEEDMPH*MPH2MPS, P = .1, I = .01)
    LK = supervisor.LK(SPEEDMPH)

    # Setup data recording
    savefile = rospy.get_param('savefile', 'test')
    inputs = {"uUser" : 0, "uOpt": 0, "blendingRatio": 0, "uOut": 0}
    dataNow = data.getDataClean() # have the proper locks to get the data
    states, r, p = roadS.step(dataNow) # Convert data to states
    recorder = recordTestData.recordTestData(dataNow,states,inputs, savefile)

    # Make sure wheel is intialized
    while not wheel.setup:
        rate.sleep()
    print("Starting Main Loop")

    control.start()

    # Main loop
    while not rospy.is_shutdown():
        #Get the data and turn it into states
        dataNow = data.getDataClean() # have the proper locks to get the data
        states, r, p = roadS.step(dataNow) # Convert data to states
        x = np.array([[states['y']], [states['nu']], [states['dPsi']], [states['r']]])
        #Get user input
        ctrl = Control(); # Init Blank Message, mainly here to be able to test without wheel
        ctrl = wheel.getControl() # Get wheel states
        CCon, LKon, quit = wheel.getFlags()
        uUser = ctrl.steering_cmd/ST_RATIO #Convert user input to vehicle space
        if (quit):
            return
        # Check if we are doing lane keeping
        if (LKon):
            #Get supervision input
            uBlend, uOpt, blend = LK.supervise(x, uUser, p)
            wheel.virtualWall(uBlend, uUser)
            ctrl.steering_cmd = uBlend*ST_RATIO # Convert blended input back to steering space
        else:
            blend = -1
            uOpt = 0
            uBlend = 0
        # Check if we are doing cruise control
        if (CCon):
            ccCommand = CC.controlSig(dataNow['Vx'])
            ctrl.throttle_cmd = ccCommand.throttle_cmd
            ctrl.brake_cmd = ccCommand.brake_cmd
        input = {"uUser" : uUser, "uOpt": uOpt, "blendingRatio": blend, "uOut": uBlend}
        # Update control publsher
        recorder.write(dataNow, states, input)
        control.update(ctrl)
        rate.sleep()

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
