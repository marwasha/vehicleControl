#!/usr/local/bin/python3.8
import rospy
import datetime
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
# TODO: 1. Update code to work without Wheel
# TODO: 2. Check why things hang when video with control
# TODO: 4. update how LK supervisor is setup
# Bonus
# TODO: 1* pygame live info

# BUGS
# Virtual wall sometimes works great othertimes not at all
# Must fix launch

# Constants/Params
ST_RATIO = 14.8 # Onelin -> 14.8 but petter's code used 12
MPH2MPS = 0.44704
RAD2DEG = 180/np.pi
DEG2RAD = np.pi/180
CONTROLRATE = 20 # Rate we are running control loop at
VEHICLERATE = 50 # Vehicle runs at 50hz
SPEEDMPH = 5 # Speed setpoint
USINGWHEEL = False # if false will just run in "AV" mode

def run():
    '''The main control loop to control the vehicle

    This first initalizes the needed modules with set params then runs the
    control loop at the set frequency.
    From ROS param uses track and savefile. track determines which route to use
    while savefile states where to save it
    '''

    rospy.init_node('LaptopCtrl', anonymous=True)
    control = controlPublisher.controlPublisher(VEHICLERATE) # Sets up module to publish control at 50hz
    data = getData.gpsData() # Set up module to get GPS data
    if USINGWHEEL:
        wheel = steeringWheel.Wheel(CONTROLRATE) # Sets up module to get steering inputs
    file_name = rospy.get_param('track', 'default') + ".csv"
    print("Using route named: " + file_name)
    CC = supervisor.CC(set=SPEEDMPH*MPH2MPS, P=.05, I=.02, dt=1/CONTROLRATE)
    LK = supervisor.LK(SPEEDMPH)
    roadS = road.road(file_name=file_name, prev_length=LK.pSteps+1, dt=LK.dt) # Set uo class for RTK to states, make sure to set route
    rate = rospy.Rate(CONTROLRATE)

    # Setup data recording
    time = datetime.datetime.now()
    savefile = rospy.get_param('savefile', time.strftime("%d-%m-%y-%H-%M-%S")) #If no rosparam set deualt to time
    print("Will dave data to file named: " + savefile)
    inputs = {"uUser" : 0, "uOpt": 0, "blendingRatio": 0, "uOut": 0, "M": 0}
    dataNow = data.getDataClean() # have the proper locks to get the data
    states, r, p = roadS.step(dataNow) # Convert data to states
    recorder = recordTestData.recordTestData(dataNow,states,inputs, savefile)

    # Make sure wheel is intialized
    while USINGWHEEL and not wheel.setup:
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
        ctrl = Control() # Init Blank Message, mainly here to be able to test without wheel
        ctrl.gear_cmd = 4
        ctrl.brake_cmd = -.01
        if USINGWHEEL:
            ctrl = wheel.getControl() # Get wheel states
            CCon, LKon, quit = wheel.getFlags()
        else:
            CCon, LKon, quit = (True, True, False)
        uUser = ctrl.steering_cmd/ST_RATIO #Convert user input to vehicle space
        if (quit):
            return
        # Check if we are doing lane keeping
        if (LKon):
            #Get supervision input
            uBlend, uOpt, blend, M = LK.supervise(x, uUser, p)
            ctrl.steering_cmd = uOpt*ST_RATIO#uBlend*ST_RATIO # Convert blended input back to steering space
            print(uOpt)
            if USINGWHEEL:
                wheel.virtualWall(uBlend, uUser) # Check this it randomly stopped before
        else:
            blend = -1
            uOpt = 0
            uBlend = 0
            M = -1
        # Check if we are doing cruise control
        if (CCon):
            ccCommand = CC.controlSig(dataNow['Vx'])
            ctrl.throttle_cmd = ccCommand.throttle_cmd
            ctrl.brake_cmd = ccCommand.brake_cmd
        input = {"uUser" : uUser, "uOpt": uOpt, "blendingRatio": blend, "uOut": uBlend, "M": M}
        # print (input)
        # Update control publsher
        recorder.write(dataNow, states, input)
        control.update(ctrl)
        rate.sleep()

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
