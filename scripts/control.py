#!/usr/local/bin/python3.8
import rospy
import datetime
import numpy as np
import threading
import json
import csv
import time as TIME
import vehicleControl.getData as getData                    # Subs to GPS data
import vehicleControl.steeringWheel as steeringWheel        # steeringwheel in loop
import vehicleControl.supervisor as supervisor              # controller/supervisor
import vehicleControl.road as road                          # rtk2state
import vehicleControl.controlPublisher as controlPublisher  # Pushes control to car
import vehicleControl.recordTestData as recordTestData      # Records tested data
from mcity_msg.msg import Control                           # MKZ rosmessage for control

# Must do
# TODO: 1. Update code to work without Wheel
# TODO: 4. update how LK supervisor is setup
# Bonus

# BUGS
# Virtual wall sometimes works great othertimes not at all
# Must fix launch

# Constants/Params
ST_RATIO = 12 #14.8 # Onelin -> 14.8 but petter's code used 12
MPH2MPS = 0.44704
RAD2DEG = 180/np.pi
DEG2RAD = np.pi/180
CONTROLRATE = 25 # Rate we are running control loop at
VEHICLERATE = 50 # Vehicle runs at 50hz
SPEEDMPH = rospy.get_param('CCSpeed', 10) # Speed setpoint
USINGWHEEL = rospy.get_param('Driver', False) # if false will just run in "AV" mode

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
    steerWheelData = getData.steeringData()
    file_name = rospy.get_param('track', 'default') + ".csv"
    print("Using route named: " + file_name)
    CC = supervisor.CC(set=SPEEDMPH*MPH2MPS, P=.05, I=.01, dt=1/CONTROLRATE)
    LK = supervisor.LK(SPEEDMPH)
    roadS = road.road(file_name=file_name, prev_length=LK.pSteps+1, dt=LK._dt) # Set uo class for RTK to states, make sure to set route
    rate = rospy.Rate(CONTROLRATE)
    if USINGWHEEL:
        wheel = steeringWheel.Wheel(CONTROLRATE) # Sets up module to get steering inputs

    # Setup data recording
    time = datetime.datetime.now()
    savefile = rospy.get_param('savefile', time.strftime("%d-%m-%y-%H-%M-%S")) #If no rosparam set deualt to time
    print("Will dave data to file named: " + savefile)
    inputs = {"uUser" : 0, "uOpt": 0, "blendingRatio": 0, "uOut": 0, "M": 0}
    dataNow = data.getDataClean() # have the proper locks to get the data
    states, r, p = roadS.step(dataNow) # Convert data to states
    u_old = 0
    u_cur = 0
    states['u_del'] = steerWheelData.get()/ST_RATIO

    recorder = recordTestData.recordTestData(dataNow, states, inputs, savefile)

    # Make sure wheel is intialized
    while USINGWHEEL and not wheel.setup:
        rate.sleep()
    print("Starting Main Loop")

    control.start()

    TIME.sleep(5)
    

    # Add a loop which gets car to speed

    # Main loop
    while not rospy.is_shutdown():
        #Get the data and turn it into states
        dataNow = data.getDataClean() # have the proper locks to get the data
        states, r, p = roadS.step(dataNow) 
        states['u_del'] = steerWheelData.get()/ST_RATIO
        x = np.array([[states['y']], [states['nu']], [states['dPsi']], [states['r']], [states['u_del']]])
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
            u_old = u_cur
            uBlend, uOpt, blend, M = LK.supervise(x, uUser, p)
            u_cur = uBlend
            ctrl.steering_cmd = u_cur*ST_RATIO # Convert blended input back to steering space
            if USINGWHEEL:
                wheel.virtualWall(u_cur, uUser) # Check this it randomly stopped before
        else:
            blend = -1
            uOpt = 0
            uBlend = uUser
            M = -1
        # Check if we are doing cruise control
        if (CCon):
            ccCommand = CC.controlSig(dataNow['Vx'])
            ctrl.throttle_cmd = ccCommand.throttle_cmd
            ctrl.brake_cmd = ccCommand.brake_cmd
        input = {"uUser" : uUser, "uOpt": uOpt, "blendingRatio": blend, "uOut": uBlend, "M": M}
        # Update control publsher
        recorder.write(dataNow, states, input)
        control.update(ctrl)
        rate.sleep()

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
