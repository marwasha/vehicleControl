#!/usr/local/bin/python3.8
import rospy
import csv
import numpy as np
import vehicleControl.getData as getData
import threading

def record():
    # Start data collection
    gps = getData.gpsData()
    rate = rospy.Rate(50)
    # Init the CSV Writer
    csv_columns = list(getData.gpsData.dataClean.keys())
    csv_file = "data/raw/ParkingLotSwerve.csv"
    # Open the file
    with open(csv_file, 'w') as csvfile:
        # Set up the CSV
        writer = csv.DictWriter(csvfile,
                                fieldnames=csv_columns,
                                quoting=csv.QUOTE_NONNUMERIC)
        writer.writeheader()
        # Record Loop
        while not rospy.is_shutdown():
            writer.writerow(gps.getDataClean())
            rate.sleep()

def run():
    # Setup
    rospy.init_node('Laptop_Record', anonymous=True) # Must be in main Loop
    # Locate Recording Thread
    recordLoop = threading.Thread(target=record, daemon = True)
    # Init Phrase
    print("Press r+enter to start")
    keyIn = ' '
    while keyIn != 'r':
        keyIn = input()
    print("Starting Recording")
    # Start Recording
    recordLoop.start()
    #Terminating Phrase
    print("Press s+enter to stop")
    while keyIn != 's':
        keyIn = input()
    print("Ending")

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
