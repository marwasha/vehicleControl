#!/usr/local/bin/python3.8
import rospy
import csv
import numpy as np
import vehicleControl.getData as getData
import threading

def record():
    ''' This function runs as the key recording thread

    This function initalizes the data collection then runs infintatly recording
    the gps data at a set frequency. Since this is run in a daemon thread, this
    thread will terminate when the main thread ends giving us control over its
    opperation
    '''
    # Start data collection
    gps = getData.gpsData()
    rate = rospy.Rate(50)
    # Init the CSV Writer
    csv_columns = list(getData.gpsData.dataClean.keys())
    file_name = rospy.get_param('track', 'default') + ".csv"  #If no rosparam set deualt to time
    print("Will record data to file: " + file_name)
    csv_file = "/home/laptopuser/mkz/data/raw/" + file_name
    # Open the file
    with open(csv_file, 'w') as csvfile:
        # Set up the CSV
        writer = csv.DictWriter(csvfile,
                                fieldnames=csv_columns,
                                quoting=csv.QUOTE_NONNUMERIC)
        writer.writeheader()
        # Record Loop
        while not rospy.is_shutdown():
            data = gps.getDataClean()
            if data['latitude'] == 0:
                continue
            writer.writerow(data)
            rate.sleep()

def run():
    ''' This function starts the recording thread and accepts keyboard inputs'''
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
