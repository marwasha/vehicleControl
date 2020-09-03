#!/usr/local/bin/python3.8
import vehicleControl.road as road
import csv
import rospy

''' This file converts a raw gps data file to a route

This is done using csv to read/write, and road to convert gps data to vehicle position
'''

# Params
nom_dist = .5; # distance between points

# File Setup
file_name = rospy.get_param('track', 'default') + ".csv" #If no rosparam set to a default
print("Will modify data from: " + file_name) # Inform the user of the file name
csv_file_in = "/home/laptopuser/mkz/data/raw/" + file_name
csv_file_out = "/home/laptopuser/mkz/data/route/" + file_name
csvfilein = open(csv_file_in, 'r', newline='')
csvfileout = open(csv_file_out, 'w')
rawReader = csv.DictReader(csvfilein, quoting=csv.QUOTE_NONNUMERIC) # QUOTE_NONNUMERIC sets that the header terms will be surronded by strings
routeWriter = csv.writer(csvfileout, quoting=csv.QUOTE_NONNUMERIC) # Makes parsing t he file for numeric data easier

# Header for reading
routeWriter.writerow(["xE", "yN", "Distance"])
# Setup first step
dist = 0
row = next(rawReader)
# Find intial X, Y of the GPS
xE, yN = road.cg_pos(row)
routeWriter.writerow([xE, yN, dist])
# Loop all data
for row in rawReader:
    xEn, yNn = road.cg_pos(row)
    rel_dist = ((xEn - xE)**2 + (yNn - yN)**2)**.5
    # Only write data if points are nom dist apart
    if rel_dist > nom_dist:
        dist += rel_dist
        xE = xEn
        yN = yNn
        routeWriter.writerow([xE, yN, dist])

csvfilein.close()
csvfileout.close()
