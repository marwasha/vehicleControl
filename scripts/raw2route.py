#!/usr/local/bin/python3.8
import vehicleControl.road as road
import pymap3d as pm
import numpy as np
import csv

# Params
nom_dist = 1;

# File Setup
file_name = "ParkingLotStraight.csv"
csv_file_in = "data/raw/" + file_name
csv_file_out = "data/route/" + file_name
csvfilein = open(csv_file_in, 'r', newline='')
csvfileout = open(csv_file_out, 'w')
rawReader = csv.DictReader(csvfilein, quoting=csv.QUOTE_NONNUMERIC)
routeWriter = csv.writer(csvfileout, quoting=csv.QUOTE_NONNUMERIC)

# Header for reading
routeWriter.writerow(["xE", "yN", "Distance"])
# Setup first step
dist = 0
row = next(rawReader)
# Find X, Y of the GPS
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
