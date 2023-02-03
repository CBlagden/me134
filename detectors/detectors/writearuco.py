#!/usr/bin/env python3
#
#   writearuco.py
#
#   Generate an ArUco marker (from a given dictionary).
#

# ROS Imports
import cv2


# Set up the ArUco detector.  Use a dictionary with the appropriate
# tags.  DICT_6x6_1000 has 1000 options (way too many).  DICT_6x6_250
# has 250 options.  DICT_6x6_50 is probably enough for our projects...
dict   = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)
params = cv2.aruco.DetectorParameters_create()

# Create a marker, ID#23, 200x200 pixels.
markerimage = cv2.aruco.drawMarker(dict, 23, 200);
cv2.imwrite("marker23.png", markerimage);

# Create a marker, ID#18, 200x200 pixels.
markerimage = cv2.aruco.drawMarker(dict, 18, 200);
cv2.imwrite("marker18.png", markerimage);

# Create a marker, ID#41, 200x200 pixels.
markerimage = cv2.aruco.drawMarker(dict, 41, 200);
cv2.imwrite("marker41.png", markerimage);

# Create a marker, ID#07, 200x200 pixels.
markerimage = cv2.aruco.drawMarker(dict,  7, 200);
cv2.imwrite("marker07.png", markerimage);
