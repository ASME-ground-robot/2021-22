from imutils.video import VideoStream
import argparse
import imutils
import time
import cv2
import sys

# Arguments
ap = argparse.ArgumentParser()
ap.add_argument("-t", "--type", type=str,
	default="DICT_4X4_50",
	help="type of ArUCo tag to detect")
args = vars(ap.parse_args())

# Defines the ArUCo tags to be used (URC uses the 4x4_50 tags)
ARUCO_DICT = {"DICT_4X4_50": cv2.aruco.DICT_4X4_50}
# Loads the tags and inputs parameters
arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT[args["type"]])
arucoParams = cv2.aruco.DetectorParameters_create()

# Opens camera
cam = VideoStream(src=0).start()
time.sleep(2.0)

# The real code begins lol
# Loops the frames from the camera and continously detects the tags
while True:
    # Grabs the frame from the threaded video stream and resize it
    frame = cam.read()
#    frame = imutils.resize(frame, width=1000)
    # Detects the ArUCo tags in the input frames
    (corners, ids, rejected) = cv2.aruco.detectMarkers(frame,
	arucoDict, parameters=arucoParams)
    # Verifies at least one ArUco tag is detected
    if len(corners) > 0:
	ids = ids.flatten()
	# Loops the coordinates of the detected ArUCo tags in the camera stream
	for (markerCorner, markerID) in zip(corners, ids):
	    # Extracts the coordinates of the tags (taken from each corner of the stream)
	    corners = markerCorner.reshape((4, 2))
	    (topLeft, topRight, bottomRight, bottomLeft) = corners
	    # Converts each of the (x, y)-coordinate pairs to integers
	    topRight = (int(topRight[0]), int(topRight[1]))
	    bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
	    bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
	    topLeft = (int(topLeft[0]), int(topLeft[1]))
	    # Draws the box that surrounds the detected tags
	    cv2.line(frame, topLeft, topRight, (0, 255, 0), 2)
	    cv2.line(frame, topRight, bottomRight, (0, 255, 0), 2)
	    cv2.line(frame, bottomRight, bottomLeft, (0, 255, 0), 2)
	    cv2.line(frame, bottomLeft, topLeft, (0, 255, 0), 2)
	    # Writes the coordinates of the center of each box
	    xcoord = int((topLeft[0] + bottomRight[0]) / 2.0)
	    ycoord = int((topLeft[1] + bottomRight[1]) / 2.0)
	    cv2.circle(frame, (xcoord, ycoord), 4, (0, 0, 255), -1)
	    # Writes the ID of the ArCUo tag
	    cv2.putText(frame, str(markerID),
		(topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,
		0.5, (0, 255, 0), 2)

    # Show the output
    cv2.imshow("Big Brain Rover", frame)

    # When q is pressed on keyboard, breaks the loop
    if cv2.waitKey(1) & 0xFF == ord("q"):
	break

# Shuts everything off
cv2.destroyAllWindows()
cam.stop()
