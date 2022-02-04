import numpy as np
import argparse
import cv2
import sys

ap = argparse.ArgumentParser()
ap.add_argument("-o", "--output", required=True,
	help="path to output image containing ArUCo tag")
ap.add_argument("-i", "--id", type=int, required=True,
	help="ID of ArUCo tag to generate")
ap.add_argument("-t", "--type", type=str,
	default="DICT_4X4_50",
	help="type of ArUCo tag to generate")
args = vars(ap.parse_args())

ARUCO_DICT = {"DICT_4X4_50": cv2.aruco.DICT_4X4_50}

if ARUCO_DICT.get(args["type"], None) is None:
    print("[INFO] ArUCo tag '{}' is not supported".format(args["type"]))
    sys.exit(0)

arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT[args["type"]])
tag = np.zeros((300,300,1), dtype="uint8")
cv2.aruco.drawMarker(arucoDict, args["id"], 300, tag, 1)

cv2.imwrite(args["output"], tag)
cv2.imshow("ArUCo Tag", tag)
cv2.waitKey(1)

