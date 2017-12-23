#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""
    This file is used to handle the calibration of the camera
"""

import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import numpy as np
import pickle
import glob
import cv2
import os

CALIBRATE_FILE = "calibration.p"

def calibrate_camera(calibrate_folder):
    """
        Method used to calibrate the camera
        **input:
            **calibrate_folder (String) Path to the store storing all images
     """
    # Check if the calibration have already been compute before
    if os.path.isfile(CALIBRATE_FILE):
        with open(CALIBRATE_FILE, "rb") as f:
            obj = pickle.load(f)
        return obj["mtx"], obj["dist"]

    # Make a list of calibrations images
    images = glob.glob(os.path.join(calibrate_folder, "calibration*.jpg"))
    # Array to store object points and images points from all the images
    objpoints = []
    imgpoints = []
    # Prepare object po*ints like (0,0,0), (1,0,0), (2,0,0) ... (7,5,0)
    objp = np.zeros((6*9, 3), np.float32)
    objp[:,:2] = np.mgrid[0:9, 0:6].T.reshape(-1, 2) # x, y coordinate
    # Variable use to get the shape of grayscale images
    gray_shape = None

    for name in images:
        img = cv2.imread(name)
        # Convert image to grayscale
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray_shape = gray.shape[::-1]
        # Find cheesbord color
        ret, corners = cv2.findChessboardCorners(gray, (9, 6), None)
        if ret == True:
            imgpoints.append(corners)
            objpoints.append(objp)

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray_shape, None, None)

    with open(CALIBRATE_FILE, "wb") as f:
        pickle.dump({"mtx": mtx, "dist": dist}, f)

    return mtx, dist
