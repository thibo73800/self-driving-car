#!/usr/bin/python3
# -*- coding: utf-8 -*-


# Plot and handle images
import matplotlib.image as mpimg
from skimage.feature import hog
import matplotlib.pyplot as plt
import numpy as np
import cv2


class Image(object):
    """
        Class use to rescale image and change parameters
    """

    #  HOG parameters
    ORIENT = 10
    PIX_PER_CELL = 16
    CELL_PER_BLOCK = 2

    # Color space (YUV) HLS
    #COLOR_SPACE = cv2.COLOR_RGB2HSV

    # Images size
    ## 64 was the orginal sampling rate, with 8 cells and 8 pix per cell
    WINDOW = 64

    @staticmethod
    def get_hog_features(img, vis=False, feature_vec=False, transform_sqrt=True):
        """
            Function to return HOG features
        """
        if vis == True:
            features, hog_image = hog(img, orientations=Image.ORIENT, pixels_per_cell=(Image.PIX_PER_CELL, Image.PIX_PER_CELL),
                                      cells_per_block=(Image.CELL_PER_BLOCK, Image.CELL_PER_BLOCK), transform_sqrt=transform_sqrt,
                                      visualise=True, feature_vector=False)
            return features, hog_image
        else:
            features = hog(img, orientations=Image.ORIENT, pixels_per_cell=(Image.PIX_PER_CELL, Image.PIX_PER_CELL),
                           cells_per_block=(Image.CELL_PER_BLOCK, Image.CELL_PER_BLOCK), transform_sqrt=transform_sqrt,
                           visualise=False, feature_vector=feature_vec)
            return features

    def __init__(self, img, scale, ystart, ystop, rescale=True):
        """
            **input:
                *img: Numpy img
                *scale: Image scale to used
                *ystart, *ystop: Area to search on
        """
        self.scale = scale
        self.tosearch = img[ystart:ystop,:,:]
        # Set the color space
        if rescale:
            self.tosearch = self.tosearch.astype(np.float32)/255
        #self.tosearch = cv2.cvtColor(self.tosearch, self.COLOR_SPACE)
        #print(self.tosearch.mean())
        #self.tosearch = cv2.cvtColor(self.tosearch, cv2.COLOR_BGR2YUV)
        # Resize image if necesary
        if scale != 1:
            imshape = self.tosearch.shape
            self.tosearch = cv2.resize(self.tosearch, (np.int(imshape[1]/scale), np.int(imshape[0]/scale)))

        # Get differents channels
        self.ch1, self.ch2, self.ch3 = self.tosearch[:,:,0], self.tosearch[:,:,1], self.tosearch[:,:,2]
        # Define blocks and steps
        nxblocks = (self.ch1.shape[1] // self.PIX_PER_CELL) - self.CELL_PER_BLOCK + 1
        nyblocks = (self.ch1.shape[0] // self.PIX_PER_CELL) - self.CELL_PER_BLOCK + 1

        self.nblocks_per_window = (self.WINDOW // self.PIX_PER_CELL) - self.CELL_PER_BLOCK + 1
        self.cells_per_step = 2  # Instead of overlap, define how many cells to step
        self.nxsteps = (nxblocks - self.nblocks_per_window) // self.cells_per_step + 1
        self.nysteps = (nyblocks - self.nblocks_per_window) // self.cells_per_step + 1
        # Compute individual channel HOG features for the entire image
        self.hog1 = Image.get_hog_features(self.ch1)
        self.hog2 = Image.get_hog_features(self.ch2)
        self.hog3 = Image.get_hog_features(self.ch3)
