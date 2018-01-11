#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""
Module used to convert the input video to a video with vehicles detected.

Usage:
    main.py <video>

Options:
  -h --help     Show this screen.
  <video>       Input video path
"""

global_i = 0

# Import everything needed to edit/save/watch video clips
from moviepy.editor import VideoFileClip
from scipy.signal import find_peaks_cwt
from IPython.display import HTML

from scipy.ndimage.measurements import label

# Sklearn methods
from sklearn.externals import joblib
from sklearn.cross_validation import train_test_split

# Plot and handle images
import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import numpy as np
import cv2

# Use to handle the HELP doc
from docopt import docopt
import pickle

# Load the current model and usefull methods
from image import Image
from car import Car

model = pickle.load(open("model.pkl", 'rb'))
X_scaler = pickle.load(open("x_scaler.pkl", 'rb'))

def plot_image(img, cmap=None, title=None):
    """
        Plot image on the screen
        **input:
            *img: Image to plot
    """
    if cmap is None:
        plt.imshow(img)
    else:
        plt.imshow(img, cmap=cmap)
    if title is not None:
        plt.title(title)
    plt.show()

def draw_boxes(img, bboxes, color=(0, 0, 255), thick=6):
    # Make a copy of the image
    imcopy = np.copy(img)
    # Iterate through the bounding boxes
    for bbox in bboxes:
        # Draw a rectangle given bbox coordinates
        cv2.rectangle(imcopy, bbox[0], bbox[1], color, thick)
    # Return the image copy with boxes drawn
    return imcopy


class CarsDetection(object):
    """
        Class used to detect cards in a video
    """

    YSTART = 380
    YSTOP = 656
    SCALES = [1., 1.4, 1.8, 2., 2.5, 3.]
    SCALES_LIMIT_MAX = [4, 3, 3, 3, 2, 1]
    SCALES_LIMIT_MIN = [0, 0, 0, 1, 1, 0]
    MAX_H = 5

    def __init__(self):
        super(CarsDetection, self).__init__()
        self.cars = []
        self.last_clusters_center = []
        self.last_heatmap = []
        self.first = True

    def _range_overlap(self, a_min, a_max, b_min, b_max):
        """
            Neither range is completely greater than the other
        """
        return False if (a_min > b_max) or (a_max < b_min) else True

    def _overlap(self, box1, box2):
        """
            Overlapping rectangles overlap both horizontally & vertically
        """
        x_overlap = self._range_overlap(box1[0][0], box1[1][0], box2[0][0], box2[1][0])
        y_overlap = self._range_overlap(box1[0][1], box1[1][1], box2[0][1], box2[1][1])
        return x_overlap and y_overlap

    def get_heatmap(self, img, shape, bbox_list, boxes_confident, show=False):
        heatmap = np.zeros(shape)
        # Iterate through list of bboxes found
        for box, confident in zip(bbox_list, boxes_confident):
            # Add += confident for all pixels inside each bbox
            # Assumingclusters.app. each "box" takes the form ((x1, y1), (x2, y2))
            heatmap[box[0][1]:box[1][1], box[0][0]:box[1][0]] += confident
        # Compute a new heat-map which is the sum of the current heat-map and the previous
        # ten heat-map.
        self.last_heatmap.append(heatmap)
        if len(self.last_heatmap) > self.MAX_H:
            self.last_heatmap = self.last_heatmap[1:]
        heatmap = np.array(self.last_heatmap).sum(axis=0)
        # Set the heatmap threshold
        threshold = max(heatmap[heatmap != 0].mean(), 5.0)
        heatmap[heatmap <= threshold] = 0

        final_clusters = []
        self.last_clusters_center = []

        labels = label(heatmap)
        for car_number in range(1, labels[1]+1):
            # Find pixels with each car_number label value
            nonzero = (labels[0] == car_number).nonzero()
            # Identify x and y values of those pixels
            nonzeroy = np.array(nonzero[0])
            nonzerox = np.array(nonzero[1])
            # Define a bounding box based on min/max x and y
            bbox = [[np.min(nonzerox), np.min(nonzeroy)], [np.max(nonzerox), np.max(nonzeroy)]]
            # Draw the box on the image
            cv2.rectangle(img, (bbox[0][0], bbox[0][1]), (bbox[1][0], bbox[1][1]), (0,240,0),6)
            final_clusters.append(bbox)

        plot_image(img)

        return final_clusters, img

    def _get_hog_features(self, image, xb, yb):
        """
            Method used to get hog features at a given position in the image
            **input:
                *image: Image class
                *xb: x step in the grid
                *yb: y step in the grid
        """
        ypos = yb * image.cells_per_step
        xpos = xb * image.cells_per_step

        hog_feat1 = image.hog1[ypos:ypos+image.nblocks_per_window, xpos:xpos+image.nblocks_per_window].ravel()
        hog_feat2 = image.hog2[ypos:ypos+image.nblocks_per_window, xpos:xpos+image.nblocks_per_window].ravel()
        hog_feat3 = image.hog3[ypos:ypos+image.nblocks_per_window, xpos:xpos+image.nblocks_per_window].ravel()
        hog_features = np.hstack((hog_feat1, hog_feat2, hog_feat3))
        return hog_features

    def _create_new_box(self, image, xb, yb):
        """
            Method used to create a new box for the image given some position in the grid (*xb, *yb)
            **input:
                *image: Image class
                *xb: x step in the grid
                *yb: y step in the grid
        """
        ypos = yb * image.cells_per_step
        xpos = xb * image.cells_per_step
        xleft = xpos*image.PIX_PER_CELL
        ytop = ypos*image.PIX_PER_CELL
        xbox_left = np.int(xleft*image.scale)
        ytop_draw = np.int(ytop*image.scale)
        win_draw = np.int(image.WINDOW*image.scale)
        # Create new boxe and append it to the list of boxes
        n_box = [[xbox_left, ytop_draw+self.YSTART],[xbox_left+win_draw,ytop_draw+win_draw+self.YSTART]]

        return n_box, [(n_box[0][0] + n_box[1][0])//2, (n_box[0][1] + n_box[1][1])//2]

    def detect_new_cars(self, img, images, show=False):
        """
            Method used to detect new cars within the image
        """
        for car in self.cars:
            car.add_activity(False) # By default the car is not detected
        # Copy original image

        # Lists to store detected boxes and detections confident (probability)
        boxes = []
        boxes_confident = []
        draw_img = np.copy(img)
        for s, scale in enumerate(self.SCALES):
            for xb in range(images[s].nxsteps):
                x_progress = xb / images[s].nxsteps
                for yb in range(images[s].nysteps):
                    # Condition to look only on the left and right of the image (except for the first time)
                    if x_progress > 0.15 and x_progress < 0.85 and not self.first:
                        n_box, center = self._create_new_box(images[s], xb, yb)
                        dists = [np.sqrt((center[0] - c[0])**2 + (center[1] - c[1])**2) for c in self.last_clusters_center]
                        if (len(dists) == 0 or np.min(dists) > 150.):
                            continue
                    # Extract HOG for this patch
                    hog_features = self._get_hog_features(images[s], xb, yb)
                    # Scale features and make a prediction
                    features = X_scaler.transform([hog_features])
                    # Prediction (softmax)
                    py = model.predict_proba(features)[0]
                    py_cls = np.argmax(py)
                    if py_cls == 1 and py[1] > 0.5: # A car is detected
                        # Create a box
                        n_box, center = self._create_new_box(images[s], xb, yb)
                        boxes.append(n_box)
                        boxes_confident.append(py[1])

        clusters, img = self.get_heatmap(np.copy(img), draw_img.shape, boxes, boxes_confident, show=show)

        for cluster in clusters:
            car_found = False
            for car in self.cars:
                if self._overlap(car.box, cluster):
                    car_found = True
                    car.add_tmp_cluster(cluster)
                    break
            if not car_found:
                self.cars.append(Car(cluster))
        for car in self.cars:
            car.commit()

        return img

    def draw_boxes(self, img):
        """
            Draw final boxes on the output image according to Cars entities.
        """
        cars = []
        for car in self.cars:
            self.last_clusters_center.append([(car.box[0][0] + car.box[1][0])//2, (car.box[0][1] + car.box[1][1])//2])
            if car.active and np.mean(car.activity) <= 0.9:
                car.confident -= 0.6
            if car.active:
                cv2.rectangle(img, (car.box[0][0], car.box[0][1]), (car.box[1][0], car.box[1][1]), (100, 0, 0), 6)
            if (car.active and car.confident > 1.) or (not car.active and np.mean(car.activity) > 0.5):
                cars.append(car)
        self.cars = cars
        return img

    def find_cars(self, img, show=False, rescale=True):
        """
            Define a single function that can extract features using hog sub-sampling and make predictions
        """
        global global_i
        if global_i < 600:
            global_i += 1
            return img
        global_i += 1
        images = [Image(img, scale, self.YSTART, self.YSTOP, rescale=rescale) for scale in self.SCALES]
        img = self.detect_new_cars(img, images, show=show)
        self.first = False
        img = self.draw_boxes(img)
        return img

def main(input_video_path):
    """
        Method used to convert the input video to a video with vehicles detected.
        **input:
            *input_video_path (String) Path to the input video file.
    """
    cars_detection = CarsDetection()

    clip = VideoFileClip(input_video_path)
    output_clip = clip.fl_image(cars_detection.find_cars)
    output_clip.write_videofile("output_tmp.mp4", audio=False)



if __name__ == '__main__':
    arguments = docopt(__doc__)
    main(arguments["<video>"])
