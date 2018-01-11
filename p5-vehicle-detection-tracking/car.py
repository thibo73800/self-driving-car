#!/usr/bin/python3
# -*- coding: utf-8 -*-

# Plot and handle images
import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import numpy as np
import cv2


class Car(object):
    """
        Class used to stored informations about a possible car on the Image
    """

    MAX_BOX_LIST = 20

    def __init__(self, box):
        super(Car, self).__init__()

        self.activity = [True]
        self.active = False
        self.confident = 1.
        self.box_list = []
        self.box = box
        self.center = [(box[0][0] + box[1][0])//2, (box[0][1] + box[1][1])//2]
        self.tmp_clusters_dist = []
        self.tmp_clusters = []

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

    def add_activity(self, activity):
        self.activity.append(activity)
        if len(self.activity) > self.MAX_BOX_LIST:
            self.activity = self.activity[1:]

    def add_tmp_cluster(self, box):
        box_size = (box[0][0] - box[1][0]) * (box[0][1] - box[1][1])
        car_size = (self.box[0][0] - self.box[1][0]) * (self.box[0][1] - self.box[1][1])
        percentage = box_size / car_size
        if percentage > 0.5:
            center = [(box[0][0] + box[1][0])//2, (box[0][1] + box[1][1])//2]
            dist_center = np.sqrt((center[0] - self.center[0])**2 + (center[1] - self.center[1])**2)
            dist_pos = abs(box[0][0] - self.box[0][0]) + abs(box[0][1] - self.box[0][1]) + abs(box[1][0] - self.box[1][0]) + abs(box[1][1] - self.box[1][1])
            self.tmp_clusters_dist.append(dist_pos)
            self.tmp_clusters.append(box)

    def commit(self):
        if len(self.tmp_clusters_dist) > 0:
            # Two box overlap with the car but these box are not Overlapping together
            if len(self.tmp_clusters_dist) == 2 and not self._overlap(self.tmp_clusters[0], self.tmp_clusters[1]):
                box = self.tmp_clusters[0]
                self.tmp_clusters_dist = []
                self.tmp_clusters = []
                self.update_box(box)
                return

            i = np.argmax(self.tmp_clusters_dist)
            box = self.tmp_clusters[i]
            self.tmp_clusters_dist = []
            self.tmp_clusters = []
            self.update_box(box)

    def update_box(self, box):
        self.box_list.append(box)
        if len(self.box_list) > 15:
            self.box_list = self.box_list[1:]
        box_list = np.array(self.box_list)
        x1 = int(box_list[:,0,0].mean())
        x2 = int(box_list[:,1,0].mean())
        y1 = int(box_list[:,0,1].mean())
        y2 = int(box_list[:,1,1].mean())

        self.box = [[x1, y1], [x2, y2]]
        self.center = [(self.box[0][0] + self.box[1][0])//2, (self.box[0][1] + self.box[1][1])//2]

        self.activity[-1] = True

        if len(self.activity) == self.MAX_BOX_LIST and np.mean(self.activity) == 1.:
            self.active = True
            self.confident += 0.1
