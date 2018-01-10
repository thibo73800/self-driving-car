#!/usr/bin/python3
# -*- coding: utf-8 -*-

# Plot and handle images
import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import numpy as np
import cv2

car_id = 0
class Car(object):
    """
        Class used to stored informations about a possible car on the Image
    """

    MAX_BOX_LIST = 30

    def __init__(self, box):
        super(Car, self).__init__()

        global car_id
        car_id += 1
        self.car_id = car_id
        self.activity = [True]
        self.active = False
        self.confident = 1.
        self.box_list = []
        self.box = box
        self.center = [(box[0][0] + box[1][0])//2, (box[0][1] + box[1][1])//2]
        self.tmp_clusters_dist = []
        self.tmp_clusters = []

    def add_activity(self, activity):
        self.activity.append(activity)
        if len(self.activity) > self.MAX_BOX_LIST:
            self.activity = self.activity[1:]

    def add_tmp_cluster(self, box):
        center = [(box[0][0] + box[1][0])//2, (box[0][1] + box[1][1])//2]
        dist_center = np.sqrt((center[0] - self.center[0])**2 + (center[1] - self.center[1])**2)
        dist_pos = abs(box[0][0] - self.box[0][0]) + abs(box[0][1] - self.box[0][1]) + abs(box[1][0] - self.box[1][0]) + abs(box[1][1] - self.box[1][1])
        self.tmp_clusters_dist.append(dist_pos)
        self.tmp_clusters.append(box)

    def commit(self):
        if len(self.tmp_clusters_dist) > 0:
            i = np.argmax(self.tmp_clusters_dist)
            box = self.tmp_clusters[i]
            self.tmp_clusters_dist = []
            self.tmp_clusters = []
            self.update_box(box)

    def update_box(self, box):
        self.box_list.append(box)
        if len(self.box_list) > 5:
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
