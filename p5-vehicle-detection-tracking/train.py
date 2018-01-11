#!/usr/bin/python3
# -*- coding: utf-8 -*-

# Sklearn methods
from sklearn.preprocessing import StandardScaler
from sklearn.cross_validation import train_test_split
from sklearn.externals import joblib
from sklearn.svm import SVC, LinearSVC

import pickle

# Plot and handle images
import matplotlib.image as mpimg
from skimage.feature import hog
import matplotlib.pyplot as plt
import numpy as np
import cv2

import time
import os

# Utils import
from utils import Utils as U

from image import Image

CAR_DIRECTORY = "vehicles"
NONCAR_DIRECTORY = "non-vehicles"

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

def dataset(car_dataset, noncar_dataset):
    """
        Load dataset
        **input:
            *car_dataset: (String) Path to the car dataset
            *noncar_dataset: (String) Path to the noncar dataset
    """
    images = []
    targets = []
    # Function used to load all files path from a directory
    def _load_file(directory, label):
        for walk in os.walk(directory):
            for img in walk[-1]:
                if ".png" in img:
                    images.append(os.path.join(walk[0], img))
                    targets.append(label)

    _load_file(car_dataset, 1)
    _load_file(noncar_dataset, 0)

    images = np.array(images)
    targets = np.array(targets)

    indexs = np.arange(len(images))
    np.random.shuffle(indexs)
    images = images[indexs]
    targets = targets[indexs]

    return images, targets

def extract_features(img_path, image=None):
    """
        Define a function to extract features from a list of images
        Have this function call bin_spatial() and color_hist()
    """
    # Read in each one by one
    size = (64, 64)
    if image is None:
        image = mpimg.imread(img_path)
    image = cv2.resize(image, size)
    # Change color space
    #feature_image = cv2.cvtColor(image, Image.COLOR_SPACE)
    # HOG features
    hog_features_1 = Image.get_hog_features(image[:,:,0])
    hog_features_2 = Image.get_hog_features(image[:,:,1])
    hog_features_3 = Image.get_hog_features(image[:,:,2])

    #plot_image(image)
    _, hog_visu = Image.get_hog_features(cv2.cvtColor(image, cv2.COLOR_RGB2GRAY), vis=True)
    #plot_image(hog_visu, cmap="gray")

    # Flatten features for each channel
    hog_features_1 = hog_features_1.ravel()
    hog_features_2 = hog_features_2.ravel()
    hog_features_3 = hog_features_3.ravel()
    # Concat features from each channel together
    hog_features = np.hstack((hog_features_1, hog_features_2, hog_features_3))
    #print("hog_features.shape", hog_features.shape)
    return hog_features

def train_dataset(features, targets):
    """
        Train the model and save it.
    """
    X_train, X_test, y_train, y_test = train_test_split(features, targets, test_size=0.2, random_state=42)
    svc = SVC(kernel="rbf", probability=True)
    t = time.time()
    print("Training SVC...")
    svc.fit(X_train, y_train)
    t2 = time.time()
    print(round(t2-t, 2), 'Seconds to train SVC...')
    # Check the score of the SVC
    print('Test Accuracy of SVC = ', round(svc.score(X_test, y_test), 4))
    # Save model
    filename = 'model.pkl'
    pickle.dump(svc, open(filename, 'wb'))

def create_dataset():
    """
        Method used to load all images, compute features of each image, normalize the features
        and save the result for futur use.
    """
    images, targets = dataset(CAR_DIRECTORY, NONCAR_DIRECTORY)
    features = []
    ntargets = []
    nb_images = len(images)
    for i, img in enumerate(images):
        features.append(extract_features(img))
        ntargets.append(targets[i])
        U.progress(i, nb_images, "Extract features...")
    features = np.array(features)
    # Normalize featues
    X_scaler = StandardScaler().fit(features)
    features = X_scaler.transform(features)
    filename = 'x_scaler.pkl'
    # Save the dataset for futur use
    pickle.dump(X_scaler, open(filename, 'wb'))
    np.save("features", features)
    np.save("targets", ntargets)
    targets = ntargets
    return features, targets

if __name__ == '__main__':
    if not os.path.isfile("features.npy"):
        features, targets = create_dataset()
    else:
        features = np.load("features.npy")
        targets = np.load("targets.npy")
    train_dataset(features, targets)
