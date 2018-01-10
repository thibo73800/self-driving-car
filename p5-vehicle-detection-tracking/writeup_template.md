## Writeup Template
### You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---

**Vehicle Detection Project**

The goals / steps of this project are the following:

* Perform a Histogram of Oriented Gradients (HOG) feature extraction on a labeled training set of images and train a classifier Linear SVM classifier
* Optionally, you can also apply a color transform and append binned color features, as well as histograms of color, to your HOG feature vector.
* Note: for those first two steps don't forget to normalize your features and randomize a selection for training and testing.
* Implement a sliding-window technique and use your trained classifier to search for vehicles in images.
* Run your pipeline on a video stream (start with the test_video.mp4 and later implement on full project_video.mp4) and create a heat map of recurring detections frame by frame to reject outliers and follow detected vehicles.
* Estimate a bounding box for vehicles detected.

## [Rubric](https://review.udacity.com/#!/rubrics/513/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

### Histogram of Oriented Gradients (HOG)

#### 1. Explain how (and identify where in your code) you extracted HOG features from the training images.

My classifier is based on HOG features. HOG features are based on the gradient magnitude and direction at each pixel. Then, the image is divided into small cells. Thus, we can compute a histogram of direction for each cell of the image. This method gives the possibility to detect the signature of an object’s shape easily. Instead of extract HOG features manually, I used the method hog from skimage.feature. This method takes mainly three important parameters: orientations, pixels_per_cell and cells_per_block. To choose the best combinations of these values, I first plotted some random pair of images with their associated HOG features as bellow.

IMAGE WITH BOTH IMG/HOG

Then, once I was happy with some combination of parameters I tried to train the model by changing HOG parameters each time in order to improve the accuracy of the test set. I finally come up with the following values: orient: 2, pixels_per_cell: 16, orient: 2.

The code for this step is contained in the file image.py in the static method get_hog_features.

#### 2. Describe how (and identify where in your code) you trained a classifier using your selected HOG features (and color features if you used them).

For this project, I trained the model using a SVC model. SVC is part of the sklearn.svm library. SVC allows two interesting things, first I can use the kernel trick to classify binary data. Then, I can get the result of the prediction as a probability for both classes (car, non-car).I will explain further in this document why this is useful.

To train the classifier, I change the color space of each image from RGB to HLS. Then I compute HOG features for each image. Before to train the classifier, I normalize the features using the StandardScaler method from sclera.preprocessing.

I train the SVC using normalized HOG features and “rbf” for the kernel trick. To come up with this choice, I tested all possible kernel by comparing each one against others, but it turns out that the best one is rbf.

Lines in the code ….

### Sliding Window Search

#### 1. Describe how (and identify where in your code) you implemented a sliding window search.  How did you decide what scales to search and how much to overlap windows?

The sliding windows search method is used to search for cars across different subregions of the images. To accelerate the processing of each image, I choose to extract HOG features only once per scale (I use 6 different scale). I reduced the area of interest: from 320px to 656px for the y-axis.

The first time the script is launch I search for a car across all the x-axis. Then I will search only new cars on the left and right side of the image. Indeed, it is unlikely to see a new car appear at the center of the image, so we don’t need to search new cars at the center. However, I am also looking for cars around the last positions where I previously detect some. Thus, I am able to track a car across a series of images, even if the car is going to the center of the image.

#### 2. Show some examples of test images to demonstrate how your pipeline is working.  What did you do to optimize the performance of your classifier?

Create images and describe them.

### Video Implementation

#### 1. Provide a link to your final video output.  Your pipeline should perform reasonably well on the entire project video (somewhat wobbly or unstable bounding boxes are ok as long as you are identifying the vehicles most of the time with minimal false positives.)
Here's a [link to my video result](./project_video.mp4)


#### 2. Describe how (and identify where in your code) you implemented some kind of filter for false positives and some method for combining overlapping bounding boxes.

False postive and overlapping bouding boxes ....

---

### Discussion

#### 1. Briefly discuss any problems / issues you faced in your implementation of this project.  Where will your pipeline likely fail?  What could you do to make it more robust?

Here I'll talk about the approach I took, what techniques I used, what worked and why, where the pipeline might fail and how I might improve it if I were going to pursue this project further.  
