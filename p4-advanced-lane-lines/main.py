#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""
Method used to convert the input video to a video with lane lines detected.

Usage:
  main.py <video>

Options:
  -h --help     Show this screen.
  <video>       Input video path
"""

# Import everything needed to edit/save/watch video clips
from moviepy.editor import VideoFileClip
from scipy.signal import find_peaks_cwt
from IPython.display import HTML

# Use to handle the HELP doc
from docopt import docopt

# Import methods used to detect lines
from detector_function import *

# Plot and handle images
from PIL import Image, ImageEnhance
import matplotlib.pyplot as plt
import numpy as np
import cv2

# Import calibration
from calibration import calibrate_camera

# Path to saved the output video
OUTPUT_PATH = "output.mp4"

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

def find_histogram_maxima(binary_warped, histogram=None):
    """
        Find maxima position into the histogram and the mean value around this maxima
        **input:
            *binary_warped (2dim numpy Array) Binary image
            *histogram: (Numpy array) [Optional] If not None, the binary_warped value is ignored
    """
    # Create histogram
    if histogram is None:
        histogram = np.sum(binary_warped[binary_warped.shape[0]//2:,:], axis=0)

    # Find all maxima position sorted
    maximas = np.argsort(histogram)[::-1]
    base = maximas[0] # By default, the higher one
    around_mean = np.mean(histogram[base-10:base+10])
    # The higher value could be at the top of a partial concave hill
    for maxima in maximas: # Search for a good maxima on the right side of the histogram
        value = histogram[maxima]
        left_mean = np.mean(histogram[:maxima])
        right_mean = np.mean(histogram[maxima:])

        if left_mean < value * 0.5 and right_mean < value * 0.5:
            base = maxima
            around_mean = np.mean(histogram[base-10:base+10])
            break

    return base, around_mean

def get_color_image(image):
    """
        Function used to enhence the contrast into the the original image
        **input:
            *image: (3dim numpy array) [Height, Width, Channels(3)]
    """
    # Enhence colors into the image
    color_img = Image.fromarray(image)
    pixelRGB = color_img.getpixel((0,0))
    R,G,B = pixelRGB
    brightness = sum([R,G,B])/3 * 1.5
    color_img = ImageEnhance.Contrast(color_img).enhance(1. + (1. * brightness/255.))
    color_img = np.asarray(color_img)
    return color_img

def get_h_filter(s_binary, h_binary, plot):
    """
        H&S: Colors channel
        Method used to combined S channel ahd H channel to produced a new H filter.
        **input:
            *image: (3dim numpy array) [Height, Width, Channels(3)]
            *s_binary: (3dim numpy array) [Height, Width]
            *h_binary: (3dim numpy array) [Height, Width]
    """
    # Warped each binary image to keep only the interesting part
    h_binary_warped, _ = perspective_transform(h_binary)
    s_binary_warped, _ = perspective_transform(s_binary)
    # Get the histogram for each part
    histogram_h = np.sum(h_binary_warped[h_binary_warped.shape[0]//2:,:], axis=0)
    histogram_s = np.sum(s_binary_warped[s_binary_warped.shape[0]//2:,:], axis=0)

    half_size = 1280 // 2

    # Get informations about maxima
    maxima_h_left, mean_max_h_left = find_histogram_maxima(None, histogram=histogram_h[:half_size])
    maxima_h_right, mean_max_h_right = find_histogram_maxima(None, histogram=histogram_h[half_size:])
    maxima_s_left, mean_max_s_left = find_histogram_maxima(None, histogram=histogram_s[:half_size])
    maxima_s_right, mean_max_s_right = find_histogram_maxima(None, histogram=histogram_s[half_size:])
    # Get the mean for each part
    mean_h_left = np.mean(histogram_h[:half_size])
    mean_h_right = np.mean(histogram_h[half_size:])
    mean_s_left = np.mean(histogram_s[:half_size])
    mean_s_right = np.mean(histogram_s[half_size:])
    # Compute the ratio for each part
    ratio_h_left = mean_max_h_left / mean_h_left * (np.log(mean_h_left) + 1)
    ratio_h_right = mean_max_h_right / mean_h_right * (np.log(mean_h_right) + 1)
    ratio_s_left =  mean_max_s_left / mean_s_left * (np.log(mean_s_left) + 1)
    ratio_s_right = mean_max_s_right / mean_s_right * (np.log(mean_s_right) + 1)

    if plot:
        plt.plot(histogram_s)
        plt.show()
        plt.plot(histogram_h)
        plt.show()

    left_h_filter = np.copy(h_binary)
    right_h_filter = np.copy(h_binary)

    if ratio_h_left > ratio_s_left:
        left_h_filter[:,half_size:] = 0
    else:
        left_h_filter[:,:] = 0
    if ratio_h_right > ratio_s_right:
        right_h_filter[:,:half_size] = 0
    else:
        right_h_filter[:,:] = 0

    return left_h_filter, right_h_filter

def color_gradient_threshold(image, plot=False):
    """
        Method is used to detect lines into the input image.
        This method return a binary image
        **input:
            *image: (3dim numpy array) [Height, Width, Channels(3)]
    """
    gradx = abs_sobel_thresh(image, orient='x', sobel_kernel=9, thresh=(9, 120))
    grady = abs_sobel_thresh(image, orient='y', sobel_kernel=9, thresh=(9, 120))
    mag_binary = mag_thresh(image, sobel_kernel=9, mag_thresh=(9, 120))
    dir_binary = dir_threshold(image, sobel_kernel=15, thresh=(0., 1.2))
    # Combined all detections
    combined = np.zeros_like(dir_binary)
    combined[((gradx == 1) & (grady == 1)) | (((mag_binary == 1) & (dir_binary == 1)))] = 1
    # Enhance colors within the image
    color_img = get_color_image(image)
    # Yellow mask
    yellow_mask = cv2.inRange(image,  np.array([150, 150, 0]), np.array([255, 255, 255]))
    # White mask
    white_mask = cv2.inRange(image,  np.array([150, 150, 150]), np.array([255, 255, 255]))
    # Combined white and yellow mask
    yw_binary = np.zeros_like(dir_binary)
    yw_binary[ ((yellow_mask == 255) | (white_mask == 255))] = 1
    if plot: plot_image(yw_binary, cmap="gray", title="Yellow + White")
    # Convert the image to HLS color space
    hls = cv2.cvtColor(color_img, cv2.COLOR_RGB2HLS).astype(np.float)
    h_channel, l_channel, s_channel = hls[:,:,0], hls[:,:,1], hls[:,:,2]
    # Threshold for S and H channel
    s_thresh, h_thresh =(120, 255), (0, 20) # Color threshold
    s_binary = np.zeros_like(dir_binary)
    h_binary = np.zeros_like(dir_binary)
    s_binary[((s_channel >= s_thresh[0]) & (s_channel <= s_thresh[1]))] = 1
    h_binary[((h_channel >= h_thresh[0]) & (h_channel <= h_thresh[1]))] = 1
    # Compare the S channel ith the H to deduce the H filter
    left_h_filter, right_h_filter = get_h_filter(s_binary, h_binary, plot)
    # Combined both channels
    colors = np.zeros_like(dir_binary)
    colors[((s_binary == 1) | (left_h_filter == 1) | (right_h_filter == 1))] = 1
    # Combind colors and gradients filter (H&S channels & Yellow and white colors & gradient threshold)
    color_grad_binary = np.zeros_like(dir_binary)
    color_grad_binary[(colors == 1) & (combined == 1) & (yw_binary == 1)] = 1

    if plot:
        # Plot the result
        f, (ax1, ax2) = plt.subplots(1, 2, figsize=(24, 9))
        f.tight_layout()
        ax1.imshow(image)
        ax1.set_title('Original Image', fontsize=50)
        ax2.imshow(combined, cmap='gray')
        ax2.set_title('Combined', fontsize=50)
        plt.subplots_adjust(left=0., right=1, top=0.9, bottom=0.)
        plt.show()

        # Plot the result
        f, (ax1, ax2) = plt.subplots(1, 2, figsize=(24, 9))
        f.tight_layout()
        ax1.imshow(image)
        ax1.set_title('Original Image', fontsize=50)
        ax2.imshow(s_binary, cmap='gray')
        ax2.set_title('Color S', fontsize=50)
        plt.subplots_adjust(left=0., right=1, top=0.9, bottom=0.)
        plt.show()

        # Plot the result
        f, (ax1, ax2) = plt.subplots(1, 2, figsize=(24, 9))
        f.tight_layout()
        ax1.imshow(image)
        ax1.set_title('Original Image', fontsize=50)
        ax2.imshow(h_binary, cmap='gray')
        ax2.set_title('Color H', fontsize=50)
        plt.subplots_adjust(left=0., right=1, top=0.9, bottom=0.)
        plt.show()

        # Plot the result
        f, (ax1, ax2) = plt.subplots(1, 2, figsize=(24, 9))
        f.tight_layout()
        ax1.imshow(image)
        ax1.set_title('Original Image', fontsize=50)
        ax2.imshow(color_grad_binary, cmap='gray')
        ax2.set_title('Color + Gradient', fontsize=50)
        plt.subplots_adjust(left=0., right=1, top=0.9, bottom=0.)
        plt.show()

    return color_grad_binary, combined, colors

def perspective_transform(binary):
    """
        Apply perspective transform on the image
        **input:
            *binary: (2dim numpy array) Binary image
    """
    # Sources points
    pt1, pt2, pt3, pt4 = [530, 450], [730, 450], [1200, 670], [200, 670]
    src_points = np.array([pt1, pt2, pt3, pt4], np.float32)

    img_size = (binary.shape[1], binary.shape[0])
    # Get height, width and margin (p)
    h, w, p = img_size[0], img_size[1], 10

    # Set destinations points
    dest_points = np.float32([[p+150, p], [h-p-150, p], [h-p-150, w-p],  [p+150, w-p]])

    # Transformation matrix
    M = cv2.getPerspectiveTransform(src_points, dest_points)
    # Inverse Transformation matrix
    Minv = cv2.getPerspectiveTransform(dest_points, src_points)

    # Warped the binary image using the Transformation matrix
    binary_warped = cv2.warpPerspective(binary, M, img_size, flags=cv2.INTER_LINEAR)

    return binary_warped, Minv

class LineDetector(object):
    """
        Class ussed to detect lines
    """
    def __init__(self):
        super(LineDetector, self).__init__()
        # STEP 1: CAMERA CALIBRATION
        self.mtx, self.dist = calibrate_camera("camera_cal")
        self.previous_left_position = []
        self.previous_right_position = []

        # Last windows to search lines
        self.left_area = None
        self.right_area = None
        self.areas_set = False
        self.previous_left_curverad = None
        self.previous_right_curverad = None

        self.left_fit_cr = []
        self.right_fit_cr = []

        self.nwindows = 9
        self.inc = 0

        # Variables for lines detection
        # Set the width of the windows +/- margin and set minimum number of pixels found to
        # recenter window
        self.margin, self.minpix = 150, 400

    def _init_areas(self, midpoint, original_image):
        """
            Method used to init the areas history
        """
        self.left_area = [[0, midpoint] for _ in range(self.nwindows)]
        self.right_area = [[midpoint, original_image.shape[1]] for _ in range(self.nwindows)]

    def _get_x_window_position(self, leftx_p, rightx_p, w, binary_warped, winh):
        """
            Method used to set the window position in x
        """
        # Identify window boundaries in y, winy[0]: low, winy[1]: high
        winy = [0, 0]
        winy[0] = binary_warped.shape[0] - (w+1)*winh
        winy[1] = binary_warped.shape[0] - w*winh

        # winx[0]: left, winx[1]: right, winx[0][0]: low, winx[0][1]: high
        winx = [[0,0], [0, 0]]

        winx[0][0] = leftx_p - min(self.margin, abs(leftx_p - self.left_area[w][0]))
        winx[0][1] = leftx_p + min(self.margin, abs(leftx_p - self.left_area[w][1]))
        winx[1][0] = rightx_p - min(self.margin, abs(leftx_p - self.right_area[w][0]))
        winx[1][1] = rightx_p + min(self.margin, abs(leftx_p - self.right_area[w][1]))

        return winx, winy

    def _get_good_lr_indices(self, nonzeroy, nonzerox, winx, winy):
        """
            Method used to detect good pixel (nonzero) on the left/right window
        """
        good_left_inds = ((nonzeroy >= winy[0]) & (nonzeroy < winy[1]) &
        (nonzerox >= winx[0][0]) &  (nonzerox < winx[0][1])).nonzero()[0]
        good_right_inds = ((nonzeroy >= winy[0]) & (nonzeroy < winy[1]) &
        (nonzerox >= winx[1][0]) &  (nonzerox < winx[1][1])).nonzero()[0]

        return good_left_inds, good_right_inds

    def detect_lines(self, original_image, binary_warped, plot=False, plot_final=False):
        """
            Detect lines into the binary image
        """
        # Get the middle point position (in x) in the image
        midpoint = original_image.shape[1] // 2
        # Init history
        if self.left_area is None and self.right_area is None: self._init_areas(midpoint, original_image)
        # Create an output image to draw on and  visualize the result
        out_img = np.dstack((binary_warped, binary_warped, binary_warped)) * 255
        # Set height of windows
        winh = np.int(binary_warped.shape[0] / self.nwindows)
        # Identify the x and y positions of all nonzero pixels in the image
        nonzero = binary_warped.nonzero()
        nonzeroy, nonzerox = np.array(nonzero[0]),  np.array(nonzero[1])
        # Create empty lists to receive left and right lane pixel indices
        left_lane_inds, right_lane_inds = [], []
        # Find maxima positions to start the detection
        leftx_p, _ = find_histogram_maxima(binary_warped[:,self.left_area[0][0]:self.left_area[0][1]])
        rightx_p, _ = find_histogram_maxima(binary_warped[:,self.right_area[0][0]:self.right_area[0][1]])
        leftx_p += self.left_area[0][0]
        rightx_p += self.right_area[0][0]
        # List used to store windows movements across the image
        x_left_moves, x_right_moves = [], []
        # List used to store additional lines points
        add_x_left, add_y_left, add_x_right, add_y_right = [], [], [], []

        for window in range(self.nwindows):
            # Identitfy window boundaries in x and y
            winx, winy = self._get_x_window_position(leftx_p, rightx_p, window, binary_warped, winh)
            # Identify the nonzero pixels in x and y within the window
            good_left_inds, good_right_inds = self._get_good_lr_indices(nonzeroy, nonzerox, winx, winy)

            # If nothing is detect in this area of the image (On the left)
            if len(good_left_inds) < self.minpix and self.areas_set:
                leftx_p = self.left_area[window][0] + self.margin
                # Identitfy window boundaries in x and y
                winx, winy = self._get_x_window_position(leftx_p, rightx_p, window, binary_warped, winh)
                # Identify the nonzero pixels in x and y within the window
                good_left_inds, good_right_inds = self._get_good_lr_indices(nonzeroy, nonzerox, winx, winy)
            # If nothing is detect in this area of the image (On the right)
            if len(good_right_inds) < self.minpix and self.areas_set:
                rightx_p = self.right_area[window][0] + self.margin
                # Identitfy window boundaries in x and y
                winx, winy = self._get_x_window_position(leftx_p, rightx_p, window, binary_warped, winh)
                # Identify the nonzero pixels in x and y within the window
                good_left_inds, good_right_inds = self._get_good_lr_indices(nonzeroy, nonzerox, winx, winy)

            # Draw the windows on the visualization image
            cv2.rectangle(out_img, (winx[0][0], winy[0]),(winx[0][1], winy[1]), (0, 1, 1), 5)
            cv2.rectangle(out_img, (winx[1][0], winy[0]),(winx[1][1], winy[1]), (0, 1, 1), 5)

            if len(good_left_inds) > self.minpix:
                nx = np.int(np.mean(nonzerox[good_left_inds]))
                x_left_moves.append(nx)
                leftx_p = nx
            elif len(x_left_moves) > 0:
                leftx_p += int((x_left_moves[-1] - x_left_moves[0]) / len(x_left_moves)) * 2
                add_x_left += [leftx_p] * winh
                add_y_left += range(winy[0], winy[1])
            elif self.areas_set and window + 1 < len(self.left_area):
                leftx_p = self.left_area[window + 1][0] + self.margin
            if len(good_right_inds) > self.minpix:
                nx = np.int(np.mean(nonzerox[good_right_inds]))
                x_right_moves.append(nx)
                rightx_p = nx
            elif len(x_right_moves) > 0:
                rightx_p += int((x_right_moves[-1] - x_right_moves[0]) / len(x_right_moves)) * 2
                add_x_right += [rightx_p] * winh
                add_y_right += range(winy[0], winy[1])
            elif self.areas_set and window + 1 < len(self.right_area):
                rightx_p = self.right_area[window + 1][0] + self.margin

            if len(good_left_inds) == 0:
                add_x_left += [leftx_p] * winh
                add_y_left += range(winy[0], winy[1])
            if len(good_right_inds) == 0:
                add_x_right += [rightx_p] * winh
                add_y_right += range(winy[0], winy[1])

            # Append these indices to the lists
            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)

            self.left_area[window] = [winx[0][0], winx[0][1]]
            self.right_area[window] = [winx[1][0], winx[1][1]]

        self.areas_set = True
        if plot: plot_image(out_img, cmap="gray")
        # Concatenate the arrays of indices
        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)
        # Extract left and right line pixel positions
        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds]
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]

        leftx = np.array(add_x_left + leftx.tolist())
        lefty = np.array(add_y_left + lefty.tolist())
        rightx = np.array(add_x_right + rightx.tolist())
        righty = np.array(add_y_right + righty.tolist())

        # Fit a second order polynomial to each
        left_fit = np.polyfit(lefty, leftx, 2)
        right_fit = np.polyfit(righty, rightx, 2)

        self.left_fit_cr.append(left_fit.tolist())
        self.right_fit_cr.append(right_fit.tolist())

        if len(self.left_fit_cr) > 10:
            self.left_fit_cr = self.left_fit_cr[-10:]
            self.right_fit_cr = self.right_fit_cr[-10:]

        lx2 = np.array(self.left_fit_cr)[:,0].mean()
        lx1 = np.array(self.left_fit_cr)[:,1].mean()
        lx = np.array(self.left_fit_cr)[:,2].mean()
        rx2 = np.array(self.right_fit_cr)[:,0].mean()
        rx1 = np.array(self.right_fit_cr)[:,1].mean()
        rx = np.array(self.right_fit_cr)[:,2].mean()

        # Generate x and y values for plotting
        #print("Left fit and right fit", left_fit, right_fit)
        ploty = np.linspace(0, binary_warped.shape[0]-1, binary_warped.shape[0] )
        # Set the formula of each lines
        left_fitx = lx2*ploty**2 + lx1*ploty + lx
        right_fitx = rx2*ploty**2 + rx1*ploty + rx

        out_img[lefty, leftx] = [0, 1, 0]
        out_img[righty, rightx] = [0, 0, 1]

        return ploty, left_fitx, right_fitx, out_img

    def detect_curvature(self, ploty, left_fitx, right_fitx):
        """
            Method used to detect the angle of each line
        """
        ## ROTATION
        # Define conversions in x and y from pixels space to meters
        ym_per_pix = 30/720 # meters per pixel in y dimension
        xm_per_pix = 3.7/700 # meters per pixel in x dimension
        y_eval = np.max(ploty)
        left_fit_cr = np.polyfit(ploty*ym_per_pix, left_fitx*xm_per_pix, 2)
        right_fit_cr = np.polyfit(ploty*ym_per_pix, right_fitx*xm_per_pix, 2)
        # Calculate the new radii of curvature
        left_curverad = ((1 + (2*left_fit_cr[0]*y_eval*ym_per_pix + left_fit_cr[1])**2)**1.5) / np.absolute(2*left_fit_cr[0])
        right_curverad = ((1 + (2*right_fit_cr[0]*y_eval*ym_per_pix + right_fit_cr[1])**2)**1.5) / np.absolute(2*right_fit_cr[0])
        # Final curvature
        curvature = (left_curverad + right_curverad) / 2
        return curvature

    def draw_lines(self, binary_warped, img, ploty, left_fitx, right_fitx, Minv):
        """
            Draw line on a new image
        """
        # Create an image to draw the lines on
        warp_zero = np.zeros_like(binary_warped).astype(np.uint8)
        color_warp = np.dstack((warp_zero, warp_zero, warp_zero))
        # Recast the x and y points into usable format for cv2.fillPoly()
        pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
        pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
        pts = np.hstack((pts_left, pts_right))
        # Draw the lane onto the warped blank image
        cv2.fillPoly(color_warp, np.int_([pts]), (0,255, 0))
        # Warp the blank back to original image space using inverse perspective matrix (Minv)
        newwarp = cv2.warpPerspective(color_warp, Minv, (img.shape[1], img.shape[0]))
        # Combine the result with the original image
        result = cv2.addWeighted(img, 1, newwarp, 0.3, 0)

        return result

    def draw_additional_informations(self, result, curvature, binary, _binary, _color):
        """
            Draw additional informations on the final image
        """
        cv2.putText(result, "Curvature: %2.fm" % (curvature), (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2, cv2.LINE_AA)

        n_win = cv2.resize(binary, (200, 100), interpolation = cv2.INTER_CUBIC)
        n_win = np.dstack((n_win, n_win, n_win)) * 255
        result[10:110,1020:1220,:] = n_win
        cv2.putText(result, "binary", (1020, 130), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2, cv2.LINE_AA)

        n_win = cv2.resize(_binary, (200, 100), interpolation = cv2.INTER_CUBIC)
        n_win = np.dstack((n_win, n_win, n_win)) * 255
        result[10:110,810:1010,:] = n_win
        cv2.putText(result, "threshold", (810, 130), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2, cv2.LINE_AA)

        n_win = cv2.resize(_color, (200, 100), interpolation = cv2.INTER_CUBIC)
        n_win = np.dstack((n_win, n_win, n_win)) * 255
        result[10:110,600:800,:] = n_win
        cv2.putText(result, "colors", (600, 130), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2, cv2.LINE_AA)

        return result


    def process_image(self, original_image, plot=False, plot_final=False):
        """
            Method used to process each image of the video
            **input:
                *original_image: (3dim numpy array) [Height, Width, Channels(3)]
        """
        #cv2.imwrite("sample_images/img_%s.jpg" % self.inc, original_image)
        self.inc += 1
        try:
            img = np.copy(original_image)
            # STEP 2 DISTORTION CORRECTION, undistort image with previously compute mtx and dist
            img = cv2.undistort(img, self.mtx, self.dist, None, self.mtx)
            # STEP 3: Color and gradient threshold
            binary, _binary, _color = color_gradient_threshold(img, plot=plot)
            # STEP 4: PERSPECTIVE TRANSFORM
            binary_warped, Minv = perspective_transform(binary)
            # Step 5: Detect lines
            ploty, left_fitx, right_fitx, out_img = self.detect_lines(
                original_image, binary_warped, plot=plot, plot_final=plot_final)
            # Step 6: Detect angles
            curvature = self.detect_curvature(ploty, left_fitx, right_fitx)
            # Step 7: Draw lines
            result = self.draw_lines(binary_warped, img, ploty, left_fitx, right_fitx, Minv)
            result = self.draw_additional_informations(result, curvature, binary, _binary, _color)
            # Plot the final image
            if plot_final: plot_image(result)
        except Exception as e:
            #raise e
            print("e = %s" % e)
            cv2.imwrite("sample_images/img_e_%s.jpg" % self.inc, original_image)
            return original_image

        return result

def main(input_video_path):
    """
        Main function
        **input:
            *input_video_path (String) Path to the input video file.
    """
    line = LineDetector()

    clip = VideoFileClip(input_video_path)
    output_clip = clip.fl_image(line.process_image)
    output_clip.write_videofile("output2.mp4", audio=False)

    plot = False
    plot_final = True

    # for i in range(950, 1080):
    # for i in range(636, 1080):
    #     print("sample_images/img_%s.jpg" % i)
    #     img = cv2.imread("sample_images/img_%s.jpg" % i)
    #     line.process_image(img, plot=plot, plot_final=plot_final)

    #img = cv2.imread("sample_images/img_580.jpg")
    #line.process_image(img, plot=plot, plot_final=plot_final)
    # img = cv2.imread("sample_images/img_580.jpg")
    # line.process_image(img, plot=plot, plot_final=plot_final)
    # img = cv2.imread("sample_images/img_890.jpg")
    # line.process_image(img, plot=plot, plot_final=plot_final)
    # img = cv2.imread("sample_images/img_620.jpg")
    # line.process_image(img, plot=plot, plot_final=plot_final)
    # # Slightly go to the left
    # img = cv2.imread("sample_images/img_539.jpg")
    # line.process_image(img, plot=plot, plot_final=plot_final)
    # img = cv2.imread("sample_images/img_614.jpg")
    # line.process_image(img, plot=plot, plot_final=plot_final)
    # img = cv2.imread("sample_images/img_985.jpg")
    # line.process_image(img, plot=plot, plot_final=plot_final)
    # img = cv2.imread("sample_images/img_965.jpg")
    # line.process_image(img, plot=plot, plot_final=plot_final)
    # img = cv2.imread("sample_images/img_992.jpg")
    # line.process_image(img, plot=plot, plot_final=plot_final)

    return

if __name__ == '__main__':
    arguments = docopt(__doc__)
    main(arguments["<video>"])
