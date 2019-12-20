#!/usr/bin/env python
"""Segmentation skeleton code for Lab 6
Course: EECS C106A, Fall 2019
Author: Grant Wang

MODIFIED For EECS106A Project F'19 Group 22
"""

import os
import numpy as np
import cv2
import imutils
from datetime import datetime

import matplotlib.pyplot as plt

def read_image(img_name, grayscale=False):
    """ reads an image

    Parameters
    ----------
    img_name : str
        name of image
    grayscale : boolean
        true if image is in grayscale, false o/w
    
    Returns
    -------
    ndarray
        an array representing the image read (w/ extension)
    """

    if not grayscale:
        img = cv2.imread(img_name)
    else:
        img = cv2.imread(img_name, 0)

    return img

def show_image(img_name, title='Fig', grayscale=False):
    """show the  as a matplotlib figure
    
    Parameters
    ----------
    img_name : str
        name of image
    tile : str
        title to give the figure shown
    grayscale : boolean
        true if image is in grayscale, false o/w
    """

    if not grayscale:
        plt.imshow(img_name)
        plt.title(title)
        plt.show()
    else:
        plt.imshow(img_name, cmap='gray')
        plt.title(title)
        plt.show()

def to_grayscale(rgb_img):
    return np.dot(rgb_img[... , :3] , [0.299 , 0.587, 0.114])

def color_space_segmentation(img, light_bound = (38, 90, 90), dark_bound = (60, 255, 255)):
    hsv_image = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv_image, light_bound, dark_bound)
    
    return cv2.bitwise_and(img, img, mask=mask)

# Contains code snippets/inspiration from:
# - Official OpenCV tutorials
# - https://www.pyimagesearch.com/2016/02/08/opencv-shape-detection/
# - https://realpython.com/python-opencv-color-spaces/
def fit_boxes(img_arr, area_thresh=2000):
    img = cv2.cvtColor(img_arr, cv2.COLOR_RGB2BGR)
    cs_segmented = color_space_segmentation(img)
    
    resized = imutils.resize(cs_segmented, width=300)
    ratio = img.shape[0] / float(resized.shape[0])

    gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (21, 21), 0)
    thresh = cv2.threshold(blurred, 40, 255, cv2.THRESH_BINARY)[1]
    kernel = np.ones((5,5), np.uint8)
    dilate = cv2.dilate(thresh, kernel, iterations=1)

    cnts = cv2.findContours(dilate.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    boxes = []
    areas = []
    for cnt in cnts:
        min_rect = cv2.minAreaRect(cnt)
        box = (cv2.boxPoints(min_rect) * ratio).astype(int)
        boxes.append(box)
        
        width, height = min_rect[1]

        areas.append(width*height)


    if len(areas) == 0 or np.max(areas) < area_thresh:
        return None, False
    
    stick_box = boxes[np.argmax(areas)]

    mask = np.zeros(img.shape[0:2])
    cv2.drawContours(mask, [stick_box], -1, (1,0, 0), -1)
    mask = mask.astype(np.uint8)

    thresh = thresh.astype(np.uint8)
    resized_thresh = imutils.resize(thresh, width = img.shape[1])

    def save_images_for_report():
        import os

        img_dir = "../exports/"
        t = datetime.now().strftime('%Y-%m-%d-%H:%M:%S')
        img_dir = img_dir + t + "/"
        os.mkdir(img_dir)

        imgs = {
            "original_img": img,
            "resized_cs_segmented": resized,
            "gray": gray,
            "blurred": blurred,
            "thresholded": thresh,
            "dilated": dilate,
            "final_mask": mask,
            "segmented": cv2.bitwise_and(resized_thresh, mask),
            "eroded": cv2.erode(cv2.bitwise_and(resized_thresh, mask), np.ones((10,10), np.uint8), iterations=4),
        }

        for file, out_image in imgs.items():
            cv2.imwrite("{}{}-{}.jpg".format(img_dir, file, t), out_image)

        assert False
    

    # save_images_for_report()

    return cv2.bitwise_and(resized_thresh, mask), True 

def segment_image(img): 
    binary, fitted = fit_boxes(img)
    if not fitted:
        return None, False

    kernel = np.ones((9,9), np.uint8) 
    binary = cv2.erode(binary, kernel, iterations=4)

    # if np.sum(binary) < 400:
    #     return None, False

    return binary, True