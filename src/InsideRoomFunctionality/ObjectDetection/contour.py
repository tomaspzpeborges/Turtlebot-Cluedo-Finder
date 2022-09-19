import cv2 as cv
import numpy as np


class Contour:

    # Constructors
    def __init__(self, contour):
        self.contour = contour;
        self.area = cv.contourArea(contour);
        self.mask = [];

    # Getters
    def get_mask(self, image):
        if not self.mask:
            self.mask = np.zeros(image.shape, np.uint8);
            cv.drawContours(self.mask, self.contour, -1, (255), 1);
        return self.mask;

    def get_area(self):
        return self.area;

    # Static functionality
    # Variables
    min_area = 50

    # Functions
    @staticmethod
    def find_contours(image):
        image_grey = cv.cvtColor(image, cv.COLOR_BGR2GRAY);
        ret, thresh = cv.threshold(image_grey, 150, 255, cv.THRESH_BINARY)
        contours, hierarchy = cv.findContours(image=thresh, mode=cv.RETR_TREE, method=cv.CHAIN_APPROX_NONE);
        return contours;

    @staticmethod
    def display_contours(image):
        image_grey = cv.cvtColor(image, cv.COLOR_BGR2GRAY);
        ret, thresh = cv.threshold(image_grey, 150, 255, cv.THRESH_BINARY)
        cv.imshow('Binary image', thresh);
