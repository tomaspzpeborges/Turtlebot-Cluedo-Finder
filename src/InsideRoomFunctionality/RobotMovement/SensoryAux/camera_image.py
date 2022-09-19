import math
from geometry_msgs.msg import Twist
import rospy
import numpy as np
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion


from nav_msgs.msg import Odometry

import cv2

class CameraImage:

    def __init__(self,name="",image=None):
        self.image=image;
        self.name=name;
        self.masks = {};
        self.mask_array = [];
        self.all_masks_combined = None;

    def get_hsv(self):
        return cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV);

    def display(self):
        cv2.namedWindow(self.name);
        cv2.imshow(self.name, self.image);
        cv2.waitKey(3);

    def add_mask(self,mask,name):
        self.masks[name]=mask;
        self.mask_array.append(mask);

    def add_image(self,image):
        self.image=image;

    def get_image(self):
        return self.image;

    #Masks
    def combine_all_masks(self):
        aux = self.mask_array[0];
        for i in range(1,self.mask_array):
            aux = cv2.bitwise_or(aux,self.mask_array[i]);
        self.all_masks_combined = aux;

    def apply_all_masks(self):
        if self.all_masks_combined==None:
            self.combine_all_masks();
        self.fully_filtered_image = cv2.bitwise_and(self.image,self.image,mask=self.all_masks_combined);


    # Static functionality
    @staticmethod
    def displayImage(image, name):
        cv2.namedWindow(name);
        cv2.imshow(name, image);
        cv2.waitKey(3);
