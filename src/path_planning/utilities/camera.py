from __future__ import division
import cv2
import rospy
import numpy as np

from sensor_msgs.msg import Image


class Camera:

    def __init__(self, display_image=False):
        self.display_image = display_image;
        self.bgr8_image = None;
        self.sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.show_camera);

    def show_camera(self, data):
        self.bgr8_image = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1);
        if self.display_image:
            cv2.namedWindow('Camera Feed')
            cv2.imshow('Camera Feed', self.bgr8_image)
            cv2.waitKey(3)
