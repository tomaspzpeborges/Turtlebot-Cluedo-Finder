from __future__ import division
import cv2
import rospy
from sensor_msgs.msg import Image
import numpy as np


class ColourDetector(object):

    def __init__(self, hsv_lower, hsv_upper, callback=None, *callback_args):
        self.sensitivity = 10
        self.colour_detected = False
        self.min_contour_area = 100
        hsv_lower[0] = hsv_lower[0] - self.sensitivity
        hsv_upper[0] = hsv_upper[0] + self.sensitivity
        if callback:
            self.subscriber = rospy.Subscriber("/camera/rgb/image_raw", Image,
                                               self.colour_detector,
                                               (hsv_lower, hsv_upper, callback,
                                                callback_args))
        else:
            self.subscriber = rospy.Subscriber("/camera/rgb/image_raw", Image,
                                               self.colour_detector,
                                               (hsv_lower, hsv_upper))

    def __del__(self):
        cv2.destroyAllWindows()

    def is_colour_detected(self):
        return self.colour_detected is True

    def unregister_subscriber(self):
        self.subscriber.unregister()

    def find_countours(self, mask, target):
        countours, hierarchy = cv2.findContours(
            mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE
        )
        c = None
        if len(countours) > 0:
            c = max(countours, key=cv2.contourArea)
            if cv2.contourArea(c) > self.min_contour_area:
                return True, c
            else:
                return False, c
        else:
            return False, c

    def colour_detector(self, data, args):
        hsv_lower = args[0]
        hsv_upper = args[1]
        callback_exists = False
        try:
            callback = args[2]
            callback_args = args[3]
            callback_exists = True
        except Exception as e:
            #rospy.loginfo(e)
            pass
        try:
            bgr8_image = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)
            hsv_image = cv2.cvtColor(bgr8_image, cv2.COLOR_BGR2HSV)
        except Exception as e:
            #rospy.loginfo(e)
            pass

        mask = cv2.inRange(hsv_image, hsv_lower, hsv_upper)
        target = cv2.bitwise_and(bgr8_image, bgr8_image, mask=mask)

        self.colour_detected, contour = self.find_countours(mask, target)
        if self.colour_detected:
            #rospy.loginfo('Colour detected.')
            if callback_exists:
                self.unregister_subscriber()
                callback(*callback_args)
        else:
            #rospy.loginfo('Colour not detected')
            pass
