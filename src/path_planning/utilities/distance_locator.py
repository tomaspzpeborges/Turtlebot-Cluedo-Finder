#!/usr/bin/env python


import rospy
from sensor_msgs.msg import LaserScan


class DistanceLocator(object):

    def __init__(self):
        self.range_min = 0
        self.range_max = 0
        self.average_distance = 0
        self.subscriber = rospy.Subscriber('/scan', LaserScan,
                                           self.distance_calculator)

    def distance_calculator(self, messages):
        sum = 0
        count = 0
        self.range_min = messages.range_min
        self.range_max = messages.range_max
        for i in range(314, 325):
            if messages.ranges[i] >= self.range_min\
                    and messages.ranges[i] <= self.range_max:
                sum += messages.ranges[i]
                count += 1
        if count == 0:
            self.average_distance = self.range_max
        else:
            self.average_distance = sum/count
