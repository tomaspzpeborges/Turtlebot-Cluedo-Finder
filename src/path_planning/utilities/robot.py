import numpy as np
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from .navigator import Navigator


class Robot(object):

    def __init__(self, rate=10):
        self.robot_pose = None
        self.robot_twist = None
        self.rate = rospy.Rate(rate)
        self.twist = Twist()
        self.navigator = Navigator()
        self.velocity_pub = rospy.Publisher(
            'mobile_base/commands/velocity',
            Twist, queue_size=10
        )
        self.odometry_sub = rospy.Subscriber('/odom', Odometry,
                                             self.location_callback)

    def move_to_location(self, x, y, theta):
        position = {'x': x, 'y': y}
        quaternion = {
            'r1': 0.000,
            'r2': 0.000,
            'r3': np.sin(theta/2.0),
            'r4': np.cos(theta/2.0)
        }
        rospy.loginfo('Go to (%s, %s) pose', x, y)
        success = self.navigator.goto(position, quaternion)
        """
        if success:
            rospy.loginfo('Hooray, reached the desired pose')
        else:
            rospy.loginfo('The base failed to reach the desired pose')
        """    
        rospy.sleep(1)
        return success

    def rotate_robot(self, theta, time, callback=None, *callback_args):
        self.twist.angular.z = theta
        for i in range(time):
            if callback:
                if callback(*callback_args) is True:
                    break
            self.velocity_pub.publish(self.twist)
            self.rate.sleep()
        self.twist.angular.z = 0

    def stop_robot(self):
        self.twist.linear.x = 0
        self.twist.angular.z = 0
        self.velocity_pub.publish(self.twist)
        #rospy.loginfo('stop robot called.')

    def cancel_goal(self):
        self.navigator.cancel_nav_goal()

    def location_callback(self, data):
        self.robot_pose = data.pose.pose
        self.robot_twist = data.twist

    def get_robot_orientation(self):
        return self.robot_pose.orientation
