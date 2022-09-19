import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
import numpy as np

from .SensoryAux.camera_image import CameraImage
from .SensoryAux.laser import Laser


class SensoryCenter:

    def __init__(self, main_diver):
        #Default params
        self.per_goal_time = 60;
        # Real time and detection
        self.real_time_detection = False;
        self.detector = None;
        self.character_params = None;
        self.display_feed_detection = False;
        # Normal feed
        self.image = CameraImage("Main camera feed");
        scan_data = rospy.wait_for_message('/scan',LaserScan);
        self.laser = Laser(scan_data);
        self.display_feed = False;
        #
        self.main_driver = main_diver;

        #Initialise the movement subscribers
        self.odometry_sub = rospy.Subscriber ('/odom', Odometry, self.location_data);
        self.camera_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.camera_data);
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_data);

        self.robot_pose = Pose();

    # Setters
    def set_real_time(self, value, detector=None, character_params=None):
        if value:
            if detector is not None and character_params is not None:
                self.real_time_detection = value;
                self.detector = detector;
                self.character_params = character_params;
        else:
            self.real_time_detvalueection = value;

    def camera_detection(self, image):
        if self.detector.predict_img(image, self.character_params):
            self.main_driver.set_finished(True);

    def display_detection_image(self, val, detector=None):
        if val:
            if detector is not None:
                self.display_feed_detection = val;
                self.detector = detector;
        else:
            self.display_feed_detection = val;

    #Input from subscribers
    def location_data(self, data):
        self.robot_pose = data.pose.pose;
        self.robot_twist = data.twist;

    def camera_data(self, data):
        image = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1);
        self.image.add_image(image);
        # Display feed
        if self.display_feed:
            self.image.display();
        # Display detection feed
        if self.display_feed_detection:
            CameraImage.displayImage(self.detector.get_image(image), "Detection image");
        # Detect character
        if self.real_time_detection and not self.main_driver.get_finished():
            self.camera_detection(image);

    def laser_data(self, data):
        self.laser.add_value(data.ranges);

    def display_camera_feed(self, val):
        self.display_feed = val;

    def resturn_robot_orientation(self):
        return self.robot_pose.orientation;

    def get_camera_image(self):
        return self.image.get_image();


