
import rospy
from PathPlanning.room import Room
from RobotMovement.main_driver import MainDriver
from ObjectDetection.object_detector import ObjectDetector


"""
Before anything, make sure you stopped all the connections to all the subscribers.
The input needed for the procedure is 
    entrance = [-1.43, 1.15];
    angle = 0 #In degrees
    center_of_correct_room = [-2.3, 5.63];
    distance_from_entrance = 1; #In meters
Parameters that cam be tweaked:
    update_rate = 10 #Rate of updates when it comes to any movement
    max_rotation_velocity = 30 #Maximum velocity (rad/sec) that enable the robot to rotate without any issues
    
    robot_clearence = 10 #Number of meters that the robot must be clear of
    distance_from_wall = 20 #Number of meters that the robot must be from the wall to take "photos"
"""

def integration():
    # Needs to have a node init(can be done sooner)
    rospy.init_node('Testing', anonymous=True);

    #Input examples
    entrance = [-1.43, 1.15];
    center_of_correct_room = [-2.3, 5.63];
    angle = 0;
    distance_from_entrance = 1;

    #Parameters examples
    update_rate = 10;
    max_rotation_velocity = 30;
    robot_clearence = 0.5;
    distance_from_wall = 1;

    md = MainDriver(update_rate, max_rotation_velocity);
    room_test = Room(center_of_correct_room, entrance, angle, distance_from_entrance, robot_clearence, distance_from_wall, True);
    md.add_room(room_test);
    objd = ObjectDetector(md.sensory);

    # room_test.map.show_map();
    #room_test.room_map.show_map();
    # md.move_around_room();
    #objd.mustard.display_image();
    #objd.mustard.display_colour(10, "D");