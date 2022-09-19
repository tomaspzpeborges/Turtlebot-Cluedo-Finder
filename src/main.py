#!/usr/local/bin/python3.9
import rospy
from InsideRoomFunctionality.PathPlanning.room import Room
from InsideRoomFunctionality.RobotMovement.main_driver import MainDriver
from character_detection.detection_structures.detector import Detector
from character_detection.detection_structures.charater_detection_params import CharacterDetectionParams
from path_planning.path_planner import room_metrics
from os.path import expanduser


# Character detection
"""
The input needed for the procedure is
    weights_path = "./character_detection/CNN/best.pt";
    model_path = "./character_detection/CNN/yolov5";
"""

def create_detector():
    weights_path = "{}/catkin_ws/src/Group4/src/character_detection/CNN/best.pt".format(expanduser('~'))
    model_path = "{}/catkin_ws/src/Group4/src/character_detection/CNN/yolov5".format(expanduser('~'))
    return Detector(model_path, weights_path);

# Room selection
"""
The input needed for the procedure is
    rotation_duration = 36;
    use_colour_detect = True;
    input_path = "/home/csunix/sc19am2/catkin_ws/src/Group4/world/input_points.yaml"
"""

def find_room(rotation_secounds=36, use_colour=True):
    # Parameters
    rotation_duration = rotation_secounds;
    use_colour_detect = use_colour;
    input_path = "{}/catkin_ws/src/Group4/world/input_points.yaml".format(expanduser('~'));
    #input_path = "/home/csunix/sc19am2/catkin_ws/src/Group4/world/input_points.yaml"
    return room_metrics(input_path, rotation_duration, use_colour_detect);

# Room path planning
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
def move_inside_room(image_detector, entrance , center_of_correct_room, angle, distance_from_entrance):
    # Input examples
    #entrance = [-1.43, 1.15];
    #center_of_correct_room = [-2.3, 5.63];
    #angle = 0;
    #distance_from_entrance = 1;

    # Parameters examples
    update_rate = 10;
    max_rotation_velocity = 30;
    robot_clearence = 0.5;
    distance_from_wall = 1;

    confidence_rating = 90;
    save_path = "{}/catkin_ws/src/Group4/deliverables/".format(expanduser('~'))
    real_time = True

    # Create main driver
    md = MainDriver(update_rate, max_rotation_velocity);

    # Add image detector
    md.add_detector(image_detector);

    #View camera feed
    #md.sensory.display_camera_feed(val=True);
    #md.sensory.display_detection_image(True,image_detector);

    #  and add room map
    room_test = Room(center_of_correct_room, entrance, angle, distance_from_entrance, robot_clearence,
                     distance_from_wall, True);
    md.add_room(room_test);

    # Character detection parameters
    character_params = CharacterDetectionParams(save_path, confidence_rating);

    # Move the robot around the room
    md.move_around_room(character_params, real_time);

# Run the script
def run():
    # Node innit
    rospy.init_node('Testing', anonymous=True);

    # Image detector innit
    image_detector = create_detector();

    # Room selection
    center_of_correct_room, entrance, angle, distance_from_entrance = find_room(36);

    # Path planning
    move_inside_room(image_detector, entrance, center_of_correct_room, angle, distance_from_entrance);

if __name__ == '__main__':
    # Your code should go here. You can break your code into several files and
    # include them in this file. You just need to make sure that your solution
    # can run by just running rosrun group_project main.py
    run();
