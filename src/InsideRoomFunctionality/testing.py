from PathPlanning.room import Room
import rospy
from RobotMovement.main_driver import MainDriver
from ObjectDetection.object_detector import ObjectDetector

def testing():
    rospy.init_node('Testing', anonymous=True);
    md = MainDriver(10, 30);
    a = [-1.43, 1.15];
    #b = [3.4, 1.15];
    c = [-2.3, 5.63];
    angle = 90;
    distance_from_entrance = 1;
    room_test = Room(c, a, angle, distance_from_entrance, 0.5, 1, True);
    #room_test = Room(c,a,b,angle,distance_from_entrance, 0.5, 1, True);
    room_test.map.show_map();
    room_test.room_map.show_map();
    md.sensory.display_camera_feed(True);
    #md.add_room(room_test);
    #md.move_around_room();

    #objd = ObjectDetector(md.sensory);
    #objd.mustard.display_image();
    #objd.mustard.display_colour(10,"D");
    #while not rospy.is_shutdown():
    #   pass;


testing();