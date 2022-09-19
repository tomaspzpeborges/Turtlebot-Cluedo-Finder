import math
from .command_center import CommandCenter
from .sensory_center import SensoryCenter

class MainDriver:
    # Constructors
    def __init__(self, update_rate, max_rotation_velocity, ):
        self.command = CommandCenter(update_rate, max_rotation_velocity);
        self.sensory = SensoryCenter(self);
        self.detector = None;
        self.room = None;
        self.finished = False;

    # Setters
    def add_room(self, room):
        self.room = room;

    def add_detector(self, detector):
        self.detector = detector;

    def set_finished(self, val):
        self.finished = val;

    # Getters
    def get_finished(self):
        return self.finished;

    # Functionality
    def real_time(self, locations, character_params):
        self.finished = False;
        self.sensory.set_real_time(True, self.detector, character_params);
        for i in locations:
            point = i.get_point().get_real();
            queue = i.get_angle_queue();
            # print(len(queue))
            # print(i.get_point().get_local())
            if len(queue) > 0:
                if self.command.move_goal(point[0], point[1], queue[0]):
                    #print("Success!");
                    self.command.rotate_fixed(self.sensory.resturn_robot_orientation(), queue[0], queue[0]);
                    if self.finished:
                        break;
                    # self.command.print_robot_orientation(self.sensory.resturn_robot_orientation());
                    for j in range(1, len(queue)):
                        #print("Success!");
                        self.command.rotate_fixed(self.sensory.resturn_robot_orientation(), queue[j], queue[j - 1]);
                        if self.finished:
                            break;
            else:
                print("Error, no direction!");
                if self.command.move_goal(point[0], point[1], 0):
                    print("Success!");
                    print(i.get_real());

    def fixed_time(self, locations, character_params):
        finished = False;
        for i in locations:
            point = i.get_point().get_real();
            queue = i.get_angle_queue();
            #print(len(queue))
            #print(i.get_point().get_local())
            if len(queue)>0:
                if self.command.move_goal(point[0], point[1], queue[0]):
                    #print("Success!");
                    self.command.rotate_fixed(self.sensory.resturn_robot_orientation(), queue[0], queue[0]);
                    #self.command.print_robot_orientation(self.sensory.resturn_robot_orientation());
                    if self.detector.predict_img(self.sensory.get_camera_image(), character_params):
                        finished = True;
                    for j in range(1, len(queue)):
                        #print("Success!");
                        self.command.rotate_fixed(self.sensory.resturn_robot_orientation(), queue[j], queue[j-1]);
                        if self.detector.predict_img(self.sensory.get_camera_image(), character_params):
                            finished = True;
                        if finished:
                            break;
                    if finished:
                        break;
            else:
                print("Error, no direction!");
                if self.command.move_goal(point[0], point[1], 0):
                    print("Success!");
                    print(i.get_real());

    def move_around_room(self, character_params, real_time_detection=False):
        locations = self.room.get_locations();
        if real_time_detection:
            self.real_time(locations, character_params);
        else:
            self.fixed_time(locations, character_params);
