import math
from geometry_msgs.msg import Twist
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
import dynamic_reconfigure.client

from .Rotation.rotation_handler import RotationHandler

class CommandCenter:

    def __init__(self,rate_update,max_angular_velocity):
        #Default params
        self.full_rotation = 360;
        self.per_goal_time = 60;
        self.rate_of_update = rate_update;
        self.null_velocity = Twist();
        self.null_velocity.linear.x = 0; 
        self.null_velocity.angular.z = 0; 
        self.rotation_handler = RotationHandler(max_angular_velocity);
        self.rotation_real_time = 1;
        self.acceptance_error_threshold = 5;

        #Initialise the movement mechanisms
        self.client = dynamic_reconfigure.client.Client("/move_base/DWAPlannerROS");
        params = { 'yaw_goal_tolerance' : 0.05, 'acc_lim_theta': 1.0, 'max_rot_vel': 1.0, 'min_rot_vel' : 0.1, 'rot_stopped_vel': 0.1 }
        self.config = self.client.update_configuration(params);
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction);
        self.movement_pub = rospy.Publisher('mobile_base/commands/velocity', Twist);
        self.rate = rospy.Rate(rate_update);
        self.move_base.wait_for_server();

    def set_per_goal_time(self,time):
        self.per_goal_time = time;

    def change_rate(self,rate):
        self.rate_of_update = rate;
        self.rate = rospy.Rate(self.rate_of_update);

    #Movement using move_base
    def move_goal_arr(self,arr):
        return self.move_goal(arr[0],arr[1],arr[2]);

    def move_goal(self,x,y,rotation):
        #Degrres to rad
        theta = math.radians(rotation);
        #Rotation to quat
        quaternion = self.rotation_handler.euler_to_quaternions(0,0,theta);
        #print(self.rotation_handler.radians_to_degrees(self.rotation_handler.quaternions_to_euler_arr(quaternion)[2]))
        # Send a goal
        self.goal_sent = True;
        goal = MoveBaseGoal();
        goal.target_pose.header.frame_id = 'map';
        goal.target_pose.header.stamp = rospy.Time.now();
        goal.target_pose.pose = Pose(Point(x, y, 0.000),Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]))
        # Start moving
        self.move_base.send_goal(goal)
        # Allow TurtleBot up to x seconds to complete task
        success = self.move_base.wait_for_result(rospy.Duration(self.per_goal_time)) 

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            # We made it!
            result = True
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False
        return result



    def print_robot_orientation(self,value):
        print("After manual rotation:" + str(self.rotation_handler.radians_to_degrees(self.rotation_handler.quaternions_to_euler(value.x,value.y,value.z,value.w)[2])))

    #Manual movement
    def pause(self,duration):
        counter =0;
        while counter<duration*self.rate_of_update:
            self.movement_pub.publish(self.null_velocity);
            counter+=1;
            self.rate.sleep();

    def move_manual(self):
        #Changeable parameters
        rate_update = 10; #Hertz
        distance_side = 2.0; #Meters
        time_distance = 5.0; #Secounds
        time_rotation = 1.0; #Secounds
        rotation_angle = 90.0; #Degrees
        slowdown_time = 0.5; #Secounds

        #Computed parameters
        forward_speed = distance_side / time_distance; #m/s
        time_duration_distance = rate_update*time_distance; #Hz 
        angular_speed = (rotation_angle * math.pi)/180 / time_rotation; #rad/s
        time_duration_rotation = rate_update*time_rotation; #Hz ;
        pause_duration = rate_update*slowdown_time; #Hz 

        self.movement_pub.publish(compute_velocity(counter));

    def rotate_distance(self,sensory,distance,angular_velocity=0, time=0, side="r"):
        #Based on the input decide speed and duration
        if time==0 and angular_velocity==0:
            speed = self.rotation_handler.max_rotation_velocity;
            duration =  self.full_rotation/speed;
        elif angular_velocity>0:
            speed = angular_velocity; 
            duration =  self.full_rotation/speed;
        elif time>0:
            duration = time;
            speed = self.full_rotation/duration;
        #Convert the rotation in radians
        speed_rad = self.rotation_handler.degress_to_radians(speed);
        #Setup the 
        counter_ideal = 0;
        difference = 20;
        #Setup the rotation loop
        counter = 0;
        velocity = Twist();
        velocity.linear.x = 0; 
        velocity.angular.z = speed_rad; 
        duration_hertz = round(duration*self.rate_of_update)+1;
        while counter < duration_hertz and self.rotation_real_time!=-1:
            #Pause the execution and save state
            if self.rotation_real_time==0:
                self.pause_procedure(duration_hertz-counter,velocity);
                break;
            if(abs(distance-sensory.laser.get_value_degree(0))<difference):
                counter_ideal = counter;
                difference=distance-sensory.laser.get_value_degree(0);
            self.movement_pub.publish(velocity);
            counter+=1;
            self.rate.sleep();
        if counter!=0:
            while counter < counter_ideal:
                #Pause the execution and save state
                self.movement_pub.publish(velocity);
                counter+=1;
                self.rate.sleep();
        return True;
        

    def rotate_fixed(self,orientation,angle,prev_angle=0):
        #Get real angle measure by the robot
        real_angle = self.rotation_handler.radians_to_degrees(self.rotation_handler.quaternions_to_euler(orientation.x,orientation.y,orientation.z,orientation.w)[2]);
        #print("Real angle: "+ str(real_angle));
        #print("Desired angle:" + str(angle));
        #Compare and adjust the previous angle
        if self.rotation_handler.distance_between_angles(real_angle,prev_angle)>self.acceptance_error_threshold:
            pa = real_angle;
        else:
            pa = prev_angle;
        if self.rotation_handler.distance_between_angles(pa,angle)>self.acceptance_error_threshold:
            #Compute rotation,speed,duration
            rotation = self.rotation_handler.decide_rotation(angle,pa);
            #print("Rotation:" + str(rotation))
            speed,duration = self.rotation_handler.velocity_duration(rotation);
            #Convert the rotation in radians
            speed_rad = self.rotation_handler.degress_to_radians(speed);
            #print(speed);
            #print(duration);
            #Setup the rotation loop
            counter = 0;
            velocity = Twist();
            velocity.linear.x = 0; 
            velocity.angular.z = speed_rad; 
            duration_hertz = round(duration*self.rate_of_update)+1;
            #print("Duration:" + str(duration_hertz))
            while counter < duration_hertz+1 and self.rotation_real_time!=-1:
                #Pause the execution and save state
                if self.rotation_real_time==0:
                    self.pause_procedure(duration_hertz-counter+1,velocity);
                    break;
                self.movement_pub.publish(velocity);
                counter+=1;
                self.rate.sleep();
        return True;

    def fully_rotate_on_spot(self, angular_velocity=0, time=0, side="r"):
        #Based on the input decide speed and duration
        if time==0 and angular_velocity==0:
            speed = self.rotation_handler.max_rotation_velocity;
            duration =  self.full_rotation/speed;
        elif angular_velocity>0:
            speed = angular_velocity; 
            duration =  self.full_rotation/speed;
        elif time>0:
            duration = time;
            speed = self.full_rotation/duration;
        #Convert the rotation in radians
        speed_rad = self.rotation_handler.degrees_to_radians(speed);
        #Setup the rotation loop
        counter = 0;
        velocity = Twist();
        velocity.linear.x = 0; 
        velocity.angular.z = speed_rad; 
        duration_hertz = round(duration*self.rate_of_update) + 1;
        while counter < duration_hertz and self.rotation_real_time!=-1:
            #Pause the execution and save state
            if self.rotation_real_time==0:
                self.pause_procedure(duration_hertz-counter,velocity);
                break;
            self.movement_pub.publish(velocity);
            counter+=1;
            self.rate.sleep();
        return True;

    #Rotation in real time control
    def stop_rotation(self):
        self.rotation_real_time = -1;

    def pause_rotation(self):
       self.rotation_real_time = 0;

    def start_rotation(self):
        self.rotation_real_time = 1;
        self.resume_rotation();
        
    def resume_rotation(self):
        counter = 0;
        self.change_rate(self.pause_rate);
        while counter < self.pause_counter and self.rotation_real_time!=-1:
            if self.rotation_real_time==0:
                self.pause_counter = self.pause_counter-counter;
                break;
            self.movement_pub.publish(self.pause_velocity);
            counter+=1;
            self.rate.sleep();

    def pause_procedure(self,duration,velocity):
        self.movement_pub.publish(self.null_velocity);
        self.pause_counter = duration;
        self.pause_velocity = velocity;
        self.pause_rate = self.rate_of_update;
