#!/usr/bin/env python

import math
import rospy
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import MapMetaData,OccupancyGrid

#Changeable parameters
rate_update = 10; #Hertz
distance_side = 2.0; #Meters
time_distance = 5.0; #Secounds
time_rotation = 1.0; #Secounds
rotation_angle = 180.0; #Degrees
slowdown_time = 0.5; #Secounds

#Computed parameters
forward_speed = distance_side / time_distance; #m/s
time_duration_distance = rate_update*time_distance; #Hz 
angular_speed = (rotation_angle * math.pi)/180 / time_rotation; #rad/s
time_duration_rotation = rate_update*time_rotation; #Hz ;
pause_duration = rate_update*slowdown_time; #Hz 

#Return the robot to its default state
def default_state():
	desired_velocity = Twist();

	desired_velocity.linear.x = 0; 
	desired_velocity.angular.z = 0;

	return desired_velocity;

#Velocity computation
def compute_velocity(counter):

	desired_velocity = Twist();
	
	#Move in a line
	if counter < time_duration_distance:
		desired_velocity.linear.x = forward_speed; 
		desired_velocity.angular.z = 0;
	#Rotate
	elif counter > time_duration_distance + pause_duration-1 and counter < time_duration_distance + time_duration_rotation + pause_duration+1:
		desired_velocity.linear.x = 0; 
		desired_velocity.angular.z = angular_speed;
	#Wait for decelaration/acceleration
	else:
		desired_velocity.linear.x = 0; 
		desired_velocity.angular.z = 0;
	
	return desired_velocity;

#Colission with bumpers
colission = False;
def processBumper(data):
	global colission;
	if (data.state == BumperEvent.PRESSED):
		colission = True;


def printlaser(data):
    print(data.angle_min)
    print(data.angle_max)
    print(len(data.ranges));
    
def printcoords(data):
	pass;

def printmap(data):
	pass;

def publisher():
	pub = rospy.Publisher('mobile_base/commands/velocity', Twist);
	rospy.Subscriber('mobile_base/events/bumper', BumperEvent, processBumper);
	rospy.Subscriber('/scan', LaserScan, printlaser);
	rospy.Subscriber('/map_metadata',MapMetaData,printcoords);
	rospy.Subscriber('/map',MapMetaData,printcoords);
	rospy.init_node('Walker', anonymous=True)
	rate = rospy.Rate(rate_update) 
	counter = 0;

	while not rospy.is_shutdown():
	    #Check for bumper colission
		if colission==False:
			#Publish the velocity states
			pub.publish(compute_velocity(counter));
			#Check counter for reinitialization(after the square is complete)
			if counter == time_duration_distance+time_duration_rotation+pause_duration+pause_duration:
				counter = -1;
				counter+=1;
				rate.sleep();
		else:
			#Return to default state and break
			pub.publish(default_state());
			break;
		

if __name__ == "__main__":
	try:
		publisher()
	except rospy.ROSInterruptException:
		pass
