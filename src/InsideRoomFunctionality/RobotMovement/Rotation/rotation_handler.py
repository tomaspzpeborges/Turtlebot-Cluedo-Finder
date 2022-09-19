#!/usr/bin/env python
import math
import numpy as np


def euler_from_quaternion(x, y, z, w):
    """
   Convert a quaternion into euler angles (roll, pitch, yaw)
   roll is rotation around x in radians (counterclockwise)
   pitch is rotation around y in radians (counterclockwise)
   yaw is rotation around z in radians (counterclockwise)
   """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return [roll_x, pitch_y, yaw_z]  # in radians

def quaternion_from_euler(roll, pitch, yaw):
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(
        yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(
        yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(
        yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(
        yaw / 2)

    return [qx, qy, qz, qw]

class RotationHandler:

    #Construtors
    def __init__(self, max_rotation_velocity):#Degrees/sec
        self.max_rotation_velocity = max_rotation_velocity;

    #Converters
    def degress_to_radians(self, deg):
        return math.radians(deg);

    def radians_to_degrees(self, rad):
        return math.degrees(rad);

    def euler_to_quaternions(self, x, y, z): #roll, pitch, yaw
        return quaternion_from_euler(x, y, z);

    def quaternions_to_euler(self, x, y, z, w):
        return euler_from_quaternion(x, y, z, w);

    def euler_to_quaternions_arr(self, arr): #roll, pitch, yaw
        return quaternion_from_euler(arr[0], arr[1], arr[2]);

    def quaternions_to_euler_arr(self, arr):
        return euler_from_quaternion(arr[0], arr[1], arr[2], arr[3]);

    def angle_to_360(self,angle):
        if angle>360 or angle<360:
            return angle%360;

    def angle_to_180(self,angle):
        if abs(angle)>360:
            angle = self.angle_to_360(angle);
        if angle>180:
            angle = 180-angle;
        return angle;

    def distance_between_angles(self,angle1,angle2):
        if (angle1>=0 and angle2>=0) or (angle1<=0 and angle2<=0):
            return abs(abs(angle1)-abs(angle2));
        elif angle1 >=0 and angle2<=0:
            right = angle1 + abs(angle2);
            left = 180-angle1 +180-abs(angle2);
        elif angle1 <=0 and angle2 >=0:
            right = angle2 + abs(angle1);
            left = 180-angle2 +180-abs(angle1);
        return min(left,right);
        
    #Core functions
    def decide_rotation(self,desired_angle,prev_angle):
        da = self.angle_to_180(desired_angle);
        pa = self.angle_to_180(prev_angle);

        if pa>=0 and da>=0 or pa<=0 and da<=0:
            return da-pa;
        elif pa >=0 and da<=0:
            right = pa + abs(da);
            left = 180-pa +180-abs(da);
        elif pa <=0 and da >=0:
            right = da + abs(pa);
            left = 180-da +180-abs(pa);

        if right>left:
            return left;
        else:
            return -right;

    def velocity_duration(self,angle,aprox=False,type="degree"):
        if type =="radians":
            angle = self.radians_to_degrees(angle);

        if angle<0:
            direction =-1;
            angle = abs(angle);
        else:
            direction =1;

        if angle<self.max_rotation_velocity:
            #return round(direction*angle/2),2;
            return round(direction*angle),1.2;

        if aprox:
            i = self.max_rotation_velocity;
            while(i>0):
                if angle%i==0:
                    return direction*i,angle/i;
                i-=1;
        else:
            return direction*self.max_rotation_velocity,angle/self.max_rotation_velocity;

        