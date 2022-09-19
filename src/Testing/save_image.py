from os import listdir
from os.path import isfile, join
import os
import cv2 as cv
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

path = os.getcwd() + "/../world/training_data/scarlet/";
name = "scarlet";
ext = ".png"

def count_files(path):
    print(os.getcwd())
    onlyfiles = [f for f in listdir(path) if isfile(join(path, f))];
    return len(onlyfiles);

def save_image(path,name,number,ext,image):
    cv.imwrite(path+name+str(number)+ext,image);
        
def main():
    rospy.init_node('CameraMan', anonymous=True)
    bridge = CvBridge();
    number = count_files(path);
    data = rospy.wait_for_message('camera/rgb/image_raw',Image);
    try:
            image = bridge.imgmsg_to_cv2(data, "bgr8");
    except CvBridgeError as ex:
            print(ex);
    save_image(path,name,number,ext,image);

main();
    
