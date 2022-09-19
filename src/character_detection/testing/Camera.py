from __future__ import division
import cv2
import numpy as np
import rospy
import sys
import torch

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


def predict_img(model, img):
    im2 = cv2.imread('test_room.jpg')[..., ::-1]  # OpenCV image (BGR to RGB)
    # Inference
    results = model(img, size=640)  # includes NMS

    # Results
    # results.show()

    # print(results.xyxy[0])  # im1 predictions (tensor)
    # results.pandas().xyxy[0]  # im1 predictions (pandas)

    tensor = results.xyxy[0]
    array = tensor.numpy()
    data = array[0]

    x1, y1, x2, y2 = data[:4]

    # print("Bouning box Dimensions x1:%d y1:%d , x2:%d , y2%d" %(x1,y1,x2,y2))

    # print("Confidance Rating: ", data[4])
    conf_rating = data[4]
    bounding_box = data[:4]

    # print("Class Index: " , data[5]) #0 Mustard , 1 Scarlet , 2 Peackock , 3 Plum
    class_index = data[5]
    if class_index == 0:
        label = "Colonel Mustard"
    elif class_index == 2:
        label = "Scralet"
    elif class_index == 1:
        label = "Peackock"
    elif class_index == 3:
        label = "Plum"
    else:
        label = "Unidentifeid"

    # point1 = (x1,y1)
    # point2 = (x2,y2)
    # print(point1,point2)

    start_point = (int(x1), int(y1))
    end_point = (int(x2), int(y2))
    color = (255, 0, 0)
    thickness = 2

    label_position = (int(x2), int(y2) - 60)
    confidance_position = (int(x2) + 150, int(y2) - 60)

    # Using cv2.rectangle() method
    # Draw a rectangle with blue line borders of thickness of 2 px
    image = cv2.rectangle(img, start_point, end_point, color, thickness)
    image = cv2.putText(image, label, label_position, cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 4)
    image = cv2.putText(image, str(data[4]), confidance_position, cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 4)

    return image, conf_rating, bounding_box


class cameraFeed():

    def __init__(self):

        # Remember to initialise a CvBridge() and set up a subscriber to the image topic you wish to use
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber("camera/rgb/image_raw", Image, self.callback)
        self.model = torch.hub.load('yolov5', 'custom', path='character_weights.pt', source='local')

        # We covered which topic to subscribe to should you wish to receive image data

    def callback(self, data):
        # Convert the received image into a opencv image
        # But remember that you should always wrap a call to this conversion method in an exception handler
        try:
            processed_img = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)
            img = predict_img(self.model, processed_img)
        except CvBridgeError as e:
            print(e)

        # Show the resultant images you have created.

        cv2.namedWindow('camera_Feed')
        cv2.imshow('camera_Feed', processed_img)
        cv2.waitKey(3)


# Create a node of your class in the main and ensure it stays up and running
# handling exceptions and such
def main(args):
    # Instantiate your class
    # And rospy.init the entire node
    cf = cameraFeed()
    # Ensure that the node continues running with rospy.spin()
    # You may need to wrap rospy.spin() in an exception handler in case of KeyboardInterrupts
    rospy.init_node('image_sub', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupts:
        print("Program Shuting Down")

    # Remember to destroy all image windows before closing node
    cv2.destroyAllWindows()


# Check if the node is executing in the main path
if __name__ == '__main__':
    main(sys.argv)
