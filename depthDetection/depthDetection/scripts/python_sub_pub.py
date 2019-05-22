"""This script receives a depth image from the pico flexx camera from a raspberry
pi 3b+ in an numpy format and publishes it to a ros topic in an opencv format
"""

import argparse
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import rospy

def callback(data):

    #retrieve the array and reshape it according to the resolution of the
    #pico flexx camera
    points = np.asarray(data.data)
    points_img = points.reshape (-1, 224)

    #Construct an image ros message and publish it to a ros topic
    points_msg = Image()
    bridge=CvBridge()
    points_msg = bridge.cv2_to_imgmsg(points_img, encoding="passthrough")
    cvImPub=rospy.Publisher('/royale_camera_driver/depth_image',Image,queue_size=1)
    cvImPub.publish(points_msg)

def listener():
    rospy.init_node('listener')
    r = rospy.Rate(10)

    #subscribe to the numpy image from the raspberry pi 3b+
    rospy.Subscriber('/numpy_image', Floats, callback)
    while not rospy.is_shutdown():
        rospy.spin()


if __name__ == '__main__':
    listener()
