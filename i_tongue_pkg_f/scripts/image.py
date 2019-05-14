#!/usr/bin/env python
PKG = 'numpy_tutorial'
import roslib; roslib.load_manifest(PKG)

import rospy
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import time
from geometry_msgs.msg import Pose2D
import cv2
import numpy as np

from PIL import Image
import scipy.misc
from scipy.misc import imread,imsave
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
from scipy.misc import toimage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt

firsttime = True

class DepthImage:
    def __init__ (self):
        self.firsttime = True
        self.points1
        self.bridge = CvBridge

    def callback(self,data):

    #points.append(171,224)
        data.data


        points_msg = Image()
        bridge=CvBridge()
        IMAGE_HEIGHT = 171
        IMAGE_WIDTH = 224
        points_msg.header.stamp = rospy.Time.now()
        points_msg.header.frame_id = "camera_depth_optical_frame"
        points_msg.encoding = "32FC1"
        points_msg.height = IMAGE_HEIGHT
        points_msg.width = IMAGE_WIDTH
        points_msg.data = bridge.cv2_to_imgmsg(points_img, '32FC1').data
        points_msg.is_bigendian = 0
        points_msg.step = points_msg.width * 4

        cvImage = bridge.cv2_to_imgmsg(points_img, encoding='passthrough')
        cvImPub=rospy.Publisher('/cvImage_array',Image,queue_size=1)
        cvImPub.publish(points_msg)

    def listener(self):
        rospy.init_node('listener')
        rospy.Subscriber('/cvImage_array',Image, self.callback)
        rospy.spin()

if __name__ == '__main__':

    dI = DepthImage()
    dI.listener()


    #listener()
