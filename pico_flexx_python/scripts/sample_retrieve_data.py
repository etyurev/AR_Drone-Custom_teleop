#!/usr/bin/python3

# Copyright (C) 2017 Infineon Technologies & pmdtechnologies ag
#
# THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
# KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
# PARTICULAR PURPOSE.

"""This is a modified version of the python sample provided with the royale sdk.
"""


#!/usr/bin/python3

# Copyright (C) 2017 Infineon Technologies & pmdtechnologies ag
#
# THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
# KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
# PARTICULAR PURPOSE.

"""This sample shows how to shows how to capture image data.
"""
import cv2
import argparse
import roypy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import time
import queue
from sample_camera_info import print_camera_info
from roypy_sample_utils import CameraOpener, add_camera_opener_options
from roypy_platform_utils import PlatformHelper
import numpy as np
import rospy


class MyListener(roypy.IDepthDataListener):
    def __init__(self, q):
        super(MyListener, self).__init__()
        self.queue = q

    def onNewData(self, data):
        zvalues = []
        for i in range(data.getNumPoints()):
            zvalues.append(data.getZ(i))
        zarray = np.asarray(zvalues)
        pubNumpy = rospy.Publisher('numpy_image', Floats, queue_size=1)
        pubNumpy.publish(zarray)

def main ():



    try:
        rospy.init_node('pico_flexx')
        r = rospy.Rate(10)
        parser = argparse.ArgumentParser (usage = __doc__)
        add_camera_opener_options (parser)
        parser.add_argument ("--seconds", type=int, default=15, help="duration to capture data")
        options = parser.parse_args()
        opener = CameraOpener (options)
        cam = opener.open_camera ()


        print_camera_info (cam)
        print("isConnected", cam.isConnected())
        print("getFrameRate", cam.getFrameRate())

        # we will use this queue to synchronize the callback with the main
        # thread, as drawing should happen in the main thread
        q = queue.Queue(maxsize=0)
        l = MyListener(q)
        cam.registerDataListener(l)
        cam.startCapture()

        while not rospy.is_shutdown():
            r.sleep()

        cam.stopCapture()
    except rospy.ROSInterruptException:
        pass


if (__name__ == "__main__"):
    main()
