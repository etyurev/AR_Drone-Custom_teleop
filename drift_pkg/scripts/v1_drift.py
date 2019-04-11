#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

# Lucas kanade params
lk_params = dict(winSize = (15, 15),
                maxLevel = 4,
                criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
# params for ShiTomasi corner detection
feature_params = dict( maxCorners = 100,
                       qualityLevel = 0.3,
                       minDistance = 7,
                       blockSize = 7 )

#def average_n_Filter(difx,dify):

def find_good_features(frame_gray):
    pts = cv2.goodFeaturesToTrack(frame_gray, mask = None, **feature_params)
    startx = []
    starty = []
    for i in range(0,len(pts)):
        startx.append(pts.ravel()[2*i])
        starty.append(pts.ravel()[2*i+1])
    #print(len(startx))
    return pts, startx, starty


def handle_newpoints(points, status,lastx, lasty):
    sumx = 0
    sumy = 0
    ite = 0
    difsum = []
    numfails = 0
    print(len(lastx))
    for i in range(0,len(points)):
        if status[i] != 0:
            x,y = points[i].ravel()
            y_dif = y-lasty[i]
            x_dif = x-lastx[i]
            #if y_dif < 2 and x_dif < 2:
            sumx += x_dif
            sumy += y_dif
            ite += 1
        else:
            numfails += 1
    if ite != 0:
        sumx /=ite
        sumy /=ite
        print('sumx',sumx)
        print('sumy',sumy)
        difsum.append(sumx)
        difsum.append(sumy)
        return difsum, numfails

def stabilize_drone(difvec,pubVel):
    vel_cmd = Twist()
    if abs(difvec[1]) > 5:
        temp = -0.002*difvec[1]
        if temp < 0.01 and temp > -0.01:
            vel_cmd.linear.x = 0
        elif temp > 0.1:
            vel_cmd.linear.x = 0.1
        elif temp < -0.1:
            vel_cmd.linear.x = -0.1
        else:
            vel_cmd.linear.x = temp
    if abs(difvec[0]) > 5:
        temp = -0.002*difvec[0]
        if temp < 0.01 and temp > -0.01:
            vel_cmd.linear.y = 0
        elif temp > 0.1:
            vel_cmd.linear.y = 0.1
        elif temp < -0.1:
            vel_cmd.linear.y = -0.1
        else:
            vel_cmd.linear.y = temp
    pubVel.publish(vel_cmd)

#### SUB ###
#Topic
#/ardrone/bottom/image_raw
#
#have_data=0
class image_getter:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/ardrone/bottom/image_raw", Image, self.callback_img_bot)
        self.have_data = 0
        self.img_bot = 0
    #def conv_image(self):
    #    self.img_bot = bridge.imgmsg_to_cv2(data, desired_encoding="passthroug")
    def callback_img_bot(self,data):
        #global img_bot
        #print('entering')
        self.have_data=1
        self.img_bot = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")

    #def get_image(self):
    #    return img_bot

class hovering_listener:
    def __init__(self):
        self.hover_service = rospy.Subscriber("/hover_mode",Bool,self.callback_hover)
        self.hover_mode_activate = False
        self.cmd_vel = Twist()
        self.velPub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.cmd_vel.linear.x = 0
        self.cmd_vel.linear.y = 0
    def callback_hover(self,data):
        print(data)
        if data.data:
            self.hover_mode_activate = True
        else:
            self.hover_mode_activate = False
            self.velPub.publish(self.cmd_vel)

    def get_hover_mode(self):
        return self.hover_mode_activate
#def callback_bot_img(data):
#    bridge = CvBridge()
#    img_bot = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
    #bridge.imgmsg_to_cv2(image_message, desired_encoding="passthrough")

if __name__ == '__main__':
    try:
        rospy.init_node('v1_drift')
        cap = cv2.VideoCapture(0)
        # Check if camera opened successfully
        pubVel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        if (cap.isOpened()== False):
            print("Error opening video stream or file")

        #ret, frame = cap.read()
        #frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ig = image_getter()
        hover = hovering_listener()
        #framen = ig.get_image()
        #frame_grayn=cv2.cvtColor(framen, cv2.COLOR_BGR2GRAY)

        x_last = []
        y_last = []
        while(ig.have_data == 0):
            print('waiting')
        print('Ready')
        frame = ig.img_bot
        frame_gray=cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        pts, x_last, y_last = find_good_features(frame_gray)
        old_gray = frame_gray.copy()
        while(True):

            # Capture frame-by-frame
            #Speficier lenght of kp
            #ret, frame = cap.read()
            if hover.get_hover_mode():
                frame=ig.img_bot
                frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                new_points, status, error = cv2.calcOpticalFlowPyrLK(old_gray, frame_gray, pts, None, **lk_params)
                old_gray = frame_gray.copy()
                pts = new_points
            #Display the resulting frame
            #ros = len(new_points)
            #print(len(x_last))
                difs, fails =handle_newpoints(new_points, status,x_last,y_last)
                stabilize_drone(difs,pubVel)

            #print(len(new_points))
                if fails > len(pts)-fails or abs(difs[0])>50 or abs(difs[1])>50:
                    pts, x_last, y_last = find_good_features(frame_gray)
                    old_gray = frame_gray.copy()
            #print all moving circles
            #frame_gray = cv2.circle(frame_gray,(x_last[1],y_last[1]),5, (0,0,255), -1)
                cv2.imshow('Frame',frame_gray)
            # Press Q on keyboard to  exit
                if cv2.waitKey(25) & 0xFF == ord('q'):
                    break
    except rospy.ROSInterruptException:
        pass




############# SURF STUFF#############
        #surf = cv2.xfeatures2d.SURF_create(1000)
        #keypoints, _ = surf.detectAndCompute(frame_gray,None)
        #print(keypoints[0].pt)
        #print(pts)
        #print(type(pts))
        #a = np.array([[10,10]], dtype=np.float32)
        #ptsarray = np.array(shape=(len(keypoints),2), dtype=np.float32)
        #print(len(keypoints))
        #for i in range(0, len(keypoints)):
        #    x = keypoints[i].pt[0]
        #    y = keypoints[i].pt[1]
        #    b = np.array([[x, y]], dtype=np.float32)
        #    if i == 0:
        #        points = np.array([[x, y]], dtype=np.float32)
        #    else:
        #        points = np.concatenate((points,b))
            #pts = np.append(pts, points)
            #print('in',points)

        #for i in range(0,len(points)):
        #    x_last.append(points.ravel()[2*i])
        #    y_last.append(points.ravel()[2*i+1])
