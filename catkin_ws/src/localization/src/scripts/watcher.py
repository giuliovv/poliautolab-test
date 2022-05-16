#!/usr/bin/env python
"""Detects duckiebots from the watchtower using their unique colors.

Publish on watchtower00/localization a PointStamped with the car coordinates.

Edited version of the original example by Simon Halle.
"""
__author__ =  'Giulio Vaccari <giulio.vaccari at mail.polimi.it>'
__version__=  '0.1'
__license__ = 'BSD'
# Python libs
import sys, time

# numpy and scipy
import numpy as np
from scipy.ndimage import filters

# OpenCV
import cv2

# Ros libraries
import roslib
import rospy

# Ros Messages
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PointStamped

VERBOSE=False

def get_cars(img):
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    hsv_color1 = np.array([100, 100, 130])
    hsv_color2 = np.array([110, 250, 170])

    mask_blue = cv2.inRange(img_hsv, hsv_color1, hsv_color2)
    kernel = np.ones((2,2), np.uint8)
    img_erosion = cv2.erode(mask_blue, kernel, iterations=2)
    img_dilation = cv2.dilate(img_erosion, kernel, iterations=10)
    rho = 1  # distance resolution in pixels of the Hough grid

    theta = np.pi / 90  # angular resolution in radians of the Hough grid
    threshold = 20  # minimum number of votes (intersections in Hough grid cell)
    min_line_length = 20  # minimum number of pixels making up a line
    max_line_gap = 15  # maximum gap in pixels between connectable line segments

    # cv2.Canny(img_dilation,100,200)
    lines = cv2.HoughLinesP(cv2.Canny(img_dilation,50,100), rho, theta, threshold, np.array([]), min_line_length, max_line_gap)

    if lines is None:
        raise ValueError("No lines found")
    _points = lines.reshape(-1, 2)

    filt = ((_points - _points[0])**2).sum(1) < 70**2

    return _points[filt], _points[~filt]

class image_feature:

    def __init__(self):
        '''Initialize ros subscriber'''

        self.coordinates_pub = rospy.Publisher("/watchtower00/localization", PointStamped)

        # subscribed Topic
        self.subscriber = rospy.Subscriber("/watchtower00/camera_node/image/compressed",
            CompressedImage, self.callback,  queue_size = 1)
        if VERBOSE :
            print("subscribed to /camera/image/compressed")


    def callback(self, ros_data):
        '''Callback function of subscribed topic. 
        Here images get converted and features detected'''
        if VERBOSE :
            print(f'received image of type: "{ros_data.format}"' )

        #### direct conversion to CV2 ####
        np_arr = np.frombuffer(ros_data.data, 'u1')
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        localized = False

        try:
            first, second = get_cars(image_np)
            cv2.circle(image_np, first[0], 20, [0,0,255], -1)
            localized = True
        except ValueError:
            print("No lines found.")

        cv2.imshow('cv_img', image_np)
        cv2.waitKey(2)

        point = PointStamped()
        point.header.stamp = rospy.Time.now()
        point.header.frame_id = "/watchtower00/localization"
        if localized:
            point.point.x = first[0,0]
            point.point.y = first[0,1]
        else:
            point.point.x = -1
            point.point.y = -1

        self.coordinates_pub.publish(point)

        


def main(args):
    '''Initializes and cleanup ros node'''
    ic = image_feature()
    rospy.init_node('image_feature', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)