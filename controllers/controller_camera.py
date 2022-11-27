import rospy
from sensor_msgs.msg import LaserScan, Range, Image
from std_msgs.msg import String
from std_msgs.msg import Float64
import socket, pickle
import numpy as np
import time
import math
import cv2
import io
from PIL import Image as ImagePIL

pose_message=[]
before_pose_message=[]
lock=0
interupted=False
saved_state=[]
BEFORE_READING=0
sub=0
port = 9988

def ranges_to_polar(ranges,object_r=0.025,angle_inc=0.031733):
    ranges = list(ranges)
    cut_ranges = []
    index_cut_ranges = []
    for i in range(3):
        min_range = min(ranges)
        ind = ranges.index(min_range)
        cut_ranges.append(min_range)
        index_cut_ranges.append(ind)
        ranges[ind] = float('inf')

    d = np.mean(cut_ranges[:3]) + object_r
    theta = np.mean(index_cut_ranges)*angle_inc
    return (d,theta)

from cv_bridge import CvBridge
bridge = CvBridge()

def get_red_pixels(cv_image):
    img_hsv=cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # lower mask (0-10)
    lower_red = np.array([0,50,50])
    upper_red = np.array([10,255,255])
    mask0 = cv2.inRange(img_hsv, lower_red, upper_red)
    # upper mask (170-180)
    lower_red = np.array([170,50,50])
    upper_red = np.array([180,255,255])
    mask1 = cv2.inRange(img_hsv, lower_red, upper_red)
    # join my masks
    mask = mask1+mask0
    # set my output img to zero everywhere except my mask
    output_img = cv_image.copy()
    output_img = cv2.bitwise_and(output_img, output_img, mask= mask)

    return output_img

def get_target(red_pixels):

    kernel = np.ones((10,10), np.uint8)
    red_pixels = cv2.erode(red_pixels,kernel=kernel)

    test=cv2.cvtColor(red_pixels, cv2.COLOR_BGR2GRAY)
    nz = np.nonzero(test)

    px = (np.mean(nz[0]))
    py = (np.mean(nz[1]))

    x,y = pixel2coords(px,py)
    print(px,py)
    print(x,y)
    return x,y

def pixel2coords(px,py,H=640,W=480,C=0.025/14.0):
    Xcv = px * C
    Ycv = py * C

    x = -Xcv + (H/2.0*C)
    y = -Ycv + (0.54 + W/2.0*C)
    print('x,y:',x,y)
    return x,y

def read_camera(msg):
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    cv2.imshow('cv_image', cv_image)
    cv2.waitKey(1)

def main():
    global sub
    rospy.init_node('robot_state_publisher2') #this is an existing topic
    sub=rospy.Subscriber("/wx200/camera_link_optical/image_raw", Image, read_camera) # Camera sensor, Image is the data type sensor_msgs/Image
    rospy.spin()

if __name__ == "__main__":
    main()
