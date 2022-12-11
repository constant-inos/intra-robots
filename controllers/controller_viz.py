import rospy
from sensor_msgs.msg import LaserScan, Range, Image, JointState
from control_msgs.msg import JointTrajectoryControllerState
from std_msgs.msg import String
from std_msgs.msg import Float64
import socket, pickle
import numpy as np
import time
import math
import cv2
from cv_bridge import CvBridge
import random

bridge = CvBridge()
sub=0
sub2 = 0
exev_status_code = 0
exev_status_text = ''



def get_objects_position(cv_image):
    # hsv format: hue, saturation, value (brightness)
    img_hsv=cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # lower red mask (0-10)
    lower_red = np.array([0,50,50])
    upper_red = np.array([10,255,255])
    red_mask0 = cv2.inRange(img_hsv, lower_red, upper_red)
    # upper red mask (170-180)
    lower_red = np.array([170,50,50])
    upper_red = np.array([180,255,255])
    red_mask1 = cv2.inRange(img_hsv, lower_red, upper_red)
    # join my masks
    red_mask = red_mask1 + red_mask0
    # lower green mask (hue 40-70)
    lower_green = np.array([40,40,40])
    upper_green = np.array([70,255,255])
    green_mask = cv2.inRange(img_hsv, lower_green, upper_green)
    # set my output img to zero everywhere except my mask
    output_img = cv_image.copy()
    only_reds = cv2.bitwise_and(output_img, output_img, mask=red_mask)
    only_greens = cv2.bitwise_and(output_img, output_img, mask=green_mask)

    objects = []

    K_reds,cr = count_objects(only_reds)
    K_greens,cg = count_objects(only_greens)

    print("Red objects found:",K_reds)
    print("Green objects found:",K_greens)

    if K_reds > 0:
        gray = cv2.cvtColor(only_reds, cv2.COLOR_BGR2GRAY)
        red_points = []
        for i in range(gray.shape[0]):
            for j in range(gray.shape[1]):
                if gray[i,j]:
                    red_points.append(np.array([i,j]))
        red_points = np.float32(red_points)
        criteria = (cv2.TERM_CRITERIA_EPS, 10, 1.0)
        ret, label, center = cv2.kmeans(red_points, K_reds, None, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)

        for c in list(center):
            objects.append((c.round(),'red'))

    if K_greens > 0:
        gray = cv2.cvtColor(only_greens, cv2.COLOR_BGR2GRAY)
        green_points = []
        for i in range(gray.shape[0]):
            for j in range(gray.shape[1]):
                if gray[i,j]:
                    green_points.append(np.array([i,j]))
        green_points = np.float32(green_points)
        criteria = (cv2.TERM_CRITERIA_EPS, 10, 1.0)
        ret, label, center = cv2.kmeans(green_points, K_greens, None, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)

        for c in list(center):
            objects.append((c.round(),'green'))

    imc = cv_image.copy()
    for i in range(len(cr)):
        cv2.drawContours(imc, cr, i, (255, 0, 0), 2)
    for i in range(len(cg)):
        cv2.drawContours(imc, cg, i, (255, 0, 0), 2)

    for o in objects:
        center_coordinates = tuple(o[0])
        center_coordinates = (center_coordinates[1],center_coordinates[0])
        text_coordinates = (int(center_coordinates[0]-15),int(center_coordinates[1]-15))
        if o[1]=='red': text_coordinates = (int(center_coordinates[0]+15),int(center_coordinates[1]+15))
        radius = 1
        color = (0,0,0)
        thickness = 2
        cv2.circle(imc, center_coordinates, radius, color, thickness)
        cv2.putText(imc,str(center_coordinates)+', '+o[1],text_coordinates, cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)


    cv2.putText(imc,"Green Boxes: "+str(K_greens),(130,400), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 1, cv2.LINE_AA)
    cv2.putText(imc,"Red Boxes:   "+str(K_reds),(130,435), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 1, cv2.LINE_AA)

    cv2.imshow('imc',imc)
    cv2.waitKey(1)

    return objects

def count_objects(image):
    blur = cv2.GaussianBlur(image, (11, 11), 0)
    canny = cv2.Canny(blur, 50,150, 3)
    dilated = cv2.dilate(canny, (1, 1), iterations=0)
    (cnt, hierarchy) = cv2.findContours(dilated.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    return len(cnt),cnt

def pixel2coords(px,py,H=480,W=640):
    C=0.025/14.0

    Xcv = px * C
    Ycv = py * C

    x = -Xcv + C*H/2
    y = -Ycv + (0.54 + C*W/2.0)

    # small perception corrections
    x = x + 0.025*x/0.4
    y = y - 0.025*(0.54-y)/0.25

    return x,y

q = []

def read_camera(msg):

    C=0.025/14.0
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    objects = get_objects_position(cv_image)

    sub.unregister()
    sub2.unregister()
    main()



def get_gripper_angle(x,y):

    if (x<0 and y>0):
        theta = np.pi - np.arctan(-y/x)
    elif (x<0 and y<0):
        theta = -np.pi + np.arctan(y/x)
    else:
        if x == 0: return np.arctan(np.inf)
        theta = np.arctan(y/x)
    return theta

def execution_status(msg):
    global exev_status_text, exec_status_code
    exev_status_text, exec_status_code = msg.status.text, msg.status.status
    print('exec_status')
    return exev_status_text, exec_status_code

from moveit_msgs.msg import MoveGroupActionResult
from actionlib_msgs.msg import GoalStatus

def main():
    global sub,sub1,sub2
    rospy.init_node('camera_observer') #this is an existing topic

    sub = rospy.Subscriber("/wx200/camera_link_optical/image_raw", Image, read_camera) # Camera sensor, Image is the data type sensor_msgs/Image
    sub2 = rospy.Subscriber("/wx200/move_group/result", MoveGroupActionResult, execution_status)
    rospy.spin()

if __name__ == "__main__":
    main()
