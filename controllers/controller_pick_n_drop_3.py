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
sub1 = 0
port = 9988

def move_to_specified_pose(value):
    global port
    arr=[]
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect(("localhost", 9988))
    arr.append("move_to_specified_pose")
    arr.append(value)
    data_string = pickle.dumps(arr)
    s.send(data_string)
    s.close()

def control_motor_angles(value):
    global port
    arr=[]
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect(("localhost", 9988))
    arr.append("control_motor_angles")
    arr.append(value)
    data_string = pickle.dumps(arr)
    s.send(data_string)
    s.close()

def move_gripper(value):
    global port
    arr=[]
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect(("localhost", 9988))
    arr.append("move_gripper")
    arr.append(value)
    data_string = pickle.dumps(arr)
    s.send(data_string)
    s.close()

def open_gripper():
    move_gripper([0.033, -0.033])
def close_gripper():
    move_gripper([0.016, -0.016])


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

    return x,y

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
busy = False
standby_pose = [0.1,0.0,0.25,0.0,0.0,0.0]

def read_camera(msg):
    global standby_pose
    global busy
    if busy: return

    move_to_specified_pose(standby_pose)

    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    red_pixels = get_red_pixels(cv_image)
    x,y = get_target(red_pixels)

    if x<0: phi = np.pi - np.arctan(-y/x)
    else: phi = np.arctan(y/x)

    pose_message=[x+0.0+0.0000001,y+0.0000001,0.2,0.0,0.0,phi]

    xr = random.random()-0.5
    yr = random.random()*0.35+0.2
    if x<0: phir = np.pi - np.arctan(-yr/xr)
    else: phir = np.arctan(yr/xr)
    random_pose=[xr+0.0+0.0000001,yr+0.0000001,0.2,0.0,0.0,phir]

    busy = True
    move_to_specified_pose(pose_message)
    lower=pose_message[:]
    lower[2]=0.14
    move_to_specified_pose(lower)
    close_gripper()
    move_to_specified_pose(pose_message)
    print(xr,yr)
    # move_to_specified_pose(random_pose)
    open_gripper()
    move_to_specified_pose(standby_pose)
    busy = False
    sub.unregister()
    main()

def is_busy(msg):
    global sub1

    d = math.dist(msg.actual.positions,msg.desired.positions)
    # print(d)
    # print(msg.desired.positions)
    # print(msg.actual.positions)

def execution_status(msg):
    global sub2
    print(msg)

from moveit_msgs.msg import MoveGroupResult
from actionlib_msgs.msg import GoalStatus
def main():
    global sub,sub1,sub2
    rospy.init_node('robot_state_publisher')#this is an existing topic
    #sub=rospy.Subscriber("/wx200/sensor/sonar_front4", LaserScan, callback) # cube big range sensor
    sub=rospy.Subscriber("/wx200/camera_link_optical/image_raw", Image, read_camera) # Camera sensor, Image is the data type sensor_msgs/Image
    # sub1 = rospy.Subscriber("/wx200/arm_controller/state", JointTrajectoryControllerState, is_busy)
    sub2 = rospy.Subscriber("/wx200/move_group/result", MoveGroupResult, execution_status)
    rospy.spin()

if __name__ == "__main__":
    main()
