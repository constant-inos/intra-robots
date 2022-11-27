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



def get_object_pixels(cv_image):
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



    if only_reds.sum() > only_greens.sum():
        output_img = only_reds
        color = 'red'
    else:
        output_img = only_greens
        color = 'green'

    output_img=cv2.cvtColor(output_img, cv2.COLOR_BGR2GRAY)
    return output_img, color

def get_target(obj_pixels):

    kernel = np.ones((10,10), np.uint8)
    obj_pixels = cv2.erode(obj_pixels,kernel=kernel)

    # test=cv2.cvtColor(red_pixels, cv2.COLOR_BGR2GRAY)
    nz = np.nonzero(obj_pixels)

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
    obj_pixels, color = get_object_pixels(cv_image)

    x,y = get_target(obj_pixels)

    if x<0: phi = np.pi - np.arctan(-y/x)
    else: phi = np.arctan(y/x)

    pose_message=[x+0.0+0.0000001,y+0.0000001,0.2,0.0,0.0,phi]

    xr = random.random()-0.5
    yr = random.random()*0.35+0.2
    if x<0: phir = np.pi - np.arctan(-yr/xr)
    else: phir = np.arctan(yr/xr)
    random_pose=[xr+0.0+0.0000001,yr+0.0000001,0.2,0.0,0.0,phir]

    if color == 'red':
        bucket = [0.35,0.0,0.25,0.0,0.0,0.0]
    else:
        bucket = [0.25,0.0,0.25,0.0,0.0,0.0]

    busy = True
    move_to_specified_pose(pose_message)
    lower=pose_message[:]
    lower[2]=0.14
    move_to_specified_pose(lower)
    close_gripper()
    move_to_specified_pose(pose_message)
    print(xr,yr)
    # move_to_specified_pose(random_pose)
    move_to_specified_pose(bucket)
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
