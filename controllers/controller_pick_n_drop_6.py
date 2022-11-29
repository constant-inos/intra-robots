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
import scipy.cluster.hierarchy as hcluster
from sklearn.cluster import MeanShift

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

    # kernel = np.ones((10,10), np.uint8)
    # only_reds = cv2.erode(only_reds,kernel=kernel)
    # only_greens = cv2.erode(only_greens,kernel=kernel)

    objects = []

    K_reds = count_objects(only_reds)
    K_greens = count_objects(only_greens)
    print(K_reds,K_greens)
    if K_reds > 0:
        gray = cv2.cvtColor(only_reds, cv2.COLOR_BGR2GRAY)
        red_points = []
        for i in range(gray.shape[0]):
            for j in range(gray.shape[1]):
                if gray[i,j]:
                    red_points.append(np.array([i,j]))
        red_points = np.float32(red_points)
        print(len(red_points))
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
        print(len(green_points))
        criteria = (cv2.TERM_CRITERIA_EPS, 10, 1.0)
        ret, label, center = cv2.kmeans(green_points, K_greens, None, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)

        for c in list(center):
            objects.append((c.round(),'green'))

    return objects

def count_objects(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (11, 11), 0)
    canny = cv2.Canny(blur, 30, 150, 3)
    dilated = cv2.dilate(canny, (1, 1), iterations=0)
    (cnt, hierarchy) = cv2.findContours(dilated.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    return len(cnt)

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
    objects = get_objects_position(cv_image)

    busy = True
    for o in objects:
        get_object(o[0],o[1])

    busy = False
    sub.unregister()
    main()


def get_object(obj_pixels,color):

    x,y = pixel2coords(obj_pixels[0],obj_pixels[1])

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


def is_busy(msg):
    global sub1

    d = math.dist(msg.actual.positions,msg.desired.positions)
    # print(d)
    # print(msg.desired.positions)
    # print(msg.actual.positions)

def execution_status(msg):
    global exev_status_text, exec_status_code
    exev_status_text, exec_status_code = msg.status.text, msg.status.status
    print(exev_status_text, exec_status_code)
    return exev_status_text, exec_status_code

from moveit_msgs.msg import MoveGroupActionResult
from actionlib_msgs.msg import GoalStatus
def main():
    global sub,sub1,sub2
    rospy.init_node('robot_state_publisher')#this is an existing topic
    #sub=rospy.Subscriber("/wx200/sensor/sonar_front4", LaserScan, callback) # cube big range sensor
    sub = rospy.Subscriber("/wx200/camera_link_optical/image_raw", Image, read_camera) # Camera sensor, Image is the data type sensor_msgs/Image
    # sub1 = rospy.Subscriber("/wx200/arm_controller/state", JointTrajectoryControllerState, is_busy)
    sub2 = rospy.Subscriber("/wx200/move_group/result", MoveGroupActionResult, execution_status)
    rospy.spin()

if __name__ == "__main__":
    main()
