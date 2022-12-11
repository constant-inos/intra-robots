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

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind(("localhost", 9989))

def receive_socket_msg(string,address="localhost",port=9989):
    # s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # s.connect((address, port))
    global s
    msg = []
    try:
        s.listen(1)
        conn, addr = s.accept()
        data = conn.recv(1024)
        data_arr = pickle.loads(data)
        msg = []
        if data_arr[0] == string:
            msg = data_arr
        s.shutdown(1)
        conn.close()
    except socket.error as socketerror:
        print("Error:",socketerror)

    return msg

def send_socket_msg(Message,address="localhost",port=9988):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((address, port))
    data_string = pickle.dumps(Message)
    s.send(data_string)
    s.close()

def move_to_specified_pose(value,orientation='standard'):
    x,y,z = value
    theta = get_gripper_angle(x,y)
    pose = [x,y,z,0.0,0.0,theta]

    arr=["move_to_specified_pose",pose]
    send_socket_msg(arr)

    msg = []
    while not msg:
        msg = receive_socket_msg('go_to_defined_pose attempt')

    print('EXECUTION:',msg[2])
    if msg[2] == 'success':
        return True
    if msg[2] == 'fail':
        return False


def control_motor_angles(value):
    arr=[]
    arr.append("control_motor_angles")
    arr.append(value)
    send_socket_msg(arr)

def move_gripper(value):
    arr=[]
    arr.append("move_gripper")
    arr.append(value)
    send_socket_msg(arr)

    msg = []
    while not msg:
        msg = receive_socket_msg('move_gripper attempt')

    return msg[1]


def open_gripper():
    return move_gripper([0.033, -0.033])
def close_gripper():
    return move_gripper([0.016, -0.016])

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

    return objects

def count_objects(image):
    blur = cv2.GaussianBlur(image, (11, 11), 0)
    canny = cv2.Canny(blur, 30, 150, 3)
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
busy = False
standby_pose = (0.1,0.0,0.25)

def read_camera(msg):
    global standby_pose
    global busy
    if busy: return

    C=0.025/14.0
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    #print("Capturing image...")
    objects = get_objects_position(cv_image)

    busy = True
    for o in objects:
        success = get_object(o[0],o[1])

    busy = False
    sub.unregister()
    sub2.unregister()
    main()

def get_object(obj_pixels,color):

    x,y = pixel2coords(obj_pixels[0],obj_pixels[1])

    if x<0: phi = np.pi - np.arctan(-y/x)
    else: phi = np.arctan(y/x)
    pose = (x,y,0.25)

    if color == 'red':
        bucket = (0.35,-0.35,0.25)
    else:
        bucket = (-0.35,-0.35,0.25)
    move_to_specified_pose(standby_pose)
    success = move_to_specified_pose(pose)
    print(success)
    if not success:
        success = move_to_specified_pose((pose[0]+0.001,pose[1]+0.001,pose[2]))
        if not success:
            print("ignoring unreachable object")
            return
    lower = (pose[0],pose[1],0.14)

    move_to_specified_pose(lower)

    close_gripper()

    move_to_specified_pose(pose)

    send_socket_msg(['check_gripper_state'],port=9988)

    msg = receive_socket_msg('check_gripper_state',port=9989)
    print(msg[1])

    move_to_specified_pose(bucket)

    open_gripper()

    move_to_specified_pose(standby_pose)


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
    rospy.init_node('robot_state_publisher') #this is an existing topic

    sub = rospy.Subscriber("/wx200/camera_link_optical/image_raw", Image, read_camera) # Camera sensor, Image is the data type sensor_msgs/Image
    sub2 = rospy.Subscriber("/wx200/move_group/result", MoveGroupActionResult, execution_status)
    rospy.spin()

if __name__ == "__main__":
    main()
