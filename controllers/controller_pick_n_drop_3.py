import rospy
from sensor_msgs.msg import LaserScan, Range, Image
from std_msgs.msg import String
from std_msgs.msg import Float64
import socket, pickle
import numpy as np
import time
import math
import cv2

pose_message=[]
before_pose_message=[]
lock=0
interupted=False
saved_state=[]
BEFORE_READING=0
sub=0
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
    # gripper: left_finger, right_finger
    #self.group2.set_joint_value_target([0.02800624132752438, -0.02622681833090827])
    #self.group2.set_joint_value_target([0.028, -0.026]) #very open
    #self.group2.set_joint_value_target([0.018, -0.016]) #almost closed
    #self.group2.set_joint_value_target([0.034, -0.034]) #almost totally open
    #self.group2.set_joint_value_target([0.016, -0.016]) #good for cube
    arr=[]
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect(("localhost", 9988))
    arr.append("move_gripper")
    arr.append(value)
    data_string = pickle.dumps(arr)
    s.send(data_string)
    s.close()

move_gripper([0.033, -0.033])


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


def callback(msg):
    start_index=0
    end_index=0
    global before_pose_message
    global lock
    global interupted
    global saved_state
    global BEFORE_READING
    global pose_message
    flag=0




    if d<2.0 and round(d,1)!=round(BEFORE_READING,1):
       #print("aaa")
       BEFORE_READING=d
       amplitude= 1.0*d
       y=round(amplitude*math.sin(theta),5)
       x=round(amplitude*math.cos(theta),5)
       if theta <= np.pi/2:
          offset=0.1
          phi=np.arcsin((y+offset)/(np.sqrt(((y+offset)**2)+((x)**2))))
       if theta > np.pi/2:
          offset=0.1
          pi_minus_theta=np.pi-theta
          pi_minus_phi=np.arcsin((y+offset)/(np.sqrt(((y+offset)**2)+((-x)**2))))
          phi=np.pi-pi_minus_phi
       #print("theta:",theta,"phi:",phi)
       pose_message=[x+0.0+0.0000001,y+offset+0.0000001,0.2,0.0,0.0,phi]
    if d>=2.0 and round(d,1)!=round(BEFORE_READING,1):
       #print("bbb")
       BEFORE_READING=d
       print(d)
       pose_message=[0.1,0.0,0.25,0.0,0.0,0.0]
    if pose_message!=before_pose_message:
       #print("ccc")
       print(d)
       before_pose_message=pose_message[:]
       if lock==0:
         move_to_specified_pose(pose_message)
         lower=pose_message[:]
         lower[2]=0.14
         move_to_specified_pose(lower)
         move_gripper([0.016, -0.016])
         move_to_specified_pose(pose_message)
         move_to_specified_pose([0.1,0.0,0.25,0.0,0.0,0.0])
         before_pose_message=[0.1,0.0,0.25,0.0,0.0,0.0]
         move_gripper([0.033, -0.033])
         #rospy.signal_shutdown("whatever")
         #main()
         sub.unregister()
         main()
         #sub=rospy.Subscriber("/wx200/sensor/sonar_front4", LaserScan, callback) # cube big range sensor
       if lock==1:
         interupted=True
         saved_state=pose_message
    if pose_message==before_pose_message and interupted==False:
       pass
    if pose_message==before_pose_message and interupted==True and lock==0:
       move_to_specified_pose(saved_state)
       #print("BBBBBBBBBBBBBBBBBBBBBBBBBBBBB")
       lower=saved_state[:]
       lower[2]=0.14
       move_to_specified_pose(lower)
       move_gripper([0.016, -0.016])
       move_to_specified_pose(pose_message)
       move_to_specified_pose([0.1,0.0,0.25,0.0,0.0,0.0])
       before_pose_message=[0.1,0.0,0.25,0.0,0.0,0.0]
       move_gripper([0.033, -0.033])
       #rospy.signal_shutdown("whatever")
       #main()
       sub.unregister()
       main()
       #sub=rospy.Subscriber("/wx200/sensor/sonar_front4", LaserScan, callback) # cube big range sensor
       saved_state=[]
       interupted=False


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

def pixel2coords(px,py,H=480,W=640):
    C=0.025/14.0

    Xcv = px * C
    Ycv = py * C

    x = -Xcv + C*H/2
    y = -Ycv + (0.54 + C*W/2.0)
    return x,y

q = []
busy = False
standby_pose = [0.1,0.0,0.25,0.0,0.0,0.0]

def read_camera(msg):
    global busy
    print(busy)
    if busy: return

    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    red_pixels = get_red_pixels(cv_image)
    x,y = get_target(red_pixels)
    print(x,y)

    if x<0:
        phi = np.pi - np.arctan(-y/x)
    else:
        phi = np.arctan(y/x)

    pose_message=[x+0.0+0.0000001,y+0.0000001,0.2,0.0,0.0,phi]

    busy = True
    move_to_specified_pose(pose_message)
    # lower=pose_message[:]
    # lower[2]=0.14
    # move_to_specified_pose(lower)
    # move_gripper([0.016, -0.016])
    # move_to_specified_pose(pose_message)
    # move_to_specified_pose(standby_pose)
    # busy = False
    # move_gripper([0.033, -0.033])
    # print('end',busy)

    sub.unregister()
    main()



def main():
    global sub
    rospy.init_node('robot_state_publisher')#this is an existing topic
    #sub=rospy.Subscriber("/wx200/sensor/sonar_front4", LaserScan, callback) # cube big range sensor
    sub=rospy.Subscriber("/wx200/camera_link_optical/image_raw", Image, read_camera) # Camera sensor, Image is the data type sensor_msgs/Image
    rospy.spin()

if __name__ == "__main__":
    main()
