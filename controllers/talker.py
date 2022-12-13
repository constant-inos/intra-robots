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

class Talker:
    def __init__(self):
        self.publisher = rospy.Publisher('talker',String,queue_size=10)
        rospy.init_node('talker')
        self.rate = rospy.Rate(10) # 10 Hz
        self.message = 'Empty Message'


    def publish(self):
        rospy.loginfo(self.message)
        self.publisher.publish(self.message)

    # def loop(self):
    #     while not rospy.is_shutdown():


T = Talker()
T.message = "Wassup my nigga?"




def main():
    while True:
        T.publish()
    # global sub,sub1,sub2
    # rospy.init_node('robot_state_publisher') #this is an existing topic
    #
    # sub = rospy.Subscriber("/wx200/camera_link_optical/image_raw", Image, read_camera) # Camera sensor, Image is the data type sensor_msgs/Image
    # sub2 = rospy.Subscriber("/wx200/move_group/result", MoveGroupActionResult, execution_status)
    # rospy.spin()

if __name__ == "__main__":
    main()
