import rospy
from sensor_msgs.msg import LaserScan, Range
from std_msgs.msg import String
from std_msgs.msg import Float64
import socket, pickle
import numpy as np
import time
import math

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
    for i in range(len(msg.ranges)):
       if math.isinf(msg.ranges[i])==False:
           start_index=i
           flag=1
       if math.isinf(msg.ranges[i])==True and flag==1:
           end_index=i
           flag=0
       if i==len(msg.ranges)-1 and flag==1:
           end_index=i
           flag=0
    mean_index=(start_index+end_index)/2
    print(mean_index)
    mean_index = int(mean_index)
    print(mean_index)
    max_theta=np.pi/2
    min_theta=-np.pi/2
    theta_step=np.pi/len(msg.ranges)
    theta=theta_step*mean_index
    print(msg.ranges[mean_index])
    if msg.ranges[mean_index]<2.0 and round(msg.ranges[mean_index],1)!=round(BEFORE_READING,1):
       #print("aaa")
       print(msg.ranges[mean_index])
       BEFORE_READING=msg.ranges[mean_index]
       amplitude= 1.0*msg.ranges[mean_index]
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
    if msg.ranges[mean_index]>=2.0 and round(msg.ranges[mean_index],1)!=round(BEFORE_READING,1):
       #print("bbb")
       BEFORE_READING=msg.ranges[mean_index]
       print(msg.ranges[mean_index])
       pose_message=[0.1,0.0,0.25,0.0,0.0,0.0]
    if pose_message!=before_pose_message:
       #print("ccc")
       print(msg.ranges[mean_index])
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



def callback2(msg):
    global lock
    if msg.ranges[0]<1.0:
      #control_motor_angles([0.0, 0.0, 0.0, 0.0, 0.0])
      #move_to_specified_pose([0.50001,0.00001,0.0,0.0,0.0,0.0])
      #move_gripper([0.030, -0.030])
      #move_gripper([0.016, -0.016])
      lock=1
    else:
      lock=0


def callback3(msg):
    #print(msg.range)
    if msg.range<0.3:
      move_to_specified_pose([0.00001,0.00001,0.45,0.0,0.0,0.0])

#def read_camera(msg):


def main():
    global sub
    rospy.init_node('robot_state_publisher')#this is an existing topic
    #rospy.init_node('robot_state_publisher', disable_signals= True)#this is an existing topic
    sub=rospy.Subscriber("/wx200/sensor/sonar_front4", LaserScan, callback) # cube big range sensor
    #sub=rospy.Subscriber("/wx200/image_raw", np.int, read_camera) # cube big range sensor
    rospy.spin()

if __name__ == "__main__":
    main()
