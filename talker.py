#!/usr/bin/env python
import rospy
import tf
from math import *
from std_msgs.msg import String
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from gazebo_msgs.msg import ModelStates
rospy.init_node('talker', anonymous=True)
pub1 = rospy.Publisher("/imu_joint1_controller/command", Float64, queue_size=10)
pub2 = rospy.Publisher("/imu_joint2_controller/command", Float64, queue_size=10)
pub3 = rospy.Publisher("/imu_joint3_controller/command", Float64, queue_size=10)
rate = rospy.Rate(10) # 10hz

erp=0
epp=0
eyp=0
sumr=0
sump=0
sumy=0
pos1=0
pos2=0
pos3=0
def callback(data):
    global erp
    global epp
    global eyp
    global sumr
    global sump
    global sumy
    global pos1
    global pos2
    global pos3
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.orientation)
    x =  data.orientation.x
    y =  data.orientation.y
    z =  data.orientation.z
    w =  data.orientation.w
    l = [x,y,z,w]
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(l)
    eroll = 0-roll
    epitch = 0-pitch
    eyaw = 0-yaw
    kp=65
    kd=80
    ki=8
    wx = kp*eroll + kd*(eroll-erp) + ki*((sumr))
    wy = kp*epitch + kd*(epitch-epp) + ki*((sump))
    wz = kp*eyaw + kd*(eyaw-eyp) + ki*((sumy))
    erp = eroll
    epp = epitch
    eyp = eyaw
    sumr+=eroll
    sump+=epitch
    sumy+=eyaw

    w1= (0.333)*(wz + (1.414*((wx*cos(yaw))-(wy*sin(yaw)))))
    w2= (0.333)*(wz+ ((1/1.414)*((sin(yaw)*((-1.732*wx)+wy))-(cos(yaw)*(wx+(1.732*wy))))))
    w3= (0.333)*(wz+ ((1/1.414)*((sin(yaw)*((1.732*wx)+wy))+(cos(yaw)*(-wx+(1.732*wy))))))
    pos1+=w1
    pos2+=w2
    pos3+=w3
    rospy.loginfo(rospy.get_caller_id() + "I heard yo ass %s", pos1)
    pub1.publish(w2)
    pub2.publish(w3)
    pub3.publish(w1)
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.Subscriber("/imu_data", Imu, callback)

    # spin() simply keeps python from exiting until this node is stopped

    rospy.spin()


if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
