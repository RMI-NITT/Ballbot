#!/usr/bin/env python
 #import required libraries
import rospy
import tf

import numpy as np

from std_msgs.msg import String
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelStates

import time
from math import sin, cos

# init a new node
rospy.init_node("controller", anonymous=True)

# make the joint state publishers for the joints
pub1 = rospy.Publisher("/imu_joint1_controller/command", Float64, queue_size=10)
pub2 = rospy.Publisher("/imu_joint2_controller/command", Float64, queue_size=10)
pub3 = rospy.Publisher("/imu_joint3_controller/command", Float64, queue_size=10)


required_x1 = 0
required_y1 = 0

t0 = 0
vwheel_speed_x = 0
vwheel_speed_y = 0
flag=0

sum_pitch = 0
sum_roll = 0
prev_pitch = 0
prev_roll = 0

"""
def get_bot_info(info):
    global pos_x
    global pos_y
    global pos_z

    global vel_x
    global vel_y
    global vel_z

    pos_x = info.pose[1].position.x - required_x1
    pos_y = info.pose[1].position.y - required_y1
    pos_z = info.pose[1].position.z

    vel_x = info.twist[1].linear.x
    vel_y = info.twist[1].linear.y
    vel_z = info.twist[1].linear.z
"""

def controller(data):
    global pub1
    global pub2
    global pub3

    global required_x1
    global required_y1

    global t0
    global vwheel_speed_x
    global vwheel_speed_y
    global vwheel_acc_x
    global vwheel_acc_y
    global flag

    global sum_pitch
    global sum_roll
    global prev_pitch
    global prev_roll
    
    # will be usefull when interpolation has been done
    x0 = required_x1
    y0 = required_y1

    # obtain the pose data from the imu in quaternion form
    x = data.orientation.x
    y = data.orientation.y
    z = data.orientation.z
    w = data.orientation.w

    # the orientation of the bot represented in quaternion form
    l = [x, y, z, w]

    # obtaining the yaw pitch and roll values from the quaternions
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(l)

    t = time.time()

    # get orientation data from ros
    ang_vel_x = data.angular_velocity.x
    ang_vel_y = data.angular_velocity.y
    ang_vel_z = data.angular_velocity.z

    lin_acc_x = data.linear_acceleration.x
    lin_acc_y = data.linear_acceleration.y

    ##############################
    # set_yaw = yaw              #
    # eroll = set_roll - roll    #
    # epitch = set_pitch - pitch #
    # eyaw = set_yaw - yaw       #
    ##############################

    # addded twp gain variables in each for future use
    # for now will only use the f1rst ones
    # refer to paper for better understanding
    k_pitch = 100
    k_roll = 100

    k_pitch_vel = 60
    k_roll_vel = 60

    k_pitch_sum = 5
    k_roll_sum = 5

    """k_x_pos = 0
    k_y_pos = 0

    k_x_vel = 0
    k_y_vel = 0"""

    vwheel_acc_y = k_pitch*(0-pitch) + k_pitch_vel*(prev_pitch-pitch) + k_pitch_sum*(0-sum_pitch)
        #k_x_pos*(x - x0) + k_x_vel*(vwheel_speed_x)
    vwheel_acc_x = k_roll*(0-roll) + k_roll_vel*(prev_roll-roll) + k_roll_sum*(0-sum_roll)
        #k_x_pos*(y - y0) + k_x_vel*(vwheel_speed_y)
    #if flag!=0:
    #    vwheel_speed_x += vwheel_acc_x*(t-t0)
    #    vwheel_speed_y += vwheel_acc_y*(t-t0)
    if flag==0:
        vwheel_speed_x = vwheel_acc_x
        vwheel_speed_y = vwheel_acc_y
        flag=1

    wx = vwheel_speed_x
    wy = vwheel_speed_y
    wz = 0

    w1= (0.333)*(wz + (1.414*((wx*cos(yaw))-(wy*sin(yaw)))))
    w2= (0.333)*(wz+ ((1/1.414)*((sin(yaw)*((-1.732*wx)+wy))-(cos(yaw)*(wx+(1.732*wy))))))
    w3= (0.333)*(wz+ ((1/1.414)*((sin(yaw)*((1.732*wx)+wy))+(cos(yaw)*(-wx+(1.732*wy))))))

    """rospy.loginfo(rospy.get_caller_id(
    ) + " i heard \nwx : %s,\nwy : %s,\nwz : %s,\npitch : %s,\nroll : %s\n", wx, wy, wz, pitch, roll)
    rospy.loginfo("\nw1 : %s\nw2 : %s\nw3 : %s\n", w1, w2, w3)

    rospy.loginfo("\nvwheel_acc_x : %s\nvwheel_acc_y : %s\n",
                  vwheel_acc_x, vwheel_acc_y)
    rospy.loginfo("\nt : %s, \nt0 : %s\n", t, t0)
    """

    pub1.publish(w2)
    pub2.publish(w3)
    pub3.publish(w1)

    print("VEL", w1,w2,w3)

    prev_pitch = pitch
    prev_roll = roll

    sum_pitch += pitch
    sum_roll += roll
    t0 = t


def vel_update(event):
    global vwheel_speed_x
    global vwheel_speed_y
    global vwheel_acc_x
    global vwheel_acc_y
    global flag

    dt = 0.1

    vwheel_speed_x += vwheel_acc_x*dt
    vwheel_speed_y += vwheel_acc_y*dt



def listener():
    rospy.Subscriber("/imu_data", Imu, controller)
    timer = rospy.Timer(rospy.Duration(100/1000.0), vel_update)
    #rospy.Subscriber("/gazebo/model_states", ModelStates, get_bot_info)

    # spin stops python from exiting this code until node is stopped
    rospy.spin()


if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
