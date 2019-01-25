#!/usr/bin/env python
import rospy
import tf
from math import *
import time
from std_msgs.msg import String
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelStates
rospy.init_node('talker', anonymous=True)
pub1 = rospy.Publisher("/imu_joint1_controller/command", Float64, queue_size=10)
pub2 = rospy.Publisher("/imu_joint2_controller/command", Float64, queue_size=10)
pub3 = rospy.Publisher("/imu_joint3_controller/command", Float64, queue_size=10)
rate = rospy.Rate(1) # 10hz

erp=0
epp=0
eyp=0
sumr=0
sump=0
sumy=0
pos1=0
pos2=0
pos3=0
x_p = 0
y_p = 0
x_vel0 = 0
y_vel0 = 0
set_r = 0
set_p = 0
set_y = 0
t0 = rospy.get_time()
pos_x = 0
pos_y = 0


def callback(data):
	global erp
	global epp
	global eyp
	global pos_x
	global pos_y
	global set_r
	global set_p
	global set_y
	global t0
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
	t = time.time()


	eroll = set_r-roll
	epitch = set_p-pitch
	eyaw = set_y-yaw
	"""
	if abs(eroll)<0.02:
		sumr = 0
	if abs(epitch)<0.02:
		sump = 0
	if abs(eyaw)<0.02:
		sumy = 0
	"""
	kp=65
	kd=85
	ki=8
	wx = kp*eroll + kd*(eroll-erp) + ki*((sumr))
	wy = kp*epitch + kd*(epitch-epp) + ki*((sump))
	wz = kp*eyaw + kd*(eyaw-eyp) + ki*((sumy))
	print("Errors Roll",sumr,eroll,(eroll-erp))
	print("Errors Pitch",sump,epitch,(epitch-epp))
	print("Errors Yaw",sumy,eyaw,(eyaw-eyp))
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

	w_x = (wx+(eroll-erp)/(t-t0))*(0.0659/0.125) + (eroll-erp)
	w_y = (wy+(epitch-epp)/(t-t0))*(0.0659/0.125) + (epitch-epp)
	w_z = (wz+(eyaw-eyp)/(t-t0))*(0.0659/0.125) + (eyaw-eyp)

	pos_x+= w_x*(t-t0)*0.125
	pos_y+= w_y*(t-t0)*0.125

	#print("POSN",pos_x,pos_y)
	d = sqrt(pos_x**2 + pos_y**2)
	ang = atan2(pos_y,pos_x)
	set_r = sin(ang)*(5/180)*3.1417
	set_p = cos(ang)*(5/180)*3.1417

	#print(w_x,w_y,w_z,"Ball")
	rospy.loginfo(rospy.get_caller_id() + "I heard yo ass %s, %s, %s", wx, wy, wz)

	pub1.publish(w2)
	pub2.publish(w3)
	pub3.publish(w1)
	t0 = t
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





"""
if abs(w1)<6.28:
w1=0
elif abs(w1)>15.727:
if w1>0:
w1=15.727
else:
w1=-15.727
if abs(w2)<6.28:
w2=0
elif abs(w2)>15.727:
if w2>0:
w2=15.727
else:
w2=-15.727
if abs(w3)<6.28:
w3=0
elif abs(w3)>15.727:
if w3>0:
w3=15.727
else:
w3=-15.727
"""
