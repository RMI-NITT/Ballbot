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
rate = rospy.Rate(10) # 10hz

req_x = 10
req_y = 10

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
pos_z = 0
v_x = 0
v_y = 0
vel_x = 0
vel_y = 0
vel_z = 0
index = 0
sum_d = 0
pre_d = 0
req_x1 = 0
req_y1 = 0


def get_pos(info):

	global pos_x
	global pos_y
	global pos_z
	global req_x1
	global req_y1
	global vel_x
	global vel_y
	global vel_z
	pos_x= info.pose[1].position.x - req_x1
	pos_y= info.pose[1].position.y - req_y1
	pos_z= info.pose[1].position.z

	vel_x= info.twist[1].linear.x
	vel_y= info.twist[1].linear.y
	vel_z= info.twist[1].linear.z


def callback(data):
	near = []
	global erp
	global epp
	global near
	global eyp
	global pos_x
	global pos_y
	global pos_z
	global vel_x
	global vel_y
	global vel_z
	global v_x
	global v_y
	global set_r
	global set_p
	global set_y
	global req_x1
	global req_y1
	global t0
	global index
	global sumr
	global sump
	global sumy
	global pos1
	global pos2
	global pos3
	global sum_d
	global pre_d

	#rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.orientation)


	#print("POSNS",pos_x,pos_y,pos_z)
	#a_x = data.linear_acceleration.x
	#a_y = data.linear_acceleration.y
	#a_z = data.linear_acceleration.z

	x =  data.orientation.x
	y =  data.orientation.y
	z =  data.orientation.z
	w =  data.orientation.w
	l = [x,y,z,w]
	roll, pitch, yaw = tf.transformations.euler_from_quaternion(l)
	t = time.time()

	set_y = yaw
	eroll = set_r-roll
	epitch = set_p-pitch
	eyaw = set_y-yaw


	kp=115
	kd=145
	ki=5
	wx = kp*eroll + kd*(eroll-erp) + ki*((sumr))
	wy = kp*epitch + kd*(epitch-epp) + ki*((sump))
	wz = kp*eyaw + kd*(eyaw-eyp) + ki*((sumy))

	print("Errors Roll",sumr,eroll,(eroll-erp))
	print("Errors Pitch",sump,epitch,(epitch-epp))
	print("Errors Yaw",sumy,eyaw,(eyaw-eyp))


	w1= (0.333)*(wz + (1.414*((wx*cos(yaw))-(wy*sin(yaw)))))
	w2= (0.333)*(wz+ ((1/1.414)*((sin(yaw)*((-1.732*wx)+wy))-(cos(yaw)*(wx+(1.732*wy))))))
	w3= (0.333)*(wz+ ((1/1.414)*((sin(yaw)*((1.732*wx)+wy))+(cos(yaw)*(-wx+(1.732*wy))))))


	des_ang = atan2(req_y,req_x)
	num_points = int(sqrt(req_x**2 + req_y**2))

	dist=0
	for i in range(1,num_points+1):
		x = cos(des_ang)*i
		y = sin(des_ang)*i
		near.append([x,y])

	print(near,"NUM")
	req_x1 = near[index][0]
	req_y1 = near[index][1]
	print("REQ",req_x1,req_y1)
	if sqrt(pos_x**2 + pos_y**2)<0.3 and index!=len(near)-1:
		index+=1

	#print("POSN",pos_x,pos_y)
	d = sqrt(pos_x**2 + pos_y**2)
	ang = atan2(pos_y,pos_x)

	kp1 = 1
	kp2 = 1.25
	kp3 = 0.00001

	xx = kp1*d + kp2*(d-pre_d) + kp3*sum_d
	print("XXXX",xx)

	set_r = abs(cos(ang))*(1.0*xx/180)*3.1417
	set_p = abs(sin(ang))*(1.0*xx/180)*3.1417
	#print("POSN",pos_x,pos_y)

	if abs(roll-set_r)<0.034:
		sum_r = 0
	if abs(pitch-set_p)<0.034:
		sum_p = 0

	if pos_x>0:
		set_p = -set_p
	if pos_y<0:
		set_r = -set_r

	if vel_x>0.2:
		set_p = -0.034
	if vel_x<-0.2:
		set_p = 0.034
	if vel_y>0.2:
		set_r = 0.034
	if vel_y<-0.2:
		set_r = -0.034
	#print("VELS",vel_x,vel_y)
	#print("SETS",set_r,set_p)


	#print(w_x,w_y,w_z,"Ball")
	rospy.loginfo(rospy.get_caller_id() + "I heard yo ass %s, %s, %s", wx, wy, wz)

	erp = eroll
	epp = epitch
	eyp = eyaw
	sumr+=eroll
	sump+=epitch
	sumy+=eyaw
	sum_d+=d
	pre_d = d

	if d<0.2:
		sum_d = 0

	if set_r>0.034:
		set_r = 0.034
	if set_r<-0.034:
		set_r = -0.034
	if set_p>0.034:
		set_p = 0.034
	if set_p<-0.034:
		set_p = -0.034

	print("SET",set_r,set_p)

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
	rospy.Subscriber("/gazebo/model_states", ModelStates, get_pos)
	#rospy.Subscriber("/odom", Imu, get_pos)



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
