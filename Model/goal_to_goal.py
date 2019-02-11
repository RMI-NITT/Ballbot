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

req_x = 10	## required position in x
req_y = 10	## required position in y

erp=0		## differential errors of Pitch
epp=0		## differential Roll
eyp=0		## differential yaw
sumr=0		## integral roll
sump=0		## integral Pitch
sumy=0		## integral yaw

set_r = 0	## desired roll angle to get to base
set_p = 0	## desired pitch angle to get to base
set_y = 0	## desired yaw angle to get to base

t0 = rospy.get_time()

pos_x = 0	## current position of system in x
pos_y = 0	## current position of system in y
pos_z = 0	## current position of system in z

vel_x = 0	## current velocity in x
vel_y = 0	## current velocity in y
vel_z = 0	## current velocity in z

index = 0	## for indexing way-points
sum_d = 0	## integral distance error to set the desired lean angle
pre_d = 0	## differential distance error to set the desired lean angle
req_x1 = 0	## required waypoint position target in x
req_y1 = 0	## required waypoint position target in y


def get_pos(info):
	global pos_x
	global pos_y
	global pos_z
	global req_x1
	global req_y1
	global vel_x
	global vel_y
	global vel_z
	pos_x= info.pose[1].position.x	## receiving current positions in x
	pos_y= info.pose[1].position.y	## receiving current positions in y
	pos_z= info.pose[1].position.z	## receiving current positions in z

	vel_x= info.twist[1].linear.x	## receiving current velocity in x
	vel_y= info.twist[1].linear.y	## receiving current velocity in y
	vel_z= info.twist[1].linear.z	## receiving current velocity in z


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
	global sum_d
	global pre_d

	#######################		finding roll pitch and Yaw	#############################

	x =  data.orientation.x
	y =  data.orientation.y
	z =  data.orientation.z
	w =  data.orientation.w
	l = [x,y,z,w]
	roll, pitch, yaw = tf.transformations.euler_from_quaternion(l)
	t = time.time()

	#######################		finding the errors for pid 	#############################

	set_y = yaw		## not considering the yaw rotation abt z in pid
	eroll = set_r-roll
	epitch = set_p-pitch
	eyaw = set_y-yaw

	#######################		finding the required planar velocities of wheel from pid	#############################

	kp=100
	kd=60
	ki=5
	wx = kp*eroll + kd*(eroll-erp) + ki*((sumr))
	wy = kp*epitch + kd*(epitch-epp) + ki*((sump))
	wz = kp*eyaw + kd*(eyaw-eyp) + ki*((sumy))

	print("Errors Roll",sumr,eroll,(eroll-erp))
	print("Errors Pitch",sump,epitch,(epitch-epp))
	print("Errors Yaw",sumy,eyaw,(eyaw-eyp))

	#######################		finding transformations of the planar velocities	#############################

	w1= (0.333)*(wz + (1.414*((wx*cos(yaw))-(wy*sin(yaw)))))
	w2= (0.333)*(wz+ ((1/1.414)*((sin(yaw)*((-1.732*wx)+wy))-(cos(yaw)*(wx+(1.732*wy))))))
	w3= (0.333)*(wz+ ((1/1.414)*((sin(yaw)*((1.732*wx)+wy))+(cos(yaw)*(-wx+(1.732*wy))))))

	#######################		calculating desired heading angle from origin and num of way-points	#############################

	des_ang = atan2(req_y,req_x)
	num_points = int(sqrt(req_x**2 + req_y**2))

	#######################		finding waypoints on st line	#############################

	dist=0
	for i in range(1,num_points+1):
		x = cos(des_ang)*i
		y = sin(des_ang)*i
		near.append([x,y])

	print(near,"NUM")
	req_x1 = near[index][0]
	req_y1 = near[index][1]
	print("REQ",req_x1,req_y1)
	if sqrt((pos_x-req_x1)**2 + (pos_y-req_y1)**2)<0.3 and index!=len(near)-1:
		index+=1

	#######################		finding distance to next waypoint and the desired ang to waypoint	#############################

	d2 = sqrt((pos_x-0)**2 + (pos_y-0)**2)
	theta = atan2(pos_y,pos_x)
	d = sqrt((pos_x-req_x1)**2 + (pos_y-req_y1)**2)
	ang = atan2((pos_y-req_y1),(pos_x-req_x1))
	#theta2 = atan2((pos_y-d2*cos(theta)*sin(ang)), (pos_x))

	#######################		applying pid to the distance to waypoint to find desired lean angle	#############################

	kp1 = 2
	kp2 = 1.25
	kp3 = 0.00001

	xx = kp1*d + kp2*(d-pre_d) + kp3*sum_d
	print("XXXX",xx)

	set_r = abs(sin(ang))*(1.0*xx/180)*3.1417
	set_p = abs(cos(ang))*(1.0*xx/180)*3.1417

	#if d2*sin(theta)>0.25:


	#######################		reset constraints	#############################

	if abs(roll-set_r)<0.034:
		sum_r = 0
	if abs(pitch-set_p)<0.034:
		sum_p = 0

	if pos_x-req_x1>0:
		set_p = -set_p
	if pos_y-req_y1<0:
		set_r = -set_r

	if vel_x>0.2:
		set_p = -0.034
	if vel_x<-0.2:
		set_p = 0.034
	if vel_y>0.2:
		set_r = 0.034
	if vel_y<-0.2:
		set_r = -0.034

	#######################		replacing derivative and integral values	#############################

	#rospy.loginfo(rospy.get_caller_id() + "I heard yo ass %s, %s, %s", wx, wy, wz)

	erp = eroll
	epp = epitch
	eyp = eyaw
	sumr+=eroll
	sump+=epitch
	sumy+=eyaw
	sum_d+=d
	pre_d = d

	#######################		reset conditions	#############################

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

	#######################		publishing to wheel	#############################

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
