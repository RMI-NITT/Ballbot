#!/usr/bin/env python
import rospy
import tf
from std_msgs.msg import String
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from gazebo_msgs.msg import ModelStates
rospy.init_node('talker', anonymous=True)
pub1 = rospy.Publisher("/imu_joint1_controller/command", Float64, queue_size=10)
pub2 = rospy.Publisher("/imu_joint2_controller/command", Float64, queue_size=10)
pub3 = rospy.Publisher("/imu_joint3_controller/command", Float64, queue_size=10)
rate = rospy.Rate(10) # 10hz

def callback(data):
    
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.orientation)
    x =  data.orientation.x
    y =  data.orientation.y
    z =  data.orientation.z
    w =  data.orientation.w
    l = [x,y,z,w]
    euler = tf.transformations.euler_from_quaternion(l)
    rospy.loginfo(rospy.get_caller_id() + "I heard yo ass %s", euler)
    pub1.publish(euler[0]*50)
    pub2.publish(euler[0]*50)
    pub3.publish(euler[0]*50)
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
