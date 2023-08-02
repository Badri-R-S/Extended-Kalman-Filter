#! /usr/bin/env python3
'''
Node that creates auxiliary topics to be used along with the 1D Kalman Filter Example
- Gets the ground truth robot position from Gazebo Model State and publishes the x component to a specified topic
- Generates noise for the model's pose x component and publishes it to a specified topic
- Calls service to reset Gazebo model position  if the current x-axis pose exceed a certain threshold value
Author: Roberto Zegers R.
Usage: rosrun [package_name] auxiliary_topics.py
'''

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from tf.transformations import euler_from_quaternion
import numpy as np

noisy_odom_pub = rospy.Publisher('/noisy_odom', Float64MultiArray, queue_size=10)
alpha = 0.8
yaw_noise_range = 2  # Specify the range of yaw noise in degrees
def noisy_pub(data):
    global alpha
    rate = rospy.Rate(5)
    ground_truth_x = data.pose.pose.position.x
    ground_truth_y = data.pose.pose.position.y
    orientation = data.pose.pose.orientation
    orientation_list = [orientation.x,orientation.y,orientation.z,orientation.w]
    roll,pitch,yaw = euler_from_quaternion(orientation_list)
    ground_truth_yaw = yaw
    sd_x = abs(alpha*ground_truth_x)
    sd_y = abs(alpha*ground_truth_y)
    sd_yaw = abs(alpha*ground_truth_yaw)
    noisy_x = np.random.normal(ground_truth_x,sd_x)
    noisy_y = np.random.normal(ground_truth_y,sd_y)
    noisy_yaw = np.random.normal(ground_truth_yaw,sd_yaw)
    noisy_list = [noisy_x,noisy_y,noisy_yaw]
    pub_msg = Float64MultiArray()
    pub_msg.data = noisy_list
    noisy_odom_pub.publish(pub_msg)


if __name__ == "__main__":
    rospy.init_node('Publish_noisy_odom',anonymous=True)
    rospy.Subscriber('/odom',Odometry,noisy_pub)
    
    rospy.spin()