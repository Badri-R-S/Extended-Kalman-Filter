#! /usr/bin/env python3

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
import math
# Define system parameters
vx = 0.5  # Constant velocity
vz = 0.5 
x_prev = np.array([-1,1,np.pi/2])
v = math.sqrt(vx**2 + vz**2)
omega_maxx = 2.84
omega = (v/vz) * omega_maxx
prev_timestamp = None
current_timestamp = None
# Define initial state estimate and error covariance matrix
#x_init = np.array([R, 0, 0])  # Initial state estimate [x, y, theta]
P = np.eye(3)  # Initial error covariance matrix

# Define process noise and measurement noise covariance matrices
Q = np.eye(3) * 0.01  # Process noise covariance
R = np.eye(3) * 0.1  # Measurement noise covariance
rate = 0.5 # Time step for filtering
filtered_odom_pub = rospy.Publisher('/filtered_odom', Float64MultiArray, queue_size=10) #Publsiher to publish filtered odm


def filter_pub(data):
    global x_prev, P, Q, R, prev_timestamp, current_timestamp
    current_timestamp = rospy.Time.now()
    if prev_timestamp is None:
        dt = 0
    else:
        dt = (current_timestamp - prev_timestamp).to_sec()

    x_hat = np.array([
        x_prev[0] + (v * np.cos(x_prev[2]) * dt),
        x_prev[1] + (v * np.sin(x_prev[2]) * dt),
        x_prev[2] + (omega * dt)
    ])

    F = np.array([
        [1, 0, -v * dt * np.sin(x_prev[2])],
        [0, 1, v * dt * np.cos(x_prev[2])],
        [0, 0, 1]
    ])

    P = F @ P @ F.T + Q

    H = np.eye(3)

    z = np.array([data.data[0], data.data[1], data.data[2]])
    z_hat = np.array([x_hat[0], x_hat[1], x_hat[2]])
    y = z - z_hat

    S = H @ P @ H.T + R
    K_gain = P @ H.T @ np.linalg.inv(S)

    x_next = x_hat + (K_gain @ y)
    diff_squared = [(x_next[0] - z[0])**2,(x_next[1] - z[1])**2]
    rms_error = np.sqrt(sum(diff_squared)/2)
    print(rms_error)
    P = (np.eye(3) - K_gain @ H) @ P
    x_prev = x_next

    pub_msg = Float64MultiArray()
    pub_msg.data = x_next
    filtered_odom_pub.publish(pub_msg)

    prev_timestamp = current_timestamp


if __name__ == "__main__":
    rospy.init_node('Publish_filtered_odom',anonymous=True)
    filter_pub.prev_time = None
    rospy.Subscriber('/noisy_odom',Float64MultiArray, filter_pub)
    
    rospy.spin()