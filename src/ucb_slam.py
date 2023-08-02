import math
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point,Twist
from nav_msgs.msg import Odometry
from tf import transformations
from itertools import islice

reward_set = {'up':0,'down':0,'east':0,'west':0,'ne':0,'nw':0,'se':0,'sw':0}    #Dictionary to store cumulative reward for each action
action_set = {'up':0.1,'down':0.1,'east':0.1,'west':0.1,'ne':0.1,'nw':0.1,'se':0.1,'sw':0.1}    #Dictionary to store number of times each action has been chosen
next_posn = {'up':[0,0.1],'down':[0,-0.1],'east':[0.1,0],'west':[-0.1,0],'ne':[0.1,0.1],'nw':[-0.1,0.1],'se':[0.1,-0.1],'sw':[-0.1,-0.1]}
node_list = []
c = 10
iter_time = 0
obs_coord = None
global obs_angle
robot_coord = None
global yaw
obstacle_coordinates = []
rew_time = 0
def laser_callback(scan_data):
    # Process the laser scan data to extract obstacle coordinates
    global obs_coord,obs_angle,obstacle_coordinates
    ranges = scan_data.ranges
    angle_min = scan_data.angle_min
    angle_increment = scan_data.angle_increment

    for i in range(len(ranges)):
        range_measurement = ranges[i]
        angle = angle_min + (i * angle_increment)

        if range_measurement > 0.0 and range_measurement < scan_data.range_max:
            obstacle_x = range_measurement * math.cos(angle)
            obstacle_y = range_measurement * math.sin(angle)
            obstacle_coordinates.append((obstacle_x, obstacle_y))
    #obstacle_coordinates.clear()

    #rospy.loginfo("Closest obstacle coordinates: (%f, %f)", obs_coord[0],obs_coord[1])    

def odom_callback(msg):
    # Extract the current position from the Odometry message
    global robot_coord,yaw
    position = msg.pose.pose.position
    robot_coord = [position.x,position.y]
    rot = msg.pose.pose.orientation
    (roll,pitch,yaw) = transformations.euler_from_quaternion([rot.x,rot.y,rot.z,rot.w])
    # Print the current position of the TurtleBot
    #rospy.loginfo("Current position: x = %f, y = %f, z = %f", position.x, position.y, position.z)    

class Node:
    def __init__(self):
        self.state = None
        self.parent = None

##Moving right###
def action_right(node):
    new_x = node.state[0] + next_posn['east'][0]
    new_y = node.state[1] + next_posn['east'][1]
    new_node = Node()
    new_node.state = [new_x,new_y]
    node_list.append(new_node)
    return new_node
#Moving left
def action_left(node):
    new_x = node.state[0] + next_posn['west'][0]
    new_y = node.state[1] + next_posn['west'][1]
    new_node = Node()
    new_node.state = [new_x,new_y]
    node_list.append(new_node)
    return new_node
#Moving up##
def action_up(node):
    new_x = node.state[0] + next_posn['up'][0]
    new_y = node.state[1] + next_posn['up'][1]
    new_node = Node()
    new_node.state = [new_x,new_y]
    node_list.append(new_node)
    return new_node
#Moving down###
def action_down(node):
    new_x = node.state[0] + next_posn['down'][0]
    new_y = node.state[1] + next_posn['down'][1]
    new_node = Node()
    new_node.state = [new_x,new_y]
    node_list.append(new_node)
    return new_node
#Moving NE##############
def action_ne(node):
    new_x = node.state[0] + next_posn['ne'][0]
    new_y = node.state[1] + next_posn['ne'][1]
    new_node = Node()
    new_node.state = [new_x,new_y]
    node_list.append(new_node)
    return new_node
#Moving NW###############
def action_nw(node):
    new_x = node.state[0] + next_posn['nw'][0]
    new_y = node.state[1] + next_posn['nw'][1]
    new_node = Node()
    new_node.state = [new_x,new_y]
    node_list.append(new_node)
    return new_node
#Moving Sw#############
def action_sw(node):
    new_x = node.state[0] + next_posn['sw'][0]
    new_y = node.state[1] + next_posn['sw'][1]
    new_node = Node()
    new_node.state = [new_x,new_y]
    node_list.append(new_node)
    return new_node
##MOving SE################
def action_se(node):
    new_x = node.state[0] + next_posn['se'][0]
    new_y = node.state[1] + next_posn['se'][1] 
    new_node = Node()
    new_node.state = [new_x,new_y]
    node_list.append(new_node)
    return new_node

def update_reward(node):
    global rew_time
    w_obs = 0.3
    w_rob = 0.7
    avg_dis = 0
    if obstacle_coordinates is not None:
        ob_dis = {}
        rob_dis = {}
        new_node = action_up(node)
        for (i,j) in obstacle_coordinates:    
            obs_dist = math.sqrt((new_node.state[0] - i)**2 + (new_node.state[1] - j)**2)
            avg_dis+=obs_dist
        ob_dis['up'] = avg_dis
        rob_dist = math.sqrt((new_node.state[0] - 0)**2 + (new_node.state[1] - 0)**2)
        rob_dis['up'] = rob_dist
        avg_dis = 0
        new_node = action_down(node)
        for (i,j) in obstacle_coordinates:    
            obs_dist = math.sqrt((new_node.state[0] - i)**2 + (new_node.state[1] - j)**2)
            avg_dis+=obs_dist
        ob_dis['down'] = avg_dis
        rob_dist = math.sqrt((new_node.state[0] - 0)**2 + (new_node.state[1] - 0)**2)
        rob_dis['down'] = rob_dist
        avg_dis = 0
        new_node = action_right(node)
        for (i,j) in obstacle_coordinates:    
            obs_dist = math.sqrt((new_node.state[0] - i)**2 + (new_node.state[1] - j)**2)
            avg_dis+=obs_dist
        ob_dis['east'] = avg_dis
        rob_dist = math.sqrt((new_node.state[0] - 0)**2 + (new_node.state[1] - 0)**2)
        rob_dis['east'] = rob_dist
        avg_dis = 0
        new_node = action_left(node)
        for (i,j) in obstacle_coordinates:    
            obs_dist = math.sqrt((new_node.state[0] - i)**2 + (new_node.state[1] - j)**2)
            avg_dis+=obs_dist
        ob_dis['west'] = avg_dis
        rob_dist = math.sqrt((new_node.state[0] - 0)**2 + (new_node.state[1] - 0)**2)
        rob_dis['west'] = rob_dist
        avg_dis = 0
        new_node = action_ne(node)
        for (i,j) in obstacle_coordinates:    
            obs_dist = math.sqrt((new_node.state[0] - i)**2 + (new_node.state[1] - j)**2)
            avg_dis+=obs_dist
        ob_dis['ne'] = avg_dis
        rob_dist = math.sqrt((new_node.state[0] - 0)**2 + (new_node.state[1] - 0)**2)
        rob_dis['ne'] = rob_dist
        avg_dis = 0
        new_node = action_nw(node)
        for (i,j) in obstacle_coordinates:    
            obs_dist = math.sqrt((new_node.state[0] - i)**2 + (new_node.state[1] - j)**2)
            avg_dis+=obs_dist
        ob_dis['nw'] = avg_dis
        rob_dist = math.sqrt((new_node.state[0] - 0)**2 + (new_node.state[1] - 0)**2)
        rob_dis['nw'] = rob_dist
        avg_dis =0
        new_node = action_se(node)
        for (i,j) in obstacle_coordinates:    
            obs_dist = math.sqrt((new_node.state[0] - i)**2 + (new_node.state[1] - j)**2)
            avg_dis+=obs_dist
        ob_dis['se'] = avg_dis
        rob_dist = math.sqrt((new_node.state[0] - 0)**2 + (new_node.state[1] - 0)**2)
        rob_dis['se'] = rob_dist
        avg_dis = 0
        new_node = action_sw(node)
        for (i,j) in obstacle_coordinates:    
            obs_dist = math.sqrt((new_node.state[0] - i)**2 + (new_node.state[1] - j)**2)
            avg_dis+=obs_dist
        ob_dis['sw'] = avg_dis
        rob_dist = math.sqrt((new_node.state[0] - 0)**2 + (new_node.state[1] - 0)**2)
        rob_dis['sw'] = rob_dist
        max_key_rob = max(rob_dis,key=rob_dis.get)
        max_key_obs = max(ob_dis,key = ob_dis.get)
        d_max_obs = ob_dis[max_key_obs]
        d_max_rob = rob_dis[max_key_rob]
        print("Max rob dis:",d_max_rob)
        min_key_obs = min(ob_dis, key = ob_dis.get)
        min_key_rob = min(rob_dis,key= rob_dis.get)
        d_min_obs = ob_dis[min_key_obs]
        d_min_rob = rob_dis[min_key_rob]
        print("Min rob dis:",d_min_rob)

        reward_set['up'] += ((ob_dis['up'] - d_min_obs) / (d_max_obs - d_min_obs))*w_obs + ((rob_dis['up'] - d_min_rob) / (d_max_rob - d_min_rob))*w_rob
        reward_set['down'] += ((ob_dis['down'] - d_min_obs) / (d_max_obs - d_min_obs))*w_obs + ((rob_dis['down'] - d_min_rob) / (d_max_rob - d_min_rob))*w_rob
        reward_set['east'] += ((ob_dis['east'] - d_min_obs) / (d_max_obs - d_min_obs))*w_obs + ((rob_dis['east'] - d_min_rob) / (d_max_rob - d_min_rob))*w_rob
        reward_set['west'] += ((ob_dis['west'] - d_min_obs) / (d_max_obs - d_min_obs))*w_obs + ((rob_dis['west'] - d_min_rob) / (d_max_rob - d_min_rob))*w_rob
        reward_set['ne'] += ((ob_dis['ne'] - d_min_obs) / (d_max_obs - d_min_obs))*w_obs + ((rob_dis['ne'] - d_min_rob) / (d_max_rob - d_min_rob))*w_rob
        reward_set['nw'] += ((ob_dis['nw'] - d_min_obs) / (d_max_obs - d_min_obs))*w_obs + ((rob_dis['nw'] - d_min_rob) / (d_max_rob - d_min_rob))*w_rob
        reward_set['se'] += ((ob_dis['se'] - d_min_obs) / (d_max_obs - d_min_obs))*w_obs + ((rob_dis['se'] - d_min_rob) / (d_max_rob - d_min_rob))*w_rob
        reward_set['sw'] += ((ob_dis['sw'] - d_min_obs) / (d_max_obs - d_min_obs))*w_obs + ((rob_dis['sw'] - d_min_rob) / (d_max_rob - d_min_rob))*w_rob
    else:
        print("Waiting for obstacle coordinates...")
        rospy.sleep(0.1)  # Wait for 0.1 seconds before checking again
        update_reward(node)  # Call the function recursively until obstacle coordinates are available
    rew_time+=1
    

def choose_action(time):
    actions = {}
    actions['up'] = reward_set['up']+ c*math.sqrt(math.log(time)/action_set['up'])
    print("UCB for up:",actions['up'])
    actions['down'] = reward_set['down']+ c*math.sqrt(math.log(time)/action_set['down'])
    print("UCB for down:",actions['down'])
    actions['east'] = reward_set['east']+ c*math.sqrt(math.log(time)/action_set['east'])
    print("UCB for east:",actions['east'])
    actions['west'] = reward_set['west']+ c*math.sqrt(math.log(time)/action_set['west'])
    print("UCB for west:",actions['west'])
    actions['ne'] = reward_set['ne']+ c*math.sqrt(math.log(time)/action_set['ne'])
    print("UCB for ne:",actions['ne'])
    actions['nw'] = reward_set['nw']+ c*math.sqrt(math.log(time)/action_set['nw'])
    print("UCB for nw:",actions['nw'])
    actions['se'] = reward_set['se']+ c*math.sqrt(math.log(time)/action_set['se'])
    print("UCB for se:",actions['se'])
    actions['sw'] = reward_set['sw']+ c*math.sqrt(math.log(time)/action_set['sw'])
    print("UCB for sw:",actions['sw'])
    sorted_dict = sorted(actions.items(), key=lambda item: item[1])
    (mid_key,mid_val) = sorted_dict[7]  
    action_set[mid_key] = int(action_set[mid_key] + 1)
    return mid_key,mid_val

def move_turtlebot(max_key,velocity_publisher,node):
    global robot_coord,yaw
    speed = Twist()
    if robot_coord is not None:
        while True:
            angle_to_goal = math.atan2(node.state[1] + next_posn[max_key][1] - robot_coord[1], node.state[0] + next_posn[max_key][0] - robot_coord[0])
            dist = math.sqrt((node.state[0] + next_posn[max_key][0] - robot_coord[0])**2+(node.state[1] + next_posn[max_key][1] - robot_coord[1])**2)
            #print("distance to target:",dist)
            if(dist<0.1):        
                speed.linear.x = 0.0
                speed.angular.z = 0.0
                velocity_publisher.publish(speed)
                break
            else:
                #print("Angle:",math.atan2(node.state[1] + next_posn[max_key][1] - robot_coord[1],node.state[0] + next_posn[max_key][0] - robot_coord[0]) - yaw)
                #print("Distance:",math.sqrt((node.state[0] + next_posn[max_key][0] - robot_coord[0])**2+(node.state[1] + next_posn[max_key][1] - robot_coord[1])**2))
                if(abs(math.atan2(node.state[1] + next_posn[max_key][1] - robot_coord[1],node.state[0] + next_posn[max_key][0] - robot_coord[0]) - yaw) >0.2):
                    #print("angle > 0.2")
                    speed.linear.x = 0.0
                    speed.angular.z = (math.atan2(node.state[1] + next_posn[max_key][1] - robot_coord[1],node.state[0] + next_posn[max_key][0] - robot_coord[0]) - yaw)*0.5            
                    #print("Publishing to rotate")
                else:
                    #print("Moving forward")
                    speed.angular.z = 0.0 
                    speed.linear.x = math.sqrt((node.state[0] + next_posn[max_key][0] - robot_coord[0])**2+(node.state[1] + next_posn[max_key][1] - robot_coord[1])**2)*0.5
                    #print("Publishing to move forward")
                velocity_publisher.publish(speed)            
    else:
        print("Waiting for robot coordinates...")
        rospy.sleep(0.1)  # Wait for 0.1 seconds before checking again
        move_turtlebot(max_key,velocity_publisher,node)  # Call the function recursively until obstacle coordinates are available        


def ucb_slam(node,velocity_publisher,iter_time):    #Number of iterations
    iter_time +=1   
    max_key,a =choose_action(iter_time) #Best key and value.
    print("Choosing action:",max_key)
    #rospy.sleep(1)
    move_turtlebot(max_key,velocity_publisher,node)
    next_node = Node()  
    next_node.state = [node.state[0] + next_posn[max_key][0],node.state[1] + next_posn[max_key][1]]
    next_node.parent = node.state
    update_reward(next_node)
    node_list.clear()
    ucb_slam(next_node,velocity_publisher,iter_time)



if __name__ == '__main__':
    rospy.init_node('multiple_subscribers_node')

    # Create laser scan subscriber
    rospy.Subscriber('/scan', LaserScan, laser_callback)

    # Create odometry subscriber
    rospy.Subscriber('/odom', Odometry, odom_callback)
    velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    x_pos = rospy.get_param('~x_pos', 0.0)
    y_pos = rospy.get_param('~y_pos', 0.0)
    z_pos = rospy.get_param('~z_pos', 0.0)
    node = Node()
    node.state = [x_pos,y_pos]
    node.parent = None
    ucb_slam(node,velocity_publisher,0)
    rospy.spin()    