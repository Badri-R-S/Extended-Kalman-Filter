import math
import numpy as np
import rospy
import tf
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point,Twist,PoseStamped,Pose
from nav_msgs.msg import Odometry,OccupancyGrid
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
global grid_data,grid_width,grid_height,resolution,transformed_position

def occupancy_grid_callback(msg):
    # Extract occupancy grid data from the message
    global grid_data,grid_width,grid_height,resolution
    grid_data = msg.data
    grid_width = msg.info.width
    grid_height = msg.info.height
    resolution = msg.info.resolution

    # Process the occupancy grid data
    for y in range(grid_height):
        for x in range(grid_width):
            # Calculate the index of the current cell in the occupancy grid
            cell_index = y * grid_width + x
            
            # Access the occupancy probability value of the current cell
            occupancy_prob = grid_data[cell_index]
            
            # Do further processing with the occupancy probability value
            # You can check if the cell is occupied, free, or unexplored based on the probability value
            # Perform actions or update your exploration strategy based on this information


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
    convert_to_occu_coord(msg)    


def convert_to_occu_coord(robot_pose):
    # Create a listener for TF transformations
    global transformed_position
    listener = tf.TransformListener()

    try:
        # Wait for the transformation to be available
        listener.waitForTransform('/map', robot_pose.header.frame_id, rospy.Time(), rospy.Duration(1.0))
        
        # Perform the transformation
        transformed_pose = listener.transformPose('/map', robot_pose)
        
        # Extract the transformed position
        transformed_position = transformed_pose.pose.position

        # Do further processing with the transformed position
        # ...
        return transformed_position
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logwarn("TF transformation failed: %s" % str(e))    

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

def convert_to_Posestamped(node):
    # Create a new PoseStamped message
    pose_stamped = PoseStamped()

    # Set the frame ID (coordinate frame) of the pose
    pose_stamped.header.frame_id = "map"  # Replace "map" with the desired frame ID

    # Create a new Pose message
    pose = Pose()

    # Set the position and orientation of the pose
    pose.position.x = node.state[0]  # Replace with the desired x-coordinate
    pose.position.y = node.state[1]  # Replace with the desired y-coordinate
    pose.position.z = 0.0  # Replace with the desired z-coordinate

    pose.orientation.x = 0.0  # Replace with the desired x-component of the quaternion
    pose.orientation.y = 0.0  # Replace with the desired y-component of the quaternion
    pose.orientation.z = 0.0  # Replace with the desired z-component of the quaternion
    pose.orientation.w = 1.0  # Replace with the desired w-component of the quaternion

    # Assign the pose to the PoseStamped message
    pose_stamped.pose = pose
    return pose_stamped    

def update_reward(node):
    new_node = action_up(node)
    pose_stamped_node = convert_to_Posestamped(new_node)
    


    

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

    rospy.Subscriber('/map', OccupancyGrid, occupancy_grid_callback)


    velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    x_pos = rospy.get_param('~x_pos', 0.0)
    y_pos = rospy.get_param('~y_pos', 0.0)
    z_pos = rospy.get_param('~z_pos', 0.0)
    node = Node()
    node.state = [x_pos,y_pos]
    node.parent = None
    ucb_slam(node,velocity_publisher,0)
    rospy.spin()    