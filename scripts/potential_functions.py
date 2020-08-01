#!/usr/bin/env python
import rospy
import time
import cv2
import sensor_msgs.msg
import numpy as np
import potential_func
import agent
import grid_world
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid

GRID_HEIGHT = 100
GRID_WIDTH = 100
NUM_AGENTS = 3
NUM_OBSTACLES = 0
NUM_STATIONS = 0

#Initial pos of bots
INITIAL_STATES = [(0, 0) for i in range(NUM_AGENTS)]
OBSTACLE_POS = []
RECHARGE_POS = ((250,250),)
DRONE_POS = [(0, 0) for i in range(NUM_AGENTS)]

DRONE_NAMES = ["firefly", "hummingbird", "pelican", "iris", "neo9"]

FIELD_GENERATORS = {
    'drones': DRONE_POS,
    'obstacles': OBSTACLE_POS,
    'recharge': RECHARGE_POS
}

potential_grid = None
position_grid = None
agent_list = None
time_passed = 0

map_recieved = False
map_arr = np.zeros((GRID_HEIGHT,GRID_WIDTH))

def generate_field():
    global potential_grid, agent_list, position_grid, FIELD_GENERATORS
    for j in range(len(agent_list)):
        #AGENT AND GRADIENTS YET TO BE COMPLETED
        # agent_list[j].agent_step(position_grid, potential_grid, FIELD_GENERATORS)

        potential_grid = potential_func.potential_compute(potential_grid, position_grid, FIELD_GENERATORS, agent_list, time_passed)

def callback(event):
    global publisher, potential_grid, map_arr, time_passed
    if map_recieved:
        res_field = (potential_grid[:,:,0] + potential_grid[:,:,1] - potential_grid[:,:,2]+255)/3
        for i in range(res_field.shape[0]):
            for j in range(res_field.shape[1]):
                #res_field[i,j] = np.min([res_field[i,j], 255])
                res_field[i,j] = np.min([res_field[i,j]+(map_arr[i,j]*255/100), 255])
        #img = np.int8(potential_grid[:,:,0])
        img = np.int8(np.flipud(np.fliplr(res_field)))

        generate_field()
        print(time_passed)

        time_passed += 1

        rosimage = sensor_msgs.msg.Image()
        rosimage.encoding = 'mono8'
        rosimage.width = img.shape[1]
        rosimage.height = img.shape[0]
        rosimage.step = img.strides[0] # For 1 Channel
        rosimage.data = img.tostring()
        rosimage.header.stamp = rospy.Time.now()
        rosimage.header.frame_id = 'map'
        publisher.publish(rosimage)


def position_callback(position, args):
    #rospy.loginfo(position)
    global GRID_HEIGHT, GRID_WIDTH
    topic = args[0]
    global DRONE_POS, FIELD_GENERATORS
    DRONE_POS[DRONE_NAMES.index(topic)] = (int(position.position.x/0.1+GRID_HEIGHT//2), int(position.position.y/0.1+GRID_WIDTH//2))
    FIELD_GENERATORS['drones'] = DRONE_POS



def map_callback(map):
    global GRID_HEIGHT, GRID_WIDTH, position_grid, potential_grid, agent_list, map_recieved, FIELD_GENERATORS, map_arr
    GRID_WIDTH = map.info.width
    GRID_HEIGHT = map.info.height
    agent_list = agent.create_agent(NUM_AGENTS, INITIAL_STATES)
    map_grid = np.array(map.data)
    map_arr = map_grid.reshape(GRID_HEIGHT, GRID_WIDTH)
    map_arr = map_arr.T
    #print('MAX', np.max(map_grid))
    #for i in range(map_arr.shape[0]):
    #    for j in range(map_arr.shape[1]):
    #        if(map_arr[i,j]==100):
    #            OBSTACLE_POS.append((i,j))

    #FIELD_GENERATORS['obstacles'] = OBSTACLE_POS

    position_grid, potential_grid = grid_world.create_grid(map_arr, GRID_HEIGHT, GRID_WIDTH, NUM_AGENTS, INITIAL_STATES)
    map_recieved = True
    rospy.loginfo("Map Recieved")
    map_sub.unregister()

# Main function initializes node and subscribers and starts the ROS loop
def initialize():
    global publisher, imagePath
    rospy.init_node('potential_publisher')
    topicName = rospy.get_param('~topic')
    publisher = rospy.Publisher(topicName, sensor_msgs.msg.Image, queue_size=10)
    drone_sub = []
    for i in range(NUM_AGENTS):
        drone_sub.append(rospy.Subscriber("" + DRONE_NAMES[i] + "/ground_truth/pose", Pose, position_callback, (DRONE_NAMES[i],)))

    global map_sub
    map_sub = rospy.Subscriber('/map', OccupancyGrid, map_callback)

    rospy.Timer(rospy.Duration(0.01), callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        initialize()
    except rospy.ROSInterruptException:
        pass






