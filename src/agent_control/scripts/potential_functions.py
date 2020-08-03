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
from std_msgs.msg import String
import json
from cv_bridge import CvBridge
import rospkg

GRID_HEIGHT = 100
GRID_WIDTH = 100
NUM_AGENTS = 5
NUM_OBSTACLES = 0
NUM_STATIONS = 0
RESOLUTION = 0.02
DOWNSAMPLE_FACTOR = 0.25
MAP_ORIGIN = [0.0, 0.0, 0.0]

#Initial pos of bots
INITIAL_STATES = [(0, 0) for i in range(NUM_AGENTS)]
OBSTACLE_POS = []
RECHARGE_POS = []#((250,250),)
DRONE_POS = [(0, 0) for i in range(NUM_AGENTS)]

DRONE_NAMES = ["firefly", "hummingbird", "pelican", "iris", "neo9"]

FIELD_GENERATORS = {
    'drones': DRONE_POS,
    'obstacles': OBSTACLE_POS,
    'recharge': RECHARGE_POS
}

METRICS = {}
METRICS["all"] = {}
METRICS["all"]["n_drones"] = NUM_AGENTS
METRICS["all"]["percent_area"] = 0
METRICS["all"]["overlap_area"] = 0
for i in DRONE_NAMES:
    METRICS[i] = {}
    METRICS[i]["percent_area"] = 0

visited_grid = None
total_visited = None
total_area = 0
potential_grid = None
position_grid = None
agent_list = None
time_passed = 0

area_image = cv2.imread(rospkg.RosPack().get_path('tan_swish')+'/cfg/true_img.jpg')

map_recieved = False
map_arr = np.zeros((GRID_HEIGHT,GRID_WIDTH))

def generate_field():
    global potential_grid, agent_list, position_grid, FIELD_GENERATORS
    potential_grid = potential_func.potential_compute(potential_grid, position_grid, FIELD_GENERATORS, agent_list, time_passed)

def callback(event):
    global publisher, potential_grid, map_arr, time_passed
    if map_recieved:
        res_field = (potential_grid[:,:,0] + potential_grid[:,:,1] - potential_grid[:,:,2]+255)/3
        res_field = np.minimum(res_field + (map_arr * 255/100), np.full(res_field.shape, 255))   
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
    global GRID_HEIGHT, GRID_WIDTH, visited_grid, total_visited, metrics_pub, map_arr, area_image, image_pub
    topic = args[0]
    global DRONE_POS, FIELD_GENERATORS
    DRONE_POS[DRONE_NAMES.index(topic)] = (int((position.position.x-MAP_ORIGIN[0])/0.02*DOWNSAMPLE_FACTOR), int((position.position.y-MAP_ORIGIN[1])/0.02*DOWNSAMPLE_FACTOR))
    FIELD_GENERATORS['drones'] = DRONE_POS

    for i in range(int((position.position.x-MAP_ORIGIN[0])/0.02*DOWNSAMPLE_FACTOR-7),int((position.position.x-MAP_ORIGIN[0])/0.02*DOWNSAMPLE_FACTOR+8)):
        for j in range(int((position.position.y-MAP_ORIGIN[1])/0.02*DOWNSAMPLE_FACTOR-7),int((position.position.y-MAP_ORIGIN[1])/0.02*DOWNSAMPLE_FACTOR+8)):
            if(i>=0 and j>=0 and i<map_arr.shape[0] and j<map_arr.shape[1]):
                if(map_arr[i,j]<255):
                    visited_grid[DRONE_NAMES.index(topic),i,j] = 1
                    total_visited[i,j] = 1
    rospy.loginfo(total_visited.shape)
    p_area = np.sum(visited_grid[DRONE_NAMES.index(topic)])/(total_area)
    rospy.loginfo("Drone " + str(DRONE_NAMES.index(topic)) + " percentage of total area covered: " + str(p_area))
    METRICS[topic]["percent_area"] = p_area
    METRICS["all"]["percent_area"] = np.sum(total_visited)/total_area
    tot=0
    for i in range(NUM_AGENTS):
        tot += METRICS[DRONE_NAMES[i]]["percent_area"]
    METRICS["all"]["overlap_area"] = (tot - METRICS["all"]["percent_area"])/tot
    d_str = json.dumps(METRICS)
    metrics_pub.publish(d_str)
    revealed_img = np.zeros((area_image.shape[0], area_image.shape[1], 3))
    revealed_img[:,:,0] = area_image[:,:,0]*total_visited
    revealed_img[:,:,1] = area_image[:,:,1]*total_visited
    revealed_img[:,:,2] = area_image[:,:,2]*total_visited
    revealed_img = np.uint8(revealed_img)
    image_pub.publish(CvBridge().cv2_to_imgmsg(revealed_img, "bgr8"))


def map_callback(map):
    global GRID_HEIGHT, GRID_WIDTH, position_grid, potential_grid, agent_list, map_recieved, FIELD_GENERATORS, map_arr, RESOLUTION, MAP_ORIGIN, visited_grid, total_area, total_visited, area_image
    GRID_WIDTH = map.info.width
    GRID_HEIGHT = map.info.height
    RESOLUTION = map.info.resolution
    MAP_ORIGIN = [map.info.origin.position.x, map.info.origin.position.y, map.info.origin.position.z]
    agent_list = agent.create_agent(NUM_AGENTS, INITIAL_STATES) 
    map_arr = np.array(map.data).reshape(GRID_HEIGHT, GRID_WIDTH)
    GRID_WIDTH = int(map.info.width * DOWNSAMPLE_FACTOR)
    GRID_HEIGHT = int(map.info.height * DOWNSAMPLE_FACTOR)
    map_arr[map_arr < 0] = 255
    map_arr[map_arr >= 100] = 255
    map_arr[map_arr < 255] = 0
    map_arr = cv2.resize(np.array(map_arr, dtype='float32'), None, fx = DOWNSAMPLE_FACTOR, fy = DOWNSAMPLE_FACTOR, interpolation = cv2.INTER_NEAREST).T
    if (map_arr.shape[1] != GRID_HEIGHT):
        map_arr = map_arr[:, 0:-1]
    #print('MAX', np.max(map_grid))
    #for i in range(map_arr.shape[0]):
    #    for j in range(map_arr.shape[1]):
    #        if(map_arr[i,j]==100):
    #            OBSTACLE_POS.append((i,j))

    #FIELD_GENERATORS['obstacles'] = OBSTACLE_POS
    area_image = cv2.resize(area_image, (map_arr.shape[1], map_arr.shape[0]))
    #area_image = cv2.imread("dota2.jpg")
    total_area = map_arr.shape[0]*map_arr.shape[1] - np.count_nonzero(map_arr)
    visited_grid =  np.zeros((NUM_AGENTS, map_arr.shape[0], map_arr.shape[1]))#np.zeros((NUM_AGENTS, GRID_HEIGHT, GRID_WIDTH))
    total_visited = np.zeros((map_arr.shape[0], map_arr.shape[1]))#np.zeros((GRID_HEIGHT, GRID_WIDTH))
    position_grid, potential_grid = grid_world.create_grid(map_arr, GRID_HEIGHT, GRID_WIDTH, NUM_AGENTS, INITIAL_STATES)
    map_recieved = True
    rospy.loginfo("Map Recieved")
    map_sub.unregister()

# Main function initializes node and subscribers and starts the ROS loop
def initialize():
    global publisher, imagePath, metrics_pub, image_pub
    rospy.init_node('potential_publisher')
    topicName = rospy.get_param('~topic')
    publisher = rospy.Publisher(topicName, sensor_msgs.msg.Image, queue_size=20)
    metrics_pub = rospy.Publisher("/drone_coverage_metrics", String, queue_size=10)
    image_pub = rospy.Publisher("/mapped_area", sensor_msgs.msg.Image, queue_size=20)
    drone_sub = []
    for i in range(NUM_AGENTS):
        drone_sub.append(rospy.Subscriber("" + DRONE_NAMES[i] + "/ground_truth/pose", Pose, position_callback, (DRONE_NAMES[i],), queue_size=5))

    global map_sub
    map_sub = rospy.Subscriber('/map', OccupancyGrid, map_callback)

    rospy.Timer(rospy.Duration(0.01), callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        initialize()
    except rospy.ROSInterruptException:
        pass

