#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
import time
from math import pow, sqrt
from visualization_msgs.msg import Marker


class DronePose():
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0

    def calculateDistance(self, x, y, z):
        return sqrt(pow((self.x - x), 2) + pow((self.y - y), 2) + pow((self.z - z), 2))


THRESHOLD_DISTANCE = 5.0
COLOR = [1.0, 1.0, 1.0]
first_goal = True
nav_goal = None

goalArray = []
dronePose = DronePose()

fov = 0.2


def drone_path_callback(data):
    global goalArray
    goalArray = data.poses
    rate = rospy.Rate(4)
    rospy.Subscriber("/firefly/ground_truth/pose", Pose, drone_pose_callback)
    while not rospy.is_shutdown():
        navigate_drone()
        rate.sleep()

count = 0
publisher = None
def drone_pose_callback(data):
    global count, COLOR
    global dronePose
    dronePose.x = data.position.x
    dronePose.y = data.position.y
    dronePose.z = data.position.z
    ellipse = Marker()
    ellipse.id = count
    ellipse.header.frame_id = "map"
    ellipse.header.stamp = rospy.Time.now()
    ellipse.type = Marker.CUBE
    ellipse.action = Marker.ADD
    ellipse.pose.position.x = dronePose.x
    ellipse.pose.position.y = dronePose.y
    ellipse.pose.position.z = dronePose.z
    ellipse.pose.orientation.x = 0
    ellipse.pose.orientation.y = 0
    ellipse.pose.orientation.z = 0
    ellipse.pose.orientation.w = 1
    ellipse.scale.x = 2 * fov
    ellipse.scale.y = 2 * fov
    ellipse.scale.z = 0.1
    ellipse.color.a = 1.0
    ellipse.color.r = COLOR[0]
    ellipse.color.g = COLOR[1]
    ellipse.color.b = COLOR[2]
    ellipse.lifetime = rospy.Duration.from_sec(10000)
    # Publish the MarkerArray
    if(count%5==0):
        publisher.publish(ellipse)

    count += 1

firefly_pub = None
def sendGoal(goal):
    global firefly_pub
    waypoint = PoseStamped()

    waypoint.pose = goal
    waypoint.header.stamp = rospy.Time.now()
    waypoint.pose.position.z = 5

    waypoint.pose.orientation.x = 0.0
    waypoint.pose.orientation.y = 0.0
    waypoint.pose.orientation.z = 0.0
    waypoint.pose.orientation.w = 1.0

    # print("Moving firefly to X: {}, Y: {}, Z: {}".format(waypoint.pose.position.x, waypoint.pose.position.y, waypoint.pose.position.z))

    firefly_pub.publish(waypoint)


def navigate_drone():
    global goalArray
    global dronePose
    global first_goal
    global nav_goal
    if goalArray:
        # print('here', len(goalArray))
        if first_goal:
            nav_goal = goalArray[0]
            sendGoal(nav_goal)
            # print(goalArray)
            goalArray = goalArray[1:]
            first_goal = False
        else:
            distance = dronePose.calculateDistance(nav_goal.position.x, nav_goal.position.y, nav_goal.position.z)
            print("Distance to waypoint: {}".format(distance))
            nav_goal = goalArray[0]
            sendGoal(nav_goal)
            if distance < THRESHOLD_DISTANCE:
                goalArray = goalArray[1:]

if __name__ == "__main__":
    global THRESHOLD_DISTANCE, fov, COLOR
    rospy.init_node("tan_swish")
    THRESHOLD_DISTANCE = rospy.get_param("~speed")
    COLOR = eval(rospy.get_param("~color"))
    fov = rospy.get_param("~fov")
    rate = rospy.Rate(5)
    firefly_pub = rospy.Publisher("/firefly/command/pose", PoseStamped, queue_size=10)
    publisher = rospy.Publisher("/firefly/covers", Marker, queue_size=10)
    sub = rospy.Subscriber("/path/firefly", PoseArray, drone_path_callback)
    rospy.spin()
