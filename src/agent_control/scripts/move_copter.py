#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
import time
from math import pow, sqrt
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud


class DronePose():
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0

    def calculateDistance(self, x, y, z):
        return sqrt(pow((self.x - x), 2) + pow((self.y - y), 2) + pow((self.z - z), 2))


THRESHOLD_DISTANCE = 1.0
first_goal = True
nav_goal = None

goalArray = []
dronePose = DronePose()
pointCloud = PointCloud()


def drone_path_callback(data):
    global goalArray
    goalArray = data.poses
    rate = rospy.Rate(4)
    while not rospy.is_shutdown():
        rospy.Subscriber("/firefly/ground_truth/pose", Pose, drone_pose_callback)
        navigate_drone()
        rate.sleep()

count = 0
publisher = None
def drone_pose_callback(data):
    global count
    global dronePose
    global pointCloud

    dronePose.x = data.position.x
    dronePose.y = data.position.y
    dronePose.z = data.position.z

    if(count % 5000 == 0):
        pointCloud.header.frame_id = "map"
        pointCloud.header.stamp = rospy.Time.now()
        point = Point32()
        point.x = dronePose.x
        point.y = dronePose.y
        point.z = dronePose.z
        pointCloud.points.append(point)

    count += 1

    # Publish the PointCloud
    publisher.publish(pointCloud)

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

    print("Moving firefly to X: {}, Y: {}, Z: {}".format(waypoint.pose.position.x, waypoint.pose.position.y, waypoint.pose.position.z))

    firefly_pub.publish(waypoint)


def navigate_drone():
    global goalArray
    global dronePose
    global first_goal
    global nav_goal
    if goalArray:
        print('here', len(goalArray))
        if first_goal:
            nav_goal = goalArray[0]
            sendGoal(nav_goal)
            print(goalArray)
            goalArray = goalArray[1:]
            first_goal = False
        else:
            distance = dronePose.calculateDistance(nav_goal.position.x, nav_goal.position.y, nav_goal.position.z)
            print("Distance to waypoint: {}".format(distance))
            if distance < THRESHOLD_DISTANCE:
                nav_goal = goalArray[0]
                sendGoal(nav_goal)
                goalArray = goalArray[1:]

if __name__ == "__main__":
    rospy.init_node("tan_swish")
    rate = rospy.Rate(5)
    firefly_pub = rospy.Publisher("/firefly/command/pose", PoseStamped, queue_size=10)
    publisher = rospy.Publisher("/firefly/covers", PointCloud, queue_size=10)
    sub = rospy.Subscriber("/path/firefly", PoseArray, drone_path_callback)
    rospy.spin()
