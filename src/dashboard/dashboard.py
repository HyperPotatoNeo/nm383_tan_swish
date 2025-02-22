#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Float32
from nav_msgs.msg import Odometry
import json
import Tkinter as tk
import thread as thread
import time
from math import sqrt
import sys

dashboard = None
start_time = None

def callback(data):    
    droneInfo = json.loads(data.data)
    # Total area
    percent = droneInfo["all"].get("percent_area", 0)
    dashboard.setTotalAreaCovered(round((percent * 100), 2))

    percent = droneInfo["all"].get("overlap_area", 0)
    dashboard.setPercentageOverlap(round((percent * 100), 2))

    percent = droneInfo["all"].get("n_drones",0)
    dashboard.setEstimatedTime(percent)

    # firefly percent area
    if dashboard.n_drones -1 >= 0:
        percent = droneInfo["firefly"].get("percent_area", 0)
        dashboard.setAreaCovered(0, round((percent * 100), 2))

    # pelican percent area
    if dashboard.n_drones -1 >= 1:
        percent = droneInfo["pelican"].get("percent_area", 0)
        dashboard.setAreaCovered(1, round((percent * 100), 2))

    # hummingbird percent area
    if dashboard.n_drones -1 >= 2:
        percent = droneInfo["hummingbird"].get("percent_area", 0)
        dashboard.setAreaCovered(2, round((percent * 100), 2))

    # iris percent area
    if dashboard.n_drones -1 >= 3:
        percent = droneInfo["iris"].get("percent_area", 0)
        dashboard.setAreaCovered(3, round((percent * 100), 2))

    # neo9 percent area
    if dashboard.n_drones -1 >= 4:
        percent = droneInfo["neo9"].get("percent_area", 0)
        dashboard.setAreaCovered(4, round((percent * 100), 2))

    seconds = (rospy.Time.now() - start_time).secs
    dashboard.setTimeTaken("{} minues {} seconds".format(seconds/60, seconds%60))


def battery_callback(msg, id):
    dashboard.setBatteryPercentage(id, round(msg.data, 1))

def distance_callback(msg, id):
    dashboard.setDistanceTravelled(id, round(msg.data, 2))

def odom_callback(msg, id):
    dashboard.setX(id, round(msg.pose.pose.position.x, 2))
    dashboard.setY(id, round(msg.pose.pose.position.y, 2))
    dashboard.setZ(id, round(msg.pose.pose.position.z, 2))

    vel = sqrt((msg.twist.twist.linear.x ** 2 + msg.twist.twist.linear.x ** 2)/2)
    dashboard.setVelocity(id, round(abs(vel), 2))


COLOR_WHITE = '#FFFFFF'
COLOR_GREY = '#F9F9F9'
COLOR_BATTERY = '#1ECB78'

def getDefaultFont(fontSize="24", style="bold"):
    return ("Times", fontSize, style)

class TotalAreaCoveredWidget(tk.Frame):
    def __init__(self, parent, *args, **kwargs):
        tk.Frame.__init__(self, parent, *args, **kwargs)
        self.label = tk.Label(self, text="Total Area Covered", font=getDefaultFont("15"))
        self.label.grid(row=0, sticky=tk.W)
        self.areaVar = tk.StringVar()
        self.areaVar.set("0.0%")
        self.area = tk.Label(self, textvariable=self.areaVar, font=getDefaultFont("50"))
        self.area.grid(row=1, sticky=tk.W)
    
    def setArea(self, area):
        self.areaVar.set(str(area)+"%")

class CommonInfoWidget(tk.Frame):
    def __init__(self, parent, *args, **kwargs):
        tk.Frame.__init__(self, parent, *args, **kwargs)
        
        self.overlapVar = tk.StringVar()
        self.overlapVar.set("Percentage Overlap: 0.0%")
        self.overlap = tk.Label(self, textvariable=self.overlapVar, font=getDefaultFont())
        self.overlap.grid(row=0, sticky=tk.W)
        
        self.timeTakenVar = tk.StringVar()
        self.timeTakenVar.set("Elapsed Time: 0 min 0 secs")
        self.timeTaken = tk.Label(self, textvariable=self.timeTakenVar, font=getDefaultFont())
        self.timeTaken.grid(row=1, sticky=tk.W)

        self.estimatedTimeVar = tk.StringVar()
        # self.estimatedTimeVar.set("Estimated Time To Completion: 0 min 0 secs")
        self.estimatedTimeVar.set("Number of drones: 0")
        self.estimatedTime = tk.Label(self, textvariable=self.estimatedTimeVar, font=getDefaultFont())
        self.estimatedTime.grid(row=2, sticky=tk.W)
    
    def setPercentageOverlap(self, overlap):
        self.overlapVar.set("Percentage Overlap: "+str(overlap)+"%")
    
    def setTimeTaken(self, time):
        self.timeTakenVar.set("Elapsed Time: {}".format(time))

    
    def setEstimatedTime(self, drones):
        # self.estimatedTimeVar.set("Estimated Time To Completion: " + time)
        self.estimatedTimeVar.set("Number of drones: {}".format(drones))
    

class TopBar(tk.Frame):
    def __init__(self, parent, *args, **kwargs):
        tk.Frame.__init__(self, parent, *args, **kwargs)
        
        self.areaWidget = TotalAreaCoveredWidget(self, padx=30, pady=30, bg=COLOR_WHITE)
        self.areaWidget.label['bg'] = COLOR_WHITE
        self.areaWidget.area['bg'] = COLOR_WHITE
        self.areaWidget.grid(row=0, column=0, sticky=tk.W)

        self.commonWidget = CommonInfoWidget(self, padx=30, pady=30, bg=COLOR_GREY)
        self.commonWidget.overlap['bg'] = COLOR_GREY
        self.commonWidget.timeTaken['bg'] = COLOR_GREY
        self.commonWidget.estimatedTime['bg'] = COLOR_GREY
        self.commonWidget.grid(row=0, column=1)
        
        self.columnconfigure(0, weight=1)
        self.columnconfigure(0, weight=2)
    
    def setTotalAreaCovered(self, area):
        self.areaWidget.setArea(area)
    
    def setPercentageOverlap(self, overlap):
        self.commonWidget.setPercentageOverlap(overlap)
    
    def setTimeTaken(self, time):
        self.commonWidget.setTimeTaken(time)
    
    def setEstimatedTime(self, time):
        self.commonWidget.setEstimatedTime(time)

class BatteryIndicator(tk.Canvas):
    def __init__(self, parent, *args, **kwargs):
        tk.Canvas.__init__(self, parent, width=60, height=120, *args, **kwargs)
        self.create_rectangle(10, 20, 50, 120, fill=COLOR_BATTERY)

    def getPercentColor(self, percent):
        RED = (220, 0, 0)
        YELLOW = (255, 220, 0)
        GREEN = (30, 203, 149)
        if(percent >= 80):
            return COLOR_BATTERY
        elif(percent <= 20):
            return "#%02x%02x%02x" % RED
        elif(percent >= 50):
            percent = ((percent - 50) * 2) / 100.0
            colorval = [int((1 - percent) * i + (percent * j)) for i,j in zip(YELLOW, GREEN)]
            return "#%02x%02x%02x" % tuple(colorval)
        else:
            percent = ((50 - percent) * 2) / 100.0
            colorval = [int((1 - percent) * i + (percent * j)) for i,j in zip(YELLOW, RED)]
            return "#%02x%02x%02x" % tuple(colorval)

    
    def setBatteryPercentage(self, percent):
        iPercent = int(percent)
        startY = 100 - iPercent + 20 
        self.delete('all')
        self.create_rectangle(10, startY, 50, 120, fill=self.getPercentColor(percent))


class DroneInfoWidget(tk.Frame):
    def __init__(self, id, bgLabels, parent, *args, **kwargs):
        tk.Frame.__init__(self, parent, *args, **kwargs)
        self.label = tk.Label(self, text="Drone "+str(id), font=getDefaultFont("18"), bg=bgLabels)
        self.label.grid(rowspan=2, columnspan=2)

        self.areaLabel = tk.Label(self, text="Area Covered: ", font=getDefaultFont("12"), bg=bgLabels)
        self.areaLabel.grid(row=2, column=0, sticky=tk.W)
        self.areaVar = tk.StringVar()
        self.areaVar.set("0%")
        self.area = tk.Label(self, textvariable=self.areaVar, font=getDefaultFont("12", style=''), bg=bgLabels)
        self.area.grid(row=2, column=1)

        self.dtLabel = tk.Label(self, text="Distance Travelled: ", font=getDefaultFont("12"), bg=bgLabels)
        self.dtLabel.grid(row=3, column=0, sticky=tk.W)
        self.dtVar = tk.StringVar()
        self.dtVar.set("0")
        self.dt = tk.Label(self, textvariable=self.dtVar, font=getDefaultFont("12", style=''), bg=bgLabels)
        self.dt.grid(row=3, column=1)

        self.velocityLabel = tk.Label(self, text="Velocity: ", font=getDefaultFont("12"), bg=bgLabels)
        self.velocityLabel.grid(row=4, column=0, sticky=tk.W)
        self.velocityVar = tk.StringVar()
        self.velocityVar.set("0")
        self.velocity = tk.Label(self, textvariable=self.velocityVar, font=getDefaultFont("12", style=''), bg=bgLabels)
        self.velocity.grid(row=4, column=1)

        self.xLabel = tk.Label(self, text="X: ", font=getDefaultFont("12"), bg=bgLabels)
        self.xLabel.grid(row=5, column=0, sticky=tk.W)
        self.xVar = tk.StringVar()
        self.xVar.set("0")
        self.x = tk.Label(self, textvariable=self.xVar, font=getDefaultFont("12", style=''), bg=bgLabels)
        self.x.grid(row=5, column=1)

        self.yLabel = tk.Label(self, text="Y: ", font=getDefaultFont("12"), bg=bgLabels)
        self.yLabel.grid(row=6, column=0, sticky=tk.W)
        self.yVar = tk.StringVar()
        self.yVar.set("0")
        self.y = tk.Label(self, textvariable=self.yVar, font=getDefaultFont("12", style=''), bg=bgLabels)
        self.y.grid(row=6, column=1)

        self.zLabel = tk.Label(self, text="Z: ", font=getDefaultFont("12"), bg=bgLabels)
        self.zLabel.grid(row=7, column=0, sticky=tk.W)
        self.zVar = tk.StringVar()
        self.zVar.set("0")
        self.z = tk.Label(self, textvariable=self.zVar, font=getDefaultFont("12", style=''), bg=bgLabels)
        self.z.grid(row=7, column=1)

        self.battery = BatteryIndicator(self, bg=COLOR_GREY)
        self.battery.grid(row=8, column=0)

        self.batteryPercentVar = tk.StringVar()
        self.batteryPercentVar.set("0%")
        self.batteryPercent = tk.Label(self, textvariable=self.batteryPercentVar, font=getDefaultFont("15"), bg=bgLabels)
        self.batteryPercent.grid(row=8, column=1)

    def setAreaCovered(self, area):
        self.areaVar.set(str(area)+"%")
    
    def setDistanceTravelled(self, distance):
        self.dtVar.set(str(distance))
    
    def setVelocity(self, velocity):
        self.velocityVar.set(str(velocity))
    
    def setX(self, x):
        self.xVar.set(str(x))
    
    def setY(self, y):
        self.yVar.set(str(y))
    
    def setZ(self, z):
        self.zVar.set(str(z))

    def setBatteryPercentage(self, percent):
        self.batteryPercentVar.set(str(percent)+"%")
        self.battery.setBatteryPercentage(percent)

class DroneCollection(tk.Frame):
    def __init__(self, n_drones, parent, *args, **kwargs):
        tk.Frame.__init__(self, parent, *args, **kwargs)
        self.n_drones = n_drones

        self.drones = []
        for i in range(n_drones):
            self.drones.append(DroneInfoWidget(i+1, COLOR_GREY, self, padx=10, pady=5, bg=COLOR_GREY))
            self.drones[i].grid(row=0, column=i, padx=5)
            self.columnconfigure(i, weight=1)
    
    def setAreaCovered(self, id, area):
        self.drones[id].setAreaCovered(area)
    
    def setDistanceTravelled(self, id, distance):
        self.drones[id].setDistanceTravelled(distance)
    
    def setVelocity(self, id, velocity):
        self.drones[id].setVelocity(velocity)
    
    def setX(self, id, x):
        self.drones[id].setX(x)
    
    def setY(self, id, y):
        self.drones[id].setY(y)
    
    def setZ(self, id, z):
        self.drones[id].setZ(z)
    
    def setBatteryPercentage(self, id, percent):
        self.drones[id].setBatteryPercentage(percent)

class Dashboard:
    def __init__(self, app, n_drones):
        self.app = app
        self.app['bg'] = COLOR_WHITE
        self.n_drones = n_drones
        self.topbar = TopBar(app, padx=30, pady=30, bg=COLOR_WHITE)
        self.topbar.pack(side=tk.TOP, fill=tk.X)

        self.droneCollection = DroneCollection(n_drones, app, padx=30, pady=0, bg=COLOR_WHITE)
        self.droneCollection.pack(side=tk.TOP, fill=tk.X)

    def setTotalAreaCovered(self, area):
        self.topbar.setTotalAreaCovered(area)
    
    def setPercentageOverlap(self, overlap):
        self.topbar.setPercentageOverlap(overlap)
    
    def setTimeTaken(self, time):
        self.topbar.setTimeTaken(time)

    def setEstimatedTime(self, time):
        self.topbar.setEstimatedTime(time)


    def setAreaCovered(self, id, area):
        self.droneCollection.setAreaCovered(id, area)
    
    def setDistanceTravelled(self, id, distance):
        self.droneCollection.setDistanceTravelled(id, distance)
    
    def setVelocity(self, id, velocity):
        self.droneCollection.setVelocity(id, velocity)
    
    def setX(self, id, x):
        self.droneCollection.setX(id, x)
    
    def setY(self, id, y):
        self.droneCollection.setY(id, y)
    
    def setZ(self, id, z):
        self.droneCollection.setZ(id, z)
    
    def setBatteryPercentage(self, id, percent):
        self.droneCollection.setBatteryPercentage(id, percent)

if __name__ == '__main__':
    rospy.init_node('dashboard', anonymous=True)
    start_time = rospy.Time.now()

    drone_count = 5
    try:
        drone_count = int(sys.argv[1])
        if(drone_count > 5):
            drone_count = 5
    except:
        drone_count = 5

    app = tk.Tk()
    app.title("Dashboard")
    app.geometry("1152x570")
    dashboard = Dashboard(app, drone_count)

    rospy.Subscriber("/drone_coverage_metrics", String, callback)
    if drone_count - 1 >= 0:
        rospy.Subscriber("/firefly/battery", Float32, battery_callback, 0)
        rospy.Subscriber("/firefly/distance_travelled", Float32, distance_callback, 0)
        rospy.Subscriber("/firefly/ground_truth/odometry", Odometry, odom_callback, 0)
    if drone_count - 1 >= 1:
        rospy.Subscriber("/pelican/battery", Float32, battery_callback, 1)
        rospy.Subscriber("/pelican/distance_travelled", Float32, distance_callback, 1)
        rospy.Subscriber("/pelican/ground_truth/odometry", Odometry, odom_callback, 1)
    if drone_count - 1 >= 2:
        rospy.Subscriber("/hummingbird/battery", Float32, battery_callback, 2)
        rospy.Subscriber("/hummingbird/distance_travelled", Float32, distance_callback, 2)
        rospy.Subscriber("/hummingbird/ground_truth/odometry", Odometry, odom_callback, 2)
    if drone_count - 1 >= 3:
        rospy.Subscriber("/iris/battery", Float32, battery_callback, 3)
        rospy.Subscriber("/iris/distance_travelled", Float32, distance_callback, 3)
        rospy.Subscriber("/iris/ground_truth/odometry", Odometry, odom_callback, 3)
    if drone_count - 1 >= 4:
        rospy.Subscriber("/neo9/battery", Float32, battery_callback, 4)
        rospy.Subscriber("/neo9/distance_travelled", Float32, distance_callback, 4)
        rospy.Subscriber("/neo9/ground_truth/odometry", Odometry, odom_callback, 4)

    app.mainloop()
