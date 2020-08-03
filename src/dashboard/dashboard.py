#!/usr/bin/env python
# import rospy
# from std_msgs.msg import String
# import json
import Tkinter as tk
import thread as thread
import time

# def callback(data):
#     droneInfo = json.loads(data.data)
#     print(droneInfo)

def testCall(dashboard):
    time.sleep(4)
    dashboard.setBatteryPercentage(1, 45)

COLOR_WHITE = '#FFFFFF'
COLOR_GREY = '#F9F9F9'
COLOR_BATTERY = '#1ECB78'

def getDefaultFont(fontSize="24", style="bold"):
    return ("Times", fontSize, style)

class TotalAreaCoveredWidget(tk.Frame):
    def __init__(self, parent, *args, **kwargs):
        tk.Frame.__init__(self, parent, *args, **kwargs)
        self.label = tk.Label(self, text="Total Area Covered", font=getDefaultFont("15"))
        self.label.grid(row=0, column=0, sticky='nsew')
        self.areaVar = tk.StringVar()
        self.areaVar.set("0.0%")
        self.area = tk.Label(self, textvariable=self.areaVar, font=getDefaultFont("50"))
        self.area.grid(row=1, column=0, sticky='nsew')

        self.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure(0, weight=1)
        self.grid_rowconfigure(1, weight=1)
    
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
        self.timeTakenVar.set("Time Taken: 0 min 0 secs")
        self.timeTaken = tk.Label(self, textvariable=self.timeTakenVar, font=getDefaultFont())
        self.timeTaken.grid(row=1, sticky=tk.W)

        self.estimatedTimeVar = tk.StringVar()
        self.estimatedTimeVar.set("Estimated Time To Completion: 0 min 0 secs")
        self.estimatedTime = tk.Label(self, textvariable=self.estimatedTimeVar, font=getDefaultFont())
        self.estimatedTime.grid(row=2, sticky=tk.W)
    
    def setPercentageOverlap(self, overlap):
        self.overlapVar.set("Percentage Overlap: "+str(overlap)+"%")
    
    def setTimeTaken(self, time):
        self.timeTakenVar.set("Time Taken: ", + time)

    
    def setEstimatedTime(self, time):
        self.estimatedTimeVar.set("Estimated Time To Completion: " + time)
    

class TopBar(tk.Frame):
    def __init__(self, parent, *args, **kwargs):
        tk.Frame.__init__(self, parent, *args, **kwargs)
        
        self.areaWidget = TotalAreaCoveredWidget(self, padx=10, pady=10, bg=COLOR_WHITE)
        self.areaWidget.label['bg'] = COLOR_WHITE
        self.areaWidget.area['bg'] = COLOR_WHITE
        self.areaWidget.grid(row=0, column=0, sticky='nsew')

        self.commonWidget = CommonInfoWidget(self, padx=10, pady=10, bg=COLOR_GREY)
        self.commonWidget.overlap['bg'] = COLOR_GREY
        self.commonWidget.timeTaken['bg'] = COLOR_GREY
        self.commonWidget.estimatedTime['bg'] = COLOR_GREY
        self.commonWidget.grid(row=0, column=1, sticky='nsew')
        
        self.rowconfigure(0, weight=1)
        self.columnconfigure(0, weight=1)
        self.columnconfigure(1, weight=4)
    
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
    
    def setBatteryPercentage(self, percent):
        iPercent = int(percent)
        startY = 100 - iPercent + 20 
        self.delete('all')
        self.create_rectangle(10, startY, 50, 120, fill=COLOR_BATTERY)


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

        self.fovLabel = tk.Label(self, text="Field of View: ", font=getDefaultFont("12"), bg=bgLabels)
        self.fovLabel.grid(row=3, column=0, sticky=tk.W)
        self.fovVar = tk.StringVar()
        self.fovVar.set("0")
        self.fov = tk.Label(self, textvariable=self.fovVar, font=getDefaultFont("12", style=''), bg=bgLabels)
        self.fov.grid(row=3, column=1)

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
        self.batteryPercentVar.set("100%")
        self.batteryPercent = tk.Label(self, textvariable=self.batteryPercentVar, font=getDefaultFont("18"), bg=bgLabels)
        self.batteryPercent.grid(row=8, column=1, sticky=tk.W)

        self.columnconfigure(0, weight=2)
        self.columnconfigure(1, weight=3)

    def setAreaCovered(self, area):
        self.areaVar.set(str(area)+"%")
    
    def setFov(self, fov):
        self.fovVar.set(str(fov))
    
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
            self.drones.append(DroneInfoWidget(i+1, COLOR_GREY, self, padx=10, pady=5,  bg=COLOR_GREY))
            self.drones[i].grid(row=0, column=i, padx=5, sticky='nsew')
            self.columnconfigure(i, weight=1)
            self.rowconfigure(0, weight=1)
    
    def setAreaCovered(self, id, area):
        self.drones[id].setAreaCovered(area)
    
    def setFov(self, id, fov):
        self.drones[id].setFov(fov)
    
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
        self.topbar = TopBar(app, padx=10, pady=10, bg=COLOR_WHITE)
        self.topbar.grid(row=0, column=0, sticky='nsew')

        self.droneCollection = DroneCollection(n_drones, app, padx=30, pady=30, bg=COLOR_WHITE)
        self.droneCollection.grid(row=1, column=0, sticky='nsew')
        self.app.grid_rowconfigure(0, weight=1)
        self.app.grid_rowconfigure(1, weight=1)
        self.app.grid_columnconfigure(0, weight=1)

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
    
    def setFov(self, id, fov):
        self.droneCollection.setFov(id, fov)
    
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
    # rospy.init_node('dashboard', anonymous=True)
    # rospy.Subscriber("/drone/info", String, callback)

    app = tk.Tk()
    app.geometry("1280x700")
    dashboard = Dashboard(app, 4)
    thread.start_new_thread(testCall, (dashboard,))
    app.mainloop()