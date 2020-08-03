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
