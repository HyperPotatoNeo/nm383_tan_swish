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