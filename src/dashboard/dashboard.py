#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import json
import Tkinter as tk

buttonA = None

def callback(data):
    global buttonA
    droneInfo = json.loads(data.data)
    print(droneInfo)
    buttonA["text"] = str(droneInfo["all"]["percent_area"]*100) + "%"

if __name__ == '__main__':
    app = tk.Tk()
    global buttonA
    buttonA = tk.Button(app, textvariable="")
    buttonA.pack()

    rospy.init_node('dashboard', anonymous=True)
    rospy.Subscriber("/drone_coverage_metrics", String, callback)

    app.mainloop()
