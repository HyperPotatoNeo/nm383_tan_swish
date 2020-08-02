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
    dashboard.setTotalAreaCovered(100)
    dashboard.setFireflyAreaCovered(200)
    dashboard.setPelicanAreaCovered(300)
    dashboard.setHummingBirdAreaCovered(400)
    dashboard.setIrisAreaCovered(500)
    dashboard.setNeoAreaCovered(600)
    

class Dashboard:
    def __init__(self, app, n_drones):
        self.n_drones = n_drones
        self.app = app
        self.app.title("ISRO DRONE COVERAGE PLANNER")

        self.allFrame = tk.Frame(self.app)
        self.allFrame.grid(row=0, sticky=tk.W)

        self.eachFrame = tk.Frame(self.app)
        self.eachFrame.grid(row=0, sticky=tk.E)

        self.app.columnconfigure(0, weight=2)
        self.app.columnconfigure(1, weight=1)

        self.totalAreaCovered = tk.StringVar()
        self.totalAreaCovered.set("Total Area Covered: ")
        self.totalLabel = tk.Label(self.allFrame, textvariable=self.totalAreaCovered, font=("Helvetica", 20), padx=50)
        self.totalLabel.grid(row=0, sticky=tk.W)

        self.fireflyFrame = tk.Frame(self.eachFrame)
        self.fireflyFrame.grid(row=0, sticky=tk.W)

        self.pelicanFrame = tk.Frame(self.eachFrame)
        self.pelicanFrame.grid(row=1, sticky=tk.W)

        self.hummingbirdFrame = tk.Frame(self.eachFrame)
        self.hummingbirdFrame.grid(row=2, sticky=tk.W)

        self.irisFrame = tk.Frame(self.eachFrame)
        self.irisFrame.grid(row=3, sticky=tk.W)

        self.neoFrame = tk.Frame(self.eachFrame)
        self.neoFrame.grid(row=4, sticky=tk.W)

        if 1 <= n_drones:
            self.fireflyArea = tk.StringVar()
            self.fireflyArea.set("Drone 1 Area Covered: ")
            self.fireflyAreaLabel = tk.Label(self.fireflyFrame, textvariable=self.fireflyArea, font=("Helvetica", 20))
            self.fireflyAreaLabel.grid(row=0, sticky=tk.W)

        if 2 <= n_drones:
            self.pelicanArea = tk.StringVar()
            self.pelicanArea.set("Drone 2 Area Covered: ")
            self.pelicanAreaLabel = tk.Label(self.pelicanFrame, textvariable=self.pelicanArea, font=("Helvetica", 20))
            self.pelicanAreaLabel.grid(row=0, sticky=tk.W)

        if 3 <= n_drones:
            self.hummingbirdArea = tk.StringVar()
            self.hummingbirdArea.set("Drone 3 Area Covered: ")
            self.hummingbirdAreaLabel = tk.Label(self.hummingbirdFrame, textvariable=self.hummingbirdArea, font=("Helvetica", 20))
            self.hummingbirdAreaLabel.grid(row=0, sticky=tk.W)
        if 4 <= n_drones:
            self.irisArea = tk.StringVar()
            self.irisArea.set("Drone 4 Area Covered: ")
            self.irisAreaLabel = tk.Label(self.irisFrame, textvariable=self.irisArea, font=("Helvetica", 20))
            self.irisAreaLabel.grid(row=0, sticky=tk.W)

        if 5 <= n_drones:
            self.neoArea = tk.StringVar()
            self.neoArea.set("Drone 5 Area Covered: ")
            self.neoAreaLabel = tk.Label(self.neoFrame, textvariable=self.neoArea, font=("Helvetica", 20))
            self.neoAreaLabel.grid(row=0, sticky=tk.W)

        self.eachFrame.rowconfigure(0, weight=2)
        self.eachFrame.rowconfigure(1, weight=2)
        self.eachFrame.rowconfigure(2, weight=2)
        self.eachFrame.rowconfigure(3, weight=2)
        self.eachFrame.rowconfigure(4, weight=2)



    def setFireflyAreaCovered(self, area):
        if 1 <= self.n_drones: 
            self.fireflyArea.set("Drone 1 Area Covered: " + str(area) + "%")

    def setPelicanAreaCovered(self, area):
        if 2 <= self.n_drones:
            self.pelicanArea.set("Drone 2 Area Covered: " + str(area) + "%")

    def setHummingBirdAreaCovered(self, area):
        if 3 <= self.n_drones:
            self.hummingbirdArea.set("Drone 3 Area Covered: " + str(area) + "%")

    def setIrisAreaCovered(self, area):
        if 4 <= self.n_drones:
            self.irisArea.set("Drone 4 Area Covered: " + str(area) + "%")
    
    def setNeoAreaCovered(self, area):
        if 5 <= self.n_drones:
            self.neoArea.set("Drone 5 Area Coverd: " + str(area) + "%")

    def setTotalAreaCovered(self, area):
        self.totalAreaCovered.set("Total Area Covered: "+ str(area) + "%")


if __name__ == '__main__':
    # rospy.init_node('dashboard', anonymous=True)
    # rospy.Subscriber("/drone/info", String, callback)

    app = tk.Tk()
    dashboard = Dashboard(app, 5)
    thread.start_new_thread(testCall, (dashboard,))
    app.mainloop()