from __future__ import print_function
import Tkinter as tk
from PIL import Image, ImageTk

COLOR_GREY = '#F9F9F9'
COLOR_WHITE = '#FFFFFF'
COLOR_GREEN = '#1ECB78'

class CommonInfo(tk.Frame):
    def __init__(self, parent, bgLabels, *args, **kwargs):
        tk.Frame.__init__(self, parent, *args, **kwargs)

        self.overlapLabel = tk.Label(self, text='Overlap', padx=10, pady=10, bg=bgLabels)
        self.overlapLabel.grid(row=0, column=0, sticky=tk.E)
        self.overlapEntry = tk.Entry(self, bg=COLOR_WHITE)
        self.overlapEntry.grid(row=0, column=1, sticky='nsw', pady=30)

        self.fovLabel = tk.Label(self, text='FOV', padx=10, pady=10, bg=bgLabels)
        self.fovLabel.grid(row=1, column=0, sticky=tk.E)
        self.fovEntry = tk.Entry(self, bg=COLOR_WHITE)
        self.fovEntry.grid(row=1, column=1, sticky='nsw', pady=30)

        self.rechargeXlabel = tk.Label(self, text='Recharge X', padx=10, pady=10, bg=bgLabels)
        self.rechargeXlabel.grid(row=2, column=0, sticky=tk.E)
        self.rechargeXEntry = tk.Entry(self, bg=COLOR_WHITE)
        self.rechargeXEntry.grid(row=2, column=1, sticky='nsw', pady=30)

        self.rechargeYlabel = tk.Label(self, text='Recharge Y', padx=10, pady=10, bg=bgLabels)
        self.rechargeYlabel.grid(row=3, column=0, sticky=tk.E)
        self.rechargeYEntry = tk.Entry(self, bg=COLOR_WHITE)
        self.rechargeYEntry.grid(row=3, column=1, sticky='nsw', pady=30)

        self.grid_rowconfigure(0, weight=1)
        self.grid_rowconfigure(1, weight=1)
        self.grid_rowconfigure(2, weight=1)
        self.grid_rowconfigure(3, weight=1)
        self.grid_columnconfigure(0, weight=1)
        self.grid_columnconfigure(1, weight=3)

        self.columnconfigure(0, weight=1)
        self.columnconfigure(1, weight=3)
        

    def getOverlap(self):
        return self.overlapEntry.get()
    
    def getFOV(self):
        return self.fovEntry.get()
    
    def getRechargeX(self):
        return self.rechargeXEntry.get()
    
    def getRechargeY(self):
        return self.rechargeYEntry.get()

class DroneInfo(tk.Frame):
    def __init__(self, id, parent, bgLabels, *args, **kwargs):
        tk.Frame.__init__(self, parent, *args, **kwargs)
        self.id = id
        self.label = tk.Label(self, text='Drone '+str(id), padx=10, pady=10, bg=bgLabels)
        self.label.grid(row=0, column=0, columnspan=2)

        self.speedLabel = tk.Label(self, text='Speed', padx=10, pady=10, bg=bgLabels)
        self.speedLabel.grid(row=1, column=0, sticky=tk.E)
        self.speedEntry = tk.Entry(self, bg=COLOR_WHITE)
        self.speedEntry.grid(row=1, column=1, sticky='ew', padx=20)

        self.rangeLabel = tk.Label(self, text='Range', padx=10, pady=10, bg=bgLabels)
        self.rangeLabel.grid(row=2, column=0, sticky=tk.E)
        self.rangeEntry = tk.Entry(self, bg=COLOR_WHITE)
        self.rangeEntry.grid(row=2, column=1, sticky='ew', padx=20)

        self.grid_columnconfigure(0, weight=1)
        self.grid_columnconfigure(1, weight=2)
        
        self.grid_rowconfigure(0, weight=1)
        self.grid_rowconfigure(1, weight=1)
        self.grid_rowconfigure(2, weight=1)
    
    def getSpeed(self):
        return self.speedEntry.get()
    
    def getRange(self):
        return self.rangeEntry.get()

class DroneGroup(tk.Frame):
    def __init__(self, n_drones, parent, *args, **kwargs):
        tk.Frame.__init__(self, parent, *args, **kwargs)
        self.n_drones = n_drones
        self.drones = []
        for i in range(n_drones):
            self.drones.append(DroneInfo(i, self, COLOR_GREY, padx=10, pady=10, bg=COLOR_GREY))
            self.drones[i].grid(row=0, column=i, rowspan=n_drones, sticky='nsew', padx=10)
            self.grid_columnconfigure(i, weight=1)
        
        self.grid_rowconfigure(0, weight=1)
    
    def getSpeed(self, id):
        return self.drones[id].getSpeed()
    
    def getRange(self, id):
        return self.drones[id].getSpeed()


class CommandCentral:
    def __init__(self, app, n_drones):
        self.app = app
        self.app['bg'] = COLOR_WHITE
        self.n_drones = n_drones

        self.commonInfo = CommonInfo(app, COLOR_GREY, padx=10, pady=20, bg=COLOR_GREY)
        self.commonInfo.grid(row=0, column=0, sticky='nsew')

        self.droneGroup = DroneGroup(n_drones, app, padx=10, pady=10, bg=COLOR_WHITE)
        self.droneGroup.grid(row=1, column=0, sticky='nsew', columnspan=2)

        self.img = Image.open('./tan_swish.jpeg')
        self.img.thumbnail((300, 300), Image.ANTIALIAS )

        self.tkImg = ImageTk.PhotoImage(self.img)

        self.tan_swish = tk.Label(app, image=self.tkImg, bg=COLOR_GREY)
        self.tan_swish.img = self.tkImg
        self.tan_swish.grid(row=0, column=1, sticky='nsew')

        self.app.grid_rowconfigure(0, weight=1)
        self.app.grid_rowconfigure(1, weight=1)
        self.app.grid_columnconfigure(0, weight=1)
        self.app.grid_columnconfigure(1, weight=1)
    
    def setCallback(self, callback):
        self.launch = tk.Button(app, text='Launch', command=callback, padx=20, pady=20, fg=COLOR_GREEN)
        self.launch.grid(row=2, column=0, pady=10, columnspan=2)

    def getOverlap(self):
        return self.commonInfo.getOverlap()
    
    def getFOV(self):
        return self.commonInfo.getFOV()
    
    def getRechargeX(self):
        return self.commonInfo.getRechargeX()
    
    def getRechargeY(self):
        return self.commonInfo.getRechargeY()
    
    def getSpeed(self, id):
        return self.droneGroup.getSpeed(id)
    
    def getRange(self, id):
        return self.droneGroup.getRange(id)
    
def callback(central):
    print("HELLO")

if __name__ == '__main__':
    app = tk.Tk()
    app.geometry('1400x700')
    central = CommandCentral(app, 5)
    central.setCallback(lambda: callback(central))
    app.mainloop()



    

        