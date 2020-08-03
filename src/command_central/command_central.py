import Tkinter as tk

COLOR_GREY = '#F9F9F9'
COLOR_WHITE = '#FFFFFF'

class CommonInfo(tk.Frame):
    def __init__(self, parent, bgLabels, *args, **kwargs):
        tk.Frame.__init__(self, parent, *args, **kwargs)

        self.overlapLabel = tk.Label(self, text='Overlap', padx=10, pady=10, bg=bgLabels)
        self.overlapLabel.grid(row=0, column=0, sticky=tk.E)
        self.overlapEntry = tk.Entry(self, bg=COLOR_WHITE)
        self.overlapEntry.grid(row=0, column=1, sticky='nsew')

        self.fovLabel = tk.Label(self, text='FOV', padx=10, pady=10, bg=bgLabels)
        self.fovLabel.grid(row=1, column=0, sticky=tk.E)
        self.fovEntry = tk.Entry(self, bg=COLOR_WHITE)
        self.fovEntry.grid(row=1, column=1, sticky='nsew')

        self.rechargeXlabel = tk.Label(self, text='Recharge X', padx=10, pady=10, bg=bgLabels)
        self.rechargeXlabel.grid(row=2, column=0, sticky=tk.E)
        self.rechargeXEntry = tk.Entry(self, bg=COLOR_WHITE)
        self.rechargeXEntry.grid(row=2, column=1, sticky='nsew')

        self.rechargeYlabel = tk.Label(self, text='Recharge Y', padx=10, pady=10, bg=bgLabels)
        self.rechargeYlabel.grid(row=3, column=0, sticky=tk.E)
        self.rechargeYEntry = tk.Entry(self, bg=COLOR_WHITE)
        self.rechargeYEntry.grid(row=3, column=1, sticky='nsew')

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
    
class CommandCentral:
    def __init__(self, app, n_drones):
        self.app = app
        self.app['bg'] = COLOR_WHITE
        self.n_drones = n_drones

        self.commonInfo = CommonInfo(app, COLOR_GREY, padx=10, pady=10, bg=COLOR_GREY)
        self.commonInfo.grid(row=0, column=0, sticky='nsew')

        self.app.grid_rowconfigure(0, weight=1)
        self.app.grid_columnconfigure(0, weight=1)
    
if __name__ == '__main__':
    app = tk.Tk()
    app.geometry('1280x700')
    central = CommandCentral(app, 5)
    app.mainloop()



    

        