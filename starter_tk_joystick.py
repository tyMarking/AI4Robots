'''
/* =======================================================================
   (c) 2015, Kre8 Technology, Inc.

   Name:          Joystick for Hamster
   By:            Qin Chen
   Last Updated:  5/10/18

   PROPRIETARY and CONFIDENTIAL
   ========================================================================*/
'''
import sys
import Tkinter as tk
import time
from HamsterAPI.comm_ble import RobotComm
#for PC, need to import from commm_usb

class Robots(object):
    def __init__(self, robotList):
        self.robotList = robotList
        return
    speed = 100
    def move_forward(self, event=None):
        print("I WAS MOVING FORWARD")
        if self.robotList:
            for robot in self.robotList:
                robot.set_wheel(0,self.speed)
                robot.set_wheel(1,self.speed)
        else:
            print "waiting for robot"

    def move_backward(self, event=None):
        if self.robotList:
            for robot in self.robotList:
                robot.set_wheel(0,-self.speed)
                robot.set_wheel(1,-self.speed)
        else:
            print "waiting for robot"

    def move_left(self, event=None):
        if self.robotList:
            for robot in self.robotList:
                robot.set_wheel(0,-self.speed)
                robot.set_wheel(1,self.speed)
        else:
            print "waiting for robot"

    def move_right(self, event=None):
        if self.robotList:
            for robot in self.robotList:
                robot.set_wheel(0,self.speed)
                robot.set_wheel(1,-self.speed)
        else:
            print "waiting for robot"

    def get_prox(self, event=None):
        print("Getting Prox")
        if self.robotList:
            return (self.robotList[0].get_proximity(0), self.robotList[0].get_proximity(1))
        else:
            return (0,0)
    def get_floor(self, event=None):
        if self.robotList:
            return (self.robotList[0].get_floor(0), self.robotList[0].get_floor(1))
        else:
            return (0,0)
    def stop_move(self, event=None):
        if self.robotList:
            for robot in self.robotList:
                robot.set_wheel(0,0)
                robot.set_wheel(1,0)
        else:
            print "waiting for robot"

    def reset_robot(self, event=None): # use Hamster API reset()
        pass

class UI(object):
    def __init__(self, root, robot_handle):
        self.root = root
        self.robot_handle = robot_handle  # handle to robot commands
        self.canvas = None
        self.prox_l_id = None
        self.prox_r_id = None
        self.floor_l_id = None
        self.floor_r_id = None
        self.initUI()
        print("Init display")
        self.display_sensor()

        return

    def initUI(self):
        ###################################################################
        # Create a Hamster joystick window which contains
        # 1. a canvas widget where "sensor readings" are displayed
        # 2. a square representing Hamster
        # 3. 4 canvas items to display floor sensors and prox sensors
        # 4. a button for exit, i.e., a call to stopProg(), given in this class
        # 5. listen to key press and key release when focus is on this window
        ###################################################################
        self.root.geometry('400x450')
        self.root.title('Hamster Control')

        self.canvas = tk.Canvas(self.root, width="400", height="400")
        self.body1 = self.canvas.create_rectangle(160,160,240,240,fill="blue")
        self.floorL = self.canvas.create_rectangle(170,170,190,180,fill="black")
        self.floorR = self.canvas.create_rectangle(210,170,230,180,fill="black")
        self.wall = self.canvas.create_line(180, 100, 220, 100, fill="black")
        self.proxL = self.canvas.create_line(180,160, 180,100, fill="red")
        self.proxR = self.canvas.create_line(220,160, 220,100, fill="red")

        self.canvas.pack()

        b2 = tk.Button(self.root, text='Exit')
        b2.pack(side='bottom')
        b2.bind('<Button-1>', self.stopProg)



        #sB = tk.Button(self.root, text='Start')
        #sB.pack(side='left')
        # sB.bind('<Button-1>', self.display_sensor())

        self.root.bind('<KeyPress>', self.keydown)
        self.root.bind('<KeyRelease>', self.keyup)


        
        
        return
    
    
    ######################################################
    # This function refreshes floor and prox sensor display every 100 milliseconds.
    # Register callback using Tkinter's after method().
    ######################################################
    def display_sensor(self):
        print("display_sensor")
        if len(self.robot_handle.robotList) == 0:
            self.canvas.after(1000, self.display_sensor)
        lProx, rProx = self.robot_handle.get_prox()
        lFloor, rFloor = self.robot_handle.get_floor()
        print("lFloor: "+str(lFloor)+"\trFloor: "+str(rFloor))
        newRY = min([160,20+(rProx)*2])
        newLY = min([160,20+(lProx)*2])
        # print("display rProx: "+str(rProx))
        # print("display lProx: "+str(lProx))
        #print("\t\t\t\tnewLY: "+str(newLY))
        self.canvas.coords(self.proxL, (180,160,180,newLY))
        self.canvas.coords(self.proxR, (220,160,220,newRY))
        self.canvas.coords(self.wall, (180,newLY, 220,newRY))

        if lFloor > 50:
            self.canvas.itemconfig(self.floorL, fill="white")
        else:
            self.canvas.itemconfig(self.floorL, fill="black")

        if rFloor > 50:
            self.canvas.itemconfig(self.floorR, fill="white")
        else:
            self.canvas.itemconfig(self.floorR, fill="black")

        self.canvas.after(100, self.display_sensor)

    ####################################################
    # Implement callback function when key press is detected
    ####################################################
    def keydown(self, event):
        # self.display_sensor()
        # print("KEYDOWN: " + (event.char))
        if event.char == 'w':
            self.robot_handle.move_forward()
        elif event.char == 's':
            self.robot_handle.move_backward()
        elif event.char == 'd':
            self.robot_handle.move_right()
        elif event.char == 'a':
            self.robot_handle.move_left()
        return

    #####################################################
    # Implement callback function when key release is detected
    #####################################################
    def keyup(self, event):
        # print("KEYUP")
        
        self.robot_handle.stop_move()

    def stopProg(self, event=None):
        self.root.quit()    # close window
        self.robot_handle.reset_robot()
        return

def main(argv=None):
    gMaxRobotNum = 2 # max number of robots to control
    comm = RobotComm(gMaxRobotNum)
    comm.start()
    print 'Bluetooth starts'
    robotList = comm.robotList

    robot_handle = Robots(robotList)
    m = tk.Tk() #root
    gui = UI(m, robot_handle)
    print("I GOT HERE")
    m.mainloop()
    comm.stop()
    comm.join()

if __name__== "__main__":
    sys.exit(main())