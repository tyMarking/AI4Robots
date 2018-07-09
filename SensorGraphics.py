'''
/* =======================================================================
   (c) 2015, Kre8 Technology, Inc.
   This is a program that is provided to students in Robot AI class.
   Students use this it to build different Hamster behaviors.

   Name:          tk_behaviors_starter.py
   By:            Qin Chen
   Last Updated:  5/10/18

   PROPRIETARY and CONFIDENTIAL
   ========================================================================*/
'''
import sys
import time
import threading
import math
import random
import Tkinter as tk
from HamsterAPI.comm_ble import RobotComm	# no dongle
#from HamsterAPI.comm_usb import RobotComm	# yes dongle

################################
# Hamster control
################################
class State():
	rProx = 0
	lProx = 0
	lFloor = 0
	rFloor = 0

state = State

class RobotBehaviorThread(threading.Thread):
	def __init__(self, robotList):
		super(RobotBehaviorThread, self).__init__()
		self.go = False
		self.done = False
		self.robotList = robotList
		return

	def run(self):
		robot=None
		while not self.done:
			for robot in self.robotList:
				if robot and self.go:
					#############################################
					# START OF YOUR WORKING AREA!!!
					#############################################

					state.lProx = robot.get_proximity(0)
					state.rProx = robot.get_proximity(1)

					#print("lProx: "+str(state.lProx)+"\trProx: "+str(state.rProx))

					time.sleep(0.05)
					#############################################
					# END OF YOUR WORKING AREA!!!
					#############################################					
		# stop robot activities, such as motion, LEDs and sound
		# clean up after exit button pressed
		if robot:
			robot.reset()
			time.sleep(0.1)
		return

class GUI(object):
	def __init__(self, root, robot_control):
		self.root = root
		self.robot_control = robot_control
		root.geometry('400x450')
		root.title('Hamster Control')

		self.canvas = tk.Canvas(root, width="400", height="400")
		self.body1 = self.canvas.create_rectangle(160,160,240,240,fill="blue")
		self.floorL = self.canvas.create_rectangle(170,170,190,180,fill="black")
		self.floorR = self.canvas.create_rectangle(210,170,230,180,fill="black")
		self.proxL = self.canvas.create_line(180,160, 180,100, fill="red")
		self.proxR = self.canvas.create_line(220,160, 220,100, fill="red")
		self.canvas.pack()

		b2 = tk.Button(root, text='Exit')
		b2.pack(side='right')
		b2.bind('<Button-1>', self.exitProg)

		self.robot_control.go = True

		cButton = tk.Button(root, text='Make Grey')
		cButton.pack(side='left')
		cButton.bind('<Button-1>', self.colorProg)
		self.proxPoll()
		return
	
	def exitProg(self, event=None):
		self.robot_control.done = True		
		self.root.quit() 	# close window
		return


	def colorProg(self, event=None):
		self.canvas.itemconfig(self.floorR, fill='Grey')
		self.canvas.itemconfig(self.floorL, fill='Grey')

	def proxPoll(self):
		#get proxes somehow?
		newRY = 10+(state.rProx)*2
		newLY = 10+(state.lProx)*2
		#print("Poll rProx: "+str(state.rProx))
		#print("Poll lProx: "+str(state.lProx))
		#print("\t\t\t\tnewLY: "+str(newLY))
		self.canvas.coords(self.proxL, (180,160,180,newRY))
		self.canvas.coords(self.proxR, (220,160,220,newLY))

		self.canvas.after(100, self.proxPoll)

#################################
# Don't change any code below!! #
#################################

def main():
	
   	# instantiate COMM object
    gMaxRobotNum = 1; # max number of robots to control
    comm = RobotComm(gMaxRobotNum)
    comm.start()
    print 'Bluetooth starts'  
    robotList = comm.robotList

    behaviors = RobotBehaviorThread(robotList)
    behaviors.start()


    frame = tk.Tk()
    GUI(frame, behaviors)
    frame.mainloop()

    comm.stop()
    comm.join()
    print("terminated!")

if __name__ == "__main__":
	sys.exit(main())
