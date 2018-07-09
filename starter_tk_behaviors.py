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
from threading import Thread
from HamsterAPI.comm_ble import RobotComm	# no dongle
#from HamsterAPI.comm_usb import RobotComm	# yes dongle

################################
# Hamster control
################################
class State():
	state = ""
	direction = 1

state = State
state.state = ""
class RobotBehaviorThread(threading.Thread):
	def __init__(self, robotList):
		super(RobotBehaviorThread, self).__init__()
		self.go = False
		self.done = False
		self.robotList = robotList
		return

	def run(self):
		colorDone = False
		robot=None
		lastMState = ""
		mazeIters = 0
		while not self.done:
			for robot in self.robotList:
				if robot and self.go:
					#############################################
					# START OF YOUR WORKING AREA!!!
					#############################################
					#robot.set_wheel(0, 30)
					#robot.set_wheel(1, 30)
					#robot.get_proximity() 0 left, 1 right
					#robot.get_floor(), 0 L, 1 R

					#square, (shy, dance), follow leader, follow line


					if state.state == "Square":

						print("state.state: "+ str(state.state))
						robot.set_wheel(0, 100)
						robot.set_wheel(1, 100)
						time.sleep(3)
						robot.set_wheel(1,0)
						time.sleep(0.53)
					
					elif state.state == "Shy":
						#shy
						diff = 25
						prox = ( robot.get_proximity(0)-diff + robot.get_proximity(1)-diff ) / 2
						#print(prox)
						#sigfoid function
						sigProx = max( ((200/(1+math.exp(-prox/5)))-100), 0)

						robot.set_wheel(0, int(-sigProx))
						robot.set_wheel(1, int(-sigProx))

					elif state.state == "Dance":
						"""
						prox = ( robot.get_proximity(0) + robot.get_proximity(1) ) / 2
						print(prox)
						if prox < 20:
							robot.set_wheel(0, 100)
							robot.set_wheel(1, 100)
							time.sleep(0.3)
						elif prox > 40:
							robot.set_wheel(0, -100)
							robot.set_wheel(1, -100)
							time.sleep(0.3)
						else:
							robot.set_wheel(0, 0)
							robot.set_wheel(1, 0)
							time.sleep(0.3)
						"""
						robot.set_wheel(0,random.randint(-100,100))
						robot.set_wheel(1,random.randint(-100,100))
						robot.set_musical_note(random.randint(0,80))
						robot.set_led(0, random.randint(0,7))
						robot.set_led(1, random.randint(0,7))
						time.sleep(random.random()*2)


					#other cases
					elif state.state == "Stopped":
						#is stopped
						#print("STOPPING")
						robot.set_wheel(0,0)
						robot.set_wheel(1,0)

					elif state.state == "Trace":
						speed = 40

						robot.set_wheel(0, speed)
						robot.set_wheel(1,speed)
						left = robot.get_floor(0)
						right = robot.get_floor(1)
						print("Left: " + str(left)+"\t"+"Right: "+str(right))
						if (right - left) > 15:
							robot.set_wheel(1,speed)
							robot.set_wheel(0, 0)
							print("Turning Left")
							time.sleep(0.05)
						elif (left - right) > 15:
							robot.set_wheel(0,speed)
							robot.set_wheel(1, 0) 
							print("Turning Right")
							time.sleep(0.03)

					elif state.state == "Intersection":
						speed = 40
						# robot.set_wheel(0, 0)
						# robot.set_wheel(1,0)
						robot.set_wheel(0, speed)
						robot.set_wheel(1,speed)
						left = robot.get_floor(0)
						right = robot.get_floor(1)
						# print("Left: " + str(left)+"\t"+"Right: "+str(right))
						
						if right < 75 and left < 75:
							#intersection??
							robot.set_wheel(0, speed)
							robot.set_wheel(1,speed)
							time.sleep(0.2)
							robot.set_wheel(0, 0)
							robot.set_wheel(1,0)

							left = robot.get_floor(0)
							right = robot.get_floor(1)
							if (True):
								robot.set_musical_note(40)
								time.sleep(0.1)
								robot.set_musical_note(0)
								#left
								tTime = 0.6
								if state.direction == 0:
									robot.set_wheel(0, -50)
									robot.set_wheel(1,50)
									time.sleep(tTime)
								elif state.direction == 2:
									robot.set_wheel(0, 50)
									robot.set_wheel(1,-50)
									time.sleep(tTime)

						elif (right - left) > 15:
							robot.set_wheel(1,speed)
							robot.set_wheel(0, 0)
							# print("Turning Left")
							time.sleep(0.03)
						elif (left - right) > 15:
							robot.set_wheel(0,speed)
							robot.set_wheel(1, 0) 
							# print("Turning Right")
							time.sleep(0.03)
						

						#both below 75 - intersection?
						print(state.direction)
					elif state.state == "Maze":
						#Hug right wall
						"""
						check right wall
						move forward half bot
						if right was availible turn right
						move forward half
						"""
						fProx = robot.get_proximity(0)
						rProx = robot.get_proximity(1)
						print("fProx: " + str(fProx)+ "\trProx: " + str(rProx))


						speed = 100
						sTime = 0.05
						#turn right
						if rProx < 32:
							robot.set_wheel(0,speed)
							#smooth turn or sharp turn
							if fProx > 42:
								robot.set_wheel(1,-speed)
							else:
								robot.set_wheel(1,speed/4)
							time.sleep(sTime)
						#turn left
						elif fProx > 35 and rProx > 30:
							robot.set_wheel(0,-speed)
							robot.set_wheel(1,speed)
							print("LOOPING")
							while fProx > 35:
								fProx = robot.get_proximity(0)
								
						#straight
						elif fProx <= 35 and rProx > 30:
							robot.set_wheel(0,speed)
							robot.set_wheel(1,speed)
							time.sleep(sTime)
						else:
							time.sleep(sTime)
						
					elif state.state == "Color":

						if not colorDone:
							print("COLOR TIME")
							print(robot.get_battery())
							color_handler = threading.Thread(name='color thread', target=self.colorThred, args=[robot])
							color_handler.daemon = True
							color_handler.start()
							colorDone = True
							# robot.set_led(0,0)
							# robot.set_led(0,0)
						

					else:
						print("Unknown state.state")
						print("state.state = " + state.state)
					
	def colorThred(self, robot):
		# print("RUNNING COLOR CHANGER")
		robot.set_led(0,random.randint(0,7))
		# print("randint: " + str(random.randint(0,7)))
		time.sleep(0.5)
		robot.set_led(1,random.randint(0,7))
		time.sleep(0.5)
		self.colorThred(robot)
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
		root.geometry('650x30')
		root.title('Hamster Control')

		b1 = tk.Button(root, text='Square')
		b1.pack(side='left')
		b1.bind('<Button-1>', self.squareProg)

		

		b3 = tk.Button(root, text='Shy')
		b3.pack(side='left')
		b3.bind('<Button-1>', self.shyProg)		

		b4 = tk.Button(root, text='Dance')
		b4.pack(side='left')
		b4.bind('<Button-1>', self.danceProg)

		b6 = tk.Button(root, text='Trace')
		b6.pack(side='left')
		b6.bind('<Button-1>', self.traceProg)

		b9 = tk.Button(root, text='Intersection')
		b9.pack(side='left')
		b9.bind('<Button-1>', self.interProg)

		b7 = tk.Button(root, text='Maze')
		b7.pack(side='left')
		b7.bind('<Button-1>', self.mazeProg)

		b2 = tk.Button(root, text='Exit')
		b2.pack(side='left')
		b2.bind('<Button-1>', self.exitProg)

		b8 = tk.Button(root, text='Color On')
		b8.pack(side='left')
		b8.bind('<Button-1>', self.colorProg)

		b5 = tk.Button(root, text='Stop')
		b5.pack(side='left')
		b5.bind('<Button-1>', self.stopProg)

		return
	
	def squareProg(self, event=None):
		self.robot_control.go = True
		state.state = "Square"
		return

	def interProg(self, event=None):
		print("YES I AM RUNNING")
		self.robot_control.go = True
		state.state = "Intersection"

		leftB = tk.Button(self.root, text="Left")
		straightB = tk.Button(self.root, text="Straight")
		rightB = tk.Button(self.root, text="Right")

		leftB.pack(side="top")
		straightB.pack(side="top")
		rightB.pack(side="top")

		leftB.bind('<Button-1>', self.leftDir)
		straightB.bind('<Button-1>', self.straightDir)
		rightB.bind('<Button-1>', self.rightDir)


	def leftDir(self, event=None):
		state.direction = 0
	def straightDir(self, event=None):
		state.direction = 1
	def rightDir(self, event=None):
		state.direction = 2

	def exitProg(self, event=None):
		self.robot_control.done = True		
		self.root.quit() 	# close window
		return

	def stopProg(self, event=None):
		self.robot_control.go = True
		state.state = "Stopped"

	def shyProg(self, event=None):
		self.robot_control.go = True
		state.state = "Shy"

	def danceProg(self, event=None):
		self.robot_control.go = True
		state.state = "Dance"

	def traceProg(self, event=None):
		self.robot_control.go = True
		state.state = "Trace"

	def mazeProg(self, event=None):
		self.robot_control.go = True
		state.state = "Maze"

	def colorProg(self, event=None):
		self.robot_control.go = True
		state.state = "Color"

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
