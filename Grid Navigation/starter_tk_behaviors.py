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

class PathFollower():
	def __init__(self):
		gMaxRobotNum = 1; # max number of robots to control
		self.comm = RobotComm(gMaxRobotNum)
		self.comm.start()
		print 'Bluetooth starts'  
		robotList = self.comm.robotList
		self.robotList = robotList
		self.go = False
		self.done = False

		return

	def run(self, path):
		robot=None
		#0:N, 1:E, 2:S, 3:W
		currentDir = 0
		while not self.done:
			for robot in self.robotList:
				if robot and self.go:
					#############################################
					# START OF YOUR WORKING AREA!!!
					#############################################
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
							nextDir = path[0]
							del path[0]
							robot.set_musical_note(40)
							time.sleep(0.1)
							robot.set_musical_note(0)
							
							tTime = 0.6
							#straight
							if nextDir == currentDir:
								print("GOING straight")
							#right
							elif nextDir == currentDir + 1 or (nextDir == 0 and currentDir == 3):
								print("TURNING RIGHT")
								robot.set_wheel(0, 50)
								robot.set_wheel(1,-50)
								time.sleep(tTime)
								#left
							elif nextDir == currentDir - 1 or (nextDir == 3 and currentDir == 0):
								print("TURNING LEFT")
								robot.set_wheel(0, -50)
								robot.set_wheel(1,50)
								time.sleep(tTime)
							elif nextDir == (currentDir + 2)%4:
								print("TURNING AROUND???")
								robot.set_wheel(0, 50)
								robot.set_wheel(1,-50)
								time.sleep(2*tTime)
							currentDir = nextDir


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
					

				



    

 	def runPath(self, path):
 		self.go = True
 		thread = threading.Thread(name="Run Thread", target=self.run, args=[path])
 		thread.daemon = True
 		thread.start()
	def exit(self):
		self.go = False
		self.done = True
		self.comm.stop()
		self.comm.join()
		print("terminated!")
		sys.exit(0)


