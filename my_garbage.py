'''
/* =======================================================================
   (c) 2015, Kre8 Technology, Inc.

   Name:          Robot Escape
   By:            Qin Chen
   Last Updated:  6/10/18

   PROPRIETARY and CONFIDENTIAL
   ========================================================================*/
'''
# This program shows how threads can be created using Thread class and your
# own functions. Another way of creating threads is subclass Thread and override
# run().
# 
"""

This is an increadibly hacking repurposing of my avoidance code. I don't know how it works so well

"""

import sys
sys.path.append('../')
import time  # sleep
import threading
import logging
import Tkinter as tk
import Queue
import random
from HamsterAPI.comm_ble import RobotComm

logging.basicConfig(level=logging.DEBUG,format='(%(threadName)-10s) %(message)s',)
count = None
class Event(object):
    def __init__(self, event_type, event_data):
      self.type = event_type #string
      self.data = event_data #list of number or character depending on type

class BehaviorThreads(object):
    Threshold_border = 50   # if floor sensor reading falls equal or below this value, border is detected
    Threshold_obstacle = 25   # if prox sensor reading is equal or higher than this, obstacle is detected
    
    def __init__(self, robot_list):
    	self.robot_list = robot_list
        self.count = 0
        self.go = False
        self.quit = False
        # events queues for communication between threads
        self.alert_q = Queue.Queue()
        self.motion_q = Queue.Queue()
        self.t_robot_watcher = None     # thread handles
        self.t_motion_handler = None
        
        # start a watcher thread
        t_robot_watcher = threading.Thread(name='watcher thread', target=self.robot_event_watcher, args=(self.alert_q, self.motion_q))
        t_robot_watcher.daemon = True
        t_robot_watcher.start()
        self.t_robot_watcher = t_robot_watcher

        ###################################
        # start a motion handler thread
        ###################################
        """
        States: moving, orientaiting, pushing, on_border
        Events: obstacle, free, border
        """
        diagram = {}
        #add diagram parts
        addPath(diagram, "moving", "free", self.moveForward, "moving")
        addPath(diagram, "moving", "obstacle", self.turn, "pushing")
        addPath(diagram, "moving", "border", self.collected, "border")
        addPath(diagram, "moving2", "free", self.moveForward, "moving2")
        addPath(diagram, "moving2", "obstacle", self.turn, "pushing")
        addPath(diagram, "moving2", "border", self.get_out, "border")
        addPath(diagram, "pushing", "free", self.moveForward, "moving")
        addPath(diagram, "pushing", "obstacle", self.turn, "pushing")
        addPath(diagram, "pushing", "border", self.collected, "border")
        addPath(diagram, "border", "free", self.moveForward, "moving2")
        addPath(diagram, "border", "obstacle", self.turn, "pushing")
        addPath(diagram, "border", "border", self.dummyFunc, "border")
        # addPath(diagram, )


        fsm = StateMachine(diagram, self.motion_q)

        t_motion_handler = threading.Thread(name='motion FSM thread', target=fsm.threadedRun, args=["moving2"])
        t_motion_handler.daemon = True
        t_motion_handler.start()
        self.t_motion_handler = t_motion_handler
        return


    def dummyFunc1Arg(self, x):
        pass 
    def dummyFunc(self):
        pass
    def moveForward(self):
        self.robot_list[0].set_wheel(0,100)
        self.robot_list[0].set_wheel(1,100)

    def turn(self, direction):
        if direction == "right":
            self.robot_list[0].set_wheel(0,30)
            self.robot_list[0].set_wheel(1,0)
        elif direction == "left":
            self.robot_list[0].set_wheel(0,0)
            self.robot_list[0].set_wheel(1,30)
        elif direction == "none":
            self.robot_list[0].set_wheel(0,100)
            self.robot_list[0].set_wheel(1,100)
        else:
            print("Unkown direction")


    ###################################
    # This function is called when border is detected
    ###################################
    def collected(self):
        self.robot_list[0].set_wheel(0,-30)
        self.robot_list[0].set_wheel(1,-30)
        time.sleep(0.5)
        if (self.robot_list[0].get_proximity(0) > 35 or self.robot_list[0].get_proximity(1) > 35):
            self.count += 1
            self.robot_list[0].set_musical_note(40)
            time.sleep(0.1)
            self.robot_list[0].set_musical_note(0)
        print("COUNT: "+str(self.count))
        if self.count >= 3:
            self.robot_list[0].set_wheel(0,0)
            self.robot_list[0].set_wheel(1,0)
            self.robot_list[0].set_musical_note(40)
            time.sleep(5)
            self.robot_list[0].set_musical_note(0)
        self.get_out()
    def get_out (self):
        self.robot_list[0].set_wheel(0,50)
        self.robot_list[0].set_wheel(1,-50)
        time.sleep(1.4)
        self.robot_list[0].set_wheel(1,50)
        while not self.motion_q.empty():
            # print("Clearing motion")
            self.motion_q.get()
        while not self.alert_q.empty():
            # print("Clearing alert")
            self.alert_q.get()
        

    # This function monitors the sensors
    def robot_event_watcher(self, q1, q2):
        count = 0

        logging.debug('starting...')
        print("EVENT WATCHER STARTED")
        while not self.quit:
            for robot in self.robot_list:
                if self.go and robot:
                    prox_l = robot.get_proximity(0)
                    prox_r = robot.get_proximity(1)
                    line_l = robot.get_floor(0)
                    line_r = robot.get_floor(1)
                    
                    if (prox_l > BehaviorThreads.Threshold_obstacle or prox_r > BehaviorThreads.Threshold_obstacle):
                        alert_event = Event("alert", [prox_l,prox_r])
                        q1.put(alert_event)
                        #logging.debug("alert event %s, %s, %s, %s", prox_l, prox_r, line_l, line_r)
	                    #time.sleep(0.01)
                        count += 1
	                    #update movement every 5 ticks
                        if (count % 5 == 0):
                            #logging.debug("obstacle detected, q2: %d %d" % (prox_l, prox_r))
                            obs_event = Event("obstacle", [prox_l, prox_r])
                            q2.put(obs_event)
                    else:
                        if (count >= 0):
                        	# free event is created when robot goes from obstacle to no obstacle
                            # logging.debug("free of obstacle")
                            free_event = Event("free",[])
                            q2.put(free_event)  # put event in motion queue
                            q1.put(free_event)  # put event in alert queue
                            count = 0

                    if (line_l < BehaviorThreads.Threshold_border or line_r < BehaviorThreads.Threshold_border):
                        # logging.debug("border detected: %d %d" % (line_l, line_r))
                        border_event = Event("border", [line_l, line_r])
                        q1.put(border_event)
                        q2.put(border_event)
                        pass
                    
                else:
                    pass
                    # print 'waiting ...'
            time.sleep(0.05)	# delay to give alert thread more processing time. Otherwise, it doesn't seem to have a chance to serve 'free' event
        return

    ##############################################################
    # Implement your motion handler. You need to get event using the passed-in queue handle and
    # decide what Hamster should do. Hamster needs to avoid obstacle while escaping. Hamster
    # stops moving after getting out of the border and remember to flush the motion queue after getting out.
    #############################################################
    """
    def robot_motion_handler(self, q):
        print("MOTION HANDLER STARTED")
        while  not self.quit:
            # print("HELLLLLOOOOO")
            for robot in self.robot_list:
                if self.go and robot:
                    event = q.get()
                    if event.type == "border":
                        print("MOTHION SIZE: "+str(q.qsize()))
                        while q.qsize() > 0:
                            q.get()
                        logging.debug("Motion Handler Border")
                        self.get_out(robot)
                        # time.sleep(0.05)
                    elif event.type == "obstacle":
                        logging.debug("Motion Handler Obq")
                        robot.set_wheel(0,30)
                        robot.set_wheel(1,-30)
                        
                    elif event.type == "free":
                        logging.debug("Motion Handler Free of obstacle")
                        robot.set_wheel(0,30)
                        robot.set_wheel(1,30)
    """

class StateMachine():
    diagram = None
    q = None
    state = None
    def __init__(self, diagram, q):
        self.diagram = diagram
        self.q = q

    def threadedRun(self, initState):
        print("STTARTING STATE MACHINE")
        self.state = initState
        while True:
            # print("STATE MACHING LOOPING")
            if not self.q.empty():
                # print("STATE: " + str(self.state))
                event = self.q.get()
                result = self.diagram[self.state][event.type]
                action = result[0]
                if event.type == "obstacle":
                    direction = "none"
                    if event.data[0] - event.data[1] > 10:
                        direction = "left"
                    elif event.data[1] - event.data[0] > 10:
                        direction = "right"
                    action(direction)
                else:
                    (action())
                self.state = result[1]


def addPath(diagram, startState, event, func, toState):
    if startState in diagram.keys():
        diagram[startState][event] = [func, toState]
    else:
        diagram[startState] = {event: [func, toState]}
                                  


class GUI(object):
    def __init__(self, root, threads_handle):
        self.root = root
        self.t_handle = threads_handle
        self.event_q = threads_handle.alert_q
        self.t_alert_handler = None
        self.canvas = None
        self.prox_l_id = None
        self.prox_r_id = None
        self.initUI()

    ##########################################################
    # 1. Create a canvas widget and three canvas items: a square, and two lines 
    # representing prox sensor readings.
    # 2. Create two button widgets, for start and exit.
    # 3. Create a thread for alert handler, which is responsible for displaying prox sensors.
    ##########################################################
    def initUI(self):
        self.root.geometry('400x450')
        self.root.title('Hamster Control')

        self.canvas = tk.Canvas(self.root, width="400", height="400")
        self.body1 = self.canvas.create_rectangle(160,160,240,240,fill="blue")
        self.wall = self.canvas.create_line(180, 100, 220, 100, fill="black")
        self.prox_l_id = self.canvas.create_line(180,160, 180,100, fill="red")
        self.prox_r_id = self.canvas.create_line(220,160, 220,100, fill="red")

        self.canvas.pack()

        b2 = tk.Button(self.root, text='Exit')
        b2.pack(side='bottom')
        b2.bind('<Button-1>', self.stopProg)

        bs = tk.Button(self.root, text='Start')
        bs.pack(side='bottom')
        bs.bind('<Button-1>', self.startRobot)

    def startRobot(self, event=None):
        print("TIME TO GO")
        self.t_handle.go = True
        self.robot_alert_handler()
        return

    def stopProg(self, event=None):
        print("TIME TO STOP")
        self.t_handle.quit = True
        print("HERE 1")
        for robot in self.t_handle.robot_list:
            robot.reset()
        print("HERE 2")
        # self.t_handle.t_motion_handler.join()
        print("HERE 3")
        # self.t_handle.t_robot_watcher.join()
        print("HERE 4")
        # self.t_alert_handler.join()
        print("HERE 5")
        self.root.quit()
        print("HERE 6")	# close GUI window
        return

    ###################################################
    # Handles prox sensor display and warning(sound).
    # Query event queue(using passed-in queue handle).
    # If there is an "alert" event, display red beams.
    # Erase the beams when "free" event is in queue.
    # This runs in the main GUI thread. Remember to schedule
    # a callback of itself after 50 milliseconds.
    ###################################################
    def robot_alert_handler(self):
        # print("ALERT HANDLER GOING")
        if self.event_q.qsize() > 0:
            event = self.event_q.get()
            if event.type == "alert":
                newRY = min([160,-50+(event.data[1])*3])
                newLY = min([160,-50+(event.data[0])*3])
                self.canvas.coords(self.prox_l_id, (180,160,180,newLY))
                self.canvas.coords(self.prox_r_id, (220,160,220,newRY))
                self.canvas.coords(self.wall, (180,newLY, 220,newRY))
            elif event.type == "free" or event.type == "border":
                self.canvas.coords(self.prox_l_id, (0,0,0,0))
                self.canvas.coords(self.prox_r_id, (0,0,0,0))
                self.canvas.coords(self.wall, (0,0,0,0))
        self.root.after(50, self.robot_alert_handler)


def main():
    max_robot_num = 1   # max number of robots to control
    comm = RobotComm(max_robot_num)
    comm.start()
    print 'Bluetooth starts'
    robotList = comm.robotList

    root = tk.Tk()
    t_handle = BehaviorThreads(robotList)
    gui = GUI(root, t_handle)
  
    root.mainloop()

    comm.stop()
    comm.join()

if __name__== "__main__":
  sys.exit(main())
  