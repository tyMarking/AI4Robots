# Robot Programming
# breadth first search
# by Dr. Qin Chen
# May, 2016

import sys
import Tkinter as tk
from starter_tk_behaviors import *
##############
# This class supports display of a grid graph. The node location on canvas
# is included as a data field of the graph, graph.node_display_locations.
##############

class GridGraphDisplay(object):
    def __init__(self, frame, graph):
        self.node_dist = 60
        self.node_size = 20
        self.gui_root = frame
        self.canvas = None
        self.graph = graph
        self.nodes_location = graph.node_display_locations
        self.start_node = graph.startNode
        self.goal_node = graph.goalNode
        self.edges = {}
        self.nodes = {}
        self.initUI()
        self.follower = PathFollower()
        return

    def initUI(self):
        self.canvas = tk.Canvas(self.gui_root, bg="white", width=400, height=400)
        self.canvas.pack()

        self.buttonFrame = tk.Frame(self.gui_root)
        self.buttonFrame.pack()

        self.goB = tk.Button(self.buttonFrame, text="Go")
        self.exitB = tk.Button(self.buttonFrame, text="Exit")

        self.goB.pack()
        self.exitB.pack()

        self.goB.bind('<Button-1>', self.goProg)
        self.exitB.bind('<Button-1>', self.exitProg)



    def exitProg(self, Event=None):
        self.follower.exit()

    def goProg(self, Event = None):
        path = self.graph.getPath()
        self.highlight_path(self.graph.node_path)
        self.follower.runPath(path)
    # draws nodes and edges in a graph
    def display_graph(self):        
        locs = self.graph.compute_node_locations()


        #Edges
        for key in self.graph.nodes.keys():
            x1 = locs[key][0]
            y1 = locs[key][1]
            edgeDict = {}
            for otherN in self.graph.nodes[key]:
                dont = False
                if otherN in self.edges.keys():
                    if self.edges[otherN][key]:
                        dont = True
                if not dont:
                    x2 = locs[otherN][0]
                    y2 = locs[otherN][1]
                    edgeDict[otherN] = self.canvas.create_line(x1,y1,x2,y2, fill='black', width=5)
            self.edges[key] = edgeDict
                    


        #Nodes
        size = 25
        for key in locs.keys():
            x = locs[key][0]
            y = locs[key][1]
            x1, y1 = (x - size), (y - size)
            x2, y2 = (x + size), (y + size)
            self.nodes[key] = self.canvas.create_oval(x1, y1, x2, y2, fill='blue')

        print(self.edges)

    # path is a list of nodes ordered from start to goal node
    def highlight_path(self, path):
        locs = self.graph.compute_node_locations()

        for i in range(len(path)-1):
            cNode = path[i]
            nNode = path[i+1]

            if nNode in self.edges[cNode].keys():
                self.canvas.itemconfig(self.edges[cNode][nNode], fill="yellow")
            elif cNode in self.edges[nNode].keys():
                self.canvas.itemconfig(self.edges[nNode][cNode], fill="yellow")

  
    # draws a node in given color. The node location info is in passed-in node object
    #EDIT: pass in the name of the node
    def draw_node(self, node, n_color):
        self.canvas.itemconfig(self.nodes[node], fill=n_color)

    # draws an line segment, between two given nodes, in given color
    def draw_edge(self, node1, node2, e_color):
        if node1 in self.edges[node2].keys():
                self.canvas.itemconfig(self.edges[node2][node1], fill=e_color)
        elif node2 in self.edges[node1].keys():
                self.canvas.itemconfig(self.edges[node1][node2], fill=e_color)
          



