'''
/* =======================================================================
   (c) 2015, Kre8 Technology, Inc.

   Name:          grid_graph_starter.py
   By:            Qin Chen
   Last Updated:  6/10/18
    
   Definition of class GridGraph. Description of all the methods is
   provided. Students are expected to implement the methods for Lab#6.
   ========================================================================*/
'''
import Tkinter as tk
import starter_grid_graph_display as display

class GridGraph(object):
    def __init__(self):
        self.nodes = {} # {node_name: set(neighboring nodes), ...}
        self.startNode = None  # string
        self.goalNode = None    # string
        self.grid_rows = None
        self.grid_columns = None
        self.obs_list = []
        self.node_display_locations=[]
        return

    # set number of rows in the grid
    def set_grid_rows(self, rows):
        self.grid_rows = rows

    # set number of columns in the grid
    def set_grid_cols(self, cols):
        self.grid_columns = cols

    # this method is used by make_grid() to create a key-value pair in self.nodes{},
    # where value is created as an empty set which is populated later while connecting
    # nodes.
    def add_node(self, name):
        self.nodes[name] = set()

    # set start node name
    def set_start(self, name):
        self.startNode = name

    # returns start node name
    def get_start_node(self):
        return self.startNode

    # set goal node name
    def set_goal(self, name):
        self.goalNode = name

    # return goal node name
    def get_goal_node(self):
        return self.goalNode

    # Given two neighboring nodes. Put them to each other's neighbors-set. This
    # method is called by self.connect_nodes() 
    def add_neighbor(self, node1, node2):
        self.nodes[node1].add(node2)
        self.nodes[node2].add(node1)

    # populate graph with all the nodes in the graph, excluding obstacle nodes
    def make_grid(self):
        for row in range(self.grid_rows):
            for col in range(self.grid_columns):
                if not [row,col] in self.obs_list:
                    self.add_node((str(row)+'-'+str(col)))

    # Based on node's name, this method identifies its neighbors and fills the 
    # set holding neighbors for every node in the graph.
    def connect_nodes(self):
        keys = self.nodes.keys()
        for key in keys:
            r = int(key[0])
            c = int(key[2])
            for i in (-1,1):
                if r+i >= 0 and r+i < self.grid_rows and not [r+i,c] in self.obs_list:
                    newName = str(r+i)+'-'+str(c)
                    self.add_neighbor(key, newName)
            for j in (-1,1):
                if c+j >= 0 and c+j < self.grid_columns  and not [r,c+j] in self.obs_list:
                    newName = str(r)+'-'+str(c+j)
                    self.add_neighbor(key, newName)

    # For display purpose, this function computes grid node location(i.e., offset from upper left corner where is (1,1)) 
    # of display area. based on node names.
    # Node '0-0' is displayed at bottom left corner 
    def compute_node_locations(self):
        width = 400
        height = 400
        buff = 50
        xOffset = (width-2*buff)/(self.grid_columns-1)
        yOffset = (height-2*buff)/(self.grid_rows-1)

        locs = {}
        for key in self.nodes.keys():
            r = int(key[0])
            c = int(key[2])
            locs[key] = ( buff + xOffset * c, buff + yOffset * (self.grid_rows - r -1))
        return locs

###########################################################
#  A testing program of your implementaion of GridGraph class.
###########################################################
def main():
    graph = GridGraph()
    # grid dimension
    graph.set_grid_rows(4)
    graph.set_grid_cols(3)

    # origin of grid is (0, 0) lower left corner
    # graph.obs_list = ([1,1],)    # in case of one obs. COMMA
    graph.obs_list = ([1,1], [3,0], [2,2])
    
    graph.set_start('0-0')
    graph.set_goal('2-1')
    
    graph.make_grid()
    graph.connect_nodes()
    print(graph.nodes)


    root = tk.Tk()
    dis = display.GridGraphDisplay(root, graph)
    dis.display_graph()
    dis.highlight_path(['0-0','1-0','2-0','2-1'])
    dis.draw_node('0-0', 'green')
    dis.draw_node('2-1', 'yellow')
    dis.draw_edge('0-0', '0-1', 'red')
    root.mainloop()
    # print(graph.nodes)
    return

if __name__ == "__main__":
    main()