###################################################################
# THIS VERSION doesn't support 'go' button as in Version3. 
# COLLISION CHECKING for simulated robot is implemented this time.
# by Qin Chen, 5/29/2018
###################################################################
import Tkinter as tk
import sys
import time
import math
import threading
from HamsterAPI.comm_ble import RobotComm
# from tk_hamster_simpackpulation_starter import *
import sys
sys.path.append('../')
import Tkinter as tk
from graph_with_edge_cost import *
from tk_hamster_GUI_Sim import *

gRobotList = []

class MotionPlanner(object):
    def __init__(self, vWorld, start, goal):
        self.vWorld = vWorld
        self.start = start
        self.goal = goal
        return

    def worker(self):
        print'MotionPlanner is called'
        vWorld = self.vWorld
        start = self.start
        goal = self.goal
        canvas_width = vWorld.canvas_width
        canvas_height = vWorld.canvas_height
        cell_list = Queue.Queue()
        cell_list.put(vWorld.area)
        # inflate obstacles to form C-space
        self.compute_c_obstacles(vWorld,28)
        obs_list = vWorld.cobs
        vWorld.goal_list = []
        f_cell_list = []
        # Cut inflated obstacles out of C-space and divide workspace into cells from cutting 
        f_cell_list = self.compute_free_cells(cell_list, obs_list)
        # determine connectivity between free cells and locate the point to go from one cell to its neighbor
        point_list = self.compute_free_points(f_cell_list)
        
        raw_input('press RETURN to show free cells')
        for cell in f_cell_list:
            x1 = cell[0]
            y1 = cell[1]
            x2 = cell[2]
            y2 = cell[3]     
            vWorld.canvas.create_rectangle(canvas_width+x1, canvas_height-y1, canvas_width+x2, canvas_height-y2, outline = "orange")
        
        raw_input('press RETURN to show start and goal')
        #create graph - nodes and edges for the point list
        myGraph = Graph()
        num_points = len(point_list)

        # creating nodes
        myGraph.add_node("s", start)
        myGraph.set_start("s")
        myGraph.add_node("g", goal)
        myGraph.set_goal("g")
        xs = start[0]
        ys = start[1]
        vWorld.canvas.create_oval(canvas_width+xs-6, canvas_height-ys-6, canvas_width+xs+6, canvas_height-ys+6, outline = "green", fill="green")
        xg = goal[0]
        yg = goal[1]
        vWorld.canvas.create_oval(canvas_width+xg-6, canvas_height-yg-6, canvas_width+xg+6, canvas_height-yg+6, outline = "purple", fill="purple")
        
        raw_input('press RETURN to show points connecting free cells, start, and goal')
        point_num = 1
        for point in point_list:
            myGraph.add_node(str(point_num), point)
            x1 = point[0]
            y1 = point[1]
            vWorld.canvas.create_oval(canvas_width+x1-4, canvas_height-y1-4, canvas_width+x1+4, canvas_height-y1+4, outline = "red")
            if self.connected(point, start, f_cell_list):
                g_cost = math.sqrt((xs-x1)*(xs-x1)+(ys-y1)*(ys-y1))
                #print "creating edge: ", "s", str(point_num), g_cost
                myGraph.add_edge("s", str(point_num), g_cost)
                vWorld.canvas.create_line(canvas_width+x1, canvas_height-y1, canvas_width+xs, canvas_height-ys, fill="black")
            if self.connected(point, goal, f_cell_list):
                g_cost = math.sqrt((xg-x1)*(xg-x1)+(yg-y1)*(yg-y1))
                #print "creating edge: ", "g", str(point_num), g_cost
                myGraph.add_edge("g", str(point_num), g_cost)
                vWorld.canvas.create_line(canvas_width+x1, canvas_height-y1, canvas_width+xg, canvas_height-yg, fill="black")
            point_num += 1

        raw_input("press RETURN to show connectivity")
        if num_points > 1:
            # creating edges
            #print "num points: ", num_points
            next_point = 2
            for i in range (1, num_points+1):
                #print "from: ", i
                point1 = point_list[i-1]
                x1 = point1[0]
                y1 = point1[1]
                for j in range (next_point, num_points+1):
                    #print "to: ", j
                    point2 = point_list[j-1]
                    x2 = point2[0]
                    y2 = point2[1]
                    if (self.connected(point1, point2, f_cell_list)):
                        g_cost = math.sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1))
                        #print "creating edge: ", str(i), str(j), g_cost
                        myGraph.add_edge(str(i), str(j), g_cost)
                        vWorld.canvas.create_line(canvas_width+x1, canvas_height-y1, canvas_width+x2, canvas_height-y2)
                next_point += 1

        raw_input('press RETURN to show path')
        #print "search: ", myGraph.queue
        myGraph.Dijkstra()
        path = Queue.LifoQueue()
        if myGraph.nodes["g"].back_pointer:
            path.put(["pose", myGraph.nodes["g"].data[0], myGraph.nodes["g"].data[1], False])
            print "Found path"
            path_node = myGraph.nodes["g"].back_pointer
            while path_node.back_pointer != False:
                px = path_node.data[0]
                py = path_node.data[1]
                path.put(["pose", px, py, False])
                vWorld.canvas.create_oval(canvas_width+px-6, canvas_height-py-6, canvas_width+px+6, canvas_height-py+6, outline = "red", fill="red")
                path_node = path_node.back_pointer
            while not path.empty():
                vWorld.goal_list.append(path.get())
                vWorld.goal_list_index = 0
            print "path: ", vWorld.goal_list
        else:
            print "failed to find path"
        return

    def compute_c_obstacles(self, vworld, d):
        time.sleep(0.01)
        # print("Compute C Obstacles called")
        # save c-space obstacle location info in vWorld.cobs[]
        for rect in vworld.map:
            vworld.cobs.append([rect[0]-d,rect[1]-d,rect[2]+d,rect[3]+d])
        points = []
        for cob in vworld.cobs:
            points.append((cob[0],cob[1]))
            points.append((cob[0],cob[3]))
            points.append((cob[2],cob[1]))
            points.append((cob[2],cob[3]))

        vworld.tempDraw(points, 'blue')
        print("Called vworld draw")


    def compute_free_cells(self, cell_list, c_obs_list):
        #return f_cell_list


        canvas_width = self.vWorld.canvas_width
        canvas_height = self.vWorld.canvas_height
        dx = canvas_width*2/4
        dy = canvas_height*2/4

        xCords = []
        for i in range(int(canvas_width*2/dx)):
            xCords.append(int(i*dx - canvas_width))
        if canvas_width not in xCords:
            xCords.append(canvas_width)
        # print(xCords)

        yCords = []
        for i in range(int(canvas_height*2/dy)):
            yCords.append(int(i*dy - canvas_height))
        if canvas_height not in yCords:
            yCords.append(canvas_height)
        # print(yCords)

        cells = []
        for x in range(len(xCords)-1):
            for y in range(len(yCords)-1):
                cells.append((xCords[x],yCords[y],xCords[x+1], yCords[y+1]))
        # print(cells)

        f_cell_list = []
        for i in range(6):
            newCells = []
            for cell in cells:
                free, continued = self.cellIsFree(cell,c_obs_list)
                if free:
                    f_cell_list.append(cell)
                elif not free and continued:
                    # print("CELL IS NOT FREE")
                    midPoint = ((cell[0]+cell[2])/2, (cell[3]+cell[1])/2)
                    newCells.append((cell[0],cell[1],midPoint[0],midPoint[1]))
                    newCells.append((cell[0],midPoint[1],midPoint[0],cell[3]))
                    newCells.append((midPoint[0],cell[1],cell[2],midPoint[1]))
                    newCells.append((midPoint[0],midPoint[1],cell[2],cell[3]))
            cells = newCells
            # print(cells)


        # self.vWorld.tempDrawRect(f_cell_list, "cyan")
        return f_cell_list

    def cellIsFree(self,cell,c_obs_list):
        free = True
        for cob in c_obs_list:
            #min and max X and Ys
            minX = cell[0]
            maxX = cell[2]
            if cell[2] < cell[0]:
                minX = cell[0]
                maxX = cell[1]
            minY = cell[1]
            maxY = cell[3]
            if cell[3] < cell[1]:
                minY = cell[3]
                maxX = cell[1]
            points = []
            points.append((cob[0], cob[1]))
            points.append((cob[0], cob[3]))
            points.append((cob[2], cob[1]))
            points.append((cob[2], cob[3]))
            for point in points:
                if point[0] >= minX and point[0] <= maxX and point[1] >= minY and point[1] <= maxY:
                        free = False
 

            points = []
            points.append((minX,minY))
            points.append((minX,maxY))
            points.append((maxX,minY))
            points.append((maxX,maxY))

            minX = cob[0]
            maxX = cob[2]
            if cob[2] < cob[0]:
                minX = cob[0]
                maxX = cob[1]
            minY = cob[1]
            maxY = cob[3]
            if cob[3] < cob[1]:
                minY = cob[3]
                maxX = cob[1]
            count = 0
            continued = True
            for point in points:
                if point[0] >= minX and point[0] <= maxX and point[1] >= minY and point[1] <= maxY:
                        free = False
                        count += 1
            if count == 4:
                continued = False

        return (free, continued)  




        """
        f_cell_list = []

        lines = []


        for cob in c_obs_list[1:2]:
            #min and max X and Ys
            minX = cob[1]
            maxX = cob[3]
            if cob[3] < cob[1]:
                minX = cob[3]
                maxX = cob[1]
            minY = cob[0]
            maxY = cob[2]
            if cob[3] < cob[0]:
                minY = cob[2]
                maxY = cob[0]

            #look for other obstacle above

            minXupDist = 1000
            minXupY = -self.vWorld.canvas_height
            minXdownDist = 1000
            minXdownY = self.vWorld.canvas_height

            maxXupDist = 1000
            maxXupY = -self.vWorld.canvas_height
            maxXdownDist = 1000
            maxXdownY = self.vWorld.canvas_height

            # minXupY = self.vWorld.canvas_height
            # minXdownY = self.vWorld.canvas_height
            # maxXupY = -self.vWorld.canvas_height
            # maxXdownY = -self.vWorld.canvas_height

            for cob2 in c_obs_list:

                

                minX2 = cob2[1]
                maxX2 = cob2[3]
                if cob2[3] < cob2[1]:
                    minX2 = cob2[3]
                    maxX2 = cob2[1]
                minY2 = cob2[0]
                maxY2 = cob2[2]
                if cob2[2] < cob2[0]:
                    minY2 = cob2[2]
                    maxY2 = cob2[0]


                if cob2 == cob:
                    continue
                #make sure vertex is not in another cob2. Set dist to 0 so no other overwrites
                #Top Left
                
                if minX >= minX2 and minX <= maxX2 and minY >= minY2 and minY <= maxY2:
                    print("inside cob2 FROM TOP LEFT")
                    minXupDist = 0
                    minXupY = maxY
                #Top Right
                if maxX >= minX2 and maxX <= maxX2 and minY >= minY2 and minY <= maxY2:
                    print("inside cob2 FROM TOP RIGHT")
                    maxXupDist = 0
                    maxXupY = maxY
                #Bottom Left
                if minX >= minX2 and minX <= maxX2 and maxY >= minY2 and maxY <= maxY2:
                    print("inside cob2 FROM BOTTOM LEFT")
                    minXdownDist = 0
                    minXdownY = minY
                #Bottom right
                if maxX >= minX2 and maxX <= maxX2 and maxY >= minY2 and maxY <= maxY2:
                    print("inside cob2 FROM BOTTOM RIGHT")
                    maxXdownDist = 0
                    maxXdownY = minY
                
                #Find distance
                #Top left
                
                if minX2 < minX and maxX2 > minX and maxY2 > minY:
                    print("INTERSECT FROM TOP LEFT at: ("+str(maxX)+","+str(minY2)+")")
                    dist = abs(minY-maxY2)
                    if dist < minXupDist:
                        minXupDist = dist
                        minXupY = maxY2

                #Top right
                if minX2 < maxX and maxX2 > maxX and maxY2 < minY:
                    print("INTERSECT FROM TOP RIGHT at: ("+str(maxX)+","+str(minY2)+")")
                    dist = abs(maxY2 - minY)
                    if dist < maxXupDist:
                        maxXupDist = dist
                        maxXupY = minY2
                
                #Bottom Left
                
                if minX2 < minX and maxX2 > minX and minY2 > maxY:
                    print("INTERSECT FROM BOTTOM LEFT at: ("+str(maxX)+","+str(minY2)+")")
                    dist = abs(minY2 - maxY)
                    if dist < minXdownDist:
                        minXdownDist = dist
                        minXdownY = minY2
                #Bottom Right
                if minX2 < maxX and maxX2 > maxX and minY2 < maxY:
                    print("INTERSECT FROM BOTTOM RIGHT at: ("+str(maxX)+","+str(minY2)+")")
                    dist = abs(minY2 - maxY)
                    if dist < minXdownDist:
                        maxXdownDist = dist
                        maxXdownY = maxY2
                
                

                lines.append((minXdownY, minX, minXupY, minX))
                lines.append((maxXdownY, maxX, maxXupY, maxX))
        self.vWorld.tempDrawLine(lines, 'red')


                
        """
               



        

        """
        for cob in c_obs_list:
            verticies = []
            verticies.append((cob[0],cob[1]))
            verticies.append((cob[0],cob[3]))
            verticies.append((cob[2],cob[1]))
            verticies.append((cob[2],cob[3]))

            for vertex in verticies:
                #find nearest up and down
                upDist = 1000
                upVert = (vertex[0],-self.vWorld.canvas_height)
                downDist = 1000
                downVert = (vertex[0],self.vWorld.canvas_height)
                for cob2 in c_obs_list:
                    if cob2 == cob:
                        continue
                    #Need to check 
                    if cob2[0] <= vertex[0] and cob2[2] >= vertex[0]:
                        if abs(vertex[1]-cob2[1])<upDist and abs(vertex[1]-cob2[1]) > 0:
                            upDist =  abs(vertex[1]-cob2[1])
                            upVert = (vertex[0],cob2[1])
                        if abs(vertex[1]-cob2[1])<downDist and abs(vertex[1]-cob2[1]) > 0:
                            downDist =  abs(vertex[1]-cob2[1])
                            downVert = (vertex[0],cob2[1])
                    if cob2[0] <= vertex[0] and cob2[2] >= vertex[0]:
                        if abs(vertex[1]-cob2[3])<upDist and abs(vertex[1]-cob2[3]) > 0:
                            upDist =  abs(vertex[1]-cob2[3])
                            upVert = (vertex[0],cob2[3])
                        if abs(vertex[1]-cob2[3])<downDist and abs(vertex[1]-cob2[3]) > 0:
                            downDist =  abs(vertex[1]-cob2[3])
                            downVert = (vertex[0],cob2[3])
                upLine = (vertex[0], vertex[1], upVert[0], upVert[1])
                downLine = (vertex[0], vertex[1], downVert[0], downVert[1])
                lines.append(upLine)
                lines.append(downLine)
            self.vWorld.tempDrawLine(lines, 'red')

        for line in lines:
            #find nearest left and right
            verticies = []
            verticies.append((line[0],line[1]))
            verticies.append((line[2],line[3]))
            LeftDist = 1000
            LeftVert = (-self.vWorld.canvas_width, line[1])
            RightDist = 1000
            RightVert = (self.vWorld.canvas_width, line[1])
            

            for line2 in lines:
                if line2 == line: 
                    continue
                #TODO

        """    



        """
        #creating a dictionary of key x val ys
        lineYs = {}
        lineXs = {}
        for cob in c_obs_list:
            #generate x cords of lines
            lineX = []
            lineX.extend([cob[0], cob[2]])
            
            #make sure all xs are in dictionary but not extras
            for x in lineX:
                if x not in lineYs.keys():
                    lineYs[x] = set()

            #add y cords for each x intercept
            lineYs[cob[0]].add(cob[1])
            lineYs[cob[0]].add(cob[3])
            lineYs[cob[2]].add(cob[1])
            lineYs[cob[2]].add(cob[3])
            for cob2 in c_obs_list:
                if cob2 != cob:
                    for x in lineX:
                        if x >= cob2[0] and x <= cob2[2]:
                            #line is intersecting the object
                            lineYs[x].add(cob2[1])
                            lineYs[x].add(cob2[3])

            lineY = []
            lineY.extend((cob[1], cob[3]))

            #make sure all ys are in dictionary but not extras
            for y in lineY:
                if y not in lineXs.keys():
                    lineXs[y] = set()

            #add x cords for each y intercept
            lineXs[cob[1]].add(cob[0])
            lineXs[cob[1]].add(cob[2])
            lineXs[cob[3]].add(cob[0])
            lineXs[cob[3]].add(cob[2])
            for cob2 in c_obs_list:
                if cob2 != cob:
                    for y in lineY:
                        if y >= cob2[1] and y <= cob2[3]:
                            #line is intersecting the object
                            lineXs[y].add(cob2[0])
                            lineXs[y].add(cob2[2])


        possCells = []
        points = []
        for x in lineYs.keys():

            for y in lineYs[x]:
                #courner of cells?
                originPoint = (x, y)
                points.append(originPoint)
        # self.vWorld.tempDraw(points, 'red')
        """






    def two_cells_connected(self, cell1, cell2):
        # Given two free cells, cell1 and cell2.
        # return connecting point[x,y] if two cells are connected
        # return False if not connected
        minX1 = cell1[0]
        maxX1 = cell1[2]
        if cell1[2] < cell1[0]:
            minX1 = cell1[0]
            maxX1 = cell1[1]
        minY1 = cell1[1]
        maxY1 = cell1[3]
        if cell1[3] < cell1[1]:
            minY1 = cell1[3]
            maxX1 = cell1[1]

        minX2 = cell2[0]
        maxX2 = cell2[2]
        if cell2[2] < cell2[0]:
            minX2 = cell2[0]
            maxX2 = cell2[1]
        minY2 = cell2[1]
        maxY2 = cell2[3]
        if cell2[3] < cell2[1]:
            minY2 = cell2[3]
            maxX2 = cell2[1]


        

    def compute_free_points(self, f_cell_list):
        # Obstacle free cells are given in f_cell_list
        # This function returns a list of points, each point is on overlapping edge of two conncted obstacle free cells.
        pass

    def connected(self, point1, point2, cell_list):
        # given two points in c-space and list of free cells.
        # return True if point1 and point2 are connected by a free cell
        # otherwise return False
        pass

class GUI(object):    
    def __init__(self, gui_root, vWorld, endCommand):
        self.gui_root = gui_root
        self.vWorld = vWorld
        gui_root.title("Motion Planner")
        self.endCommand = endCommand
        self.start = [200, 0]

        # instance variables
        self.button4=None
        self.button5=None
        self.button6=None
        self.button8=None
        self.button11=None
        self.rCanvas = None
        self.initUI()

        self.offset = 0.5

    def initUI(self):
        print("UI IS INITIALIZED")
        self.gui_root.title("Hamster Simulator")
        canvas_width = 440 # half width
        canvas_height = 300 # half height
        self.vWorld.canvas_width = canvas_width
        self.vWorld.canvas_height = canvas_height
        vRobot = self.vWorld.vrobot
        #creating tje virtual appearance of the robot
        
        self.rCanvas = tk.Canvas(self.gui_root, bg="white", width=canvas_width*2, height=canvas_height*2)
        self.rCanvas.pack(side="top")
        self.vWorld.canvas = self.rCanvas
        self.vWorld.canvas_width = canvas_width
        self.vWorld.canvas_height = canvas_height

        # visual elements of the virtual robot 
        poly_points = [0,0,0,0,0,0,0,0]
        vRobot.poly_id = self.rCanvas.create_polygon(poly_points, fill='blue') #robot
        vRobot.prox_l_id = self.rCanvas.create_line(0,0,0,0, fill="red") #prox sensors
        vRobot.prox_r_id = self.rCanvas.create_line(0,0,0,0, fill="red")
        vRobot.floor_l_id = self.rCanvas.create_oval(0,0,0,0, outline="white", fill="white") #floor sensors
        vRobot.floor_r_id = self.rCanvas.create_oval(0,0,0,0, outline="white", fill="white")
        #time.sleep(1)
        frame = tk.Frame(self.gui_root)
        frame.pack(side='bottom')
        button0 = tk.Button(frame,text="Grid")
        button0.pack(side='left')
        button0.bind('<Button-1>', self.drawGrid)

        button1 = tk.Button(frame,text="Clear")
        button1.pack(side='left')
        button1.bind('<Button-1>', self.clearCanvas)

        button2 = tk.Button(frame,text="Reset")
        button2.pack(side='left')
        button2.bind('<Button-1>', self.resetvRobot)

        button3 = tk.Button(frame,text="Map")
        button3.pack(side='left')
        button3.bind('<Button-1>', self.drawMap)

        self.button4 = tk.Button(frame,text="Trace")
        self.button4.pack(side='left')
        self.button4.bind('<Button-1>', self.toggleTrace)

        self.button5 = tk.Button(frame,text="Prox Dots")
        self.button5.pack(side='left')
        self.button5.bind('<Button-1>', self.toggleProx)

        self.button6 = tk.Button(frame,text="Floor Dots")
        self.button6.pack(side='left')
        self.button6.bind('<Button-1>', self.toggleFloor)

        self.button11 = tk.Button(frame,text="Real Robot")
        self.button11.pack(side='left')
        self.button11.bind('<Button-1>', self.toggleRobot)

        button9 = tk.Button(frame,text="Exit")
        button9.pack(side='left')
        button9.bind('<Button-1>', self.exit_prog)

        self.gui_root.bind("<KeyPress>", self.keydown)
        self.gui_root.bind("<KeyRelease>", self.keyup)

        self.rCanvas.bind("<Button-1>", self.getGoal)

        return

    def keydown(self, event):
        if event.char == 'w':
            self.move_up()
        elif event.char == 's':
            self.move_down()
        elif event.char == 'a':
            self.move_left()
        elif event.char == 'd':
            self.move_right()
        return

    def keyup(self, event):
        if event.char == 'w' or event.char == 's' or event.char == 'a' or event.char == 'd':
            self.stop_move()
        return

    def drawGrid(self, event=None):
        print "draw Grid"
        canvas_width = self.vWorld.canvas_width
        canvas_height = self.vWorld.canvas_height
        x1 = 0
        x2 = canvas_width*2
        y1 = 0
        y2 = canvas_height*2
        del_x = 20
        del_y = 20
        num_x = x2 / del_x
        num_y = y2 / del_y
        # draw center (0,0)
        self.rCanvas.create_rectangle(canvas_width-3,canvas_height-3,canvas_width+3,canvas_height+3, fill="red")
        # horizontal grid
        for i in range (0,num_y):
            y = i * del_y
            self.rCanvas.create_line(x1, y, x2, y, fill="yellow")
        # verticle grid
        for j in range (0, num_x):
            x = j * del_x
            self.rCanvas.create_line(x, y1, x, y2, fill="yellow")
        return

    def drawMap(self, event=None):
        self.vWorld.draw_map()
        return

    def resetvRobot(self, event=None):
        #vRobot.reset_robot()
        self.vWorld.vrobot.x = 200
        self.vWorld.vrobot.y = 0
        self.vWorld.vrobot.a = -1.571
        self.vWorld.goal_achieved = True
        self.vWorld.goal_list_index = 0
        return

    def clearCanvas(self, event=None):
        self.rCanvas.delete("all")
        poly_points = [0,0,0,0,0,0,0,0]
        self.vWorld.vrobot.poly_id = self.rCanvas.create_polygon(poly_points, fill='blue')
        self.vWorld.vrobot.prox_l_id = self.rCanvas.create_line(0,0,0,0, fill="red")
        self.vWorld.vrobot.prox_r_id = self.rCanvas.create_line(0,0,0,0, fill="red")
        self.vWorld.vrobot.floor_l_id = self.rCanvas.create_oval(0,0,0,0, outline="white", fill="white")
        self.vWorld.vrobot.floor_r_id = self.rCanvas.create_oval(0,0,0,0, outline="white", fill="white")
        

    def toggleTrace(self, event=None):
        if self.vWorld.trace == True:
            self.vWorld.trace = False
            self.button4["text"] = "Trace"
        else:
            self.vWorld.trace = True
            self.button4["text"] = "No Trace"
        return

    def toggleProx(self, event=None):
        if self.vWorld.prox_dots == True:
            self.vWorld.prox_dots = False
            self.button5["text"] = "Prox Dots"
        else:
            self.vWorld.prox_dots = True
            self.button5["text"] = "No Prox Dots"
        return

    def toggleFloor(self, event=None):
        if self.vWorld.floor_dots == True:
            self.vWorld.floor_dots = False
            self.button6["text"] = "Floor Dots"
        else:
            self.vWorld.floor_dots = True
            self.button6["text"] = "No Floor Dots"
        return

    def toggleRobot(self, event=None):
        if self.vWorld.real_robot:
            robot = self.vWorld.real_robot
            robot.set_wheel(0,0)
            robot.set_wheel(1,0)
            self.vWorld.real_robot = False
            print "simulated robot"
            self.button11["text"] = "Real Robot"
        else:
            if (len(gRobotList) > 0):
                self.vWorld.real_robot = gRobotList[0]
                robot = self.vWorld.real_robot
                robot.set_wheel(0, self.vWorld.vrobot.sl)
                robot.set_wheel(1, self.vWorld.vrobot.sr) 
                self.button11["text"] = "Simulation"
                print "connected to robot", self.vWorld.real_robot
            else:
                print "please turn on robot"
                self.vWorld.voronoi()
        return

    def toggleGo(self, event=None):
        if self.vWorld.go:
            self.vWorld.go = False
            print "Pause"
            self.button8["text"] = "Go"
        else:
            self.vWorld.go = True
            print "Go"
            self.button8["text"] = "Pause"
        return

    def getGoal(self, event):
        self.vWorld.canvas.create_oval(event.x-4, event.y-4, event.x+4, event.y+4, outline = "blue")
        canvas_width = self.vWorld.canvas_width
        canvas_height = self.vWorld.canvas_height
        self.vWorld.goal_x = event.x - canvas_width
        self.vWorld.goal_y = canvas_height - event.y 
        print "selected goal: ", self.vWorld.goal_x, self.vWorld.goal_y
        s_point = self.start
        g_point = [self.vWorld.goal_x, self.vWorld.goal_y]
        print 'start pose(%s, %s): ' % (s_point[0], s_point[1])
        print 'goal pose(%s, %s): ' % (g_point[0], g_point[1]) 
        mp = MotionPlanner(self.vWorld, s_point, g_point)
        mp.worker()
        return

    def move_up(self, event=None):
        self.vWorld.vrobot.sl = 30
        self.vWorld.vrobot.sr = 30
        robot = self.vWorld.real_robot
        if robot:
            robot.set_wheel(0, self.vWorld.vrobot.sl)
            robot.set_wheel(1, self.vWorld.vrobot.sr)

        return

    def move_down(self, event=None):
        self.vWorld.vrobot.sl = -30
        self.vWorld.vrobot.sr = -30
        robot = self.vWorld.real_robot
        if robot:
            robot.set_wheel(0, self.vWorld.vrobot.sl)
            robot.set_wheel(1, self.vWorld.vrobot.sr)
        return

    

    def move_left(self, event=None):
        self.vWorld.vrobot.sl = 10
        self.vWorld.vrobot.sr = 30
        robot = self.vWorld.real_robot
        if robot:
            # robot.set_wheel(0, int(self.vWorld.vrobot.sl - self.offset))
            # robot.set_wheel(1, int(self.vWorld.vrobot.sr + self.offset))
            robot.set_wheel(0, 10)
            robot.set_wheel(1, 33)
        return

    def move_right(self, event=None):
        self.vWorld.vrobot.sl = 30
        self.vWorld.vrobot.sr = 10
        robot = self.vWorld.real_robot
        if robot:
            # robot.set_wheel(0, int(self.vWorld.vrobot.sl + self.offset))
            # robot.set_wheel(1, int(self.vWorld.vrobot.sr - self.offset))

            robot.set_wheel(0, 33)
            robot.set_wheel(1, 10)
        return

    def stop_move(self, event=None):
        self.vWorld.vrobot.sl = 0
        self.vWorld.vrobot.sr = 0
        robot = self.vWorld.real_robot
        if robot:
            robot.set_wheel(0, self.vWorld.vrobot.sl)
            robot.set_wheel(1, self.vWorld.vrobot.sr)
        return

    def exit_prog(self, event=None):
        self.vWorld.gQuit = True
        self.gui_root.quit()
        for robot in gRobotList:
            robot.reset()
        return

    def draw_virtual_world(self):       
        self.vWorld.draw_robot()
        self.vWorld.draw_prox("left")
        self.vWorld.draw_prox("right")
        self.vWorld.draw_floor("left")
        self.vWorld.draw_floor("right")
        self.gui_root.after(50, self.draw_virtual_world)
        return

class VirtualWorld(object):
    def __init__(self, gui_root):
        self.gui_root = gui_root
        self.gui_handle = None
        self.vrobot = None
        self.vworld = None
        self.gQuit = False
        self.create_world()
        return

    def create_world(self):       
        self.vrobot = virtual_robot()
        self.vrobot.t = time.time()

        #create the virtual worlds that contains the virtual robot
        self.vworld = virtual_world()
        self.vworld.vrobot = self.vrobot
        
        #objects in the world
        self.vworld.map = []

        # take out map to test real robot
        # local obstacle avoidance using APF
        #project 3-1
        #rect1 = [-50, 80, 50, 120]
        #rect2 = [100, -50, 140, 50]
        #rect3 = [-160, -50, -120, 50]
        #rect4 = [-50,-180, 50, -140]

        #project 3-2
        #rect1 = [-20, 80, 20, 120]
        #rect2 = [100, -20, 140, 20]
        #rect3 = [-20, -120, 20, -80]
        #rect4 = [-260,-30, -220, 30]
        #rect5 = [-220, -70, -180, -30]
        #rect6 = [-220, 30, -180, 70]

        #bounder of board
        rect1 = [-100, -180, 0, -140]
        rect2 = [-140, -180, -100, -80]
        rect3 = [-100, 140, 0, 180]
        rect4 = [-140, 80, -100, 180]
        rect5 = [0, -50, 40, 50]
        rect6 = [-260, -20, -220, 20]
        rect7 = [40, 60, 140, 100]

        self.vworld.area = [-300,-200,300,200]

        self.vworld.add_obstacle(rect1)
        self.vworld.add_obstacle(rect2)
        self.vworld.add_obstacle(rect3)
        self.vworld.add_obstacle(rect4)
        self.vworld.add_obstacle(rect5)
        self.vworld.add_obstacle(rect6)
        self.vworld.add_obstacle(rect7)


        # set initial pose of robot wrt robot coordinate system
        self.vworld.vrobot.x = 200  # 200 pixels to the right from center(origin of robot coor. syst.)
        self.vworld.vrobot.y = 0
        self.vworld.vrobot.a = 1.5*3.1415    # facing west
        # self.gui_handle = GUI(self.gui_root, self.vworld, self.stopProg)
        
        return

    def stopProg(self, event=None):
            self.gui_root.quit()
            return

    # Update data components of the simulator.
    def update_virtual_world(self):
        waiting_for_robot = True

        while waiting_for_robot and self.vworld.real_robot:
            if len(gRobotList) > 0:
                robot = gRobotList[0] 
                waiting_for_robot = False
                print "connected to robot"
            else:
                print "waiting for robot to connect"
            time.sleep(0.1)

        noise_prox = 35 # noise level for proximity
        noise_floor = 20 #floor ambient color - if floor is darker, set higher noise
        p_factor = 1.3 #proximity conversion - assuming linear
        #prox_conv_l = [0, 0, 0, 60, 50, 40, 30, 20, 20]
        prox_conv_l = [85,85,82,75,67,59,51,47,40,35]
        prox_conv_r = [85,85,82,75,67,59,51,47,40,35]

        #################################
        # Find the value of distance factor for your Hamster

        d_factor = 1.025 # travel distance conversion
        #################################

        #a_factor = 17.5 #turning speed of -15 vs angle 
        b = 31.5 #distance in mm, between two wheels

        vrobot = self.vworld.vrobot

        while not self.gQuit:
            t = time.time()
            del_t = t - vrobot.t
            vrobot.t = t # update the tick

            # compute new pose
            ms = (vrobot.sl*del_t+vrobot.sr*del_t)/2 #speed of the center
            new_vrobot_a = vrobot.a + (vrobot.sl-vrobot.sr)*del_t/b
            new_vrobot_x = vrobot.x + ms * math.sin(vrobot.a) * d_factor
            new_vrobot_y = vrobot.y + ms * math.cos(vrobot.a) * d_factor

            ##########################
            # If in collision, Hamster should stop moving, which means no update
            # of its pose.
            ##########################
            # if not self.vworld.in_collision(new_vrobot_a, new_vrobot_x, new_vrobot_y):
            if True:
                vrobot.a = new_vrobot_a
                vrobot.x = new_vrobot_x
                vrobot.y = new_vrobot_y

            while (vrobot.a >= 3.1415):
                vrobot.a -= 6.283

            # update sensors
            if (self.vworld.real_robot):    # convert prox sensor readings to number of pixels for the red beams
                robot = self.vworld.real_robot
                prox_l = robot.get_proximity(0)
                prox_r = robot.get_proximity(1)

                if (prox_l > noise_prox):
                    i = 0
                    while (prox_conv_l[i] > prox_l) and (i < 9):
                        i += 1
                    if (vrobot.dist_l and vrobot.sl == 0 and vrobot.sr == 0):
                        vrobot.dist_l = vrobot.dist_l*2.0 + i*10 + 10 - (prox_l - prox_conv_l[i])*10/(prox_conv_l[i-1] - prox_conv_l[i])
                        vrobot.dist_l = vrobot.dist_l / 3.0
                    else:
                        vrobot.dist_l = i*10 + 10 - (prox_l - prox_conv_l[i])*10/(prox_conv_l[i-1] - prox_conv_l[i])
                else:
                    vrobot.dist_l = False

                if (prox_r > noise_prox):
                    i = 0
                    while (prox_conv_r[i] > prox_r) and (i < 9):
                        i += 1
                    if (vrobot.dist_r and vrobot.sl == 0 and vrobot.sr == 0):
                        vrobot.dist_r = vrobot.dist_r*2.0 +i*10 + 10 - (prox_r - prox_conv_r[i])*10/(prox_conv_r[i-1] - prox_conv_r[i])
                        vrobot.dist_r = vrobot.dist_r / 3.0
                    else:
                        vrobot.dist_r = i*10 + 10 - (prox_r - prox_conv_r[i])*10/(prox_conv_r[i-1] - prox_conv_r[i])
                else:
                    vrobot.dist_r = False
            else:               # simulate prox sensors
                self.vworld.get_vrobot_prox("left")
                self.vworld.get_vrobot_prox("right")

            if (self.vworld.real_robot):
                floor_l = robot.get_floor(0)
                floor_r = robot.get_floor(1)
            else:
                floor_l = 100 #white
                floor_r = 100
            if (floor_l < noise_floor):
                vrobot.floor_l = floor_l
            else:
                vrobot.floor_l = False

            if (floor_r < noise_floor):
                vrobot.floor_r = floor_r
            else:
                vrobot.floor_r = False

            time.sleep(0.05)
        return

def main():
    global gRobotList

    gMaxRobotNum = 0 # max number of robots to control
    comm = RobotComm(gMaxRobotNum)
    comm.start()
    print 'Bluetooth starts'
    gRobotList = comm.robotList
    
    root = tk.Tk() #root
    root.geometry="630x880"
    world_handle = VirtualWorld(root)
    t_update_world = threading.Thread(name='update_world', target=world_handle.update_virtual_world)
    t_update_world.daemon = True
    t_update_world.start()

    gui_handle = GUI(root, world_handle.vworld, world_handle.stopProg)
    gui_handle.draw_virtual_world()     # this method runs in main thread

    root.mainloop()

    comm.stop()
    comm.join()

if __name__== "__main__":
    sys.exit(main())