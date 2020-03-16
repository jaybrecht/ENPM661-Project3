import cv2
import numpy as np
import math
import time
import os
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
from matplotlib.collections import PatchCollection
from matplotlib.patches import Ellipse, Circle, Wedge, Polygon, Arrow
import matplotlib.pyplot as plt

class Robot:
    def __init__(self,maze,userInput):
        self.maze = maze
        self.pos_thresh = .5
        self.ang_thresh = 30
        self.goal_radius = 1.5

        if userInput:
            self.get_user_nodes()
        else:
            self.start = (50,30,30)
            self.goal = (150,150)
            self.d = 10
            self.clearance = 3
            self.radius = 5
        
        self.offset=self.clearance+self.radius

        s_circle = Circle((self.start[0],self.start[1]), self.goal_radius, color='green',alpha=.4)
        self.maze.ax.add_patch(s_circle)
        g_circle = Circle((self.goal[0],self.goal[1]), self.goal_radius, color='red',alpha=.4)
        self.maze.ax.add_patch(g_circle)

        
    def move(self,point,direction):
        d=self.d
        x = point[0]
        y = point[1]
        theta = np.deg2rad(point[2])

        if direction == 'left60':
            phi=np.deg2rad(60)
        elif direction == 'left30':
            phi=np.deg2rad(30)
        elif direction == 'straight':
            phi=0
        elif direction == 'right30':
            phi=np.deg2rad(-30)
        elif direction == 'right60':
            phi=np.deg2rad(-60)

        new_x=x+d*math.cos(theta+phi)
        new_y=y+d*math.sin(theta+phi)

        new_theta = round(np.rad2deg(theta+phi))
        if new_theta >= 360:
            new_theta = new_theta-360
        if new_theta <= -360:
            new_theta = new_theta+360

        new_point = (new_x,new_y,new_theta)

        return new_point


    def check_neighbors(self,cur_node):
        directions = ['left60','left30','straight','right30','right60']

        neighbors = []
        for direction in directions:
            new_point = self.move(cur_node,direction)
            if self.maze.in_bounds(new_point):
                if not self.maze.in_obstacle(new_point,self.offset):
                    neighbors.append(new_point)

        return neighbors


    def trunc(self,a,thresh):
        dec_a = a % 1
        int_a = a//1

        if dec_a % thresh < thresh/100:
            trunc_a = int_a + dec_a
        else: 
            for val in np.arange(0,1,thresh):
                if(dec_a-val)<=thresh:
                    if abs(dec_a-(val)) < abs(dec_a-(val+thresh)):
                        trunc_a = int_a+val
                    else:
                        trunc_a = int_a+(val+thresh)
                    break

        return trunc_a


    def discretize(self,point):
        x=point[0]
        y=point[1]
        theta=point[2]

        x = int(self.trunc(x,self.pos_thresh)*(1/self.pos_thresh))
        y = int(self.trunc(y,self.pos_thresh)*(1/self.pos_thresh))
        if theta < 0:
            theta = 360+round(theta)
        theta = int(theta*(1/self.ang_thresh))


        new_point=(x,y,theta)
        return new_point


    def A_star(self):
        def take_second(elem):
            return elem[1]
        # each node = (x,y,theta) <- floats
        # nodes = [node1,node2,..,node_n]
        self.nodes = []
        self.nodes.append(self.start)

        # visited_nodes = binary 3D matrix 1 for have visited 0 for haven't
        size_x = int(self.maze.width/self.pos_thresh)
        size_y = int(self.maze.height/self.pos_thresh)
        size_th = int(360/self.ang_thresh)
        visited_nodes = np.zeros((size_x,size_y,size_th))
        start_disc = self.discretize(self.start)
        visited_nodes[start_disc[0],start_disc[1],start_disc[2]] = 1 #set start node as checked
        
        # costs = 3D matrix where at discretized point is tuple (cost2come,cost2goal)
        self.costs2come = np.full((visited_nodes.shape[0],visited_nodes.shape[1],visited_nodes.shape[2]),np.inf)
        self.costs2goal = np.full((visited_nodes.shape[0],visited_nodes.shape[1],visited_nodes.shape[2]),np.inf)
        cost2come = 0
        self.costs2come[start_disc[0],start_disc[1],start_disc[2]] = cost2come
        cost2goal = math.sqrt((self.goal[0] - self.start[0])**2 + (self.goal[1] - self.start[1])**2)
        
        # parents = 3D matrix of size h/thresh,w/thresh,360/th_thresh index is ind of parent in nodes
        self.parents = np.full((visited_nodes.shape[0],visited_nodes.shape[1],visited_nodes.shape[2]),np.nan)
        self.parents[start_disc[0],start_disc[1],start_disc[2]] = -1 #set parent of start node to -1
        
        # queue needs to be a list of tuples (node_ind,cost2come+cost2goal)
        queue = [(0,cost2come+cost2goal)]
    
        self.foundGoal = False    

        while queue:
            #sort queue
            queue.sort(key = take_second)
            # Set the current node as the top of the queue and remove it
            parent,distance = queue.pop(0)

            cur_node = self.nodes[parent]
            cur_disc = self.discretize(cur_node)
            cost2come = self.costs2come[cur_disc[0],cur_disc[1],cur_disc[2]]

            neighbors = self.check_neighbors(cur_node)

            for p in neighbors:
                cost2goal = math.sqrt((self.goal[0] - p[0])**2 + (self.goal[1] - p[1])**2)
                disc_p = self.discretize(p)
                if visited_nodes[disc_p[0],disc_p[1],disc_p[2]] == 0: 
                    visited_nodes[disc_p[0],disc_p[1],disc_p[2]] = 1
                    self.costs2come[disc_p[0],disc_p[1],disc_p[2]] = cost2come+self.d
                    self.parents[disc_p[0],disc_p[1],disc_p[2]] = parent
                    self.nodes.append(p)
                    queue.append((len(self.nodes)-1,cost2goal))
                elif cost2come + self.d < self.costs2come[disc_p[0],disc_p[1],disc_p[2]]:
                    self.costs2come[disc_p[0],disc_p[1],disc_p[2]] = cost2come+self.d
                    self.parents[disc_p[0],disc_p[1],disc_p[2]] = parent
                if cost2goal<self.goal_radius:
                    self.foundGoal = True
                    queue.clear()
                    break


    def generate_path(self):
        #Assume the last item in nodes is the goal node
        goal = self.nodes[-1]
        disc_goal = self.discretize(goal)
        parent = int(self.parents[disc_goal[0],disc_goal[1],disc_goal[2]])
        path_nodes = [parent]
        while parent != -1:
            node = self.nodes[path_nodes[-1]]
            disc_node = self.discretize(node)
            parent = int(self.parents[disc_node[0],disc_node[1],disc_node[2]])
            path_nodes.append(parent)
        self.path = [goal]
        for ind in path_nodes:
            if ind == -1:
                break
            else:
                self.path.insert(0,self.nodes[ind])


    def get_user_nodes(self):
        valid_input = False
        while not valid_input:
            valid_pt = False
            while not valid_pt:
                print('Please enter a start point (x,y,theta)')
                start_str_x = input('start x: ')
                start_str_y = input('start y: ')
                start_str_th = input('start theta: ')
                try:
                    start_point = (float(start_str_x),float(start_str_y),int(start_str_th))
                except ValueError:
                    print('Please enter a number')
                else:
                    # Check if start point is valid in maze 
                    if self.maze.in_bounds(start_point):
                        if self.maze.in_obstacle(start_point,0):
                            print("The start point is in an obstacle")
                        else:
                            valid_pt = True
                    else:
                        print("The start point is not valid")

            valid_pt = False
            while not valid_pt:
                print('Please enter a goal point (x,y)')
                goal_str_x = input('goal x: ')
                goal_str_y = input('goal y: ')
                try:
                    goal_point = (float(goal_str_x),float(goal_str_y))
                except ValueError:
                    print('Please enter a number')
                else:
                    # Check if goal point is valid in maze 
                    if self.maze.in_bounds(goal_point):
                        if self.maze.in_obstacle(goal_point,0):
                            print("The goal point is in an obstacle")
                        else:
                            valid_pt = True
                    else:
                        print("The goal point is not valid")

            # Check that start is not goal
            distance = math.sqrt((goal_point[0] - start_point[0])**2 + (goal_point[1] - start_point[1])**2)
            if distance <= self.goal_radius:
                print('The start cannot be within the goal')
            else:
                valid_input = True

        valid_d = False
        while not valid_d:
            print('Please enter the distance your robot can travel per move')
            d_str = input('distance: ')
            try:
                d = float(d_str)
                if 1 <= d <= 10:
                    valid_d = True
                else:
                    print('The value must be between 1 and 10')
            except ValueError:
                print('Please enter a number')


        valid_clear = False
        while not valid_clear:
            print('Please enter the desired clearance to obstacles')
            clearance_str = input('clearance: ')
            try:
                self.clearance = float(clearance_str)
                valid_clear = True
            except ValueError:
                print('Please enter a number')

        valid_radius = False
        while not valid_radius:
            print('Please enter the radius of your robot')
            radius_str = input('radius: ')
            try:
                self.radius = float(radius_str)
                valid_radius = True
            except ValueError:
                print('Please enter a number')

        self.start = start_point
        self.goal = goal_point
        self.d = d


    def plotter(self,start_pos, end_pos,color="black"):
        x_s=start_pos[0]
        y_s=start_pos[1]
        theta=start_pos[2]

        x_f=end_pos[0]
        y_f=end_pos[1]

        dx=x_f-x_s
        dy=y_f-y_s
        # head_width=0.5,length_includes_head=True, head_length=0.5,
        arrow = plt.Arrow(x_s, y_s, dx, dy, color=color)

        return arrow

        


    def visualize(self,output,show):
        if output:
            fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')
            filename = 'output/rigid_robot_plot.mp4'
            fps_out = 35
            
            if os.path.exists(filename):
                os.remove(filename)
            
            out_plt = cv2.VideoWriter(filename, fourcc, fps_out, (1200,800))
            canvas = FigureCanvas(self.maze.fig)

            print('Writing to video. Please Wait.')
        
        # Show the searched nodes
        for i,point in enumerate(self.nodes):
            neighborhood=self.check_neighbors(point)
            for neighbor in neighborhood:
                arrow = self.plotter(point,neighbor,color='cyan')
                self.maze.ax.add_artist(arrow)

                if output:
                    self.maze.fig.canvas.draw()
                    maze_img = np.frombuffer(self.maze.fig.canvas.tostring_rgb(), dtype=np.uint8).reshape(self.maze.fig.canvas.get_width_height()[::-1] + (3,))
                    maze_img = cv2.cvtColor(maze_img,cv2.COLOR_RGB2BGR)
                    cv2.imshow('Visualization',maze_img)

                if show:
                    if cv2.waitKey(1) == ord('q'):
                        exit()
                    out_plt.write(maze_img)

                arrow.remove()
                arrow = self.plotter(point,neighbor,color='gray')
                self.maze.ax.add_artist(arrow)

        robot_circle=plt.Circle((self.path[0][0],self.path[0][1]), self.offset, color='orange')
        self.maze.ax.add_artist(robot_circle)

        if output:          
            self.maze.fig.canvas.draw()
            maze_img = np.frombuffer(self.maze.fig.canvas.tostring_rgb(), dtype=np.uint8).reshape(self.maze.fig.canvas.get_width_height()[::-1] + (3,))
            maze_img = cv2.cvtColor(maze_img,cv2.COLOR_RGB2BGR)
            out_plt.write(maze_img)

        if show:
            cv2.imshow('Visualization',maze_img)
            if cv2.waitKey(1) == ord('q'):
                exit()

        # Draw the path
        for i in range(len(self.path)-1):
            #Remove the previous circle
            robot_circle.remove()

            # Plot the path arrows
            arrow = self.plotter(self.path[i],self.path[i+1],color='red')
            self.maze.ax.add_artist(arrow)

            # Plot the robot
            robot_circle=plt.Circle((self.path[i+1][0],self.path[i+1][1]), self.offset, color='orange')
            self.maze.ax.add_artist(robot_circle)

            if output:          
                self.maze.fig.canvas.draw()
                maze_img = np.frombuffer(self.maze.fig.canvas.tostring_rgb(), dtype=np.uint8).reshape(self.maze.fig.canvas.get_width_height()[::-1] + (3,))
                maze_img = cv2.cvtColor(maze_img,cv2.COLOR_RGB2BGR)
                out_plt.write(maze_img)

            if show:
                cv2.imshow('Visualization',maze_img)
                if cv2.waitKey(1) == ord('q'):
                    exit()

        if output:
            out_plt.release()
            


