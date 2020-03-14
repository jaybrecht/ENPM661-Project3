import cv2
import numpy as np
import math
import time
from plotter import*

class Robot:
    def __init__(self,maze,userInput):
        self.maze = maze
        self.pos_thresh = .5
        self.ang_thresh = 30
        if userInput:
            self.get_user_nodes()
        else:
            self.start = (25,100,0)
            self.goal = (75,100,0)
            self.d = 1
        

    def move(self,point,direction):
        d=self.d
        x = point[0]
        y = point[1]
        theta = np.deg2rad(point[2])

        if direction == 'left60':
            phi=np.deg2rad(60)
            x=x+d*math.cos(phi)*math.cos(phi)
            y=x+d*math.sin(phi)*math.sin(phi)

        elif direction == 'left30':
            phi=np.deg2rad(30)
            x=x+d*math.cos(phi)*math.cos(phi)
            y=x+d*math.sin(phi)*math.sin(phi)

        elif direction == 'straight':
            phi=0
            x=x+d*math.cos(theta)
            y=y+d*math.sin(theta)

        elif direction == 'right30':
            phi=np.deg2rad(-30)
            x=x+d*math.cos(phi)*math.cos(phi)
            y=x+d*math.sin(phi)*math.sin(phi)

        elif direction == 'right60':
            phi=np.deg2rad(-60)
            x=x+d*math.cos(phi)*math.cos(phi)
            y=x+d*math.sin(phi)*math.sin(phi)

        theta = round(np.rad2deg(theta+phi))
        if theta >= 360:
            theta = theta-360

        new_point = (x,y,theta)
        plotter(self.maze.ax,point,new_point)
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
            parent = queue.pop(0)[0]

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
                    queue.append((len(self.nodes)-1,cost2come+self.d+cost2goal))
                elif cost2come + self.d < self.costs2come[disc_p[0],disc_p[1],disc_p[2]]:
                    self.costs2come[disc_p[0],disc_p[1],disc_p[2]] = cost2come+self.d
                    self.parents[disc_p[0],disc_p[1],disc_p[2]] = parent
                if disc_p == self.discretize(self.goal):
                    self.foundGoal = True
                    queue.clear()
                    break


    def generate_path(self):
        nodes = self.nodes
        parents = self.parents
        #Assume the last item in nodes is the goal node
        goal = nodes[-1]
        parent = parents[goal[1],goal[0]]
        path_nodes = [parent]
        while parent != -1:
            parent_node = nodes[path_nodes[-1]]
            parent = parents[parent_node[1],parent_node[0]]
            path_nodes.append(parent)
        self.path = [goal]
        for ind in path_nodes:
            if ind == -1:
                break
            else:
                self.path.insert(0,nodes[ind])


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
                goal_str_th = input('goal theta: ')
                try:
                    goal_point = (float(goal_str_x),float(goal_str_y),int(goal_str_th))
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
            if self.discretize(start_point) == self.discretize(goal_point):
                print('The start and goal cannot be the same point')
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


        self.start = start_point
        self.goal = goal_point
        self.d = d


class PointRobot(Robot):
    def __init__(self,maze,userInput):
        super().__init__(maze,userInput)
        self.offset = 0

    def visualize(self,show,output,stepsize):
        if output:
            fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')
            frame_size = (self.maze.image.shape[1], self.maze.image.shape[0])
            today = time.strftime("%m-%d__%H.%M.%S")
            videoname=str(today)
            fps_out = 60
            out = cv2.VideoWriter(str(videoname)+".mp4", fourcc, fps_out, frame_size)
            print("Writing to Video, Please Wait")

        cur_frame = 1
        tot_frames = (len(self.nodes)//stepsize)+1
  
        for i,point in enumerate(self.nodes):
            if self.maze.scale == 1:
                self.image[point[1],point[0]] = (0,255,255)
            else:
                sx = point[0]*self.maze.scale
                sy = (self.maze.height-point[1])*self.maze.scale
                ex = sx+self.maze.scale
                ey = sy+self.maze.scale
                cv2.rectangle(self.maze.image,(sx,sy),(ex,ey),(0,255,255),-1)
            if i == 5000:
                cv2.imwrite('Images/searched_nodes.png',self.maze.image)
            
            if i%stepsize == 0:
                if output:
                    print('Frame number:' + str(cur_frame) + ' of ' + str(tot_frames))
                    out.write(self.maze.image)
                    time.sleep(0.005)
                    cur_frame += 1
                if show:
                    cv2.imshow('Maze Visualization',self.maze.image)
                if cv2.waitKey(1) == ord('q'):
                    exit()

        for point in self.path:
            sx = point[0]*self.maze.scale
            sy = (self.maze.height-point[1])*self.maze.scale
            cv2.circle(self.maze.image,(sx,sy),self.maze.scale,(0,0,255),-1)
            if output:
                out.write(self.maze.image)
                time.sleep(0.005)
            if show:
                cv2.imshow('Maze Visualization',self.maze.image)
            if cv2.waitKey(1) == ord('q'):
                exit()
            if point == self.maze.goal:
                # cv2.imwrite('Images/solution.png',self.maze.image)
                cv2.waitKey(0)
        if output:
            out.release()


class RigidRobot(Robot):
    def __init__(self,maze):
        super().__init__(maze)
        self.get_params()

    def get_params(self):
        print('Please enter the size of your robot')
        size_str = input('radius: ')
        if size_str.isdigit():
            self.radius = int(size_str)
        else:
            print('Please enter a number')
            exit()
        
        print('Please enter the clearance for your robot')
        clear_str = input('clearance: ')
        if clear_str.isdigit():
            self.clearance = int(clear_str)
        else:
            print('Please enter a number')
            exit()

        self.offset = self.radius+self.clearance


    def visualize(self,show,output,stepsize):
        node_color = (102, 255, 255)
        if output:
            fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')
            frame_size = (self.maze.image.shape[1], self.maze.image.shape[0])
            today = time.strftime("%m-%d__%H.%M.%S")
            videoname=str(today)
            fps_out = 60
            out = cv2.VideoWriter(str(videoname)+".mp4", fourcc, fps_out, frame_size)
            print("Writing to Video, Please Wait")

        cur_frame = 1
        tot_frames = (len(self.nodes)//stepsize)+1
  
        for i,point in enumerate(self.nodes):
            if self.maze.scale == 1:
                self.image[point[1],point[0]] = node_color
            else:
                sx = point[0]*self.maze.scale
                sy = (self.maze.height-point[1])*self.maze.scale
                ex = sx+self.maze.scale
                ey = sy+self.maze.scale
                cv2.rectangle(self.maze.image,(sx,sy),(ex,ey),node_color,-1)

            if i%stepsize == 0:
                if output:
                    print('Frame number:' + str(cur_frame) + ' of ' + str(tot_frames))
                    out.write(self.maze.image)
                    time.sleep(0.005)
                    cur_frame += 1
                if show:
                    cv2.imshow('Maze Visualization',self.maze.image)
                    
                if cv2.waitKey(1) == ord('q'):
                    exit()

        # self.maze.contract_obstacles(self.radius)

        for point in self.path:
            sx = point[0]*self.maze.scale
            sy = (self.maze.height-point[1])*self.maze.scale
            cv2.circle(self.maze.image,(sx,sy),self.maze.scale*self.radius,(0,0,255),-1)
            if output:
                out.write(self.maze.image)
                time.sleep(0.005)
            if show:
                cv2.imshow('Maze Visualization',self.maze.image)
            if cv2.waitKey(1) == ord('q'):
                exit()
            if point == self.maze.goal:
                #cv2.imwrite('searched_nodes.png',self.maze.image)
                cv2.waitKey(0)

        if output:
            out.release()
