import cv2
import numpy as np
import math
import time
from collections import deque

class Robot:
    def __init__(self,maze):
        self.maze = maze

    def move(self,point,direction):
        x = point[0]
        y = point[1]
        if direction == 'E':
            new_point = (x-1,y)
        elif direction == 'NE':
            new_point = (x-1,y+1)
        elif direction == 'N':
            new_point = (x,y+1)
        elif direction == 'NW':
            new_point = (x+1,y+1)
        elif direction == 'W':
            new_point = (x+1,y)
        elif direction == 'SW':
            new_point = (x+1,y-1)
        elif direction == 'S':
            new_point = (x,y-1)
        elif direction == 'SE':
            new_point = (x-1,y-1)

        return new_point


    def check_neighbors(self,cur_node):
        directions = ['E','NE','N','NW','W','SW','S','SE']

        neighbors = []
        for direction in directions:
            new_point = self.move(cur_node,direction)
            if self.maze.in_maze(new_point):
                if len(direction) == 2:
                    cost = math.sqrt(2)
                else:
                    cost = 1

                neighbors.append((new_point,cost))

        return neighbors


    def BFS(self):
        start_point = self.maze.start
        goal_point = self.maze.goal
        maze = self.maze.maze
        nodes = []

        # Checked points are additionally stored in a set which is much faster for 
        # checking if the node has already been visited
        points = {start_point}
        parents = np.full((maze.shape[0],maze.shape[1]),np.nan,dtype=np.int32)

         #set start node to have parent of -1 and cost of 0
        nodes.append(start_point) #add the start node to nodes
        parents[start_point[1],start_point[0]] = -1

        # The queue is strucuted as a deque which allows for much faster operation
        # when accessing the first item in the list
        queue = deque()
        queue.append(0) #set the start_node as the first node in the queue

        isgoal = False

        while queue:
            # Set the current node as the top of the queue and remove it
            parent = queue.popleft()
            cur_node = nodes[parent]
            neighbors = self.check_neighbors(cur_node)

            for n in neighbors:
                p = n[0]
                if p not in points:
                    nodes.append(p)
                    points.add(p)
                    queue.append(len(nodes)-1)
                    parents[p[1],p[0]] = parent
                if p == goal_point:
                    isgoal = True
                    queue.clear()
                    break

        self.nodes = nodes
        self.parents = parents
        self.foundGoal = isgoal

    def Dijkstra(self):
        start_point = self.maze.start
        goal_point = self.maze.goal
        maze = self.maze.maze
        
        nodes = []

        # Checked points are additionally stored in a set which is much faster for 
        # checking if the node has already been visited
        points = {start_point}
        costs = np.full((maze.shape[0],maze.shape[1]),np.inf)
        parents = np.full((maze.shape[0],maze.shape[1]),np.nan,dtype=np.int32)

         #set start node to have parent of -1 and cost of 0
        nodes.append(start_point) #add the start node to nodes
        costs[start_point[1],start_point[0]] = 0
        parents[start_point[1],start_point[0]] = -1

        # The queue is strucuted as a deque which allows for much faster operation
        # when accessing the first item in the list
        queue = deque()
        queue.append(0) #set the start_node as the first node in the queue

        isgoal = False
        cost2come = 0

        while queue:
            # Set the current node as the top of the queue and remove it
            parent = queue.popleft()
            cur_node = nodes[parent]
            cost2come = costs[cur_node[1],cur_node[0]]
            neighbors = self.check_neighbors(cur_node)

            for n in neighbors:
                p = n[0]
                c = n[1]
                if p not in points:
                    nodes.append(p)
                    points.add(p)
                    queue.append(len(nodes)-1)
                if cost2come + c < costs[p[1],p[0]]:
                    costs[p[1],p[0]] = cost2come + c
                    parents[p[1],p[0]] = parent
                if p == goal_point:
                    isgoal = True
                    queue.clear()
                    break

        self.nodes = nodes
        self.parents = parents
        self.costs = costs
        self.foundGoal = isgoal


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

    


class PointRobot(Robot):
    def __init__(self,maze):
        super().__init__(maze)

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