import cv2 
import math
import numpy as np
# from shapely.geometry import Polygon

class Maze:
    def __init__(self,filename,scale):
        # instatiates an object of class maze
       
        self.filename = filename
        self.scale = scale
        self.read_obstacles()

        self.image = np.zeros((self.height*scale,self.width*scale,3),np.uint8)
        self.maze = np.zeros((self.height,self.width),dtype=np.uint8)

        for obs in self.obstacles:
            if obs['type'] == 'c': # circle
                self.draw_circle(obs,0,obs['color'])
                self.define_circle(obs, 0)
                
            elif obs['type'] == 'p': # polygon
                self.draw_polygon(obs,0,obs['color'])
                self.define_polygon(obs, 0)

            elif obs['type'] == 'e': # ellipse
                self.draw_ellipse(obs,0,obs['color'])
                self.define_ellipse(obs, 0)
                
            elif obs['type'] == 'rr': # rotate rect
                self.draw_rotated_rect(obs,0,obs['color'])
                self.define_rotated_rect(obs,0)

        # maze_not_scaled = cv2.resize(self.image,(self.width,self.height))
        # inds=np.nonzero(maze_not_scaled)
        
        # for i in range(len(inds[0])):
        #     x = inds[1][i]
        #     y = inds[0][i]
        #     self.maze[y,x] = 1


    def in_maze(self,point):
        # Checks whether a point is in bounds and not an obstacle
        x = point[0]
        y = point[1]
        if 0<=x<self.width and 0<=y<self.height:
            if self.maze[y,x] != 1:
                return True
        return False

    def draw_circle(self,obs,offset,color):
        # Draws a circle on the maze image
        x = obs['center'][0]
        y = self.height - obs['center'][1]
        center = (x*self.scale,y*self.scale)
        radius = obs['radius']*self.scale

        self.image = cv2.circle(self.image,center,radius+(offset*self.scale),color,-1)


    def define_circle(self,obs,offset):
        # Write code that modifies that attribute maze to have 1s everywhere inside
        # of the obstacle obs. Should expand the obstacle by the offset
        center = obs['center']
        radius = obs['radius']+offset
        topx = center[0]-radius
        topy = center[1]-radius
        for x in range(topx, topx+2*radius+1):
            for y in range(topy, topy+2*radius+1):
                if ((x-center[0])**2 + (y-center[1])**2 <= radius**2):
                    if self.in_maze((x,y)):
                        self.maze[y,x]=1


    def draw_polygon(self,obs,offset,color):
        # Draws a polygon on the maze image
        points = []
        for p in obs['points']:
            points.append((p[0]*self.scale,(self.height*self.scale)-p[1]*self.scale))
        
        contour = np.array(points, dtype=np.int32)
        
        self.image = cv2.drawContours(self.image,[contour],-1,color,-1) 
        # else:
            # off_contour = np.squeeze(contour)
            # polygon = Polygon(off_contour)
            # offset_poly = polygon.buffer(offset*self.scale,cap_style=2, join_style=2)
            # off_points = offset_poly.exterior.coords
            # off_contour = np.array(off_points, dtype=np.int32)
            # self.image = cv2.drawContours(self.image,[off_contour],-1,color,-1) 


    def define_polygon(self,obs,offset):
        # Write code that modifies that attribute maze to have 1s everywhere inside
        # of the obstacle obs. Should expand the obstacle by the offset
        points=obs['points'].copy()
        points.append(points[0])
        contour = np.array(obs['points'], dtype=np.int32)    
        topx,topy,w,h = cv2.boundingRect(contour)
        topx -= offset
        topy -= offset
        w += offset*2
        h += offset*2
        # cv2.rectangle(self.maze,(topx,topy),(topx+w,topy+h),(255),1)
        test = (topx+w//2, topy+h//2)

        point_sets = []
        for i in range(len(points)-1):
            p = np.array([points[i][0],points[i][1],0])
            q = np.array([points[i+1][0],points[i+1][1],0])
            v1 = q-p
            v1_hat = v1/ np.linalg.norm(v1)
            z = np.array([0,0,1])
            off_hat = np.cross(z,v1_hat)
            off_vect = off_hat*offset
            new_p = p+off_vect
            new_q = q+off_vect
            point_sets.append([(new_p[0],new_p[1]),(new_q[0],new_q[1])])

        a,b,c=[],[],[]
        for i in range(len(point_sets)):
            p1 = point_sets[i][0]
            p2 = point_sets[i][1]
            ai=(p1[1]-p2[1])
            bi=(p2[0]-p1[0])
            ci=((p1[0]*p2[1])-(p2[0]*p1[1]))
            if (ai*test[0]+bi*test[1]+ci<0):
                ai*=-1
                bi*=-1
                ci*=-1
            a.append(ai)
            b.append(bi)
            c.append(ci)
        
        for x in range(topx, topx+w+1):
            for y in range(topy, topy+h+1):
                count=0    
                for i in range(len(a)):
                    if ((a[i]*x + b[i]*y + c[i]) >= 0):
                        count+=1
                if count==len(a):
                    if offset == 0:
                        if self.in_maze((x,y)):
                            self.maze[y,x]=1
                    else:
                        if self.in_maze((x,y)):
                            self.maze[y,x]=1
                

    def draw_ellipse(self,obs,offset,color):
        # Draws an ellipse on the maze image
        x = obs['center'][0]
        y = self.height - obs['center'][1]
        center = (x*self.scale,y*self.scale)
        axis = (obs['axis'][0]*self.scale+(offset*self.scale),
                       obs['axis'][1]*self.scale+(offset*self.scale))
        self.image = cv2.ellipse(self.image, center, axis, obs['angle'],
                        obs['start'], obs['end'],color,-1)


    def define_ellipse(self,obs,offset):
        # Write code that modifies that attribute maze to have 1s everywhere inside
        # of the obstacle obs. Should expand the obstacle by the offset
        center = obs['center']
        a1 = obs['axis'][0]+offset
        a2 = obs['axis'][1]+offset
        topx = center[0]-a1
        topy = center[1]-a2
        for x in range(topx, topx+2*a1+1):
            for y in range(topy, topy+2*a2+1):
                if ((((x-center[0])**2)/a1**2) + (((y-center[1])**2)/a2**2) <= 1):
                    if self.in_maze((x,y)):
                        self.maze[y,x]=1


    def draw_rotated_rect(self,obs,offset,color):
        # Draws a rotated rectangle on the maze image
        w = obs['width']
        h = obs['height']
        ang1 = math.radians(obs['angle'])
        ang2 = math.radians(90-obs['angle'])
        p1 = obs['start_point']
        p2 = (p1[0]-(w*math.cos(ang1)),p1[1]+(w*math.sin(ang1)))
        p3 = (p1[0]+(h*math.cos(ang2)),p1[1]+(h*math.sin(ang2)))
        p4 = (p3[0]-(w*math.cos(ang1)),p3[1]+(w*math.sin(ang1)))
        points = [p1,p2,p4,p3]
        spoints = []
        for p in points:
            spoints.append((p[0]*self.scale,(self.height*self.scale)-p[1]*self.scale))
        contour = np.array(spoints, dtype=np.int32)
        
        if offset == 0:
            self.image = cv2.drawContours(self.image,[contour],-1,color,-1) 
        else:
            off_contour = np.squeeze(contour)
            polygon = Polygon(off_contour)
            offset_poly = polygon.buffer(offset*self.scale,cap_style=2, join_style=2)
            off_points = offset_poly.exterior.coords
            off_contour = np.array(off_points, dtype=np.int32)
            self.image = cv2.drawContours(self.image,[off_contour],-1,color,-1)


    def define_rotated_rect(self,obs,offset):
        # Write code that modifies that attribute maze to have 1s everywhere inside
        # of the obstacle obs. Should expand the obstacle by the offset, may be easier
        # to just define the points and pass them into define_polygon
        w = obs['width']
        h = obs['height']
        ang1 = math.radians(obs['angle'])
        ang2 = math.radians(90-obs['angle'])
        p1 = obs['start_point']
        p2 = (p1[0]-(w*math.cos(ang1)),p1[1]+(w*math.sin(ang1)))
        p3 = (p1[0]+(h*math.cos(ang2)),p1[1]+(h*math.sin(ang2)))
        p4 = (p3[0]-(w*math.cos(ang1)),p3[1]+(w*math.sin(ang1)))
        points = [p1,p2,p4,p3]
        obs = {'points': points,'type':'r'}
        self.define_polygon(obs, offset)
        

    def expand_obstacles(self,offset):
        off_color = (255,102,0)
        for obs in self.obstacles:
            if obs['type'] == 'c': # circle
                # self.draw_circle(obs,offset,off_color)
                # self.draw_circle(obs,0,obs['color'])
                self.define_circle(obs, offset)

            elif obs['type'] == 'p': # polygon
                # self.draw_polygon(obs,offset,off_color)    
                # self.draw_polygon(obs,0,obs['color'])  
                self.define_polygon(obs, offset)       

            elif obs['type'] == 'e': # ellipse
                # self.draw_ellipse(obs,offset,off_color)
                # self.draw_ellipse(obs,0,obs['color'])
                self.define_ellipse(obs, offset)

            elif obs['type'] == 'rr': # rotate rect
                # self.draw_rotated_rect(obs,offset,off_color)
                # self.draw_rotated_rect(obs,0,obs['color'])
                self.define_rotated_rect(obs, offset)

        # maze_not_scaled = cv2.resize(self.image,(self.width,self.height))
        # self.maze = np.zeros((self.height,self.width),dtype=np.uint8)
        # inds=np.nonzero(maze_not_scaled)
        
        # for i in range(len(inds[0])):
        #     x = inds[1][i]
        #     y = self.height-inds[0][i]
        #     if self.in_maze((x,y)):
        #         self.maze[y,x] = 1


    def contract_obstacles(self,offset):
        empty_color = (0,0,0)
        for obs in self.obstacles:
            if obs['type'] == 'c': # circle
                self.draw_circle(obs,offset,empty_color)
                self.draw_circle(obs,0,obs['color'])

            elif obs['type'] == 'p': # polygon
                self.draw_polygon(obs,offset,empty_color)    
                self.draw_polygon(obs,0,obs['color'])        

            elif obs['type'] == 'e': # ellipse
                self.draw_ellipse(obs,offset,empty_color)
                self.draw_ellipse(obs,0,obs['color'])

            elif obs['type'] == 'rr': # rotate rect
                self.draw_rotated_rect(obs,offset,empty_color)
                self.draw_rotated_rect(obs,0,obs['color'])
                


    def read_obstacles(self):
        # reads in obstacles from a text file. Supports four types of obstacles,
        # circles,polygons,ellipses,and rotatedrects. Returns a list where each
        # obstacle is a dictionary 
        maze_file = open(self.filename,"r")
        lines = maze_file.readlines()
        default_color = (255,0,0)
        obstacles = []

        for i,line in enumerate(lines):
            if line.split(':')[0].strip() == 'height':
                h = line.split(':')[-1]
                self.height = int(h)
            if line.split(':')[0].strip() == 'width':
                w = line.split(':')[-1]
                self.width = int(w)
            if line == 'circle\n':
                j = i+1
                next_line = lines[j]
                center = radius = None
                color = default_color
                while next_line != '\n':
                    if next_line.split(':')[0].strip() == 'center':
                        p = next_line.split(':')[-1].split(',')
                        center = (int(p[0]),int(p[1]))
                    if next_line.split(':')[0].strip() == 'radius':
                        radius = int(next_line.split(':')[-1])
                    if next_line.split(':')[0].strip() == 'color':
                        bgr = next_line.split(':')[-1].split(',')
                        color = (int(bgr[0]),int(bgr[1]),int(bgr[2]))
                    j += 1
                    if j<len(lines):
                        next_line = lines[j]
                    else:
                        break

                if radius!=None and center!=None:
                    obs = {'type':'c','center':center,'radius':radius,'color':color}
                    obstacles.append(obs)
            
            if line == 'polygon\n':
                points = []
                color = default_color
                j = i+1
                next_line = lines[j]
                while next_line != '\n':
                    if next_line.split(':')[0].strip() == 'point':
                        p = next_line.split(':')[-1].split(',')
                        points.append((int(p[0]),int(p[1])))
                    if next_line.split(':')[0].strip() == 'color':
                        bgr = next_line.split(':')[-1].split(',')
                        color = (int(bgr[0]),int(bgr[1]),int(bgr[2]))
                    j += 1
                    if j<len(lines):
                        next_line = lines[j]
                    else:
                        break

                if len(points)>0:
                    obs = {'type':'p','points':points,'color':color}
                    obstacles.append(obs)
            
            if line == 'ellipse\n': 
                center = axis = None
                angle = start = 0
                end = 360
                color = default_color

                j = i+1
                next_line = lines[j]

                while next_line != '\n':
                    if next_line.split(':')[0].strip() == 'center':
                        p = next_line.split(':')[-1].split(',')
                        center = (int(p[0]),int(p[1]))
                    if next_line.split(':')[0].strip() == 'axis':
                        p = next_line.split(':')[-1].split(',')
                        axis = (int(p[0]),int(p[1]))
                    if next_line.split(':')[0].strip() == 'angle':
                        angle = int(next_line.split(':')[-1])
                    if next_line.split(':')[0].strip() == 'start':
                        radius = int(next_line.split(':')[-1])
                    if next_line.split(':')[0].strip() == 'end':
                        radius = int(next_line.split(':')[-1])
                    if next_line.split(':')[0].strip() == 'color':
                        bgr = next_line.split(':')[-1].split(',')
                        color = (int(bgr[0]),int(bgr[1]),int(bgr[2]))
                    j += 1
                    if j<len(lines):
                        next_line = lines[j]
                    else:
                        break
                
                if radius!=None and axis!=None:
                    obs = {'type':'e','center':center,'axis':axis,'angle':angle,
                           'start':start,'end':end,'color':color}
                    obstacles.append(obs)

            if line == 'rotatedrect\n': 
                start_point = height = width = angle = None
                color = default_color

                j = i+1
                next_line = lines[j]

                while next_line != '\n':
                    if next_line.split(':')[0].strip() == 'start_point':
                        p = next_line.split(':')[-1].split(',')
                        start_point = (int(p[0]),int(p[1]))
                    if next_line.split(':')[0].strip() == 'l1':
                        height = int(next_line.split(':')[-1])
                    if next_line.split(':')[0].strip() == 'l2':
                        width = int(next_line.split(':')[-1])
                    if next_line.split(':')[0].strip() == 'angle':
                        angle = int(next_line.split(':')[-1])
                    if next_line.split(':')[0].strip() == 'color':
                        bgr = next_line.split(':')[-1].split(',')
                        color = (int(bgr[0]),int(bgr[1]),int(bgr[2]))
                    j += 1
                    if j<len(lines):
                        next_line = lines[j]
                    else:
                        break
                
                if start_point!=None and height!=None and width!=None and angle!=None:
                    obs = {'type':'rr','start_point':start_point,'height':height,
                           'width':width,'angle':angle,'color':color}
                    obstacles.append(obs)

        maze_file.close()
        self.obstacles = obstacles
    

    def get_user_nodes(self):
        print('Please enter a start point (x,y)')
        start_str_x = input('start x: ')
        start_str_y = input('start y: ')
        start_point = (int(start_str_x),int(start_str_y))

        # Check if start point is valid in maze 
        if self.in_maze(start_point):
            pass
        else:
            print("The start point is not valid")
            exit()
            
        print('Please enter a goal point (x,y)')
        start_str_x = input('start x: ')
        start_str_y = input('start y: ')
        goal_point = (int(start_str_x),int(start_str_y))

        # Check if goal point is valid in maze 
        if self.in_maze(goal_point):
            pass
        else:
            print("The goal point is not valid")
            exit()

        self.start = start_point
        self.goal = goal_point


if __name__ == '__main__':
    maze = 'maze1'
    mymaze = Maze(maze+'.txt',5)
    cv2.imwrite('Images/maze1.png',mymaze.image)

