import cv2 
import math
import numpy as np
# from shapely.geometry import Polygon
from matplotlib.collections import PatchCollection
from matplotlib.patches import Ellipse, Circle, Wedge, Polygon
import matplotlib.pyplot as plt

class Maze:
    def __init__(self,filename,scale,ax):
        # instatiates an object of class maze
        self.filename = filename
        self.scale = scale
        self.patches=[]
        self.ax=ax
        self.read_obstacles()        

        self.image = np.zeros((self.height*scale,self.width*scale,3),np.uint8)

        for obs in self.obstacles:
            if obs['type'] == 'c': # circle
                self.draw_circle(obs,0,obs['color'])
                
            elif obs['type'] == 'p': # polygon
                self.draw_polygon(obs,0,obs['color'])

            elif obs['type'] == 'e': # ellipse
                self.draw_ellipse(obs,0,obs['color'])
                
            elif obs['type'] == 'rr': # rotate rect
                self.draw_rotated_rect(obs,0,obs['color'])
                self.get_rr_points(obs)


    def in_bounds(self,point):
        x = point[0]
        y = point[1]
        if 0<=x<=self.width-1 and 0<=y<=self.height-1:
            return True
        else:
            return False


    def in_obstacle(self,node,offset):
        x = node[0]
        y = node[1]
       
        for obs in self.obstacles:
            if obs['type'] == 'c':
                center = obs['center']
                radius = obs['radius']+ offset
                if ((x-center[0])**2 + (y-center[1])**2 <= radius**2):
                    return True
            elif obs['type'] == 'e':
                center = obs['center']
                a1 = obs['axis'][0]+ offset
                a2 = obs['axis'][1]+ offset
                if ((((x-center[0])**2)/a1**2) + (((y-center[1])**2)/a2**2) <= 1):
                    return True
            elif obs['type'] == 'p' or obs['type'] == 'r':
                points=obs['points'].copy()
                points.append(points[0])
                contour = np.array(obs['points'], dtype=np.int32)    
                topx,topy,w,h = cv2.boundingRect(contour)
                topx -= offset
                topy -= offset
                w += offset*2
                h += offset*2
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

                a,b,c = [],[],[]
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
                count=0
  
                for i in range(len(a)):
                    if ((a[i]*x + b[i]*y + c[i]) >= 0):
                        count+=1
                    if count==len(a):
                        return True
        return False


    def draw_circle(self,obs,offset,color):
        # Draws a circle on the maze image
        x = obs['center'][0]
        y = self.height - obs['center'][1]
        center = (x*self.scale,y*self.scale)
        radius = obs['radius']*self.scale

        self.image = cv2.circle(self.image,center,radius+(offset*self.scale),color,-1)
        circle = Circle((x, y), radius)
        self.patches.append(circle)
        # print(self.patches)


    def draw_polygon(self,obs,offset,color):
        # Draws a polygon on the maze image
        points = []
        for p in obs['points']:
            points.append((p[0]*self.scale,(self.height*self.scale)-p[1]*self.scale))
        
        contour = np.array(points, dtype=np.int32)
        
        self.image = cv2.drawContours(self.image,[contour],-1,color,-1)
        polygon = Polygon(points, True)
        self.patches.append(polygon)
        # print(self.patches)
                

    def draw_ellipse(self,obs,offset,color):
        # Draws an ellipse on the maze image
        x = obs['center'][0]
        y = self.height - obs['center'][1]
        center = (x*self.scale,y*self.scale)
        axis = (obs['axis'][0]*self.scale+(offset*self.scale),
                       obs['axis'][1]*self.scale+(offset*self.scale))
        self.image = cv2.ellipse(self.image, center, axis, obs['angle'],
                        obs['start'], obs['end'],color,-1)
        ellipse = Ellipse((x,y),axis[0],axis[1])
        self.patches.append(ellipse)
        # print(self.patches)


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
            polygon = Polygon(points, True)
            self.patches.append(polygon) 
        else:
            off_contour = np.squeeze(contour)
            polygon = Polygon(off_contour)
            offset_poly = polygon.buffer(offset*self.scale,cap_style=2, join_style=2)
            off_points = offset_poly.exterior.coords
            off_contour = np.array(off_points, dtype=np.int32)
            self.image = cv2.drawContours(self.image,[off_contour],-1,color,-1)


    def get_rr_points(self,obs):
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
        obs['points'] = points           


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

    

if __name__ == '__main__':
    maze = 'maze2'
    mymaze = Maze(maze+'.txt',1)
    cv2.imshow('Maze2',mymaze.image)
    cv2.waitKey(0)
    print(mymaze.in_obstacle((251.5,150),2))