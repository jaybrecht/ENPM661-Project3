import cv2 
from maze import Maze
from robot import PointRobot

write_to_video = False
show_visualization = True
search_type = 'D' # D for Dijkstra, B for BFS
stepsize = 100 #controls the number of nodes shown in each frame of visualization

# Construct maze object
scale = 3
maze = Maze('maze2.txt',scale)

# Ask user for start point and goal point
maze.get_user_nodes()

# Contstruct the robot
robot = PointRobot(maze)

# Run Search
if search_type == 'D':
    robot.Dijkstra()
if search_type == 'B':
    robot.BFS()

if robot.foundGoal:
    robot.generate_path()
else:
    print('The goal could not be found')
    exit()

# Visualize the path
robot.visualize(show_visualization,write_to_video,stepsize)
