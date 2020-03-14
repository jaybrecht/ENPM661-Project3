import cv2 
from maze import Maze
from robot import PointRobot
import matplotlib.pyplot as plt
from matplotlib.collections import PatchCollection
import numpy as np

write_to_video = False
show_visualization = False
userInput = False
search_type = 'A' # D for Dijkstra, B for BFS, A for A*
stepsize = 100 #controls the number of nodes shown in each frame of visualization

# Construct maze object
maze = Maze('maze2.txt')
print("Maze created")

# Contstruct the robot
robot = PointRobot(maze,userInput)
print("Robot created")

# Run Search
if search_type == 'D':
    robot.Dijkstra()
if search_type == 'B':
    robot.BFS()
if search_type == 'A':
    robot.A_star()


if robot.foundGoal:
    robot.generate_path()
    for i in range(len(robot.path)-1):
        robot.plotter(robot.path[i],robot.path[i+1])
    print(robot.path)
else:
    print('The goal could not be found')
    exit()

plt.show()
# # # Visualize the path
# # # robot.visualize(show_visualization,write_to_video,stepsize)
