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


# Set up plotter
fig, ax = plt.subplots()
plt.xlim(0,300)
plt.ylim(0,200)
plt.grid()
plt.xlabel('X')
plt.ylabel('Y')
plt.title("Maze")
ax.set_aspect('equal')
# ax.imshow(x, origin='upper', interpolation=interp)
# ax.yaxis.set_ticks(np.arange(200, 0, 20)) # set y-ticks

# Construct maze object
scale = 1
maze = Maze('maze2.txt', scale,ax)

# print(maze.patches)
p = PatchCollection(maze.patches, alpha=1)
colors = 100*np.random.rand(len(maze.patches))
p.set_array(np.array(colors))
maze.ax.add_collection(p)
plt.show()




# # Contstruct the robot
# robot = PointRobot(maze,userInput)

# # Run Search
# # if search_type == 'D':
# #     robot.Dijkstra()
# # if search_type == 'B':
# #     robot.BFS()
# if search_type == 'A':
#     robot.A_star()

# if robot.foundGoal:
#     print('Yay')
#     # robot.generate_path()
# else:
#     print('The goal could not be found')
#     exit()

# # Visualize the path
# # robot.visualize(show_visualization,write_to_video,stepsize)
