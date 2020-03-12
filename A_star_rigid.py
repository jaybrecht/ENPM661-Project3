import cv2 
from maze import Maze
from robot import RigidRobot

write_to_video = False
show_visualization = True
search_type = 'D' # D for Dijkstra, B for BFS
stepsize = 100 #controls the number of nodes shown in each frame of visualization

# Construct maze object
scale = 5
maze = Maze('maze2.txt',scale)

# Ask user for start point and goal point
maze.get_user_nodes()

# Construct the robot
robot = RigidRobot(maze)

# Expand obstacles
robot.maze.expand_obstacles(robot.radius+robot.clearance)

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
