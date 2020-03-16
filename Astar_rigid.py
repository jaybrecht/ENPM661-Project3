# Import Python functions
import cv2 
from datetime import datetime as dtime

# Import our classes
from maze import Maze
from robot import Robot

write_to_video = False
show_visualization = True
userInput = True

# Construct maze object
maze = Maze('maze.txt')

# Contstruct the robot
robot = Robot(maze,userInput)

# Record the start time so we can compute run time at the end
starttime = dtime.now()

# Run Search
robot.A_star()

if robot.foundGoal:
    searchtime=dtime.now()
    searchtime=searchtime-starttime
    robot.generate_path()
    print('Found Path in '+str(searchtime)+' (hours:min:sec)')

else:
    print('The goal could not be found')
    exit()

# Visualize the path
robot.visualize(write_to_video,show_visualization)
endtime = dtime.now()
runtime=endtime-starttime
print("Finished in "+str(runtime)+" (hours:min:sec)")


