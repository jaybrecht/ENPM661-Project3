import matplotlib.pyplot as plt
import numpy as np


fig, ax = plt.subplots()

def plotter(start_pos,end_pos):
	# start_pos= (x,y,theta)
	x_s=np.array((start_pos[0]))
	y_s=np.array((start_pos[1]))
	# theta_s=start_pos[2]

	x_f=np.array((end_pos[0]))
	y_f=np.array((end_pos[1]))
	# theta_f=end_pos[2]

	dx=x_f-x_s
	dy=y_f-y_s

	q = ax.arrow(x_s, y_s, dx, dy, head_width=0.5,length_includes_head=True, head_length=0.5, fc='k', ec='k')
	# ax.annotate('', xy=(x_s,y_s), xytext=(x_f,y_f), arrowprops={'arrowstyle': '->'}, va='center')





plotter((1,2),(5,3))
plotter((5,3),(4,4))
plotter((4,4),(1,3))

plt.xlim(0,30)
plt.ylim(0,20)
plt.grid()
plt.xlabel('X')
plt.ylabel('Y')
plt.title("Moves")
plt.show()





