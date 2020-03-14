import matplotlib.pyplot as plt
import numpy as np
import math


fig, ax = plt.subplots()
plt.xlim(0,30)
plt.ylim(0,20)
plt.grid()
plt.xlabel('X')
plt.ylabel('Y')
plt.title("Moves")

def plotter(start_pos,end_pos,color="black"):
	# start_pos= (x,y,theta)
	x_s=np.array((start_pos[0]))
	y_s=np.array((start_pos[1]))
	# theta_s=start_pos[2]

	x_f=np.array((end_pos[0]))
	y_f=np.array((end_pos[1]))
	# theta_f=end_pos[2]

	dx=x_f-x_s
	dy=y_f-y_s

	q = ax.arrow(x_s, y_s, dx, dy, head_width=0.5,length_includes_head=True, head_length=0.5, fc=color, ec=color)
	# ax.annotate('', xy=(x_s,y_s), xytext=(x_f,y_f), arrowprops={'arrowstyle': '->'}, va='center')




start_pos=[1,6]
end_pos=[0,0]
end_pos[0]=start_pos[0]+4*math.cos(0.523599)
end_pos[1]=start_pos[1]+4*math.sin(0.523599)
plotter(start_pos,end_pos)
end_pos[0]=start_pos[0]+4*math.cos(-0.523599)
end_pos[1]=start_pos[1]+4*math.sin(-0.523599)
plotter(start_pos,end_pos)

end_pos[0]=start_pos[0]+4*math.cos(2*0.523599)
end_pos[1]=start_pos[1]+4*math.sin(2*0.523599)
plotter(start_pos,end_pos)
end_pos[0]=start_pos[0]+4*math.cos(-2*0.523599)
end_pos[1]=start_pos[1]+4*math.sin(-2*0.523599)
plotter(start_pos,end_pos)

end_pos[0]=start_pos[0]+4
end_pos[1]=start_pos[1]
plotter(start_pos,end_pos,'r')





start_pos[0]=end_pos[0]
start_pos[1]=end_pos[1]





end_pos[0]=start_pos[0]+4*math.cos(0.523599)
end_pos[1]=start_pos[1]+4*math.sin(0.523599)
plotter(start_pos,end_pos)
end_pos[0]=start_pos[0]+4*math.cos(-0.523599)
end_pos[1]=start_pos[1]+4*math.sin(-0.523599)
plotter(start_pos,end_pos)

end_pos[0]=start_pos[0]+4*math.cos(2*0.523599)
end_pos[1]=start_pos[1]+4*math.sin(2*0.523599)
plotter(start_pos,end_pos,'r')
end_pos[0]=start_pos[0]+4*math.cos(-2*0.523599)
end_pos[1]=start_pos[1]+4*math.sin(-2*0.523599)
plotter(start_pos,end_pos)

end_pos[0]=start_pos[0]+4
end_pos[1]=start_pos[1]
plotter(start_pos,end_pos)


plt.show()





