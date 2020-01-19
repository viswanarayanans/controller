import numpy as np
import matplotlib.pyplot as plt
import os
import sys
import time
import re
from mpl_toolkits.mplot3d import Axes3D



load = "Load = 1 kg "
motor_constant = "MC = 8.548e-6 "
controller = "PID"
sim_time = "Sim_time = 76 sec "



m = 0
t_traj = 0
t_odom = 0
label_x = []
label_y = []
label_z = []
label_time = []
label_x_traj = []
label_y_traj = []
label_z_traj = []
label_time_traj = []
traj_mat_time = []
odom_time = []

with open("../bag/data.txt","r") as  file:
	for i in file.readlines():
		k = i.split(":")

		if(k[0] == "header1"):
			m = m+1
		
		if(m != 0):
			m = m+1

		if(m == 6):
			l = i.split(": ")
			l[1] = l[1].rstrip()
			label_time.append(l[1])
			# print(label_time)

		if(m == 12):
			l = i.split(": ")
			l[1] = l[1].rstrip()
			label_x.append(l[1])

		if(m == 13):
			l = i.split(": ")
			l[1] = l[1].rstrip()
			label_y.append(l[1])			

		if(m == 14):
			m = 0
			l = i.split(": ")
			l[1] = l[1].rstrip()
			label_z.append(l[1])


with open("data.txt","r") as file:
	for i in file.readlines():
		k = i.split(":")
		# print i
		if(k[0] == "points"):
			m = m+1

		if(m != 0):
			m = m+1

		if(m == 7):
			l = i.split(": ")
			l[1] = l[1].rstrip()
			label_x_traj.append(l[1])	

		if(m == 8):
			l = i.split(": ")
			l[1] = l[1].rstrip()
			label_y_traj.append(l[1])

		if(m == 9):
			m = 0
			l = i.split(": ")
			l[1] = l[1].rstrip()
			label_z_traj.append(l[1])


for i in range(len(label_x)):
	label_x[i] = float(label_x[i])
	label_y[i] = float(label_y[i])
	label_z[i] = float(label_z[i])

label_x_samp = []
label_y_samp = []
label_z_samp = []
j = 0

for i in range(len(label_x)):
	
	if(i%22000 == 0):
		i = i+1

	if(i%1 == 0):
		label_x_samp.append(label_x[i])
		label_y_samp.append(label_y[i])
		label_z_samp.append(label_z[i])
		j = j+1





for i in range(len(label_x_traj)):
	label_x_traj[i] = float(label_x_traj[i])
	label_y_traj[i] = float(label_y_traj[i])
	label_z_traj[i] = float(label_z_traj[i])

t_samp = range(0,len(label_x_samp))
t = range(0,len(label_x))
t_traj = range(0,len(label_x_traj))


print len(t_samp)  
print len(t_traj)

plt.figure()
plt.title(load + motor_constant + sim_time + controller)
plt.xlabel("Time")
plt.ylabel("X position")
plt.plot(t_samp	,label_x_samp,color = "b",label = "Ground Truth")
plt.plot(t_traj,label_x_traj,color = "r",label = "Generated Trajectory")
plt.legend(bbox_to_anchor=(1,0), loc="lower right")
plt.savefig('trace_x.png')
plt.show()

plt.figure()
plt.title(load + motor_constant + sim_time + controller)
plt.xlabel("Time")
plt.ylabel("Y position")
plt.plot(t_samp,label_y_samp,color = "b",label = "Ground Truth")
plt.plot(t_traj,label_y_traj,color = "r",label = "Generated Trajectory")
plt.legend(bbox_to_anchor=(1,0), loc="lower right")
plt.savefig('trace_y.png')
plt.show()

plt.figure()
plt.title(load + motor_constant + sim_time + controller)
plt.xlabel("Time")
plt.ylabel("Z position")
plt.plot(t_samp,label_z_samp,color = "b",label = "Ground Truth")
plt.plot(t_traj,label_z_traj,color = "r",label = "Generated Trajectory")
plt.legend(bbox_to_anchor=(1,0), loc="lower right")
plt.savefig('trace_z.png')
plt.show()

ax = Axes3D(plt.figure())
ax.set_title(load + motor_constant + sim_time + controller)
ax.set_xlabel("X position")
ax.set_ylabel("Y position")
ax.set_zlabel("Z position")
ax.plot(label_x_samp,label_y_samp,label_z_samp,color="b", label = "Ground Truth")
ax.plot(label_x_traj,label_y_traj,label_z_traj, color = "r", label = "Generated Trajectory")
ax.legend(bbox_to_anchor=(1,0), loc="lower right")
plt.savefig('trace_traj.png')
plt.show()