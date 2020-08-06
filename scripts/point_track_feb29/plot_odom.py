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
n = 0
t_traj = 0
t_odom = 0
label_x = [0]
label_y = [0]
label_z = [0]
traj_x = [0]
traj_y = [0]
traj_z = [0]
label_time = [0]
traj_time = [0]
label_x_traj = []
label_y_traj = []
label_z_traj = []
label_time_traj = []
traj_mat_time = []
odom_time = []
time = 0.0

with open("../bag/data.txt","r") as  file:
	for i in file.readlines():
		k = i.split(":")

		if(k[0] == "header"):		
			m = 0	

		m += 1

		if(k[0].lstrip() == "secs"):
			l = i.split(": ")
			time = float(l[1])

		if(k[0].lstrip() == "nsecs"):
			l = i.split(": ")
			time += float(l[1])/1000000000
			label_time.append(time)

		if(m == 11):
			l = i.split(": ")
			t = float(l[1])
			label_x.append(float(l[1]))

		if(m == 12):
			l = i.split(": ")
			label_y.append(float(l[1]))

		if(m == 13):
			l = i.split(": ")
			label_z.append(float(l[1]))

		if(k[0] == "transforms"):
			n = 0
		n += 1

		if(n == 4):
			l = i.split(": ")
			traj_x.append(traj_x[-1])			
			traj_x.append(float(l[1]))

		if(n == 5):
			l = i.split(": ")
			traj_y.append(traj_y[-1])			
			traj_y.append(float(l[1]))

		if(n == 6):
			l = i.split(": ")
			traj_z.append(traj_z[-1])			
			traj_z.append(float(l[1]))
			traj_time.append(time)
			traj_time.append(time)




# with open("../bag/data.txt","r") as  file:
# 	for i in file.readlines():
# 		k = i.split(":")

# 		if(k[0] == "transforms"):		
# 			m = 0	
# 			pass

# 		m += 1

# 		# if(k[0].lstrip() == "secs"):
# 		# if(m == 4):
# 		# 	l = i.split(": ")
# 		# 	time = float(l[1])

# 		# # if(k[0].lstrip() == "nsecs"):
# 		# if(m == 5):
# 		# 	l = i.split(": ")
# 		# 	# print(l)
# 		# 	time += float(l[1])/1000000000
# 		traj_time.append(time)
# 		traj_time.append(time)

# 		if(m == 4):
# 			l = i.split(": ")
# 			# print(l)
# 			t = float(l[1])
# 			traj_x.append(traj_x[-1])
# 			traj_x.append(float(l[1]))

# 		if(m == 5):
# 			l = i.split(": ")
# 			traj_y.append(traj_y[-1])
# 			traj_y.append(float(l[1]))

# 		if(m == 6):
# 			l = i.split(": ")
# 			traj_z.append(traj_z[-1])
# 			traj_z.append(float(l[1]))

traj_time.append(label_time[-1])
traj_x.append(traj_x[-1])
traj_y.append(traj_y[-1])
traj_z.append(traj_z[-1])


# for i in range(10):
# 	traj_time[-10+i] = traj_time[i+1]
	# traj_time[-10+i+1] = traj_time[i+2]

print(len(label_time), len(label_x))
# print(traj_x, traj_y, traj_z)
print(traj_time)

plt.figure()
plt.title(load + motor_constant + sim_time + controller)
plt.xlabel("Time")
plt.ylabel("X position")
plt.plot(label_time[:3000]	,label_x[:3000],color = "b",label = "Ground Truth")
# plt.plot(traj_time,traj_x,color = "r",label = "Generated Trajectory")
plt.legend(bbox_to_anchor=(1,0), loc="lower right")
plt.savefig('trace_x.png')
plt.show()

# plt.figure()
# plt.title(load + motor_constant + sim_time + controller)
# plt.xlabel("Time")
# plt.ylabel("Y position")
# plt.plot(label_time,label_y,color = "b",label = "Ground Truth")
# plt.plot(traj_time,traj_x,color = "r",label = "Generated Trajectory")
# plt.legend(bbox_to_anchor=(1,0), loc="lower right")
# plt.savefig('trace_y.png')
# plt.show()

# plt.figure()
# plt.title(load + motor_constant + sim_time + controller)
# plt.xlabel("Time")
# plt.ylabel("Z position")
# plt.plot(label_time,label_z,color = "b",label = "Ground Truth")
# plt.plot(traj_time,traj_x,color = "r",label = "Generated Trajectory")
# plt.legend(bbox_to_anchor=(1,0), loc="lower right")
# plt.savefig('trace_z.png')
# plt.show()

# ax = Axes3D(plt.figure())
# ax.set_title(load + motor_constant + sim_time + controller)
# ax.set_xlabel("X position")
# ax.set_ylabel("Y position")
# ax.set_zlabel("Z position")
# ax.plot(label_x,label_y,label_z,color="b", label = "Ground Truth")
# # ax.plot(label_x_traj,label_y_traj,label_z_traj, color = "r", label = "Generated Trajectory")
# ax.legend(bbox_to_anchor=(1,0), loc="lower right")
# plt.savefig('trace_traj.png')
# plt.show()