import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

odom_x = [0]
odom_y = [0]
odom_z = [0.1]
odom_time = [0]
odom_x1 = [0]
odom_y1 = [0]
odom_z1 = [0.1]
odom_time1 = [0]
odom_x2 = [0]
odom_y2 = [0]
odom_z2 = [0.1]
odom_time2 = [0]
traj_x = [0]
traj_y = [0]
traj_z = [0.1]
traj_time = [0]

with open('odom_smc_hi.txt') as file:
	for i in file.readlines():
		k = i.split(':')

		if k[0] == 'X':
			odom_x.append(float(k[1]))
		if k[0] == 'Y':
			odom_y.append(float(k[1]))
		if k[0] == 'Z':
			odom_z.append(float(k[1]))
		if k[0] == 'secs':
			time = float(k[1])
		if k[0] == 'nsecs':
			time += float(k[1])/1000000000
			odom_time.append(time)


# with open('odom_smc1.txt') as file:
# 	for i in file.readlines():
# 		k = i.split(':')

# 		if k[0] == 'X':
# 			odom_x1.append(float(k[1]))
# 		if k[0] == 'Y':
# 			odom_y1.append(float(k[1]))
# 		if k[0] == 'Z':
# 			odom_z1.append(float(k[1]))
# 		if k[0] == 'secs':
# 			time = float(k[1])
# 		if k[0] == 'nsecs':
# 			time += float(k[1])/1000000000
# 			odom_time1.append(time)


with open('odom_asmc.txt') as file:
	for i in file.readlines():
		k = i.split(':')

		if k[0] == 'X':
			odom_x2.append(float(k[1]))
		if k[0] == 'Y':
			odom_y2.append(float(k[1]))
		if k[0] == 'Z':
			odom_z2.append(float(k[1]))
		if k[0] == 'secs':
			time = float(k[1])
		if k[0] == 'nsecs':
			time += float(k[1])/1000000000
			odom_time2.append(time)


with open('traj_smc_hi.txt') as file:
	for i in file.readlines():
		k = i.split(':')

		if k[0] == 'x':
			traj_x.append(traj_x[-1])
			traj_x.append(float(k[1]))
		if k[0] == 'y':
			traj_y.append(traj_y[-1])
			traj_y.append(float(k[1]))
		if k[0] == 'z':
			traj_z.append(traj_z[-1])
			traj_z.append(float(k[1]))
		if k[0] == 'time':
			traj_time.append(float(k[1])-5)
			traj_time.append(traj_time[-1])



print(odom_time[2])

traj_time.append(odom_time[-1])
traj_x.append(traj_x[-1])
traj_y.append(traj_y[-1])
traj_z.append(traj_z[-1])


plt.plot(odom_time, odom_x, label="SMC")
# # plt.plot(odom_time1, odom_x1, label="Default lambda")
plt.plot(odom_time2, odom_x2, label="ASMC")
plt.plot(traj_time, traj_x, label="Desired")
plt.legend()
plt.show()
plt.plot(odom_time, odom_y, label="High lambda")
# # plt.plot(odom_time1, odom_y1, label="Default lambda")
plt.plot(odom_time2, odom_y2, label="ASMC")
plt.plot(traj_time, traj_y, label="Desired")
plt.legend()
plt.show()
plt.plot(odom_time, odom_z, label="High lambda")
# plt.plot(odom_time1, odom_z1, label="Default lambda")
plt.plot(odom_time2, odom_z2, label="ASMC")
plt.plot(traj_time, traj_z, label="Desired")
plt.legend()
plt.show()