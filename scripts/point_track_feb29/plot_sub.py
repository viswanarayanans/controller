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

with open('odom_pd.txt') as file:
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


with open('odom_smc.txt') as file:
	for i in file.readlines():
		k = i.split(':')

		if k[0] == 'X':
			odom_x1.append(float(k[1]))
		if k[0] == 'Y':
			odom_y1.append(float(k[1]))
		if k[0] == 'Z':
			odom_z1.append(float(k[1]))
		if k[0] == 'secs':
			time = float(k[1])
		if k[0] == 'nsecs':
			time += float(k[1])/1000000000
			odom_time1.append(time)


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


with open('traj_asmc.txt') as file:
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
		if k[0] == 'secs':
			# print("time")
			time = float(k[1])
		if k[0] == 'nsecs':
			time += float(k[1])/1000000000
			traj_time.append(time)
			traj_time.append(traj_time[-1])
		# if k[0] == 'time':
		# 	traj_time.append(float(k[1])-5)
		# 	traj_time.append(traj_time[-1])



# print(len(traj_time))

traj_time.append(odom_time2[-1])
traj_x.append(traj_x[-1])
traj_y.append(traj_y[-1])
traj_z.append(traj_z[-1])


# plt.plot(odom_time, odom_x, label="PID")
# plt.plot(odom_time1, odom_x1, label="SMC")
# plt.plot(odom_time2, odom_x2, label="ASMC")
# plt.plot(traj_time, traj_x, label="Desired")
# plt.legend()
# # plt.savefig("random.png")
# plt.show()
# plt.plot(odom_time, odom_y, label="PID")
# plt.plot(odom_time1, odom_y1, label="SMC")
# plt.plot(odom_time2, odom_y2, label="ASMC")
# plt.plot(traj_time, traj_y, label="Desired")
# plt.legend()
# plt.show()
# plt.plot(odom_time, odom_z, label="PID")
# plt.plot(odom_time1, odom_z1, label="SMC")
# plt.plot(odom_time2, odom_z2, label="ASMC")
# plt.plot(traj_time, traj_z, label="Desired")
# plt.legend()
# plt.show()

traj_index = 0
odom_index_asmc = 0
odom_index_smc = 0
odom_index_pd = 0

traj_x_ = []
traj_y_ = []
traj_z_ = []

odom_x_asmc = []
odom_y_asmc = []
odom_z_asmc = []
odom_x_smc = []
odom_y_smc = []
odom_z_smc = []
odom_x_pd = []
odom_y_pd = []
odom_z_pd = []


for i in range(int(odom_time2[-1]*100)):
	ts = float(i)/100
	if ts in traj_time:
		traj_index = traj_time.index(ts)
	traj_x_.append(traj_x[traj_index+1])
	traj_y_.append(traj_y[traj_index+1])
	traj_z_.append(traj_z[traj_index+1])

	if ts in odom_time2:
		odom_index_asmc = odom_time2.index(ts)
	if ts in odom_time1:
		odom_index_smc = odom_time1.index(ts)
	if ts in odom_time:
		odom_index_pd = odom_time.index(ts)

	odom_x_asmc.append(odom_x2[odom_index_asmc])
	odom_y_asmc.append(odom_y2[odom_index_asmc])
	odom_z_asmc.append(odom_z2[odom_index_asmc])
	odom_x_smc.append(odom_x1[odom_index_smc])
	odom_y_smc.append(odom_y1[odom_index_smc])
	odom_z_smc.append(odom_z1[odom_index_smc])
	odom_x_pd.append(odom_x[odom_index_pd])
	odom_y_pd.append(odom_y[odom_index_pd])
	odom_z_pd.append(odom_z[odom_index_pd])




err_x_asmc = np.array(odom_x_asmc) - np.array(traj_x_)
err_x_smc = np.array(odom_x_smc) - np.array(traj_x_)
err_x_pd = np.array(odom_x_pd) - np.array(traj_x_)
err_y_asmc = np.array(odom_y_asmc) - np.array(traj_y_)
err_y_smc = np.array(odom_y_smc) - np.array(traj_y_)
err_y_pd = np.array(odom_y_pd) - np.array(traj_y_)
err_z_asmc = np.array(odom_z_asmc) - np.array(traj_z_)
err_z_smc = np.array(odom_z_smc) - np.array(traj_z_)
err_z_pd = np.array(odom_z_pd) - np.array(traj_z_)

fig = plt.figure(figsize=(16,16))
plt.subplot(311)
plt.plot(list(range(len(err_x_asmc))), err_x_asmc, label="ASMC")
plt.plot(list(range(len(err_x_asmc))), err_x_smc, label="SMC")
# plt.plot(list(range(len(err_x_asmc))), err_x_pd, label="PID")
plt.plot(list(range(len(err_x_asmc))), err_x_pd, label="PD")
plt.legend()
# plt.show()

plt.subplot(312)
plt.plot(list(range(len(err_x_asmc))), err_y_asmc, label="ASMC")
plt.plot(list(range(len(err_x_asmc))), err_y_smc, label="SMC")
# plt.plot(list(range(len(err_x_asmc))), err_y_pd, label="PID")
plt.plot(list(range(len(err_x_asmc))), err_y_pd, label="PD")
plt.legend()
# plt.show()

plt.subplot(313)
plt.plot(list(range(len(err_x_asmc))), err_z_asmc, label="ASMC")
plt.plot(list(range(len(err_x_asmc))), err_z_smc, label="SMC")
# plt.plot(list(range(len(err_x_asmc))), err_z_pd, label="PID")
plt.plot(list(range(len(err_x_asmc))), err_z_pd, label="PD")
plt.legend()
plt.show()
fig.savefig("error.png")