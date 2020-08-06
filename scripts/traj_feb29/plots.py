import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D



odom_pd_x = [0]
odom_pd_y = [0]
odom_pd_z = [0.1]
odom_pd_time = [0]
odom_smc_x = [0]
odom_smc_y = [0]
odom_smc_z = [0.1]
odom_smc_time = [0]
odom_asmc_x = [0]
odom_asmc_y = [0]
odom_asmc_z = [0.1]
odom_asmc_time = [0]
err_pd_x = [0]
err_pd_y = [0]
err_pd_z = [0.1]
err_pd_time = [0]
err_smc_x = [0]
err_smc_y = [0]
err_smc_z = [0.1]
err_smc_time = [0]
err_asmc_x = [0]
err_asmc_y = [0]
err_asmc_z = [0.1]
err_asmc_time = [0]
traj_x = [0]
traj_y = [0]
traj_z = [0.1]
traj_time = [0]

with open('odom_pd.txt') as file:
	for i in file.readlines():
		k = i.split(':')

		if k[0] == 'X':
			odom_pd_x.append(float(k[1]))
		if k[0] == 'Y':
			odom_pd_y.append(float(k[1]))
		if k[0] == 'Z':
			odom_pd_z.append(float(k[1]))
		if k[0] == 'secs':
			time = float(k[1])
		if k[0] == 'nsecs':
			time += float(k[1])/1000000000
			odom_pd_time.append(time)


with open('odom_smc.txt') as file:
	for i in file.readlines():
		k = i.split(':')

		if k[0] == 'X':
			odom_smc_x.append(float(k[1]))
		if k[0] == 'Y':
			odom_smc_y.append(float(k[1]))
		if k[0] == 'Z':
			odom_smc_z.append(float(k[1]))
		if k[0] == 'secs':
			time = float(k[1])
		if k[0] == 'nsecs':
			time += float(k[1])/1000000000
			odom_smc_time.append(time)


with open('odom_asmc.txt') as file:
	for i in file.readlines():
		k = i.split(':')

		if k[0] == 'X':
			odom_asmc_x.append(float(k[1]))
		if k[0] == 'Y':
			odom_asmc_y.append(float(k[1]))
		if k[0] == 'Z':
			odom_asmc_z.append(float(k[1]))
		if k[0] == 'secs':
			time = float(k[1])
		if k[0] == 'nsecs':
			time += float(k[1])/1000000000
			odom_asmc_time.append(time)


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


with open('error_asmc.txt') as file:
	for i in file.readlines():
		k = i.split(':')

		if k[0] == 'Err_X':
			# err_asmc_x.append(traj_x[-1])
			err_asmc_x.append(float(k[1]))
		if k[0] == 'Err_Y':
			# err_asmc_y.append(traj_y[-1])
			err_asmc_y.append(float(k[1]))
		if k[0] == 'Err_Z':
			# err_asmc_z.append(traj_z[-1])
			err_asmc_z.append(float(k[1]))
		if k[0] == 'secs':
			# print("time")
			time = float(k[1])
		if k[0] == 'nsecs':
			time += float(k[1])/1000000000
			err_asmc_time.append(time)



with open('error_smc.txt') as file:
	for i in file.readlines():
		k = i.split(':')

		if k[0] == 'Err_X':
			# err_smc_x.append(traj_x[-1])
			err_smc_x.append(float(k[1]))
		if k[0] == 'Err_Y':
			# err_smc_y.append(traj_y[-1])
			err_smc_y.append(float(k[1]))
		if k[0] == 'Err_Z':
			# err_smc_z.append(traj_z[-1])
			err_smc_z.append(float(k[1]))
		if k[0] == 'secs':
			# print("time")
			time = float(k[1])
		if k[0] == 'nsecs':
			time += float(k[1])/1000000000
			err_smc_time.append(time)



with open('error_pd.txt') as file:
	for i in file.readlines():
		k = i.split(':')

		if k[0] == 'Err_X':
			# err_pd_x.append(traj_x[-1])
			err_pd_x.append(float(k[1]))
		if k[0] == 'Err_Y':
			# err_pd_y.append(traj_y[-1])
			err_pd_y.append(float(k[1]))
		if k[0] == 'Err_Z':
			# err_pd_z.append(traj_z[-1])
			err_pd_z.append(float(k[1]))
		if k[0] == 'secs':
			# print("time")
			time = float(k[1])
		if k[0] == 'nsecs':
			time += float(k[1])/1000000000
			err_pd_time.append(time)


# falling_index = err_smc_time.index(39)
# falling_index = 1500



plt.plot(traj_time, traj_x, label='Traj')
plt.plot(odom_pd_time, odom_pd_x, label='PD')
plt.plot(odom_smc_time, odom_smc_x, label="SMC")
plt.plot(odom_asmc_time, odom_asmc_x, label="ASMC")
plt.legend()
plt.show()
plt.savefig('traj_x.png')

plt.plot(traj_time, traj_y, label='Traj')
plt.plot(odom_pd_time, odom_pd_y, label='PD')
plt.plot(odom_smc_time, odom_smc_y, label="SMC")
plt.plot(odom_asmc_time, odom_asmc_y, label="ASMC")
plt.legend()
plt.show()
plt.savefig('traj_y.png')


plt.plot(traj_time, traj_z, label='Traj')
plt.plot(odom_pd_time, odom_pd_z, label='PD')
plt.plot(odom_smc_time, odom_smc_z, label="SMC")
plt.plot(odom_asmc_time, odom_asmc_z, label="ASMC")
plt.legend()
plt.show()
plt.savefig('traj_z.png')


plt.plot(err_pd_time, err_pd_x, label='PD')
plt.plot(err_smc_time, err_smc_x, label="SMC")
plt.plot(err_asmc_time, err_asmc_x, label="ASMC")
plt.plot([0,err_asmc_time[-1]], [0,0])
plt.legend()
plt.show()
plt.savefig('err_x.png')


plt.plot(err_pd_time, err_pd_y, label='PD')
plt.plot(err_smc_time, err_smc_y, label="SMC")
plt.plot(err_asmc_time, err_asmc_y, label="ASMC")
plt.plot([0,err_asmc_time[-1]], [0,0])
plt.legend()
plt.show()
plt.savefig('err_y.png')


plt.plot(err_pd_time, err_pd_z, label='PD')
plt.plot(err_smc_time, err_smc_z, label="SMC")
plt.plot(err_asmc_time, err_asmc_z, label="ASMC")
plt.plot([0,err_asmc_time[-1]], [0,0])
plt.legend()
plt.show()
plt.savefig('err_z.png')


RSME_asmc_ = np.zeros(3)
RSME_asmc_[0] = np.linalg.norm(err_asmc_x)
RSME_asmc_[1] = np.linalg.norm(err_asmc_y)
RSME_asmc_[2] = np.linalg.norm(err_asmc_z)
RSME_asmc = np.linalg.norm(RSME_asmc_)

print(RSME_asmc_, RSME_asmc)

RSME_smc_ = np.zeros(3)
RSME_smc_[0] = np.linalg.norm(err_smc_x)
RSME_smc_[1] = np.linalg.norm(err_smc_y)
RSME_smc_[2] = np.linalg.norm(err_smc_z)
RSME_smc = np.linalg.norm(RSME_smc_)

print(RSME_smc_, RSME_smc)

RSME_pd_ = np.zeros(3)
RSME_pd_[0] = np.linalg.norm(err_pd_x)
RSME_pd_[1] = np.linalg.norm(err_pd_y)
RSME_pd_[2] = np.linalg.norm(err_pd_z)
RSME_pd = np.linalg.norm(RSME_pd_)

print(RSME_pd_, RSME_pd)