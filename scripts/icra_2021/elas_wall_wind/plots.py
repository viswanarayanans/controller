import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
# from geometry_msgs.msg import PoseWithCovarianceStamped
import geometry_msgs
from tf.transformations import euler_from_quaternion



odom_pd_x = [0]
odom_pd_y = [0]
odom_pd_z = [0.1]
odom_pd_time = [0]
odom_peli_wind_x = [0]
odom_peli_wind_y = [0]
odom_peli_wind_z = [0.1]
odom_peli_wind_time = [0]
odom_elas_wind_x = [0]
odom_elas_wind_y = [0]
odom_elas_wind_z = [0.1]
odom_elas_wind_time = [0]
odom_pd_ax = [0]
odom_pd_ay = [0]
odom_pd_az = [0]
odom_pd_aw = [0]
odom_peli_wind_ax = [0]
odom_peli_wind_ay = [0]
odom_peli_wind_az = [0]
odom_peli_wind_aw = [0]
odom_elas_wind_ax = [0]
odom_elas_wind_ay = [0]
odom_elas_wind_az = [0]
odom_elas_wind_aw = [1]
err_pd_x = [0]
err_pd_y = [0]
err_pd_z = [0.1]
err_pd_phi = [0]
err_pd_theta = [0]
err_pd_psi = [0]
err_pd_time = [0]
err_peli_wind_x = [0]
err_peli_wind_y = [0]
err_peli_wind_z = [0.1]
err_peli_wind_time = [0]
err_peli_wind_phi = [0]
err_peli_wind_theta = [0]
err_peli_wind_psi = [0]
err_elas_wind_x = [0]
err_elas_wind_y = [0]
err_elas_wind_z = [0.1]
err_elas_wind_phi = [0]
err_elas_wind_theta = [0]
err_elas_wind_psi = [0]
err_elas_wind_time = [0]
traj_x = [0]
traj_y = [0]
traj_z = [0.1]
traj_time = [0]
traj_ax = [0]
traj_ay = [0]
traj_az = [0]
traj_aw = [1]

with open('odom_pd.txt') as file:
	for i in file.readlines():
		k = i.split(':')

		if k[0] == 'X':
			odom_pd_x.append(float(k[1]))
		if k[0] == 'Y':
			odom_pd_y.append(float(k[1]))
		if k[0] == 'Z':
			odom_pd_z.append(float(k[1]))
		if k[0] == 'a_X':
			odom_pd_ax.append(float(k[1]))
		if k[0] == 'a_Y':
			odom_pd_ay.append(float(k[1]))
		if k[0] == 'a_Z':
			odom_pd_az.append(float(k[1]))
		if k[0] == 'a_W':
			odom_pd_aw.append(float(k[1]))
		if k[0] == 'secs':
			time = float(k[1])
		if k[0] == 'nsecs':
			time += float(k[1])/1000000000
			odom_pd_time.append(time)

euler_pd_phi = []
euler_pd_theta = []
euler_pd_psi = []
for i in range(len(odom_pd_ax)):
	euler = euler_from_quaternion(np.array([odom_pd_ax[i], odom_pd_ay[i], odom_pd_az[i], odom_pd_aw[i]]))
	euler_pd_phi.append(euler[0])
	euler_pd_theta.append(euler[1])
	euler_pd_psi.append(euler[2])



with open('odom_peli_wind.txt') as file:
	for i in file.readlines():
		k = i.split(':')

		if k[0] == 'X':
			odom_peli_wind_x.append(float(k[1]))
		if k[0] == 'Y':
			odom_peli_wind_y.append(float(k[1]))
		if k[0] == 'Z':
			odom_peli_wind_z.append(float(k[1]))
		if k[0] == 'a_X':
			odom_peli_wind_ax.append(float(k[1]))
		if k[0] == 'a_Y':
			odom_peli_wind_ay.append(float(k[1]))
		if k[0] == 'a_Z':
			odom_peli_wind_az.append(float(k[1]))
		if k[0] == 'a_W':
			odom_peli_wind_aw.append(float(k[1]))
			# print("Nothin")

		if k[0] == 'secs':
			time = float(k[1])
		if k[0] == 'nsecs':
			time += float(k[1])/1000000000
			odom_peli_wind_time.append(time)

euler_peli_wind_phi = []
euler_peli_wind_theta = []
euler_peli_wind_psi = []
for i in range(len(odom_peli_wind_ax)):
	euler = euler_from_quaternion(np.array([odom_peli_wind_ax[i], odom_peli_wind_ay[i], odom_peli_wind_az[i], odom_peli_wind_aw[i]]))
	euler_peli_wind_phi.append(euler[0])
	euler_peli_wind_theta.append(euler[1])
	euler_peli_wind_psi.append(euler[2])


with open('odom_elas_wind.txt') as file:
	for i in file.readlines():
		k = i.split(':')

		if k[0] == 'X':
			odom_elas_wind_x.append(float(k[1]))
		if k[0] == 'Y':
			odom_elas_wind_y.append(float(k[1]))
		if k[0] == 'Z':
			odom_elas_wind_z.append(float(k[1]))
		if k[0] == 'a_X':
			odom_elas_wind_ax.append(float(k[1]))
		if k[0] == 'a_Y':
			odom_elas_wind_ay.append(float(k[1]))
		if k[0] == 'a_Z':
			odom_elas_wind_az.append(float(k[1]))
		if k[0] == 'a_W':
			odom_elas_wind_aw.append(float(k[1]))
		if k[0] == 'secs':
			time = float(k[1])
		if k[0] == 'nsecs':
			time += float(k[1])/1000000000
			odom_elas_wind_time.append(time)


euler_elas_wind_phi = []
euler_elas_wind_theta = []
euler_elas_wind_psi = []
for i in range(len(odom_elas_wind_ax)):
	euler = euler_from_quaternion(np.array([odom_elas_wind_ax[i], odom_elas_wind_ay[i], odom_elas_wind_az[i], odom_elas_wind_aw[i]]))
	euler_elas_wind_phi.append(euler[0])
	euler_elas_wind_theta.append(euler[1])
	euler_elas_wind_psi.append(euler[2])


with open('traj_elas_wind.txt') as file:
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
		if k[0] == 'a_x':
			traj_ax.append(traj_ax[-1])
			traj_ax.append(float(k[1]))
		if k[0] == 'a_y':
			traj_ay.append(traj_ay[-1])
			traj_ay.append(float(k[1]))
		if k[0] == 'a_z':
			traj_az.append(traj_az[-1])
			traj_az.append(float(k[1]))
		if k[0] == 'a_w':
			traj_aw.append(traj_aw[-1])
			traj_aw.append(float(k[1]))
		if k[0] == 'secs':
		# print("time")
			time = float(k[1])
		if k[0] == 'nsecs':
			time += float(k[1])/1000000000
			traj_time.append(time)
			traj_time.append(traj_time[-1])

traj_phi = []
traj_theta = []
traj_psi = []
for i in range(len(traj_ax)):
	euler = euler_from_quaternion(np.array([traj_ax[i], traj_ay[i], traj_az[i], traj_aw[i]]))
	traj_phi.append(euler[0])
	traj_theta.append(euler[1])
	traj_psi.append(euler[2])


with open('error_elas_wind.txt') as file:
	for i in file.readlines():
		k = i.split(':')

		if k[0] == 'Err_X':
		# err_elas_wind_x.append(traj_x[-1])
			err_elas_wind_x.append(float(k[1]))
		if k[0] == 'Err_Y':
		# err_elas_wind_y.append(traj_y[-1])
			err_elas_wind_y.append(float(k[1]))
		if k[0] == 'Err_Z':
		# err_elas_wind_z.append(traj_z[-1])
			err_elas_wind_z.append(float(k[1]))
		if k[0] == 'Err_phi':
		# err_elas_wind_x.append(traj_x[-1])
			err_elas_wind_phi.append(float(k[1])*180/np.pi)
		if k[0] == 'Err_theta':
		# err_elas_wind_y.append(traj_y[-1])
			err_elas_wind_theta.append(float(k[1])*180/np.pi)
		if k[0] == 'Err_psi':
		# err_elas_wind_z.append(traj_z[-1])
			err_elas_wind_psi.append(float(k[1])*180/np.pi)
		if k[0] == 'secs':
		# print("time")
			time = float(k[1])
		if k[0] == 'nsecs':
			time += float(k[1])/1000000000
			err_elas_wind_time.append(time)



with open('error_peli_wind.txt') as file:
	for i in file.readlines():
		k = i.split(':')

		if k[0] == 'Err_X':
		# err_peli_wind_x.append(traj_x[-1])
			err_peli_wind_x.append(float(k[1]))
		if k[0] == 'Err_Y':
		# err_peli_wind_y.append(traj_y[-1])
			err_peli_wind_y.append(float(k[1]))
		if k[0] == 'Err_Z':
		# err_peli_wind_z.append(traj_z[-1])
			err_peli_wind_z.append(float(k[1]))
		if k[0] == 'Err_phi':
		# err_elas_wind_x.append(traj_x[-1])
			err_peli_wind_phi.append(float(k[1])*180/np.pi)
		if k[0] == 'Err_theta':
		# err_elas_wind_y.append(traj_y[-1])
			err_peli_wind_theta.append(float(k[1])*180/np.pi)
		if k[0] == 'Err_psi':
		# err_elas_wind_z.append(traj_z[-1])
			err_peli_wind_psi.append(float(k[1])*180/np.pi)
		if k[0] == 'secs':
		# print("time")
			time = float(k[1])
		if k[0] == 'nsecs':
			time += float(k[1])/1000000000
			err_peli_wind_time.append(time)



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
		if k[0] == 'Err_phi':
		# err_elas_wind_x.append(traj_x[-1])
			err_pd_phi.append(float(k[1])*180/np.pi)
		if k[0] == 'Err_theta':
		# err_elas_wind_y.append(traj_y[-1])
			err_pd_theta.append(float(k[1])*180/np.pi)
		if k[0] == 'Err_psi':
		# err_elas_wind_z.append(traj_z[-1])
			err_pd_psi.append(float(k[1])*180/np.pi)
		if k[0] == 'secs':
		# print("time")
			time = float(k[1])
		if k[0] == 'nsecs':
			time += float(k[1])/1000000000
			err_pd_time.append(time)

traj_time.append(odom_peli_wind_time[-1])
traj_x.append(traj_x[-1])
traj_y.append(traj_y[-1])
traj_z.append(traj_z[-1])
# falling_index = err_peli_wind_time.index(39)
# falling_index = 1500
# print(max(err_pd_psi))


# fig = plt.figure(figsize=(6,12))
fig, axes = plt.subplots(nrows=3, ncols=1, figsize=(16,8))
plt.subplot(311)
plt.plot(traj_time, traj_x, label='Desired trajectory', lw=2)
# plt.plot(odom_pd_time, odom_pd_x, label='PD', lw=2)
plt.plot(odom_peli_wind_time, odom_peli_wind_x, label="Pelican", lw=2)
plt.plot(odom_elas_wind_time, odom_elas_wind_x, label="Elasticopter", lw=2)
plt.axis([0, 15, -1, 1])
# plt.ylabel('x')
plt.title('Position tracking: x (m)')
plt.legend(loc=2, prop={'size': 8}, ncol=4)
# plt.show()
# fig.savefig('traj_x.png')
plt.subplot(312)
plt.plot(traj_time, traj_y, label='Desired trajectory', lw=2)
# plt.plot(odom_pd_time, odom_pd_y, label='PD', lw=2)
plt.plot(odom_peli_wind_time, odom_peli_wind_y, label="Pelican", lw=2)
plt.plot(odom_elas_wind_time, odom_elas_wind_y, label="Elasticopter", lw=2)
plt.axis([0, 15, -5, 10])
plt.title('Position tracking: y (m)')
# plt.ylabel('y')
plt.legend(loc=2, prop={'size': 8}, ncol=4)
# fig.savefig('traj_y.png')

plt.subplot(313)
plt.plot(traj_time, traj_z, label='Desired trajectory', lw=2)
# plt.plot(odom_pd_time, odom_pd_z, label='PD', lw=2)
plt.plot(odom_peli_wind_time, odom_peli_wind_z, label="Pelican", lw=2)
plt.plot(odom_elas_wind_time, odom_elas_wind_z, label="Elasticopter", lw=2)
# plt.ylabel('z')
plt.axis([0, 15, 0, 5])
plt.legend(loc=2, prop={'size': 8}, ncol=4)
# plt.xlabel('time')
plt.title('Position tracking: z (m)')
plt.xlabel('time (s)')


fig.tight_layout()
plt.show()
fig.savefig('traj_wall.png')


# fig = plt.figure(figsize=(6,12))
fig, axes = plt.subplots(nrows=3, ncols=1, figsize=(16,8))
plt.subplot(311)
# plt.plot(err_pd_time[:len(err_peli_wind_time)], err_pd_x[:len(err_peli_wind_time)], label='PD', lw=2)
plt.plot(err_peli_wind_time[:len(err_peli_wind_time)], err_peli_wind_x[:len(err_peli_wind_time)], label="Pelican", lw=2)
plt.plot(err_elas_wind_time[:len(err_peli_wind_time)], err_elas_wind_x[:len(err_peli_wind_time)], label="Elasticopter", lw=2)
# plt.plot([0,err_pd_time[-1]], [0,0])
# plt.ylabel('x_err')
plt.title('Position error: x (m)')
plt.axis([0, 15, -1, 1])
plt.legend(loc=2, prop={'size': 10}, ncol=4)
# plt.show()
# fig.savefig('err_x.png')


plt.subplot(312)
# plt.plot(err_pd_time[:len(err_peli_wind_time)], err_pd_y[:len(err_peli_wind_time)], label='PD', lw=2)
plt.plot(err_peli_wind_time[:len(err_peli_wind_time)], err_peli_wind_y[:len(err_peli_wind_time)], label="Pelican", lw=2)
plt.plot(err_elas_wind_time[:len(err_peli_wind_time)], err_elas_wind_y[:len(err_peli_wind_time)], label="Elasticopter", lw=2)
# plt.plot([0,err_pd_time[-1]], [0,0])
# plt.ylabel('y_err')
plt.title('Position error: y (m)')
plt.axis([0, 15, -1, 1])
plt.legend(loc=3, prop={'size': 10}, ncol=4)
# plt.show()
# fig.savefig('err_y.png')


plt.subplot(313)
# plt.plot(err_pd_time[:len(err_peli_wind_time)], err_pd_z[:len(err_peli_wind_time)], label='PD', lw=2)
plt.plot(err_peli_wind_time[:len(err_peli_wind_time)], err_peli_wind_z[:len(err_peli_wind_time)], label="Pelican", lw=2)
plt.plot(err_elas_wind_time[:len(err_peli_wind_time)], err_elas_wind_z[:len(err_peli_wind_time)], label="Elasticopter", lw=2)
# plt.plot([0,err_pd_time[-1]], [0,0])
# plt.ylabel('z_err')
plt.title('Position error: z (m)')
plt.xlabel('time (s)')
plt.axis([0, 15, -1, 1])
plt.legend(loc=3, prop={'size': 10}, ncol=4)

fig.tight_layout(h_pad=0.4)
plt.show()
fig.savefig('error_pos_wall.png')


# fig = plt.figure(figsize=(6,12))
fig, axes = plt.subplots(nrows=3, ncols=1, figsize=(16,8))
plt.subplot(311)
# plt.plot(err_pd_time[:len(err_peli_wind_time)], err_pd_phi[:len(err_peli_wind_time)], label='PD', lw=2)
plt.plot(err_peli_wind_time[:len(err_peli_wind_time)], err_peli_wind_phi[:len(err_peli_wind_time)], label="Pelican", lw=2)
plt.plot(err_elas_wind_time[:len(err_peli_wind_time)], err_elas_wind_phi[:len(err_peli_wind_time)], label="Elasticopter", lw=2)
# plt.plot([0,err_pd_time[-1]], [0,0])
# plt.ylabel('x_err')
plt.title('Attitude error: $\phi$ (deg)')
plt.axis([0, 15, -10, 10])
plt.legend(loc=3, prop={'size': 10}, ncol=4)
# plt.show()
# fig.savefig('err_x.png')


plt.subplot(312)
# plt.plot(err_pd_time[:len(err_peli_wind_time)], err_pd_theta[:len(err_peli_wind_time)], label='PD', lw=2)
plt.plot(err_peli_wind_time[:len(err_peli_wind_time)], err_peli_wind_theta[:len(err_peli_wind_time)], label="Pelican", lw=2)
plt.plot(err_elas_wind_time[:len(err_peli_wind_time)], err_elas_wind_theta[:len(err_peli_wind_time)], label="Elasticopter", lw=2)
# plt.plot([0,err_pd_time[-1]], [0,0])
# plt.ylabel('y_err')
plt.title('Attitude error: $\\theta$ (deg)')
plt.axis([0, 15, -20, 20])
plt.legend(loc=2, prop={'size': 10}, ncol=4)
# plt.show()
# fig.savefig('err_y.png')


plt.subplot(313)
# plt.plot(err_pd_time[:len(err_peli_wind_time)], err_pd_psi[:len(err_peli_wind_time)], label='PD', lw=2)
plt.plot(err_peli_wind_time[:len(err_peli_wind_time)], err_peli_wind_psi[:len(err_peli_wind_time)], label="Pelican", lw=2)
plt.plot(err_elas_wind_time[:len(err_peli_wind_time)], err_elas_wind_psi[:len(err_peli_wind_time)], label="Elasticopter", lw=2)
# plt.plot([0,err_pd_time[-1]], [0,0])
# plt.ylabel('z_err')
plt.title('Attitude error: $\psi$ (deg)')
plt.xlabel('time (s)')
plt.axis([0, 15, -10, 10])
plt.legend(loc=2, prop={'size': 10}, ncol=4)

fig.tight_layout(h_pad=0.4)
plt.show()
fig.savefig('error_att_wall.png')

RSME_elas_wind_ = np.zeros(3)
RSME_elas_wind_[0] = np.sqrt(np.mean([x**2 for x in err_elas_wind_x]))
RSME_elas_wind_[1] = np.sqrt(np.mean([y**2 for y in err_elas_wind_y]))
RSME_elas_wind_[2] = np.sqrt(np.mean([z**2 for z in err_elas_wind_z]))
RSME_elas_wind = np.sqrt(np.mean(RSME_elas_wind_**2))

print(RSME_elas_wind_, RSME_elas_wind)

RSME_peli_wind_ = np.zeros(3)
RSME_peli_wind_[0] = np.sqrt(np.mean([x**2 for x in err_peli_wind_x]))
RSME_peli_wind_[1] = np.sqrt(np.mean([x**2 for x in err_peli_wind_y]))
RSME_peli_wind_[2] = np.sqrt(np.mean([x**2 for x in err_peli_wind_z]))
RSME_peli_wind = np.sqrt(np.mean(RSME_peli_wind_**2))

print(RSME_peli_wind_, RSME_peli_wind)



RSME_elas_wind_ = np.zeros(3)
RSME_elas_wind_[0] = np.sqrt(np.mean([x**2 for x in err_elas_wind_phi]))
RSME_elas_wind_[1] = np.sqrt(np.mean([y**2 for y in err_elas_wind_theta]))
RSME_elas_wind_[2] = np.sqrt(np.mean([z**2 for z in err_elas_wind_psi]))
RSME_elas_wind = np.sqrt(np.mean(RSME_elas_wind_**2))

print(RSME_elas_wind_, RSME_elas_wind)

RSME_peli_wind_ = np.zeros(3)
RSME_peli_wind_[0] = np.sqrt(np.mean([x**2 for x in err_peli_wind_phi]))
RSME_peli_wind_[1] = np.sqrt(np.mean([x**2 for x in err_peli_wind_theta]))
RSME_peli_wind_[2] = np.sqrt(np.mean([x**2 for x in err_peli_wind_psi]))
RSME_peli_wind = np.sqrt(np.mean(RSME_peli_wind_**2))

print(RSME_peli_wind_, RSME_peli_wind)

RSME_pd_ = np.zeros(3)
# RSME_pd_[0] = np.sqrt(np.mean(err_pd_x[:len(err_peli_wind_time)]))
# RSME_pd_[1] = np.sqrt(np.mean(err_pd_y[:len(err_peli_wind_time)]))
# RSME_pd_[2] = np.sqrt(np.mean(err_pd_z[:len(err_peli_wind_time)]))
# RSME_pd = np.sqrt(np.mean(RSME_pd_))
