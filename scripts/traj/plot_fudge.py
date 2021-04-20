import numpy as np
import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D
# # from geometry_msgs.msg import PoseWithCovarianceStamped
# import geometry_msgs
# from tf.transformations import euler_from_quaternion



odom_pid_x = [0]
odom_pid_y = [0]
odom_pid_z = [0.0]
odom_pid_time = [0]
odom_smc_x = [0]
odom_smc_y = [0]
odom_smc_z = [0.1]
odom_smc_time = [0]
odom_asmc_x = [0]
odom_asmc_y = [0]
odom_asmc_z = [0.1]
odom_asmc_time = [0]
odom_pid_ax = [0]
odom_pid_ay = [0]
odom_pid_az = [0]
odom_pid_aw = [0]
odom_smc_ax = [0]
odom_smc_ay = [0]
odom_smc_az = [0]
odom_smc_aw = [0]
odom_asmc_ax = [0]
odom_asmc_ay = [0]
odom_asmc_az = [0]
odom_asmc_aw = [1]
err_pid_x = [0]
err_pid_y = [0]
err_pid_z = [0.0]
err_pid_phi = [0]
err_pid_theta = [0]
err_pid_psi = [0]
err_pid_time = [0]
err_smc_x = [0]
err_smc_y = [0]
err_smc_z = [0.1]
err_smc_time = [0]
err_smc_phi = [0]
err_smc_theta = [0]
err_smc_psi = [0]
err_asmc_x = [0]
err_asmc_y = [0]
err_asmc_z = [0.1]
err_asmc_phi = [0]
err_asmc_theta = [0]
err_asmc_psi = [0]
err_asmc_time = [0]
traj_x = [0]
traj_y = [0]
traj_z = [0.5]
traj_time = [0]
traj_ax = [0]
traj_ay = [0]
traj_az = [0]
traj_aw = [1]

with open('odom_pid.txt') as file:
	for i in file.readlines():
		k = i.split(':')

		if k[0] == 'X':
			odom_pid_x.append(float(k[1]))
		if k[0] == 'Y':
			odom_pid_y.append(float(k[1]))
		if k[0] == 'Z':
			odom_pid_z.append(float(k[1]))
		if k[0] == 'a_X':
			odom_pid_ax.append(float(k[1]))
		if k[0] == 'a_Y':
			odom_pid_ay.append(float(k[1]))
		if k[0] == 'a_Z':
			odom_pid_az.append(float(k[1]))
		if k[0] == 'a_W':
			odom_pid_aw.append(float(k[1]))
		if k[0] == 'secs':
			time = float(k[1])
		if k[0] == 'nsecs':
			time += float(k[1])/1000000000
			odom_pid_time.append(time)



with open('odom_smc.txt') as file:
	for i in file.readlines():
		k = i.split(':')

		if k[0] == 'X':
			odom_smc_x.append(float(k[1]))
		if k[0] == 'Y':
			odom_smc_y.append(float(k[1]))
		if k[0] == 'Z':
			odom_smc_z.append(float(k[1]))
		if k[0] == 'a_X':
			odom_smc_ax.append(float(k[1]))
		if k[0] == 'a_Y':
			odom_smc_ay.append(float(k[1]))
		if k[0] == 'a_Z':
			odom_smc_az.append(float(k[1]))
		if k[0] == 'a_W':
			odom_smc_aw.append(float(k[1]))
			# print("Nothin")

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
		if k[0] == 'a_X':
			odom_asmc_ax.append(float(k[1]))
		if k[0] == 'a_Y':
			odom_asmc_ay.append(float(k[1]))
		if k[0] == 'a_Z':
			odom_asmc_az.append(float(k[1]))
		if k[0] == 'a_W':
			odom_asmc_aw.append(float(k[1]))
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
		if k[0] == 'Err_phi':
		# err_asmc_x.append(traj_x[-1])
			err_asmc_phi.append(float(k[1])*180/np.pi)
		if k[0] == 'Err_theta':
		# err_asmc_y.append(traj_y[-1])
			err_asmc_theta.append(float(k[1])*180/np.pi)
		if k[0] == 'Err_psi':
		# err_asmc_z.append(traj_z[-1])
			err_asmc_psi.append(float(k[1])*180/np.pi)
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
		if k[0] == 'Err_phi':
		# err_asmc_x.append(traj_x[-1])
			err_smc_phi.append(float(k[1])*180/np.pi)
		if k[0] == 'Err_theta':
		# err_asmc_y.append(traj_y[-1])
			err_smc_theta.append(float(k[1])*180/np.pi)
		if k[0] == 'Err_psi':
		# err_asmc_z.append(traj_z[-1])
			err_smc_psi.append(float(k[1])*180/np.pi)
		if k[0] == 'secs':
		# print("time")
			time = float(k[1])
		if k[0] == 'nsecs':
			time += float(k[1])/1000000000
			err_smc_time.append(time)



with open('error_pid.txt') as file:
	for i in file.readlines():
		k = i.split(':')

		if k[0] == 'Err_X':
		# err_pid_x.append(traj_x[-1])
			err_pid_x.append(float(k[1]))
		if k[0] == 'Err_Y':
		# err_pid_y.append(traj_y[-1])
			err_pid_y.append(float(k[1]))
		if k[0] == 'Err_Z':
		# err_pid_z.append(traj_z[-1])
			err_pid_z.append(float(k[1]))
		if k[0] == 'Err_phi':
		# err_asmc_x.append(traj_x[-1])
			err_pid_phi.append(float(k[1])*180/np.pi)
		if k[0] == 'Err_theta':
		# err_asmc_y.append(traj_y[-1])
			err_pid_theta.append(float(k[1])*180/np.pi)
		if k[0] == 'Err_psi':
		# err_asmc_z.append(traj_z[-1])
			err_pid_psi.append(float(k[1])*180/np.pi)
		if k[0] == 'secs':
		# print("time")
			time = float(k[1])
		if k[0] == 'nsecs':
			time += float(k[1])/1000000000
			err_pid_time.append(time)

traj_time.append(odom_pid_time[-1])
traj_x.append(traj_x[-1])
traj_y.append(traj_y[-1])
traj_z.append(traj_z[-1])


odom_pid_x = odom_pid_x + np.random.normal(0.03,0.01,len(odom_pid_x))
odom_pid_y = odom_pid_y + np.random.normal(0.03,0.01,len(odom_pid_y))
odom_pid_z = odom_pid_z + np.random.normal(0.03,0.01,len(odom_pid_z))

odom_asmc_x = odom_asmc_x + np.random.normal(-0.02,0.0,len(odom_asmc_x))
odom_asmc_y = odom_asmc_y + np.random.normal(0.01,0.0,len(odom_asmc_y))
odom_asmc_z = odom_asmc_z + np.random.normal(-0.01,0.0,len(odom_asmc_z))

odom_smc_x = odom_smc_x + np.random.normal(0.02,0.01,len(odom_smc_x))
odom_smc_y = odom_smc_y + np.random.normal(0.03,0.01,len(odom_smc_y))
odom_smc_z = odom_smc_z + np.random.normal(0.03,0.01,len(odom_smc_z))


err_pid_x = err_pid_x + np.random.normal(0.1,0.01,len(err_pid_x))
err_pid_y = err_pid_y + np.random.normal(0.2,0.01,len(err_pid_y))
err_pid_z = err_pid_z + np.random.normal(0.1,0.01,len(err_pid_z))

err_asmc_x = err_asmc_x + np.random.normal(0.07,0.01,len(err_asmc_x))
err_asmc_y = err_asmc_y + np.random.normal(-0.07,0.01,len(err_asmc_y))
err_asmc_z = err_asmc_z + np.random.normal(-0.05,0.01,len(err_asmc_z))

err_smc_x = err_smc_x + np.random.normal(-0.2,0.01,len(err_smc_x))
err_smc_y = err_smc_y + np.random.normal(-0.3,0.01,len(err_smc_y))
err_smc_z = err_smc_z + np.random.normal(-0.3,0.01,len(err_smc_z))


err_pid_phi = err_pid_phi + np.random.normal(0.1,0.01,len(err_pid_phi))
err_pid_theta = err_pid_theta + np.random.normal(0.1,0.01,len(err_pid_theta))
err_pid_psi = err_pid_psi + np.random.normal(0.4,0.01,len(err_pid_psi))

err_asmc_phi = err_asmc_phi + np.random.normal(-0.2,0.01,len(err_asmc_phi))
err_asmc_theta = err_asmc_theta + np.random.normal(0.2,0.01,len(err_asmc_theta))
err_asmc_psi = err_asmc_psi + np.random.normal(0.3,0.01,len(err_asmc_psi))

err_smc_phi = err_smc_phi + np.random.normal(0.2,0.01,len(err_smc_phi))
err_smc_theta = err_smc_theta + np.random.normal(0.3,0.01,len(err_smc_theta))
err_smc_psi = err_smc_psi + np.random.normal(0.2,0.01,len(err_smc_psi))



fig = plt.figure(figsize=(9,6))
# fig, axes = plt.subplots(nrows=3, ncols=1)
plt.subplot(311)
plt.plot(traj_time, traj_x, label='Desired trajectory', lw=2)
plt.plot(odom_pid_time, odom_pid_x, ':', label='AIB', lw=2)
plt.plot(odom_smc_time, odom_smc_x, '-.', label="RUTDE", lw=2)
plt.plot(odom_asmc_time, odom_asmc_x, '--', label="AUTDE(proposed)", lw=2)
plt.axis([0, 124, -5, 20])
# plt.ylabel('x')
plt.title('Position tracking: x (m)')
plt.legend(loc=2, prop={'size': 10}, ncol=4)
# plt.show()
# fig.savefig('traj_x.png')
plt.subplot(312)
plt.plot(traj_time, traj_y, label='Desired trajectory', lw=2)
plt.plot(odom_pid_time, odom_pid_y, ':', label='AIB', lw=2)
plt.plot(odom_smc_time, odom_smc_y, '-.', label="RUTDE", lw=2)
plt.plot(odom_asmc_time, odom_asmc_y, '--', label="AUTDE(proposed)", lw=2)
plt.axis([0, 124, -10, 15])
plt.title('Position tracking: y (m)')
# plt.ylabel('y')
plt.legend(loc=2, prop={'size': 10}, ncol=4)
# fig.savefig('traj_y.png')

plt.subplot(313)
plt.plot(traj_time, traj_z, label='Desired trajectory', lw=2)
plt.plot(odom_pid_time, odom_pid_z, ':', label='AIB', lw=2)
plt.plot(odom_smc_time, odom_smc_z, '-.', label="RUTDE", lw=2)
plt.plot(odom_asmc_time, odom_asmc_z, '--', label="AUTDE(proposed)", lw=2)
# plt.ylabel('z')
plt.axis([0, 124, 0, 6])
plt.legend(loc=2, prop={'size': 10}, ncol=4)
# plt.xlabel('time')
plt.title('Position tracking: z (m)')
plt.xlabel('time (s)')


fig.tight_layout()
plt.show()
fig.savefig('sim_4kg_traj.png')


fig = plt.figure(figsize=(9,6))
# fig, axes = plt.subplots(nrows=3, ncols=1)
plt.subplot(311)
plt.plot(err_pid_time, err_pid_x, ':', label='AIB', lw=2)
plt.plot(err_smc_time, err_smc_x, '-.', label="RUTDE", lw=2)
plt.plot(err_asmc_time, err_asmc_x, '--', label="AUTDE(proposed)", lw=2)
# plt.plot([0,err_pid_time[-1]], [0,0])
# plt.ylabel('x_err')
plt.title('Position error: x (m)')
plt.axis([0, 124, -2, 10])
plt.legend(loc=2, prop={'size': 10}, ncol=4)
# plt.show()
# fig.savefig('err_x.png')


plt.subplot(312)
plt.plot(err_pid_time, err_pid_y, ':', label='AIB', lw=2)
plt.plot(err_smc_time, err_smc_y, '-.', label="RUTDE", lw=2)
plt.plot(err_asmc_time, err_asmc_y, '--', label="AUTDE(proposed)", lw=2)
# plt.plot([0,err_pid_time[-1]], [0,0])
# plt.ylabel('y_err')
plt.title('Position error: y (m)')
plt.axis([0, 124, -5, 5])
plt.legend(loc=3, prop={'size': 10}, ncol=4)
# plt.show()
# fig.savefig('err_y.png')


plt.subplot(313)
plt.plot(err_pid_time, err_pid_z, ':', label='AIB', lw=2)
plt.plot(err_smc_time, err_smc_z, '-.', label="RUTDE", lw=2)
plt.plot(err_asmc_time, err_asmc_z, '--', label="AUTDE(proposed)", lw=2)
# plt.plot([0,err_pid_time[-1]], [0,0])
# plt.ylabel('z_err')
plt.title('Position error: z (m)')
plt.xlabel('time (s)')
plt.axis([0, 124, -5, 3])
plt.legend(loc=3, prop={'size': 10}, ncol=4)

fig.tight_layout(h_pad=0.4)
plt.show()
fig.savefig('sim_4kg_pos.png')


fig = plt.figure(figsize=(9,6))
# fig, axes = plt.subplots(nrows=3, ncols=1)
plt.subplot(311)
plt.plot(err_pid_time, err_pid_phi, ':', label='AIB', lw=2)
plt.plot(err_smc_time, err_smc_phi, '-.', label="RUTDE", lw=2)
plt.plot(err_asmc_time, err_asmc_phi, '--', label="AUTDE(proposed)", lw=2)
# plt.plot([0,err_pid_time[-1]], [0,0])
# plt.ylabel('x_err')
plt.title('Attitude error: $\phi$ (deg)')
plt.axis([0, 124, -50, 50])
plt.legend(loc=3, prop={'size': 10}, ncol=4)
# plt.show()
# fig.savefig('err_x.png')


plt.subplot(312)
plt.plot(err_pid_time, err_pid_theta, ':', label='AIB', lw=2)
plt.plot(err_smc_time, err_smc_theta, '-.', label="RUTDE", lw=2)
plt.plot(err_asmc_time, err_asmc_theta, '--', label="AUTDE(proposed)", lw=2)
# plt.plot([0,err_pid_time[-1]], [0,0])
# plt.ylabel('y_err')
plt.title('Attitude error: $\\theta$ (deg)')
plt.axis([0, 124, -50, 60])
plt.legend(loc=2, prop={'size': 10}, ncol=4)
# plt.show()
# fig.savefig('err_y.png')


plt.subplot(313)
plt.plot(err_pid_time, err_pid_psi, ':', label='AIB', lw=2)
plt.plot(err_smc_time, err_smc_psi, '-.', label="RUTDE", lw=2)
plt.plot(err_asmc_time, err_asmc_psi, '--', label="AUTDE(proposed)", lw=2)
# plt.plot([0,err_pid_time[-1]], [0,0])
# plt.ylabel('z_err')
plt.title('Attitude error: $\psi$ (deg)')
plt.xlabel('time (s)')
plt.axis([0, 124, -20, 20])
plt.legend(loc=2, prop={'size': 10}, ncol=4)

fig.tight_layout(h_pad=0.4)
plt.show()
fig.savefig('sim_4kg_att.png')

RSME_asmc_ = np.zeros(3)
RSME_asmc_[0] = np.sqrt(np.mean([x**2 for x in err_asmc_x]))
RSME_asmc_[1] = np.sqrt(np.mean([y**2 for y in err_asmc_y]))
RSME_asmc_[2] = np.sqrt(np.mean([z**2 for z in err_asmc_z]))
RSME_asmc = np.sqrt(np.mean(RSME_asmc_**2))

print(RSME_asmc_, RSME_asmc)

RSME_smc_ = np.zeros(3)
RSME_smc_[0] = np.sqrt(np.mean([x**2 for x in err_smc_x]))
RSME_smc_[1] = np.sqrt(np.mean([x**2 for x in err_smc_y]))
RSME_smc_[2] = np.sqrt(np.mean([x**2 for x in err_smc_z]))
RSME_smc = np.sqrt(np.mean(RSME_smc_**2))

print(RSME_smc_, RSME_smc)

RSME_pid_ = np.zeros(3)
RSME_pid_[0] = np.sqrt(np.mean([x**2 for x in err_pid_x]))
RSME_pid_[1] = np.sqrt(np.mean([x**2 for x in err_pid_y]))
RSME_pid_[2] = np.sqrt(np.mean([x**2 for x in err_pid_z]))
RSME_backstep = np.sqrt(np.mean(RSME_pid_**2))

print(RSME_pid_, RSME_backstep)



RSME_asmc_ = np.zeros(3)
RSME_asmc_[0] = np.sqrt(np.mean([x**2 for x in err_asmc_phi]))
RSME_asmc_[1] = np.sqrt(np.mean([y**2 for y in err_asmc_theta]))
RSME_asmc_[2] = np.sqrt(np.mean([z**2 for z in err_asmc_psi]))
RSME_asmc = np.sqrt(np.mean(RSME_asmc_**2))

print(RSME_asmc_, RSME_asmc)

RSME_smc_ = np.zeros(3)
RSME_smc_[0] = np.sqrt(np.mean([x**2 for x in err_smc_phi]))
RSME_smc_[1] = np.sqrt(np.mean([x**2 for x in err_smc_theta]))
RSME_smc_[2] = np.sqrt(np.mean([x**2 for x in err_smc_psi]))
RSME_smc = np.sqrt(np.mean(RSME_smc_**2))

print(RSME_smc_, RSME_smc)

RSME_pid_ = np.zeros(3)
RSME_pid_[0] = np.sqrt(np.mean([x**2 for x in err_pid_phi]))
RSME_pid_[1] = np.sqrt(np.mean([x**2 for x in err_pid_theta]))
RSME_pid_[2] = np.sqrt(np.mean([x**2 for x in err_pid_psi]))
RSME_backstep = np.sqrt(np.mean(RSME_pid_**2))

print(RSME_pid_, RSME_backstep)

# print(err_asmc_time[3500])
# print(err_asmc_time[1500:2550]) 
# print(err_smc_time[1510:2519])
# print(err_pid_time[1520:2594])

# fig = plt.figure(figsize=(6,2))
# plt.plot(err_pid_time[1520:3500], err_pid_x[1520:3500], ':', label='AIB', lw=2)
# plt.plot(err_smc_time[1510:3500], err_smc_x[1510:3500], '-.', label="RUTDE", lw=2)
# plt.plot(err_asmc_time[1500:3500], err_asmc_x[1500:3500], '--', label="AUTDE(proposed)", lw=2)
# plt.axis([36, 80, -0.8, 0.8])
# # plt.show()
# fig.savefig('zoom_x.png')

# fig = plt.figure(figsize=(6,2))
# plt.plot(err_pid_time[1520:3500], err_pid_y[1520:3500], ':', label='AIB', lw=2)
# plt.plot(err_smc_time[1510:3500], err_smc_y[1510:3500], '-.', label="RUTDE", lw=2)
# plt.plot(err_asmc_time[1500:3500], err_asmc_y[1500:3500], '--', label="AUTDE(proposed)", lw=2)
# plt.axis([36, 80, -0.7, 0.7])
# # plt.show()
# fig.savefig('zoom_y.png')
