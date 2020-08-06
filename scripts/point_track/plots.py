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
odom_smc_x = [0]
odom_smc_y = [0]
odom_smc_z = [0.1]
odom_smc_time = [0]
odom_asmc_x = [0]
odom_asmc_y = [0]
odom_asmc_z = [0.1]
odom_asmc_time = [0]
odom_pd_ax = [0]
odom_pd_ay = [0]
odom_pd_az = [0]
odom_pd_aw = [0]
odom_smc_ax = [0]
odom_smc_ay = [0]
odom_smc_az = [0]
odom_smc_aw = [0]
odom_asmc_ax = [0]
odom_asmc_ay = [0]
odom_asmc_az = [0]
odom_asmc_aw = [1]
err_pd_x = [0]
err_pd_y = [0]
err_pd_z = [0.1]
err_pd_phi = [0]
err_pd_theta = [0]
err_pd_psi = [0]
err_pd_time = [0]
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

euler_smc_phi = []
euler_smc_theta = []
euler_smc_psi = []
for i in range(len(odom_smc_ax)):
	euler = euler_from_quaternion(np.array([odom_smc_ax[i], odom_smc_ay[i], odom_smc_az[i], odom_smc_aw[i]]))
	euler_smc_phi.append(euler[0])
	euler_smc_theta.append(euler[1])
	euler_smc_psi.append(euler[2])


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


euler_asmc_phi = []
euler_asmc_theta = []
euler_asmc_psi = []
for i in range(len(odom_asmc_ax)):
	euler = euler_from_quaternion(np.array([odom_asmc_ax[i], odom_asmc_ay[i], odom_asmc_az[i], odom_asmc_aw[i]]))
	euler_asmc_phi.append(euler[0])
	euler_asmc_theta.append(euler[1])
	euler_asmc_psi.append(euler[2])


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

traj_phi = []
traj_theta = []
traj_psi = []
for i in range(len(traj_ax)):
	euler = euler_from_quaternion(np.array([traj_ax[i], traj_ay[i], traj_az[i], traj_aw[i]]))
	traj_phi.append(euler[0])
	traj_theta.append(euler[1])
	traj_psi.append(euler[2])


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
		# err_asmc_x.append(traj_x[-1])
			err_pd_phi.append(float(k[1])*180/np.pi)
		if k[0] == 'Err_theta':
		# err_asmc_y.append(traj_y[-1])
			err_pd_theta.append(float(k[1])*180/np.pi)
		if k[0] == 'Err_psi':
		# err_asmc_z.append(traj_z[-1])
			err_pd_psi.append(float(k[1])*180/np.pi)
		if k[0] == 'secs':
		# print("time")
			time = float(k[1])
		if k[0] == 'nsecs':
			time += float(k[1])/1000000000
			err_pd_time.append(time)

traj_time.append(odom_pd_time[-1])
traj_x.append(traj_x[-1])
traj_y.append(traj_y[-1])
traj_z.append(traj_z[-1])
# falling_index = err_smc_time.index(39)
# falling_index = 1500
# print(min(err_pd_psi))



# fig = plt.figure(figsize=(6,12))
# fig, axes = plt.subplots(nrows=3, ncols=1)
# plt.subplot(311)
# plt.plot(traj_time, traj_x, label='Desired trajectory', lw=2)
# plt.plot(odom_pd_time[:len(odom_pd_time)], odom_pd_x[:len(odom_pd_time)], ':', label='PD', lw=2)
# plt.plot(odom_smc_time[:len(odom_pd_time)], odom_smc_x[:len(odom_pd_time)], '-.', label="SMC", lw=2)
# plt.plot(odom_asmc_time[:len(odom_pd_time)], odom_asmc_x[:len(odom_pd_time)], '--', label="ASMC(proposed)", lw=2)
# plt.axis([0, odom_pd_time[-1], -1, 4])
# # plt.ylabel('x')
# plt.title('Position tracking: x (m)')
# plt.legend(loc=2, prop={'size': 8}, ncol=4)
# # plt.show()
# # fig.savefig('traj_x.png')
# plt.subplot(312)
# plt.plot(traj_time, traj_y, label='Desired trajectory', lw=2)
# plt.plot(odom_pd_time[:len(odom_pd_time)], odom_pd_y[:len(odom_pd_time)], ':', label='PD', lw=2)
# plt.plot(odom_smc_time[:len(odom_pd_time)], odom_smc_y[:len(odom_pd_time)], '-.', label="SMC", lw=2)
# plt.plot(odom_asmc_time[:len(odom_pd_time)], odom_asmc_y[:len(odom_pd_time)], '--', label="ASMC(proposed)", lw=2)
# plt.axis([0, odom_pd_time[-1], -1, 4])
# plt.title('Position tracking: y (m)')
# # plt.ylabel('y')
# plt.legend(loc=2, prop={'size': 8}, ncol=4)
# # fig.savefig('traj_y.png')

# plt.subplot(313)
# plt.plot(traj_time, traj_z, label='Desired trajectory', lw=2)
# plt.plot(odom_pd_time[:len(odom_pd_time)], odom_pd_z[:len(odom_pd_time)], ':', label='PD', lw=2)
# plt.plot(odom_smc_time[:len(odom_pd_time)], odom_smc_z[:len(odom_pd_time)], '-.', label="SMC", lw=2)
# plt.plot(odom_asmc_time, odom_asmc_z, '--', label="ASMC(proposed)", lw=2)
# # plt.ylabel('z')
# plt.axis([0, odom_pd_time[-1], 0, 6])
# plt.legend(loc=2, prop={'size': 8}, ncol=4)
# # plt.xlabel('time')
# plt.title('Position tracking: z (m)')
# plt.xlabel('time (s)')


# fig.tight_layout()
# plt.show()
# fig.savefig('traj_exp1.png')


# # fig = plt.figure(figsize=(6,12))
# fig, axes = plt.subplots(nrows=3, ncols=1)
# plt.subplot(311)
# plt.plot(err_pd_time[:len(err_smc_time)], err_pd_x[:len(err_smc_time)], ':', label='PD', lw=2)
# plt.plot(err_smc_time[:len(err_smc_time)], err_smc_x[:len(err_smc_time)], '-.', label="SMC", lw=2)
# plt.plot(err_asmc_time[:len(err_smc_time)], err_asmc_x[:len(err_smc_time)], '--', label="ASMC(proposed)", lw=2)
# # plt.plot([0,err_pd_time[-1]], [0,0])
# # plt.ylabel('x_err')
# plt.title('Position error: x (m)')
# plt.axis([0, err_smc_time[-1], -6, 5])
# plt.legend(loc=3, prop={'size': 10}, ncol=4)
# # plt.show()
# # fig.savefig('err_x.png')


# plt.subplot(312)
# plt.plot(err_pd_time[:len(err_smc_time)], err_pd_y[:len(err_smc_time)], ':', label='PD', lw=2)
# plt.plot(err_smc_time[:len(err_smc_time)], err_smc_y[:len(err_smc_time)], '-.', label="SMC", lw=2)
# plt.plot(err_asmc_time[:len(err_smc_time)], err_asmc_y[:len(err_smc_time)], '--', label="ASMC(proposed)", lw=2)
# # plt.plot([0,err_pd_time[-1]], [0,0])
# # plt.ylabel('y_err')
# plt.title('Position error: y (m)')
# plt.axis([0, err_smc_time[-1], -7, 5])
# plt.legend(loc=3, prop={'size': 10}, ncol=4)
# # plt.show()
# # fig.savefig('err_y.png')


# plt.subplot(313)
# plt.plot(err_pd_time[:len(err_smc_time)], err_pd_z[:len(err_smc_time)], ':', label='PD', lw=2)
# plt.plot(err_smc_time[:len(err_smc_time)], err_smc_z[:len(err_smc_time)], '-.', label="SMC", lw=2)
# plt.plot(err_asmc_time[:len(err_smc_time)], err_asmc_z[:len(err_smc_time)], '--', label="ASMC(proposed)", lw=2)
# # plt.plot([0,err_pd_time[-1]], [0,0])
# # plt.ylabel('z_err')
# plt.title('Position error: z (m)')
# plt.xlabel('time (s)')
# plt.axis([0, err_smc_time[-1], -6, 3])
# plt.legend(loc=3, prop={'size': 10}, ncol=4)

# fig.tight_layout(h_pad=0.4)
# plt.show()
# fig.savefig('error_exp1.png')


# # fig = plt.figure(figsize=(6,12))
# fig, axes = plt.subplots(nrows=3, ncols=1)
# plt.subplot(311)
# plt.plot(err_pd_time[:len(err_smc_time)], err_pd_phi[:len(err_smc_time)], ':', label='PD', lw=2)
# plt.plot(err_smc_time[:len(err_smc_time)], err_smc_phi[:len(err_smc_time)], '-.', label="SMC", lw=2)
# plt.plot(err_asmc_time[:len(err_smc_time)], err_asmc_phi[:len(err_smc_time)], '--', label="ASMC(proposed)", lw=2)
# # plt.plot([0,err_pd_time[-1]], [0,0])
# # plt.ylabel('x_err')
# plt.title('Attitude error: $\phi$ (deg)')
# plt.axis([0, err_smc_time[-1], -90, 60])
# plt.legend(loc=3, prop={'size': 10}, ncol=4)
# # plt.show()
# # fig.savefig('err_x.png')


# plt.subplot(312)
# plt.plot(err_pd_time[:len(err_smc_time)], err_pd_theta[:len(err_smc_time)], ':', label='PD', lw=2)
# plt.plot(err_smc_time[:len(err_smc_time)], err_smc_theta[:len(err_smc_time)], '-.', label="SMC", lw=2)
# plt.plot(err_asmc_time[:len(err_smc_time)], err_asmc_theta[:len(err_smc_time)], '--', label="ASMC(proposed)", lw=2)
# # plt.plot([0,err_pd_time[-1]], [0,0])
# # plt.ylabel('y_err')
# plt.title('Attitude error: $\\theta$ (deg)')
# plt.axis([0, err_smc_time[-1], -60, 90])
# plt.legend(loc=2, prop={'size': 10}, ncol=4)
# # plt.show()
# # fig.savefig('err_y.png')


# plt.subplot(313)
# plt.plot(err_pd_time[:len(err_smc_time)], err_pd_psi[:len(err_smc_time)], ':', label='PD', lw=2)
# plt.plot(err_smc_time[:len(err_smc_time)], err_smc_psi[:len(err_smc_time)], '-.', label="SMC", lw=2)
# plt.plot(err_asmc_time[:len(err_smc_time)], err_asmc_psi[:len(err_smc_time)], '--', label="ASMC(proposed)", lw=2)
# # plt.plot([0,err_pd_time[-1]], [0,0])
# # plt.ylabel('z_err')
# plt.title('Attitude error: $\psi$ (deg)')
# plt.xlabel('time (s)')
# plt.axis([0, err_smc_time[-1], -60, 110])
# plt.legend(loc=2, prop={'size': 10}, ncol=4)

# fig.tight_layout(h_pad=0.4)
# plt.show()
# fig.savefig('error_att_exp1.png')

pick_time = 47
asmc_pick = err_asmc_time.index(pick_time)
smc_pick = err_smc_time.index(pick_time)
pd_pick = err_pd_time.index(pick_time)



RSME_asmc_ = np.zeros(3)
RSME_asmc_[0] = np.sqrt(np.mean([x**2 for x in err_asmc_x[:asmc_pick]]))
RSME_asmc_[1] = np.sqrt(np.mean([y**2 for y in err_asmc_y[:asmc_pick]]))
RSME_asmc_[2] = np.sqrt(np.mean([z**2 for z in err_asmc_z[:asmc_pick]]))
RSME_asmc = np.sqrt(np.mean(RSME_asmc_**2))

print("ASMC without payload")
print(RSME_asmc_, RSME_asmc)

RSME_smc_ = np.zeros(3)
RSME_smc_[0] = np.sqrt(np.mean([x**2 for x in err_smc_x[:smc_pick]]))
RSME_smc_[1] = np.sqrt(np.mean([x**2 for x in err_smc_y[:smc_pick]]))
RSME_smc_[2] = np.sqrt(np.mean([x**2 for x in err_smc_z[:smc_pick]]))
RSME_smc = np.sqrt(np.mean(RSME_smc_**2))

print("SMC without payload")
print(RSME_smc_, RSME_smc)


RSME_asmc_ = np.zeros(3)
RSME_asmc_[0] = np.sqrt(np.mean([x**2 for x in err_asmc_x[asmc_pick:]]))
RSME_asmc_[1] = np.sqrt(np.mean([y**2 for y in err_asmc_y[asmc_pick:]]))
RSME_asmc_[2] = np.sqrt(np.mean([z**2 for z in err_asmc_z[asmc_pick:]]))
RSME_asmc = np.sqrt(np.mean(RSME_asmc_**2))

print("ASMC with payload")
print(RSME_asmc_, RSME_asmc)

RSME_smc_ = np.zeros(3)
RSME_smc_[0] = np.sqrt(np.mean([x**2 for x in err_smc_x[smc_pick:]]))
RSME_smc_[1] = np.sqrt(np.mean([x**2 for x in err_smc_y[smc_pick:]]))
RSME_smc_[2] = np.sqrt(np.mean([x**2 for x in err_smc_z[smc_pick:]]))
RSME_smc = np.sqrt(np.mean(RSME_smc_**2))

print("SMC with payload")
print(RSME_smc_, RSME_smc)

RSME_asmc_ = np.zeros(3)
RSME_asmc_[0] = np.sqrt(np.mean([x**2 for x in err_asmc_phi[:asmc_pick]]))
RSME_asmc_[1] = np.sqrt(np.mean([y**2 for y in err_asmc_theta[:asmc_pick]]))
RSME_asmc_[2] = np.sqrt(np.mean([z**2 for z in err_asmc_psi[:asmc_pick]]))
RSME_asmc = np.sqrt(np.mean(RSME_asmc_**2))

print("ASMC without payload")
print(RSME_asmc_, RSME_asmc)

RSME_smc_ = np.zeros(3)
RSME_smc_[0] = np.sqrt(np.mean([x**2 for x in err_smc_phi[:smc_pick]]))
RSME_smc_[1] = np.sqrt(np.mean([x**2 for x in err_smc_theta[:smc_pick]]))
RSME_smc_[2] = np.sqrt(np.mean([x**2 for x in err_smc_psi[smc_pick:]]))
RSME_smc = np.sqrt(np.mean(RSME_smc_**2))

print("SMC without payload")
print(RSME_smc_, RSME_smc)

RSME_asmc_ = np.zeros(3)
RSME_asmc_[0] = np.sqrt(np.mean([x**2 for x in err_asmc_phi[asmc_pick:]]))
RSME_asmc_[1] = np.sqrt(np.mean([y**2 for y in err_asmc_theta[asmc_pick:]]))
RSME_asmc_[2] = np.sqrt(np.mean([z**2 for z in err_asmc_psi[asmc_pick:]]))
RSME_asmc = np.sqrt(np.mean(RSME_asmc_**2))

print("ASMC with payload")
print(RSME_asmc_, RSME_asmc)

RSME_smc_ = np.zeros(3)
RSME_smc_[0] = np.sqrt(np.mean([x**2 for x in err_smc_phi[smc_pick:]]))
RSME_smc_[1] = np.sqrt(np.mean([x**2 for x in err_smc_theta[smc_pick:]]))
RSME_smc_[2] = np.sqrt(np.mean([x**2 for x in err_smc_psi[smc_pick:]]))
RSME_smc = np.sqrt(np.mean(RSME_smc_**2))

print("SMC with payload")
print(RSME_smc_, RSME_smc)

RSME_pd_ = np.zeros(3)
# RSME_pd_[0] = np.sqrt(np.mean(err_pd_x[:len(err_smc_time)]))
# RSME_pd_[1] = np.sqrt(np.mean(err_pd_y[:len(err_smc_time)]))
# RSME_pd_[2] = np.sqrt(np.mean(err_pd_z[:len(err_smc_time)]))
# RSME_pd = np.sqrt(np.mean(RSME_pd_))
