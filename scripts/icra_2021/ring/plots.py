import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
# from geometry_msgs.msg import PoseWithCovarianceStamped
# import geometry_msgs
# from tf.transformations import euler_from_quaternion



odom_pd_x = [0]
odom_pd_y = [0]
odom_pd_z = [0.1]
odom_pd_time = [0]
odom_smc_x = [0]
odom_smc_y = [0]
odom_smc_z = [0.1]
odom_smc_time = [0]
odom_sb_x = [0]
odom_sb_y = [0]
odom_sb_z = [0.1]
odom_sb_time = [0]
odom_rsb_x = [0]
odom_rsb_y = [0]
odom_rsb_z = [0.1]
odom_rsb_time = [0]
odom_pd_ax = [0]
odom_pd_ay = [0]
odom_pd_az = [0]
odom_pd_aw = [0]
odom_smc_ax = [0]
odom_smc_ay = [0]
odom_smc_az = [0]
odom_smc_aw = [0]
odom_rsb_ax = [0]
odom_rsb_ay = [0]
odom_rsb_az = [0]
odom_rsb_aw = [1]
odom_sb_ax = [0]
odom_sb_ay = [0]
odom_sb_az = [0]
odom_sb_aw = [1]
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
err_smc_thrust = [0]
err_smc_mx = [0]
err_smc_my = [0]
err_smc_mz = [0]
err_smc_time = [0]
err_smc_phi = [0]
err_smc_theta = [0]
err_smc_psi = [0]
err_sb_x = [0]
err_sb_y = [0]
err_sb_z = [0.1]
err_sb_thrust = [0]
err_sb_mx = [0]
err_sb_my = [0]
err_sb_mz = [0]
err_sb_phi = [0]
err_sb_theta = [0]
err_sb_psi = [0]
err_sb_time = [0]
err_rsb_x = [0]
err_rsb_y = [0]
err_rsb_z = [0.1]
err_rsb_thrust = [0]
err_rsb_mx = [0]
err_rsb_my = [0]
err_rsb_mz = [0]
err_rsb_phi = [0]
err_rsb_theta = [0]
err_rsb_psi = [0]
err_rsb_time = [0]
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

# euler_pd_phi = []
# euler_pd_theta = []
# euler_pd_psi = []
# for i in range(len(odom_pd_ax)):
# 	euler = euler_from_quaternion(np.array([odom_pd_ax[i], odom_pd_ay[i], odom_pd_az[i], odom_pd_aw[i]]))
# 	euler_pd_phi.append(euler[0])
# 	euler_pd_theta.append(euler[1])
# 	euler_pd_psi.append(euler[2])



with open('odom_smc1_ring_wind.txt') as file:
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

# euler_smc_phi = []
# euler_smc_theta = []
# euler_smc_psi = []
# for i in range(len(odom_smc_ax)):
# 	euler = euler_from_quaternion(np.array([odom_smc_ax[i], odom_smc_ay[i], odom_smc_az[i], odom_smc_aw[i]]))
# 	euler_smc_phi.append(euler[0])
# 	euler_smc_theta.append(euler[1])
# 	euler_smc_psi.append(euler[2])


with open('odom_sb.txt') as file:
	for i in file.readlines():
		k = i.split(':')

		if k[0] == 'X':
			odom_sb_x.append(float(k[1]))
		if k[0] == 'Y':
			odom_sb_y.append(float(k[1]))
		if k[0] == 'Z':
			odom_sb_z.append(float(k[1]))
		if k[0] == 'a_X':
			odom_sb_ax.append(float(k[1]))
		if k[0] == 'a_Y':
			odom_sb_ay.append(float(k[1]))
		if k[0] == 'a_Z':
			odom_sb_az.append(float(k[1]))
		if k[0] == 'a_W':
			odom_sb_aw.append(float(k[1]))
		if k[0] == 'secs':
			time = float(k[1])
		if k[0] == 'nsecs':
			time += float(k[1])/1000000000
			odom_sb_time.append(time)


# euler_sb_phi = []
# euler_sb_theta = []
# euler_sb_psi = []
# for i in range(len(odom_sb_ax)):
# 	euler = euler_from_quaternion(np.array([odom_sb_ax[i], odom_sb_ay[i], odom_sb_az[i], odom_sb_aw[i]]))
# 	euler_sb_phi.append(euler[0])
# 	euler_sb_theta.append(euler[1])
# 	euler_sb_psi.append(euler[2])


with open('error_sb.txt') as file:
	for i in file.readlines():
		k = i.split(':')

		if k[0] == 'Err_X':
		# err_sb_x.append(traj_x[-1])
			err_sb_x.append(float(k[1]))
		if k[0] == 'Err_Y':
		# err_sb_y.append(traj_y[-1])
			err_sb_y.append(float(k[1]))
		if k[0] == 'Err_Z':
		# err_sb_z.append(traj_z[-1])
			err_sb_z.append(float(k[1]))
		if k[0] == 'Err_phi':
		# err_sb_x.append(traj_x[-1])
			err_sb_phi.append(float(k[1])*180/np.pi)
		if k[0] == 'Err_theta':
		# err_sb_y.append(traj_y[-1])
			err_sb_theta.append(float(k[1])*180/np.pi)
		if k[0] == 'Err_psi':
		# err_sb_z.append(traj_z[-1])
			err_sb_psi.append(float(k[1])*180/np.pi)
		if k[0] == 'Thrust':
		# err_sb_z.append(traj_z[-1])
			err_sb_thrust.append(float(k[1]))
		if k[0] == 'Moments_x':
		# err_sb_z.append(traj_z[-1])
			err_sb_mx.append(float(k[1]))
		if k[0] == 'Moments_y':
		# err_sb_z.append(traj_z[-1])
			err_sb_my.append(float(k[1]))
		if k[0] == 'Moments_z':
		# err_sb_z.append(traj_z[-1])
			err_sb_mz.append(float(k[1]))
		if k[0] == 'secs':
		# print("time")
			time = float(k[1])
		if k[0] == 'nsecs':
			time += float(k[1])/1000000000
			err_sb_time.append(time)

with open('odom_rsb1_ring_wind.txt') as file:
	for i in file.readlines():
		k = i.split(':')

		if k[0] == 'X':
			odom_rsb_x.append(float(k[1]))
		if k[0] == 'Y':
			odom_rsb_y.append(float(k[1]))
		if k[0] == 'Z':
			odom_rsb_z.append(float(k[1]))
		if k[0] == 'a_X':
			odom_rsb_ax.append(float(k[1]))
		if k[0] == 'a_Y':
			odom_rsb_ay.append(float(k[1]))
		if k[0] == 'a_Z':
			odom_rsb_az.append(float(k[1]))
		if k[0] == 'a_W':
			odom_rsb_aw.append(float(k[1]))
		if k[0] == 'secs':
			time = float(k[1])
		if k[0] == 'nsecs':
			time += float(k[1])/1000000000
			odom_rsb_time.append(time)


# euler_rsb_phi = []
# euler_rsb_theta = []
# euler_rsb_psi = []
# for i in range(len(odom_rsb_ax)):
# 	euler = euler_from_quaternion(np.array([odom_rsb_ax[i], odom_rsb_ay[i], odom_rsb_az[i], odom_rsb_aw[i]]))
# 	euler_rsb_phi.append(euler[0])
# 	euler_rsb_theta.append(euler[1])
# 	euler_rsb_psi.append(euler[2])


with open('traj_rsb1_ring_wind.txt') as file:
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

# traj_phi = []
# traj_theta = []
# traj_psi = []
# for i in range(len(traj_ax)):
# 	euler = euler_from_quaternion(np.array([traj_ax[i], traj_ay[i], traj_az[i], traj_aw[i]]))
# 	traj_phi.append(euler[0])
# 	traj_theta.append(euler[1])
# 	traj_psi.append(euler[2])


with open('error_rsb1_ring_wind.txt') as file:
	for i in file.readlines():
		k = i.split(':')

		if k[0] == 'Err_X':
		# err_rsb_x.append(traj_x[-1])
			err_rsb_x.append(float(k[1]))
		if k[0] == 'Err_Y':
		# err_rsb_y.append(traj_y[-1])
			err_rsb_y.append(float(k[1]))
		if k[0] == 'Err_Z':
		# err_rsb_z.append(traj_z[-1])
			err_rsb_z.append(float(k[1]))
		if k[0] == 'Err_phi':
		# err_rsb_x.append(traj_x[-1])
			err_rsb_phi.append(float(k[1])*180/np.pi)
		if k[0] == 'Err_theta':
		# err_rsb_y.append(traj_y[-1])
			err_rsb_theta.append(float(k[1])*180/np.pi)
		if k[0] == 'Err_psi':
		# err_rsb_z.append(traj_z[-1])
			err_rsb_psi.append(float(k[1])*180/np.pi)
		if k[0] == 'Thrust':
		# err_rsb_z.append(traj_z[-1])
			err_rsb_thrust.append(float(k[1]))
		if k[0] == 'Moments_x':
		# err_rsb_z.append(traj_z[-1])
			err_rsb_mx.append(float(k[1]))
		if k[0] == 'Moments_y':
		# err_rsb_z.append(traj_z[-1])
			err_rsb_my.append(float(k[1]))
		if k[0] == 'Moments_z':
		# err_rsb_z.append(traj_z[-1])
			err_rsb_mz.append(float(k[1]))
		if k[0] == 'secs':
		# print("time")
			time = float(k[1])
		if k[0] == 'nsecs':
			time += float(k[1])/1000000000
			err_rsb_time.append(time)



with open('error_smc1_ring_wind.txt') as file:
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
		# err_rsb_x.append(traj_x[-1])
			err_smc_phi.append(float(k[1])*180/np.pi)
		if k[0] == 'Err_theta':
		# err_rsb_y.append(traj_y[-1])
			err_smc_theta.append(float(k[1])*180/np.pi)
		if k[0] == 'Err_psi':
		# err_rsb_z.append(traj_z[-1])
			err_smc_psi.append(float(k[1])*180/np.pi)
		if k[0] == 'Thrust':
		# err_rsb_z.append(traj_z[-1])
			err_smc_thrust.append(float(k[1]))
		if k[0] == 'Moments_x':
		# err_rsb_z.append(traj_z[-1])
			err_smc_mx.append(float(k[1]))
		if k[0] == 'Moments_y':
		# err_rsb_z.append(traj_z[-1])
			err_smc_my.append(float(k[1]))
		if k[0] == 'Moments_z':
		# err_rsb_z.append(traj_z[-1])
			err_smc_mz.append(float(k[1]))
		if k[0] == 'secs':
		# print("time")
			time = float(k[1])
		if k[0] == 'nsecs':
			time += float(k[1])/1000000000
			err_smc_time.append(time)



# with open('error_pd.txt') as file:
# 	for i in file.readlines():
# 		k = i.split(':')

# 		if k[0] == 'Err_X':
# 		# err_pd_x.append(traj_x[-1])
# 			err_pd_x.append(float(k[1]))
# 		if k[0] == 'Err_Y':
# 		# err_pd_y.append(traj_y[-1])
# 			err_pd_y.append(float(k[1]))
# 		if k[0] == 'Err_Z':
# 		# err_pd_z.append(traj_z[-1])
# 			err_pd_z.append(float(k[1]))
# 		if k[0] == 'Err_phi':
# 		# err_rsb_x.append(traj_x[-1])
# 			err_pd_phi.append(float(k[1])*180/np.pi)
# 		if k[0] == 'Err_theta':
# 		# err_rsb_y.append(traj_y[-1])
# 			err_pd_theta.append(float(k[1])*180/np.pi)
# 		if k[0] == 'Err_psi':
# 		# err_rsb_z.append(traj_z[-1])
# 			err_pd_psi.append(float(k[1])*180/np.pi)
# 		if k[0] == 'secs':
# 		# print("time")
# 			time = float(k[1])
# 		if k[0] == 'nsecs':
# 			time += float(k[1])/1000000000
# 			err_pd_time.append(time)

traj_time.append(odom_rsb_time[-1])
traj_x.append(traj_x[-1])
traj_y.append(traj_y[-1])
traj_z.append(traj_z[-1])
# falling_index = err_smc_time.index(39)
# falling_index = 1500
# print(max(err_pd_psi))


# fig = plt.figure(figsize=(6,12))
# fig, axes = plt.subplots(nrows=3, ncols=1, figsize=(8,4))
# plt.subplot(311)
# plt.plot(traj_time, traj_x, label='Desired trajectory', c='cyan', lw=4)
# # plt.plot(odom_pd_time[:len(odom_smc_time)], odom_pd_x[:len(odom_smc_time)], label='PD', lw=2)
# plt.plot(odom_smc_time[:len(odom_smc_time)], odom_smc_x[:len(odom_smc_time)], '--', c='orange', label="SMC", lw=2)
# plt.plot(odom_rsb_time[:len(odom_smc_time)], odom_rsb_x[:len(odom_smc_time)], 'b-.', label="RSB(proposed)", lw=2)
# # plt.plot(odom_sb_time[:len(odom_sb_time)], odom_sb_x[:len(odom_sb_time)], label="SB", lw=2)
# plt.axis([0, odom_smc_time[-1], -5, 5])
# # plt.ylabel('x')
# plt.title('Position tracking: x (m)')
# plt.legend(loc=2, prop={'size': 8}, ncol=4)
# # plt.show()
# # fig.savefig('traj_x.png')
# plt.subplot(312)
# plt.plot(traj_time, traj_y, c='cyan', label='Desired trajectory', lw=4)
# # plt.plot(odom_pd_time[:len(odom_smc_time)], odom_pd_y[:len(odom_smc_time)], label='PD', lw=2)
# plt.plot(odom_smc_time[:len(odom_smc_time)], odom_smc_y[:len(odom_smc_time)], '--', c='orange', label="SMC", lw=2)
# plt.plot(odom_rsb_time[:len(odom_smc_time)], odom_rsb_y[:len(odom_smc_time)], 'b-.', label="RSB(proposed)", lw=2)
# # plt.plot(odom_sb_time[:len(odom_sb_time)], odom_sb_y[:len(odom_sb_time)], label="SB", lw=2)
# plt.axis([0, odom_smc_time[-1], -3, 8])
# plt.title('Position tracking: y (m)')
# # plt.ylabel('y')
# plt.legend(loc=2, prop={'size': 8}, ncol=4)
# # fig.savefig('traj_y.png')

# plt.subplot(313)
# plt.plot(traj_time, traj_z, c='cyan', label='Desired trajectory', lw=4)
# # plt.plot(odom_pd_time[:len(odom_smc_time)], odom_pd_z[:len(odom_smc_time)], label='PD', lw=2)
# plt.plot(odom_smc_time[:len(odom_smc_time)], odom_smc_z[:len(odom_smc_time)], '--', c='orange', label="SMC", lw=2)
# plt.plot(odom_rsb_time, odom_rsb_z, 'b-.', label="RSB(proposed)", lw=2)
# # plt.plot(odom_sb_time, odom_sb_z, label="SB", lw=2)
# # plt.ylabel('z')
# plt.axis([0, odom_smc_time[-1], -0.1, 4])
# plt.legend(loc=2, prop={'size': 8}, ncol=4)
# # plt.xlabel('time')
# plt.title('Position tracking: z (m)')
# plt.xlabel('time (s)')


# fig.tight_layout()
# plt.show()
# fig.savefig('ring_wind_traj_exp3.png')


# # # fig = plt.figure(figsize=(6,12))
# fig, axes = plt.subplots(nrows=3, ncols=1, figsize=(12,4))
# plt.subplot(311)
# # plt.plot(err_pd_time[:len(err_smc_time)], err_pd_x[:len(err_smc_time)], label='PD', lw=2)
# plt.plot(err_smc_time[:len(err_smc_time)], err_smc_x[:len(err_smc_time)], '--', c='orange', label="SMC", lw=2)
# plt.plot(err_rsb_time[:len(err_smc_time)], err_rsb_x[:len(err_smc_time)], label="RSB(proposed)", lw=2)
# plt.plot(np.array([0,err_smc_time[-1]]), np.array([0.3,0.3]), 'r-', label='bounds', lw=0.5)
# plt.plot(np.array([0,err_smc_time[-1]]), np.array([-0.3,-0.3]), 'r-' , lw=0.5)
# # plt.plot(err_sb_time[:len(err_sb_time)], err_sb_x[:len(err_sb_time)], label="SB", lw=2)
# # plt.plot([0,err_pd_time[-1]], [0,0])
# # plt.ylabel('x_err')
# plt.title('Position error: x (m)')
# plt.axis([0, err_smc_time[-1], -1, 1])
# plt.legend(loc=3, prop={'size': 10}, ncol=4)
# # plt.show()
# # fig.savefig('err_x.png')


# plt.subplot(312)
# # plt.plot(err_pd_time[:len(err_smc_time)], err_pd_y[:len(err_smc_time)], label='PD', lw=2)
# plt.plot(err_smc_time[:len(err_smc_time)], err_smc_y[:len(err_smc_time)], '--', c='orange', label="SMC", lw=2)
# plt.plot(err_rsb_time[:len(err_smc_time)], err_rsb_y[:len(err_smc_time)], label="RSB(proposed)", lw=2)
# plt.plot(np.array([0,err_smc_time[-1]]), np.array([0.3,0.3]), 'r-' , label='bounds', lw=0.5)
# plt.plot(np.array([0,err_smc_time[-1]]), np.array([-0.3,-0.3]), 'r-' , lw=0.5)
# # plt.plot(err_sb_time[:len(err_sb_time)], err_sb_y[:len(err_sb_time)], label="SB", lw=2)
# # plt.plot([0,err_pd_time[-1]], [0,0])
# # plt.ylabel('y_err')
# plt.title('Position error: y (m)')
# plt.axis([0, err_smc_time[-1], -1, 8])
# plt.legend(loc=2, prop={'size': 10}, ncol=4)
# # plt.show()
# # fig.savefig('err_y.png')


# plt.subplot(313)
# # plt.plot(err_pd_time[:len(err_smc_time)], err_pd_z[:len(err_smc_time)], label='PD', lw=2)
# plt.plot(err_smc_time[:len(err_smc_time)], err_smc_z[:len(err_smc_time)], '--', c='orange', label="SMC", lw=2)
# plt.plot(err_rsb_time[:len(err_smc_time)], err_rsb_z[:len(err_smc_time)], label="RSB(proposed)", lw=2)
# plt.plot(np.array([0,err_smc_time[-1]]), np.array([0.16,0.16]), 'r-' , label='bounds', lw=0.5)
# plt.plot(np.array([0,err_smc_time[-1]]), np.array([-0.16,-0.16]), 'r-' , lw=0.5)
# # plt.plot(err_sb_time[:len(err_sb_time)], err_sb_z[:len(err_sb_time)], label="SB", lw=2)
# # plt.plot([0,err_pd_time[-1]], [0,0])
# # plt.ylabel('z_err')
# plt.title('Position error: z (m)')
# plt.xlabel('time (s)')
# plt.axis([0, err_smc_time[-1], -0.5, 0.2])
# plt.legend(loc=3, prop={'size': 10}, ncol=4)

# fig.tight_layout(h_pad=0.4)
# plt.show()
# fig.savefig('ring_wind_pos_error_exp3.png')

# # # fig = plt.figure(figsize=(6,12))
# fig, axes = plt.subplots(nrows=4, ncols=1, figsize=(8,4))
# plt.subplot(411)
# # plt.plot(err_pd_time[:len(err_smc_time)], err_pd_thrust[:len(err_smc_time)], label='PD', lw=2)
# plt.plot(err_smc_time[:len(err_smc_time)], err_smc_thrust[:len(err_smc_time)], '--', c='orange', label="SMC", lw=2)
# plt.plot(err_rsb_time[:len(err_smc_time)], err_rsb_thrust[:len(err_smc_time)], label="RSB(proposed)", lw=2)
# # plt.plot(err_sb_time[:len(err_sb_time)], err_sb_thrust[:len(err_sb_time)], c='red', label="SB", lw=2)
# # plt.plot([0,err_pd_time[-1]], [0,0])
# # plt.ylabel('x_err')
# plt.title('Thrust')
# plt.axis([0, err_smc_time[-1], -5, 40])
# plt.legend(loc=2, prop={'size': 10}, ncol=4)
# # plt.show()
# # fig.savefig('err_x.png')


# plt.subplot(412)
# # plt.plot(err_pd_time[:len(err_smc_time)], err_pd_mx[:len(err_smc_time)], label='PD', lw=2)
# plt.plot(err_smc_time[:len(err_smc_time)], err_smc_mx[:len(err_smc_time)], '--', c='orange', label="SMC", lw=2)
# plt.plot(err_rsb_time[:len(err_smc_time)], err_rsb_mx[:len(err_smc_time)], label="RSB(proposed)", lw=2)
# # plt.plot(err_sb_time[:len(err_sb_time)], err_sb_mx[:len(err_sb_time)], c='red', label="SB", lw=2)
# # plt.plot([0,err_pd_time[-1]], [0,0])
# # plt.ylabel('y_err')
# plt.title('Moment: x')
# plt.axis([0, err_smc_time[-1], -0.5, 0.5])
# plt.legend(loc=3, prop={'size': 10}, ncol=4)
# # plt.show()
# # fig.savefig('err_y.png')


# plt.subplot(413)
# # plt.plot(err_pd_time[:len(err_smc_time)], err_pd_my[:len(err_smc_time)], label='PD', lw=2)
# plt.plot(err_smc_time[:len(err_smc_time)], err_smc_my[:len(err_smc_time)], '--', c='orange', label="SMC", lw=2)
# plt.plot(err_rsb_time[:len(err_smc_time)], err_rsb_my[:len(err_smc_time)], label="RSB(proposed)", lw=2)
# # plt.plot(err_sb_time[:len(err_sb_time)], err_sb_my[:len(err_sb_time)], c='red', label="SB", lw=2)
# # plt.plot([0,err_pd_time[-1]], [0,0])
# # plt.ylabel('y_err')
# plt.title('Moment: y')
# plt.axis([0, err_smc_time[-1], -0.5, 0.5])
# plt.legend(loc=3, prop={'size': 10}, ncol=4)
# # plt.show()
# # fig.savefig('err_y.png')


# plt.subplot(414)
# # plt.plot(err_pd_time[:len(err_smc_time)], err_pd_mz[:len(err_smc_time)], label='PD', lw=2)
# plt.plot(err_smc_time[:len(err_smc_time)], err_smc_mz[:len(err_smc_time)], '--', c='orange', label="SMC", lw=2)
# plt.plot(err_rsb_time[:len(err_smc_time)], err_rsb_mz[:len(err_smc_time)], label="RSB(proposed)", lw=2)
# # plt.plot(err_sb_time[:len(err_sb_time)], err_sb_mz[:len(err_sb_time)], c='red', label="SB", lw=2)
# # plt.plot([0,err_pd_time[-1]], [0,0])
# # plt.ylabel('z_err')
# plt.title('Moment: z')
# plt.xlabel('time (s)')
# plt.axis([0, err_smc_time[-1], -0.04, 0.04])
# plt.legend(loc=3, prop={'size': 10}, ncol=4)

# fig.tight_layout(h_pad=0.4)
# plt.show()
# fig.savefig('ring_wind_control_op_exp3.png')


# # fig = plt.figure(figsize=(6,12))
# fig, axes = plt.subplots(nrows=3, ncols=1, figsize=(16,8))
# plt.subplot(311)
# # plt.plot(err_pd_time[:len(err_smc_time)], err_pd_phi[:len(err_smc_time)], label='PD', lw=2)
# plt.plot(err_smc_time[:len(err_smc_time)], err_smc_phi[:len(err_smc_time)], '--', c='orange', label="SMC", lw=2)
# plt.plot(err_rsb_time[:len(err_smc_time)], err_rsb_phi[:len(err_smc_time)], label="RSB(proposed)", lw=2)
# plt.plot(np.array([0,err_smc_time[-1]]), np.array([28,28]), 'r-' , label='bounds', lw=0.5)
# plt.plot(np.array([0,err_smc_time[-1]]), np.array([-28,-28]), 'r-' , lw=0.5)
# # plt.plot(err_sb_time[:len(err_sb_time)], err_sb_phi[:len(err_sb_time)], label="SB", lw=2)
# # plt.plot([0,err_pd_time[-1]], [0,0])
# # plt.ylabel('x_err')
# plt.title('Attitude error: $\phi$ (deg)')
# plt.axis([0, err_smc_time[-1], -35, 35])
# plt.legend(loc=3, prop={'size': 10}, ncol=4)
# # plt.show()
# # fig.savefig('err_x.png')


# plt.subplot(312)
# # plt.plot(err_pd_time[:len(err_smc_time)], err_pd_theta[:len(err_smc_time)], label='PD', lw=2)
# plt.plot(err_smc_time[:len(err_smc_time)], err_smc_theta[:len(err_smc_time)], '--', c='orange', label="SMC", lw=2)
# plt.plot(err_rsb_time[:len(err_smc_time)], err_rsb_theta[:len(err_smc_time)], label="RSB(proposed)", lw=2)
# plt.plot(np.array([0,err_smc_time[-1]]), np.array([28,28]), 'r-' , label='bounds', lw=0.5)
# plt.plot(np.array([0,err_smc_time[-1]]), np.array([-28,-28]), 'r-' , lw=0.5)
# # plt.plot(err_sb_time[:len(err_sb_time)], err_sb_theta[:len(err_sb_time)], label="SB", lw=2)
# # plt.plot([0,err_pd_time[-1]], [0,0])
# # plt.ylabel('y_err')
# plt.title('Attitude error: $\\theta$ (deg)')
# plt.axis([0, err_smc_time[-1], -35, 35])
# plt.legend(loc=2, prop={'size': 10}, ncol=4)
# # plt.show()
# # fig.savefig('err_y.png')


# plt.subplot(313)
# # plt.plot(err_pd_time[:len(err_smc_time)], err_pd_psi[:len(err_smc_time)], label='PD', lw=2)
# plt.plot(err_smc_time[:len(err_smc_time)], err_smc_psi[:len(err_smc_time)], '--', c='orange', label="SMC", lw=2)
# plt.plot(err_rsb_time[:len(err_smc_time)], err_rsb_psi[:len(err_smc_time)], label="RSB(proposed)", lw=2)
# plt.plot(np.array([0,err_smc_time[-1]]), np.array([14,14]), 'r-' , label='bounds', lw=0.5)
# plt.plot(np.array([0,err_smc_time[-1]]), np.array([-14,-14]), 'r-' , lw=0.5)
# # plt.plot(err_sb_time[:len(err_sb_time)], err_sb_psi[:len(err_sb_time)], label="SB", lw=2)
# # plt.plot([0,err_pd_time[-1]], [0,0])
# # plt.ylabel('z_err')
# plt.title('Attitude error: $\psi$ (deg)')
# plt.xlabel('time (s)')
# plt.axis([0, err_smc_time[-1], -15, 15])
# plt.legend(loc=2, prop={'size': 10}, ncol=4)

# fig.tight_layout(h_pad=0.4)
# plt.show()
# fig.savefig('ring_wind_error_att_exp2.png')


np.savetxt('rsb_data.txt', (odom_rsb_time, odom_rsb_x, odom_rsb_y, odom_rsb_z), delimiter=',')
np.savetxt('smc_data.txt', (odom_smc_time, odom_smc_x, odom_smc_y, odom_smc_z), delimiter=',')
np.savetxt('rsb_control.txt', (err_rsb_time, err_rsb_thrust, err_rsb_mx, err_rsb_my, err_rsb_mz), delimiter=',')
np.savetxt('smc_control.txt', (err_smc_time, err_smc_thrust, err_smc_mx, err_smc_my, err_smc_mz), delimiter=',')
np.savetxt('rsb_error.txt', (err_rsb_time, err_rsb_x, err_rsb_y, err_rsb_z, err_rsb_phi, err_rsb_theta, err_rsb_psi), delimiter=',')
np.savetxt('smc_error.txt', (err_smc_time, err_smc_x, err_smc_y, err_smc_z, err_smc_phi, err_smc_theta, err_smc_psi), delimiter=',')

RSME_rsb_ = np.zeros(3)
RSME_rsb_[0] = np.sqrt(np.mean([x**2 for x in err_rsb_x]))
RSME_rsb_[1] = np.sqrt(np.mean([y**2 for y in err_rsb_y]))
RSME_rsb_[2] = np.sqrt(np.mean([z**2 for z in err_rsb_z]))
RSME_rsb = np.sqrt(np.mean(RSME_rsb_**2))

print(RSME_rsb_, RSME_rsb)

RSME_smc_ = np.zeros(3)
RSME_smc_[0] = np.sqrt(np.mean([x**2 for x in err_smc_x]))
RSME_smc_[1] = np.sqrt(np.mean([x**2 for x in err_smc_y]))
RSME_smc_[2] = np.sqrt(np.mean([x**2 for x in err_smc_z]))
RSME_smc = np.sqrt(np.mean(RSME_smc_**2))

print(RSME_smc_, RSME_smc)



RSME_rsb_ = np.zeros(3)
RSME_rsb_[0] = np.sqrt(np.mean([x**2 for x in err_rsb_phi]))
RSME_rsb_[1] = np.sqrt(np.mean([y**2 for y in err_rsb_theta]))
RSME_rsb_[2] = np.sqrt(np.mean([z**2 for z in err_rsb_psi]))
RSME_rsb = np.sqrt(np.mean(RSME_rsb_**2))

print(RSME_rsb_, RSME_rsb)

RSME_smc_ = np.zeros(3)
RSME_smc_[0] = np.sqrt(np.mean([x**2 for x in err_smc_phi]))
RSME_smc_[1] = np.sqrt(np.mean([x**2 for x in err_smc_theta]))
RSME_smc_[2] = np.sqrt(np.mean([x**2 for x in err_smc_psi]))
RSME_smc = np.sqrt(np.mean(RSME_smc_**2))

print(RSME_smc_, RSME_smc)

RSME_pd_ = np.zeros(3)
# RSME_pd_[0] = np.sqrt(np.mean(err_pd_x[:len(err_smc_time)]))
# RSME_pd_[1] = np.sqrt(np.mean(err_pd_y[:len(err_smc_time)]))
# RSME_pd_[2] = np.sqrt(np.mean(err_pd_z[:len(err_smc_time)]))
# RSME_pd = np.sqrt(np.mean(RSME_pd_))


index = np.where((np.array(err_smc_time)>40) & (np.array(err_smc_time)<75))[0]
print(index.astype(int))

# fig = plt.figure()
# plt.plot(err_smc_time[index[0]:index[-1]], err_smc_y[index[0]:index[-1]], '--', c='orange', label="SMC", lw=2)
# plt.plot(err_rsb_time[index[0]:index[-1]], err_rsb_y[index[0]:index[-1]], label="RSB(proposed)", lw=2)
# plt.plot(np.array([40, 75]), np.array([0.3,0.3]), 'r-' , label='bounds', lw=0.5)
# plt.plot(np.array([40, 75]), np.array([-0.3,-0.3]), 'r-' , lw=0.5)
# # plt.plot(err_sb_time[:len(err_sb_time)], err_sb_y[:len(err_sb_time)], label="SB", lw=2)
# # plt.plot([0,err_pd_time[-1]], [0,0])
# # plt.ylabel('y_err')
# plt.title('Position error: y (m)')
# plt.axis([40, 75, -0.5, 0.5])
# plt.show()
# fig.savefig("pos_err_y_zoom.png")