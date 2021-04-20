import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
# from geometry_msgs.msg import PoseWithCovarianceStamped
import geometry_msgs


def load_odom_data(file_name):
	odom_x = [0]
	odom_y = [0]
	odom_z = [0.1]
	odom_time = [0]
	with open(file_name) as file:
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
	odom = [odom_x, odom_y, odom_z, odom_time]
	return odom

def load_traj_data(file_name):
	traj_x = [0]
	traj_y = [0]
	traj_z = [0.1]
	traj_time = [0]
	with open(file_name) as file:
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
				time = float(k[1])
			if k[0] == 'nsecs':
				time += float(k[1])/1000000000
				traj_time.append(time)
				traj_time.append(traj_time[-1])
	traj = [traj_x, traj_y, traj_z, traj_time]
	return traj

def load_err_data(file_name):
	err_x = [0]
	err_y = [0]
	err_z = [0]
	err_phi = [0]
	err_theta = [0]
	err_psi = [0]
	err_time = [0]
	with open(file_name) as file:
		for i in file.readlines():
			k = i.split(':')

			if k[0] == 'Err_X':
			# err_asmc_x.append(traj_x[-1])
				err_x.append(float(k[1]))
			if k[0] == 'Err_Y':
			# err_asmc_y.append(traj_y[-1])
				err_y.append(float(k[1]))
			if k[0] == 'Err_Z':
			# err_asmc_z.append(traj_z[-1])
				err_z.append(float(k[1]))
			if k[0] == 'Err_phi':
			# err_asmc_x.append(traj_x[-1])
				err_phi.append(float(k[1])*180/np.pi)
			if k[0] == 'Err_theta':
			# err_asmc_y.append(traj_y[-1])
				err_theta.append(float(k[1])*180/np.pi)
			if k[0] == 'Err_psi':
			# err_asmc_z.append(traj_z[-1])
				err_psi.append(float(k[1])*180/np.pi)
			if k[0] == 'secs':
			# print("time")
				time = float(k[1])
			if k[0] == 'nsecs':
				time += float(k[1])/1000000000
				err_time.append(time)
	err = [err_x, err_y, err_z, err_phi, err_theta, err_psi, err_time]
	return err


def plot_data(type,title,amp,time,labels,colors,shapes,line_width):
	if type==1:
		time_limit = min(time)
	elif type==2:
		time_limit = min(time[1:])
	else:
		print("Number of inputs should be 3 or 4")
		return -1

	for i in range(len(time)):
		plt.plot(time[i], amp[i], shapes[i], c=colors[i], label=labels[i], lw=line_width[i])
	plt.title(title)

def print_rmse(error):
	RSME = np.zeros(3)
	for i in range(3):
		RSME[i] = np.sqrt(np.mean([x**2 for x in error[i]]))
	RSME_ = np.sqrt(np.mean(RSME**2))
	print(RSME, RSME_)


def main():
	odom_pid = load_odom_data('odom_pid.txt')
	odom_smc = load_odom_data('odom_smc.txt')
	odom_sac = load_odom_data('odom_sac.txt')
	traj = load_traj_data('traj_sac.txt')
	err_pid = load_err_data('error_pid.txt')
	err_smc = load_err_data('error_smc.txt')
	err_sac = load_err_data('error_sac.txt')
	traj[3].append(odom_pid[3][-1])
	traj[0].append(traj[0][-1])
	traj[1].append(traj[1][-1])
	traj[2].append(traj[2][-1])

	track = [traj, odom_pid, odom_smc, odom_sac]
	shapes = ['-','--','-.',':']
	labels = ['Desired','PID','SMC','SAC']
	colors = ['b', 'r', 'orange', 'g']
	line_width = [2,2,2,3]
	order = ['X','Y','Z','Psi','Theta','Phi']
	time_limit = 131
	upper_limit = [4,4,3]
	lower_limit = [-3.5,-3,-0.2]


	# fig = plt.figure(figsize=(9,6))
	# for i in range(3):
	# 	plt.subplot('31'+str(i+1))
	# 	for j in range(len(track)):
	# 		plt.plot(track[j][3],track[j][i],shapes[j],c=colors[j],label=labels[j],lw=line_width[j])
	# 	plt.axis([0, time_limit, lower_limit[i], upper_limit[i]])
	# 	plt.legend(loc=2, ncol=4)
	# 	plt.title('Position tracking: ' + order[i] + '(m)')
	# plt.xlabel('time (s)')
	# plt.tight_layout()
	# plt.show()
	# fig.savefig("ring_traj.png")

	error = [err_pid, err_smc, err_sac]
	# lower_limit = [-3,-2.5,-2.1]
	# upper_limit = [1.5,4,1.5]
	# fig = plt.figure(figsize=(9,6))
	# for i in range(3):
	# 	plt.subplot('31'+str(i+1))
	# 	for j in range(len(error)):
	# 		plt.plot(error[j][-1],error[j][i],shapes[j+1],c=colors[j+1],label=labels[j+1],lw=line_width[j+1])
	# 	plt.axis([0, time_limit, lower_limit[i], upper_limit[i]])
	# 	plt.legend(loc=2, ncol=4)
	# 	plt.title('Position error: ' + order[i] + '(m)')
	# plt.xlabel('time (s)')
	# plt.tight_layout()
	# plt.show()
	# fig.savefig("ring_error_pos.png")

	lower_limit = [-40,-30,-50]
	upper_limit = [40,40,50]

	fig = plt.figure(figsize=(9,6))
	for i in range(3):
		plt.subplot('31'+str(i+1))
		for j in range(len(error)):
			plt.plot(error[j][-1],error[j][i+3],shapes[j+1],c=colors[j+1],label=labels[j+1],lw=line_width[j+1])
		plt.axis([0, time_limit, lower_limit[i], upper_limit[i]])
		plt.legend(loc=2, ncol=4)
		plt.title('Attitude error: ' + order[i+3] + '(deg)')
	plt.xlabel('time (s)')
	plt.tight_layout()
	plt.show()
	fig.savefig("ring_error_att.png")

	print_rmse(err_pid[:3])
	print_rmse(err_smc[:3])
	print_rmse(err_sac[:3])
	print("")
	print_rmse(err_pid[3:6])
	print_rmse(err_smc[3:6])
	print_rmse(err_sac[3:6])


if __name__ == "__main__":
	main()


# # falling_index = err_smc_time.index(39)
# # falling_index = 1500
# # print(max(err_pd_psi))




# # fig = plt.figure(figsize=(6,12))
# fig, axes = plt.subplots(nrows=3, ncols=1, figsize=(16,8))
# plt.subplot(311)
# plt.plot(traj_time, traj_x, label='Desired trajectory', lw=2)
# plt.plot(odom_pd_time[:len(odom_asmc_time)], odom_pd_x[:len(odom_asmc_time)], label='PD', lw=2)
# plt.plot(odom_smc_time[:len(odom_asmc_time)], odom_smc_x[:len(odom_asmc_time)], label="SMC", lw=2)
# plt.plot(odom_asmc_time[:len(odom_asmc_time)], odom_asmc_x[:len(odom_asmc_time)], label="ASMC(proposed)", lw=2)
# plt.axis([0, odom_asmc_time[-1], -5, 15])
# # plt.ylabel('x')
# plt.title('Position tracking: x (m)')
# plt.legend(loc=2, prop={'size': 8}, ncol=4)
# # plt.show()
# # fig.savefig('traj_x.png')
# plt.subplot(312)
# plt.plot(traj_time, traj_y, label='Desired trajectory', lw=2)
# plt.plot(odom_pd_time[:len(odom_asmc_time)], odom_pd_y[:len(odom_asmc_time)], label='PD', lw=2)
# plt.plot(odom_smc_time[:len(odom_asmc_time)], odom_smc_y[:len(odom_asmc_time)], label="SMC", lw=2)
# plt.plot(odom_asmc_time[:len(odom_asmc_time)], odom_asmc_y[:len(odom_asmc_time)], label="ASMC(proposed)", lw=2)
# plt.axis([0, odom_asmc_time[-1], -10, 10])
# plt.title('Position tracking: y (m)')
# # plt.ylabel('y')
# plt.legend(loc=2, prop={'size': 8}, ncol=4)
# # fig.savefig('traj_y.png')

# plt.subplot(313)
# plt.plot(traj_time, traj_z, label='Desired trajectory', lw=2)
# plt.plot(odom_pd_time[:len(odom_asmc_time)], odom_pd_z[:len(odom_asmc_time)], label='PD', lw=2)
# plt.plot(odom_smc_time[:len(odom_asmc_time)], odom_smc_z[:len(odom_asmc_time)], label="SMC", lw=2)
# plt.plot(odom_asmc_time, odom_asmc_z, label="ASMC(proposed)", lw=2)
# # plt.ylabel('z')
# plt.axis([0, odom_asmc_time[-1], 0, 5])
# plt.legend(loc=2, prop={'size': 8}, ncol=4)
# # plt.xlabel('time')
# plt.title('Position tracking: z (m)')
# plt.xlabel('time (s)')


# fig.tight_layout()
# plt.show()
# fig.savefig('traj_exp2.png')


# # fig = plt.figure(figsize=(6,12))
# fig, axes = plt.subplots(nrows=3, ncols=1, figsize=(16,8))
# plt.subplot(311)
# plt.plot(err_pd_time[:len(err_asmc_time)], err_pd_x[:len(err_asmc_time)], label='PD', lw=2)
# plt.plot(err_smc_time[:len(err_asmc_time)], err_smc_x[:len(err_asmc_time)], label="SMC", lw=2)
# plt.plot(err_asmc_time[:len(err_asmc_time)], err_asmc_x[:len(err_asmc_time)], label="ASMC(proposed)", lw=2)
# # plt.plot([0,err_pd_time[-1]], [0,0])
# # plt.ylabel('x_err')
# plt.title('Position error: x (m)')
# plt.axis([0, err_asmc_time[-1], -8, 12])
# plt.legend(loc=2, prop={'size': 10}, ncol=4)
# # plt.show()
# # fig.savefig('err_x.png')


# plt.subplot(312)
# plt.plot(err_pd_time[:len(err_asmc_time)], err_pd_y[:len(err_asmc_time)], label='PD', lw=2)
# plt.plot(err_smc_time[:len(err_asmc_time)], err_smc_y[:len(err_asmc_time)], label="SMC", lw=2)
# plt.plot(err_asmc_time[:len(err_asmc_time)], err_asmc_y[:len(err_asmc_time)], label="ASMC(proposed)", lw=2)
# # plt.plot([0,err_pd_time[-1]], [0,0])
# # plt.ylabel('y_err')
# plt.title('Position error: y (m)')
# plt.axis([0, err_asmc_time[-1], -15, 5])
# plt.legend(loc=3, prop={'size': 10}, ncol=4)
# # plt.show()
# # fig.savefig('err_y.png')


# plt.subplot(313)
# plt.plot(err_pd_time[:len(err_asmc_time)], err_pd_z[:len(err_asmc_time)], label='PD', lw=2)
# plt.plot(err_smc_time[:len(err_asmc_time)], err_smc_z[:len(err_asmc_time)], label="SMC", lw=2)
# plt.plot(err_asmc_time[:len(err_asmc_time)], err_asmc_z[:len(err_asmc_time)], label="ASMC(proposed)", lw=2)
# # plt.plot([0,err_pd_time[-1]], [0,0])
# # plt.ylabel('z_err')
# plt.title('Position error: z (m)')
# plt.xlabel('time (s)')
# plt.axis([0, err_asmc_time[-1], -4, 2])
# plt.legend(loc=3, prop={'size': 10}, ncol=4)

# fig.tight_layout(h_pad=0.4)
# plt.show()
# fig.savefig('pre_error_exp2.png')


# # fig = plt.figure(figsize=(6,12))
# fig, axes = plt.subplots(nrows=3, ncols=1, figsize=(16,8))
# plt.subplot(311)
# plt.plot(err_pd_time[:len(err_asmc_time)], err_pd_phi[:len(err_asmc_time)], label='PD', lw=2)
# plt.plot(err_smc_time[:len(err_asmc_time)], err_smc_phi[:len(err_asmc_time)], label="SMC", lw=2)
# plt.plot(err_asmc_time[:len(err_asmc_time)], err_asmc_phi[:len(err_asmc_time)], label="ASMC(proposed)", lw=2)
# # plt.plot([0,err_pd_time[-1]], [0,0])
# # plt.ylabel('x_err')
# plt.title('Attitude error: $\phi$ (deg)')
# plt.axis([0, err_asmc_time[-1], -60, 60])
# plt.legend(loc=3, prop={'size': 10}, ncol=4)
# # plt.show()
# # fig.savefig('err_x.png')


# plt.subplot(312)
# plt.plot(err_pd_time[:len(err_asmc_time)], err_pd_theta[:len(err_asmc_time)], label='PD', lw=2)
# plt.plot(err_smc_time[:len(err_asmc_time)], err_smc_theta[:len(err_asmc_time)], label="SMC", lw=2)
# plt.plot(err_asmc_time[:len(err_asmc_time)], err_asmc_theta[:len(err_asmc_time)], label="ASMC(proposed)", lw=2)
# # plt.plot([0,err_pd_time[-1]], [0,0])
# # plt.ylabel('y_err')
# plt.title('Attitude error: $\\theta$ (deg)')
# plt.axis([0, err_asmc_time[-1], -60, 60])
# plt.legend(loc=2, prop={'size': 10}, ncol=4)
# # plt.show()
# # fig.savefig('err_y.png')


# plt.subplot(313)
# plt.plot(err_pd_time[:len(err_asmc_time)], err_pd_psi[:len(err_asmc_time)], label='PD', lw=2)
# plt.plot(err_smc_time[:len(err_asmc_time)], err_smc_psi[:len(err_asmc_time)], label="SMC", lw=2)
# plt.plot(err_asmc_time[:len(err_asmc_time)], err_asmc_psi[:len(err_asmc_time)], label="ASMC(proposed)", lw=2)
# # plt.plot([0,err_pd_time[-1]], [0,0])
# # plt.ylabel('z_err')
# plt.title('Attitude error: $\psi$ (deg)')
# plt.xlabel('time (s)')
# plt.axis([0, err_asmc_time[-1], -60, 40])
# plt.legend(loc=2, prop={'size': 10}, ncol=4)

# fig.tight_layout(h_pad=0.4)
# plt.show()
# fig.savefig('error_att_exp2.png')
