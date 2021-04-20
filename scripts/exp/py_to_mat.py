import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
# from geometry_msgs.msg import PoseWithCovarianceStamped
import geometry_msgs
from tf.transformations import euler_from_quaternion



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

euler_pid_phi = []
euler_pid_theta = []
euler_pid_psi = []
for i in range(len(odom_pid_ax)):
	euler = euler_from_quaternion(np.array([odom_pid_ax[i], odom_pid_ay[i], odom_pid_az[i], odom_pid_aw[i]]))
	euler_pid_phi.append(euler[0])
	euler_pid_theta.append(euler[1])
	euler_pid_psi.append(euler[2])



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

