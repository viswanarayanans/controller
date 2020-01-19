import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

controller = "ASMC"
exp = 'Obs'
payload = '0.5 kg'

title_x = controller + ': ' + exp + ': ' + payload + ': X'
title_y = controller + ': ' + exp + ': ' + payload + ': Y'
title_z = controller + ': ' + exp + ': ' + payload + ': Z'
fig_x = 'obs/' + controller + '_' + exp + '_X.png'
fig_y = 'obs/' + controller + '_' + exp + '_Y.png'
fig_z = 'obs/' + controller + '_' + exp + '_Z.png'




odom_x_asmc = [0]
odom_y_asmc = [0]
odom_z_asmc = [0.1]
odom_time_asmc = [0]
odom_x_smc = [0]
odom_y_smc = [0]
odom_z_smc = [0.1]
odom_time_smc = [0]
odom_x_pid = [0]
odom_y_pid = [0]
odom_z_pid = [0.1]
odom_time_pid = [0]
odom_x_pd = [0]
odom_y_pd = [0]
odom_z_pd = [0.1]
odom_time_pd = [0]
traj_x = [0]
traj_y = [0]
traj_z = [0.1]
traj_time = [0]

with open('obs/odom_asmc.txt') as file:
	for i in file.readlines():
		k = i.split(':')

		if k[0] == 'X':
			odom_x_asmc.append(float(k[1]))
		if k[0] == 'Y':
			odom_y_asmc.append(float(k[1]))
		if k[0] == 'Z':
			odom_z_asmc.append(float(k[1]))
		if k[0] == 'secs':
			time = float(k[1])
		if k[0] == 'nsecs':
			time += float(k[1])/1000000000
			odom_time_asmc.append(time)

with open('obs/odom_smc.txt') as file:
	for i in file.readlines():
		k = i.split(':')

		if k[0] == 'X':
			odom_x_smc.append(float(k[1]))
		if k[0] == 'Y':
			odom_y_smc.append(float(k[1]))
		if k[0] == 'Z':
			odom_z_smc.append(float(k[1]))
		if k[0] == 'secs':
			time = float(k[1])
		if k[0] == 'nsecs':
			time += float(k[1])/1000000000
			odom_time_smc.append(time)

with open('obs/odom_pid.txt') as file:
	for i in file.readlines():
		k = i.split(':')

		if k[0] == 'X':
			odom_x_pid.append(float(k[1]))
		if k[0] == 'Y':
			odom_y_pid.append(float(k[1]))
		if k[0] == 'Z':
			odom_z_pid.append(float(k[1]))
		if k[0] == 'secs':
			time = float(k[1])
		if k[0] == 'nsecs':
			time += float(k[1])/1000000000
			odom_time_pid.append(time)

with open('obs/odom_pd.txt') as file:
	for i in file.readlines():
		k = i.split(':')

		if k[0] == 'X':
			odom_x_pd.append(float(k[1]))
		if k[0] == 'Y':
			odom_y_pd.append(float(k[1]))
		if k[0] == 'Z':
			odom_z_pd.append(float(k[1]))
		if k[0] == 'secs':
			time = float(k[1])
		if k[0] == 'nsecs':
			time += float(k[1])/1000000000
			odom_time_pd.append(time)

with open('obs/traj_asmc.txt') as file:
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
			traj_time.append(float(k[1]))
			traj_time.append(traj_time[-1])

print(odom_time_asmc[-1], odom_time_smc[-1], odom_time_pid[-1], odom_time_pd[-1])



# err_x_asmc = odom_x_asmc[index_traj_asmc] - traj_x 



traj_time.append(odom_time_asmc[-1])
traj_x.append(traj_x[-1])
traj_y.append(traj_y[-1])
traj_z.append(traj_z[-1])

traj_time_round = []
odom_time_asmc_round = []
odom_time_smc_round = []
odom_time_pid_round = []
odom_time_pd_round = []
odom_index_asmc = []
odom_index_smc = []
odom_index_pid = []
odom_index_pd = []


for i in range(len(odom_time_asmc)):
	odom_time_asmc_round.append(round(odom_time_asmc[i],2))

for i in range(len(odom_time_smc)):
	odom_time_smc_round.append(round(odom_time_smc[i],2))

for i in range(len(odom_time_pid)):
	odom_time_pid_round.append(round(odom_time_pid[i],2))

for i in range(len(odom_time_pd)):
	odom_time_pd_round.append(round(odom_time_pd[i],2))

for i in range(len(traj_time)):
	traj_time_round.append(round(traj_time[i],2))
# 	odom_index_asmc.append(odom_time_asmc.index(traj_time_round[-1]))
# 	odom_index_smc.append(odom_time_smc.index(traj_time_round[-1]))
# 	odom_index_pid.append(odom_time_pid.index(traj_time_round[-1]))
# 	odom_index_pd.append(odom_time_pd.index(traj_time_round[-1]))

traj_prev_x = 0
traj_prev_y = 0
traj_prev_z = 0
traj_index = 0

asmc_prev_x = 0
asmc_prev_y = 0
asmc_prev_z = 0
asmc_index = 0

smc_prev_x = 0
smc_prev_y = 0
smc_prev_z = 0
smc_index = 0

pid_prev_x = 0
pid_prev_y = 0
pid_prev_z = 0
pid_index = 0

pd_prev_x = 0
pd_prev_y = 0
pd_prev_z = 0
pd_index = 0

# print(len(traj_time))
# print(len(odom_time_asmc), len(odom_x_asmc))
# print(odom_time_asmc[-1])
# print(len(odom_time_smc))
# print(len(odom_time_pid))
# print(len(odom_time_pd))

for i in range(int(odom_time_asmc[-1]*100)):
# for i in range(10387):
	t = float(i)/100
	if t in traj_time_round:
		# print("traj", i)
		traj_index = traj_time_round.index(t)
		try:
			if traj_time_round[traj_index+1] == t:
				traj_index += 1
		except IndexError:
			print("Index not found in Trajectory", traj_index)
		traj_prev_x = traj_x[traj_index]
		traj_prev_y = traj_y[traj_index]
		traj_prev_z = traj_z[traj_index]

	if t not in traj_time_round:
		# print("traj not", i)
		# pass
		traj_time.insert(traj_index+1,t)
		traj_time_round.insert(traj_index+1,t)
		traj_x.insert(traj_index+1, traj_prev_x)
		traj_y.insert(traj_index+1, traj_prev_y)
		traj_z.insert(traj_index+1, traj_prev_z)
		traj_index += 1

	if t in odom_time_asmc_round:
		# print("odom", i)
		asmc_index = odom_time_asmc_round.index(t)
		try:
			if odom_time_asmc_round[asmc_index+1] == t:
				asmc_index += 1
		except IndexError:
			print("Index not found in Trajectory", asmc_index)
		asmc_prev_x = odom_x_asmc[asmc_index]
		asmc_prev_y = odom_y_asmc[asmc_index]
		asmc_prev_z = odom_z_asmc[asmc_index]

	if t not in odom_time_asmc_round:
		# pass
		odom_time_asmc.insert(asmc_index+1,t)
		odom_time_asmc_round.insert(asmc_index+1,t)
		odom_x_asmc.insert(asmc_index+1, asmc_prev_x)
		odom_y_asmc.insert(asmc_index+1, asmc_prev_y)
		odom_z_asmc.insert(asmc_index+1, asmc_prev_z)
		asmc_index += 1

	if t in odom_time_smc_round:
		# print("smc", i)
		smc_index = odom_time_smc_round.index(t)
		try:
			if odom_time_smc_round[smc_index+1] == t:
				smc_index += 1
		except IndexError:
			print("Index not found in Trajectory", smc_index)
		smc_prev_x = odom_x_smc[smc_index]
		smc_prev_y = odom_y_smc[smc_index]
		smc_prev_z = odom_z_smc[smc_index]

	if t not in odom_time_smc_round:
		# pass
		odom_time_smc.insert(smc_index+1,t)
		odom_time_smc_round.insert(smc_index+1,t)
		odom_x_smc.insert(smc_index+1, smc_prev_x)
		odom_y_smc.insert(smc_index+1, smc_prev_y)
		odom_z_smc.insert(smc_index+1, smc_prev_z)
		smc_index += 1

	if t in odom_time_pid_round:
		# print("pid", i)
		pid_index = odom_time_pid_round.index(t)
		try:
			if odom_time_pid_round[pid_index+1] == t:
				pid_index += 1
		except IndexError:
			print("Index not found in Trajectory", pid_index)
		pid_prev_x = odom_x_pid[pid_index]
		pid_prev_y = odom_y_pid[pid_index]
		pid_prev_z = odom_z_pid[pid_index]

	if t not in odom_time_pid_round:
		# pass
		odom_time_pid.insert(pid_index+1,t)
		odom_time_pid_round.insert(pid_index+1,t)
		odom_x_pid.insert(pid_index+1, pid_prev_x)
		odom_y_pid.insert(pid_index+1, pid_prev_y)
		odom_z_pid.insert(pid_index+1, pid_prev_z)
		pid_index += 1

	if t in odom_time_pd_round:
		# print("pd", i)
		pd_index = odom_time_pd_round.index(t)
		try:
			if odom_time_pd_round[pd_index+1] == t:
				pd_index += 1
		except IndexError:
			print("Index not found in Trajectory", pd_index)
		pd_prev_x = odom_x_pd[pd_index]
		pd_prev_y = odom_y_pd[pd_index]
		pd_prev_z = odom_z_pd[pd_index]

	if t not in odom_time_pd_round:
		# pass
		odom_time_pd.insert(pd_index+1,t)
		odom_time_pd_round.insert(pd_index+1,t)
		odom_x_pd.insert(pd_index+1, pd_prev_x)
		odom_y_pd.insert(pd_index+1, pd_prev_y)
		odom_z_pd.insert(pd_index+1, pd_prev_z)
		pd_index += 1





time_max_traj = traj_time.index(odom_time_asmc[-1])
time_max_smc = odom_time_smc.index(odom_time_asmc[-1])
time_max_pid = odom_time_pid.index(odom_time_asmc[-1])
time_max_pd = odom_time_pd.index(odom_time_asmc[-1])


print(len(traj_time[:time_max_traj]))
print(len(odom_time_asmc), len(odom_x_asmc))
# print(len(odom_time_smc[:time_max_smc]))
# print(len(odom_time_pid[:time_max_pid]))
# print(len(odom_time_pd[:time_max_pd]))

traj_x_ = []
traj_y_ = []
traj_z_ = []

print(traj_time_round[3000:3020])



# print(len(traj_x_))
# err_x_asmc = np.array(odom_x_asmc) - np.array(traj_x[:time_max_traj])
# print(err_x_asmc)


# plt.plot(traj_time_round[:5000], traj_x[:5000])

			
		# traj_time_x.append(float(i)/100)
	
	

# print(traj_time_round)

# print(traj_time_round.index(93.17), traj_time_round.index(103.87))

# odom_index_asmc = traj_time_round.index(odom_time_asmc)

# print(odom_time_asmc_round.index(traj_time_round[:10]))
# index_traj = odom_time_asmc_round.index(traj_time_round)
# print(index_traj)

# fig = plt.figure(figsize=(16,6))
# plt.plot(odom_time_asmc, odom_x_asmc, label='ASMC trajectory')
# plt.plot(odom_time_smc[:time_max_smc], odom_x_smc[:time_max_smc], label='SMC trajectory')
# plt.plot(odom_time_pid[:time_max_pid], odom_x_pid[:time_max_pid], label='PID trajectory')
# plt.plot(odom_time_pd[:time_max_pd], odom_x_pd[:time_max_pd], label='PD trajectory')
# plt.plot(traj_time, traj_x, label='Desired trajectory')
# plt.xlabel("Time in s")
# plt.ylabel("X")
# plt.legend()
# plt.title(title_x)
# # plt.show()
# fig.savefig(fig_x)

# fig = plt.figure(figsize=(16,6))
# plt.plot(odom_time_asmc, odom_y_asmc, label='ASMC trajectory')
# plt.plot(odom_time_smc[:time_max_smc], odom_y_smc[:time_max_smc], label='SMC trajectory')
# plt.plot(odom_time_pid[:time_max_pid], odom_y_pid[:time_max_pid], label='PID trajectory')
# plt.plot(odom_time_pd[:time_max_pd], odom_y_pd[:time_max_pd], label='PD trajectory')
# plt.plot(traj_time, traj_y, label='Desired trajectory')
# plt.title(title_y)
# plt.legend()
# # plt.show()
# fig.savefig(fig_y)

# fig = plt.figure(figsize=(16,6))
# plt.plot(odom_time_asmc, odom_z_asmc, label='ASMC trajectory')
# plt.plot(odom_time_smc[:time_max_smc], odom_z_smc[:time_max_smc], label='SMC trajectory')
# plt.plot(odom_time_pid[:time_max_pid], odom_z_pid[:time_max_pid], label='PID trajectory')
# plt.plot(odom_time_pd[:time_max_pd], odom_z_pd[:time_max_pd], label='PD trajectory')
# plt.plot(traj_time, traj_z, label='Desired trajectory')
# plt.title(title_z)
# plt.legend()
# # plt.show()
# fig.savefig(fig_z)
