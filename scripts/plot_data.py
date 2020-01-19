import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


x_value = []
y_value = []
z_value = []


with open("../bag/data.txt","r") as  file:
	for i in file.readlines():
		k = i.split(':')
		if k[0] == 'x':
			x_value.append(float(k[1]))
		if k[0] == 'y':
			y_value.append(float(k[1]))
		if k[0] == 'z':
			z_value.append(float(k[1]))


plt.plot(x_value)
plt.show()

plt.plot(y_value)
plt.show()

plt.plot(z_value)
plt.show()