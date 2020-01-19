import rospy
import numpy as np
from tf.transformations import quaternion_from_euler


l = [0, 2, 3, 4]
# l.insert(1, 1)
if 1 not in l:
	print(l)