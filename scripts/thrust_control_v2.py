#!/usr/bin/env python

import numpy as np
import math
import tf
from scipy.integrate import odeint
from geometry_msgs.msg import Point, PoseStamped
import rospy
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from nav_msgs.msg import *
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tf.transformations import *

Phi_p = np.zeros((3, 3)) #postive gain matrix
Lamd_p = np.zeros((3, 3))

# np.fill_diagonal(Phi_p, [1.5, 1.5, 2.2])
# np.fill_diagonal(Lamd_p, [1.5, 1.5, 0.6])

np.fill_diagonal(Phi_p, [1.5, 1.5, 2])
np.fill_diagonal(Lamd_p, [1.5, 1.5, 0.5])

g = 9.8 #acceleration due to gravity 
Kp0, Kp1, m_est = 0.01, 0.01, 0.01

Kpa, Kda, Kia = 1.1, 4, 0.1

Kq0, Kq1, Kq2 = 0.001, 0.001, 0.001

Lamd_q = np.zeros((3, 3))
np.fill_diagonal(Lamd_q, [0.25, 0.25, 0.5])

Phi_q = np.zeros((3, 3)) #postive gain matrix
np.fill_diagonal(Phi_q, [0.35, 0.35, 0.35])

class fcuModes:
    def __init__(self):
        pass

    def setTakeoff(self):
    	rospy.wait_for_service('mavros/cmd/takeoff')
    	try:
    		takeoffService = rospy.ServiceProxy('mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
    		takeoffService(altitude = 3)
    	except rospy.ServiceException, e:
    		print "Service takeoff call failed: %s"%e

    def setArm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException, e:
            print "Service arming call failed: %s"%e

    def setDisarm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(False)
        except rospy.ServiceException, e:
            print "Service disarming call failed: %s"%e

    def setStabilizedMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='STABILIZED')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Stabilized Mode could not be set."%e

    def setOffboardMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Offboard Mode could not be set."%e

    def setAltitudeMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='ALTCTL')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Altitude Mode could not be set."%e

    def setPositionMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='POSCTL')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Position Mode could not be set."%e

    def setAutoLandMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='AUTO.LAND')
        except rospy.ServiceException, e:
               print "service set_mode call failed: %s. Autoland Mode could not be set."%e


class Controller:
    # initialization method
    def __init__(self):
        self.sp = PositionTarget()
        self.sp.type_mask = int('010111111000', 2)
        # LOCAL_NED
        self.sp.coordinate_frame = 1
        self.sp.position.x = 0.0
        self.sp.position.y = 0.0
        self.sp.position.z = 4
    	self.state = State()

    	self.x, self.y, self.z = 0, 0, 0
    	self.xd, self.yd, self.zd = 5, 5, 5

    	self.vx, self.vy, self.vz = 0, 0, 0
    	self.vxd, self.vyd, self.vzd = 0, 0, 0

    	self.q0, self.q1, self.q2, self.q3 = 0, 0, 0, -1 
    	self.wx, self.wy, self.wz = 0,0,0

    	self.roll, self.pitch, self.yaw = 0,0,0 #current roll, pitch, yaw
        self.rd, self.pd, self.yawd = 0, 0, 0 #desired rpy

        self.qd_dot = np.array([0, 0, 0]) #desired angular acceleration

    def stateCb(self, msg):
        self.state = msg

    def updateSp(self):
        self.sp.position.x = self.x
        self.sp.position.y = self.y

    def odomCb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z

        self.q0 = msg.pose.pose.orientation.w
    	self.q1 = msg.pose.pose.orientation.x
    	self.q2 = msg.pose.pose.orientation.y
    	self.q3 = msg.pose.pose.orientation.z

        self.vx = msg.twist.twist.linear.x
        self.vy = msg.twist.twist.linear.y
        self.vz = msg.twist.twist.linear.z

        self.wx = msg.twist.twist.angular.x
        self.wy = msg.twist.twist.angular.y
        self.wz = msg.twist.twist.angular.z

        orientation_list = [self.q1, self.q2, self.q3, self.q0]
    	(self.roll, self.pitch, self.yaw) = euler_from_quaternion(orientation_list)

    def getTime(self, time):
        self.time = time

    def thrust(self):
     	ep_dot = np.array([self.vx - self.vxd, self.vy - self.vyd, self.vz - self.vzd])
     	ep = np.array([self.x - self.xd, self.y - self.yd, self.z -self.zd])
     	self.svp = ep_dot + Phi_p.dot(ep)

     	self.zeta_p = np.array([ep, ep_dot])
     	zeta_norm_p = (np.linalg.norm(self.zeta_p))

        Kp0 = self.sol_Kp(self.time,0.01,1)
        Kp1 = self.sol_Kp(self.time,0.01,2) 

     	Rho_p = Kp0 + Kp1*(zeta_norm_p)
     	self.gp = np.array([0,0,g])

        m_est = self.sol_m(self.time, 0.01)
     	self.tou_p = - Lamd_p.dot(self.svp) - Rho_p*np.sign(self.svp) + m_est*self.gp

        R, R_inv ,R_transp = self.GRotationMatricn(self.roll, self.pitch, self.yaw)
        self.tou_p = R_inv.dot(self.tou_p)
        self.tou_p = np.around(self.tou_p, decimals = 5)
        

    def desired_xy_acceleeration(self): #PID for x and y error
        # Cxi = Cxi + (self.xd - self.x)
        # Cyi = Cyi + (self.yd - self.y)

        xd_acc = Kpa*(self.xd - self.x) + Kda*(self.vxd - self.vx) #+ Kia*(Cxi)
        yd_acc = Kpa*(self.yd - self.y) + Kda*(self.vyd - self.vy) #+ Kia*(Cyi)

        return xd_acc, yd_acc

    def desired_rp(self): #desired roll and pitch
        # print("desired_rp")
        xd_acc, yd_acc = self.desired_xy_acceleeration()
        self.rd = (1/g)* (xd_acc * np.sin(self.yawd) - yd_acc*np.cos(self.yawd))
        self.pd = (1/g)* (xd_acc * np.cos(self.yawd) + yd_acc*np.sin(self.yawd))

    def rpyRate(self):
        self.desired_rp()
        R, R_inv ,R_transp = self.GRotationMatricn(self.roll, self.pitch, self.yaw)
        Rd, Rd_inv ,Rd_transp = self.GRotationMatricn(self.rd, self.pd, self.yawd)

        eq = np.array([self.rd - self.roll, self.pd - self.pitch, self.yawd - self.yaw])
        eq = (eq + math.pi) % (2 * math.pi) - math.pi
        eq_dot = - np.array([self.wx, self.wy, self.wz]) + (Rd_transp.dot(R)).dot(self.qd_dot)
        self.svq = eq_dot + Phi_q.dot(eq)

        self.zeta_q = np.array([eq, eq_dot])
        zeta_norm_q = (np.linalg.norm(self.zeta_q))    

        Kq0 = self.sol_Kq(self.time, 0.001, 1)
        Kq1 = self.sol_Kq(self.time, 0.001, 2)
        Kq2 = self.sol_Kq(self.time, 0.001, 3)
        # print(Kq0, Kq1, Kq2)
        
        Rho_q = Kq0 + Kq1*(zeta_norm_q) + Kq2*(zeta_norm_q)*(zeta_norm_q)
        self.tou_q = - Lamd_q.dot(self.svq) - Rho_q*np.sign(self.svq)     

    def GRotationMatricn(self, roll, pitch, yaw): #R_W_B current roll pitvh yaw
        Rhomo = euler_matrix(roll, pitch, yaw)
        R = Rhomo[:-1,:-1]
        Rinv = np.linalg.inv(R)
        Rtransp = np.transpose(R)
        return R, Rinv, Rtransp

    def sol_m(self, t, y0):
        y0 = 1
        alpham = 50
        b = -(self.gp).dot(self.svp)
        # print(b)
        a = alpham
        c = y0 - (b/a)
        y = (b/a) + c*np.exp(-a*t)
        # print(b, b/a)
        return y

    def sol_Kp(self,t,y0,k): #k=1,2 KP0 and KP1
        alphap = 1
        a = alphap
        b = np.linalg.norm(self.svp)*np.power(np.linalg.norm(self.zeta_p),k)
        c = y0 - (b/a)
        y = (b/a) + c*np.exp(-a*t)
        return y

    def sol_Kq(self,t,y0,k): #k=1,2,3  KQ0, KQ1, KQ2
        alphaq = 10
        a = alphaq
        b = np.linalg.norm(self.svq)*np.power(np.linalg.norm(self.zeta_q),k)
        c = y0 - (b/a)
        y = (b/a) + c*np.exp(-a*t)
        return y


def main():
    rospy.init_node('setpoint_node', anonymous=True)
    movement_cmd = AttitudeTarget()
    
    modes = fcuModes() # flight mode object
    cnt = Controller()
    rate = rospy.Rate(50.0)

    rospy.Subscriber('mavros/state', State, cnt.stateCb)
    rospy.Subscriber('mavros/local_position/odom', Odometry, cnt.odomCb)

    thrust_pub = rospy.Publisher('mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)   
    sp_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)

    # Make sure the drone is armed
    while not cnt.state.armed:
        modes.setArm()
        rate.sleep()

    k=0
    while k < 10:
        sp_pub.publish(cnt.sp)
        rate.sleep()
        k = k + 1
    modes.setOffboardMode() # activate OFFBOARD mode

    # ROS main loop
    now1 = rospy.Time.now()
    time1 = now1.secs + now1.nsecs/1e9
    
    while not rospy.is_shutdown():

        now = rospy.Time.now()
        time = (now.secs + now.nsecs/1e9) - time1
        cnt.getTime(time)

    	cnt.thrust()
        cnt.rpyRate()
        movement_cmd.body_rate.x = -cnt.tou_q[0]
        movement_cmd.body_rate.y = -cnt.tou_q[1]
        movement_cmd.body_rate.z = -cnt.tou_q[2]

        #averaging the 10 values
        num_array = [0,0,0,0,0,0,0] #take 10 elements
        num_array.append(num_array.pop(0)) 
        num_array[-1]=cnt.tou_p[2]
        Avg = sum(num_array)/np.size(num_array)

    	# movement_cmd.thrust = cnt.tou_p[2]
        movement_cmd.thrust = Avg
        print(cnt.tou_p)

    	thrust_pub.publish(movement_cmd)
        # rate.sleep()


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass

