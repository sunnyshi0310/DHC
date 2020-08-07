#!/usr/bin/env python

import math
import numpy as np
from crazyflieParser import CrazyflieParser
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion as q2e
from crazyflie_driver.msg import GenericLogData
import matplotlib.pyplot as plt
import scipy.io
from numpy.linalg import matrix_rank



class mim_cf():
	def __init__(self):
		index = 1   # for cf1
		initialPosition = [0,0,0] # x,y,z coordinate for this crazyflie
		self.cfs = CrazyflieParser(index, initialPosition)
		self.cf = self.cfs.crazyflies[0]
		self.time = self.cfs.timeHelper
		self.number = 5


		self.cf.setParam("commander/enHighLevel", 1)
		self.cf.setParam("stabilizer/estimator", 2) # Use EKF
		self.cf.setParam("stabilizer/controller", 2) # Use mellinger controller
		#cf.setParam("ring/effect", 7)


		self.input = [] # Set it as cmdposition ow
		self.state = [] # unsampled X
		self.rank = [] # check the rank
		self.prediction = []
		self.new_cmd = []
		self.motor_in = []
		# 34:1.4 40:1.2 48:1.0 60:0.8 80:0.6 100:0.48 120:0.4 150:0.32
		self.k = 60 # number of time step
		self.t = 20 # number of training time
		self.m = 3 # number of inputs
		self.n = 2 # number of states
		self.hz = 20
		self.height = 0.165+0.023*3

		self.Xr = np.zeros((self.number*(self.k+20)+1,self.m))
		self.difference = np.zeros((self.k,self.n))

		self.i = 0.0
		self.reference() # set the reference


		# Phase1: for test
		self.cf.takeoff(targetHeight = self.height+0.01, duration = 3.0)
		self.time.sleep(3.0)

		while not rospy.is_shutdown():

			if self.i < self.t:
				self.cmd_u = np.array([self.Xr[self.i,0],self.Xr[self.i,1]])
				self.cf.cmdPosition(np.array([self.cmd_u[0],0,self.cmd_u[1]]),0)
				self.i = self.i + 1
				#self.rate.sleep()
				self.time.sleep(0.1)
			elif self.i < self.number*(self.k+20):
				self.update_model()
				self.controller()
				self.i = self.i+1
			elif self.i == self.number*(self.k+20):
				self.cf.land(targetHeight = 0.0, duration = 5.0)
				self.time.sleep(5.0)
				self.graph()

			self.odom_information()
			self.input.append(self.cmd_u)



	def reference(self):
		for ite in range(self.number):
			for k in range(self.k/2):
				#self.Xr[k,:] = np.array([0.5*math.sin(2*k*np.pi/self.k),1.0-0.5*math.cos(k*2*np.pi/self.k),k*2*np.pi/self.k])
				self.Xr[k+ite*(20+self.k),:] = np.array([4.8 *k/self.k,self.height,0])
			for i in range(10):
				self.Xr[i+self.k/2+ite*(20+self.k),:] = np.array([2.4,self.height,0])
			for j in range(self.k/2):
				self.Xr[j+self.k/2+10+ite*(20+self.k),:] = np.array([2.4-4.8*j/self.k,self.height,0])
			for t in range(10):
				self.Xr[ite*(20+self.k)+t+self.k+10,:] = np.array([0,self.height,0])
				#self.Xr[k,:] = np.array([0.5*math.sin(2*k*np.pi/self.k),1-0.5*math.cos(k*2*np.pi/self.k),k*2*np.pi/self.k,0.5*math.cos(2*k*np.pi/self.k),0.5*math.sin(k*2*np.pi/self.k)])

	def update_model(self):
		noise = np.random.rand(self.n+self.m,3)
		self.U = np.array(self.state)

		self.X = np.array(self.input)
		self.P = np.array(self.motor_in)
		X_MIM = self.X[self.i-(self.n+self.m+1):self.i-1,:]
		X_PMIM = self.X[self.i+1-(self.n+self.m+1):self.i,:]
		U_MIM = self.U[self.i-(self.n+self.m+1):self.i-1,:]
		print(U_MIM)
		print(noise)
		self.Omega_MIM = np.zeros((self.n+self.m,self.n+self.m))
		self.Omega_MIM[:,0:self.n] = X_MIM
		self.Omega_MIM[:,self.n:self.n+self.m] = U_MIM
		estimate_AB = np.zeros((self.n+self.m,self.n))
		for i in range(self.n):
			(a,b,c,d) = np.linalg.lstsq(self.Omega_MIM,X_PMIM[:,i],rcond = -1)
			estimate_AB[:,i] = np.matrix(a)
		AB = np.transpose(estimate_AB)
		self.rank.append(matrix_rank(AB))
		self.A = AB[:,0:self.n]
		self.B = AB[:,self.n:self.n+self.m]

	def controller(self):
		u_previous = self.X[self.i-2,:]
		u = np.zeros((1,2))
		a = np.transpose(np.matrix(self.A)*np.transpose(np.matrix(self.X[self.i-1,:]))+np.matrix(self.B)*np.transpose(np.matrix(self.Xr[self.i-1,:])))
		u[0,:] = np.array([a[0,0],a[0,1]])
		print(self.Xr[self.i,0])
		print("our 0 = "+str(u[0,0]))
		print(self.Xr[self.i,1])
		print("our 0 = "+str(u[0,1]))
		#for j in range(self.n):
		#	if (u[0,j]-self.U[self.i-1,j])>0.01:
		#		u[0,j] = self.U[self.i-1,j]+0.01
		#for j in range(self.n):
		#	if (u[0,j]-self.U[self.i-1,j])<-0.01:
		#		u[0,j] = self.U[self.i-1,j]-0.01
		for j in range(self.n):
			if (u[0,j]-self.Xr[self.i,j])>0.01:
				u[0,j] = self.Xr[self.i-1,j]+0.01
		for j in range(self.n):
			if (u[0,j]-self.Xr[self.i-1,j])<-0.01:
				u[0,j] = self.Xr[self.i-1,j]-0.01
		# Constrain the updating rate of input
		#if abs(u[0,0]-u_previous[0])>0.05:
		#	u[0,0] = u_previous[0]+np.sign(u[0,0]-u_previous[0])*0.05
		#if abs(u[0,1]-u_previous[1])>0.05:
		#	u[0,1] = u_previous[1]+np.sign(u[0,1]-u_previous[1])*0.05

		# Reference
		self.cmd_u = np.array([self.Xr[self.i,0],self.Xr[self.i,1]])

		# DHC Input
		#self.cmd_u = np.array([u[0,0],u[0,1]])


		self.cf.cmdPosition(np.array([self.cmd_u[0],0,self.cmd_u[1]]),0)
		#self.rate.sleep()
		self.new_cmd.append(u)
		self.time.sleep(0.1)

	def odom_information(self):
		translation = self.cfs.trans
		self.motor_in.append(self.cfs.mot)
		self.state.append(np.array([translation[0],translation[1],self.cfs.roll]))

	def graph(self):
		newPosition = np.array(self.new_cmd)
		scipy.io.savemat('/home/ee144-nuc12/catkin_ws/src/ee245/scripts/rank/pi.mat',mdict = {'r':self.rank})
		#scipy.io.savemat('/home/ee144-nuc12/catkin_ws/src/ee245/scripts/rank/mim_69_140_input.mat',mdict = {'X':self.X})
		scipy.io.savemat('/home/ee144-nuc12/catkin_ws/src/ee245/scripts/rank/mim_23_40_state.mat',mdict = {'U':self.U})

		plt.figure(1)
		#plt.plot(self.U[:,0],'r-',label='x(t)')
		plt.plot(self.U[:,1],'k-')
		plt.ylabel('height')
		plt.xlabel('time steps')
		plt.axis([0,self.number*(self.k+20),self.height-0.06,self.height+0.06])
		plt.legend(loc='best')
		plt.title('state evolution')

		plt.figure(2)
		#plt.plot(self.X[:,0],'r-',label='x(t)')
		plt.plot(self.X[:,1],'k-')
		plt.ylabel('height')
		plt.xlabel('time steps')
		plt.axis([0,self.number*(self.k+20),self.height-0.03,self.height+0.03])
		plt.legend(loc='best')
		plt.title('Inputs')
		plt.show()

if __name__ == '__main__':
	try:
		mim_cf()
	except rospy.ROSInterruptException:
		rospy.loginfo("Action terminated.")
