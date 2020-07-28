#!/usr/bin/env python
'''
We will see

'''

import rospy
from std_msgs.msg import Int64, Float64MultiArray, Float64, Float32MultiArray, Bool

from PID_class import *
from simple_pid import PID

# from camera_radar_msg.msg import fused_data
# from camera_radar_msg.msg import fused_data_
import sys, time

class Controls(object):


	def __init__(self):
		#Initialize ROS node
		rospy.init_node('test_controls', anonymous=True)

		#Steering
		self.BodyAngle = 0.0
		self.RadianAngle = 0.0
		self.PrevRadianAngle = 1.0
		self.RadianFactor = 57.3248
		self.MotorToBodyFactor = 36
		
		#Motor Speed 
		self.AccelMotorSpeed  = 1
		self.BrakeMotorSpeed = 2
		self.SteerMotorSpeed = 1
		
		#Speed related 
		self.ObjectPresent = False
		self.ReferenceSpeed = 0
		self.BrakeAngle = 0.0
		self.CurrentVelocity = 0.0
		self.EmergencyBrakeMotorSpeed = 0.05
		self.EmergencyBrakeAngle = 129
		# self.STOPBoardDistance = 15.0

		#Limiting Variables
		self.AccelMax = 100
		self.BrakeMax = 100

		self.AccelAngle = 1
		self.ThrottlePos = 1
		
		self.Brake = Float64MultiArray()
		self.Accel = Float64MultiArray()
		self.Steer = Float64MultiArray()
		self.BrakeEmergency = Float64MultiArray()

		self.FBSteeringAngle = 0
		self.CurrentSteeringAngle = 0
		self.i =0
		self.CurrentSteeringAngleBuf = []
		for self.i in range(0,100):
			self.CurrentSteeringAngleBuf.append(0)
		self.i = 0

		#Accel PID Parameters
		self.AccelKp = 13.0
		self.AccelKi = 0.1
		self.AccelKd = 1.5
		self.Accelsetpoint = self.ReferenceSpeed
		self.AccelPID = PID(Kp = self.AccelKp, Ki = self.AccelKi, Kd = self.AccelKd, setpoint = self.Accelsetpoint)
		self.AccelPID.output_limits = (0, 50)

		#Brake PID Parameters
		self.BrakeKp = -19.5 
		self.BrakeKi = -0.6
		self.BrakeKd = -5.0
		self.Brakesetpoint = self.ReferenceSpeed
		self.BrakePID = PID(Kp = self.BrakeKp, Ki = self.BrakeKi, Kd = self.BrakeKd, setpoint = self.Brakesetpoint)
		self.BrakePID.output_limits = (0, self.BrakeMax)

		if not rospy.is_shutdown():
			#Subscribers
			# self.OffsetValue = rospy.Subscriber("Vision/Lane_Offset", Int64, self.cal_angle)
			rospy.Subscriber("Input/Inputs", Float32MultiArray, self.get_data , queue_size=1)
			rospy.Subscriber("Output/Feedback", Float32MultiArray, self.get_FBdata , queue_size=1)
			# rospy.Subscriber("/fused_data", fused_data_, self.update_dist)
			rospy.Subscriber("/manual_speed", Float64, self.call_manualSPeed)
			# rospy.Subscriber("/object_dis", Float64, self.get_obj_dis)
			# rospy.Subscriber("/speedCommand", Float64, self.get_MPCspeed, queue_size=1)
			# rospy.Subscriber("/steerCommand", Float64, self.get_MPCsteer, queue_size=1)
			# rospy.Subscriber("/planning/gideon/velocityToFollow", Float64, self.get_vel, queue_size=1)
			# rospy.Subscriber("/planning/gideon/emergencyFlag", Bool, self.get_emergency, queue_size=1)
			#rospy.Subscriber("/Controls/debug", , self.get_emergency, queue_size=1)

			#Publisher
			self.SteerPub = rospy.Publisher('/Output/Steering_Angle', Float64MultiArray, queue_size=1)   # For steering motor
			self.BrakePub = rospy.Publisher('/Output/Brake_Angle', Float64MultiArray, queue_size=1)       # For brake motor
			self.AccelPub = rospy.Publisher('/Output/Speed_Angle', Float64MultiArray, queue_size=1)   # For accelerator motor
			self.SpeedPub = rospy.Publisher('/OBD', Float64, queue_size=1)   # For accelerator motor


	def call_manualSPeed(self, data):
		self.ReferenceSpeed = data.data
		print("manual_speed",self.ReferenceSpeed)

	def get_vel(self, msg):

		# self.ObstaclePresent = msg.data[0]
		# # self.RadianAngle = msg.data[1]
		# self.StopBoardPresent = msg.data[1]
		if (self.ObjectPresent == False):
			self.ReferenceSpeed = (msg.data * 3.6) #Data is converted from m/s to km/h
			#print("Velocity Status:",self.ReferenceSpeed, self.CurrentVelocity)
		# self.StopBoardPresentdDistance = msg.data[3]

	def get_FBdata(self,msg):
		self.FBSteeringAngle = msg.data[5]

	def get_MPCspeed(self, msg):
		if (self.ObjectPresent == False):
			self.ReferenceSpeed = (msg.data) #Data is converted from m/s to km/h
		#	print("Velocity Status:",self.ReferenceSpeed, self.CurrentVelocity)
		else:
			self.ReferenceSpeed = 0;
	def get_MPCsteer(self, msg):
		self.RadianAngle = msg.data

	def get_emergency(self, msg):
		# self.ReferenceSpeed = 0
		self.ObjectPresent = msg.data
		if self.ObjectPresent == True:
			self.ReferenceSpeed = 0

	def get_data(self, data):
		self.CurrentVelocity = data.data[12]
		self.ThrottlePos = data.data[11]
		self.BrakePot = data.data[4]
		self.OBD = Float64()
		self.OBD.data = self.CurrentVelocity / 3.6
		self.SpeedPub.publish(self.OBD)

	def EmergencyBraking(self):
		self.Accel.data = [1, 1]
		self.AccelPub.publish(self.Accel)
		time.sleep(0.05)
		
		print("Obstacle Present, Emergency Braking ! ! !")
		self.BrakeEmergency.data = [self.EmergencyBrakeAngle, self.EmergencyBrakeMotorSpeed]
		self.BrakePub.publish(self.BrakeEmergency)
		time.sleep(0.05)
		

	def cruizer(self):

		'''	
		
							if (obstacle_dis/StopSign)
									|
					-----------------------------------------------------------------
					|                                                               |
				YES |                                                            NO |
					|                                                               |
					|                                                               |
					V                                                               |  
		-------------------------------------------                                 |
		|                       |                  |                                |
		| obstacle_dis          | obstacle_dis     |                                |
		|  < 4                  |    4 - 20        |                                |
		V                       V                  V                                | 
	Emergency braking     Soft braking         Do Nothing                           | 
							  PID                                                   |
																					V
														------------------------------------------------
														|                           |                   |
														|                           |                   |
														|                           |                   |
												current speed <               current speed ==        current speed > refernce speed +2     
												reference speed - 2             reference speed +- 2    |
														|                           |                   |
														|                           |                   |
														V                           V                   V
													accelerate & Release brake    do nothing     brake & deaccelrate
															PID
		'''
		print(self.ObjectPresent,self.CurrentVelocity,self.ReferenceSpeed,self.AccelAngle,self.AccelMotorSpeed,self.BrakeAngle,self.BrakeMotorSpeed)
		if(self.ObjectPresent):
			print("Object Detected")
			self.EmergencyBraking()
			
		else:
			if(self.ReferenceSpeed >= 100):
				
				self.AccelPID.auto_mode = False
				self.BrakePID.auto_mode = False

				self.BrakeAngle = 1
				self.BrakeMotorSpeed = 1
				self.Brake.data = [self.BrakeAngle, self.BrakeMotorSpeed]
				self.BrakePub.publish(self.Brake)
				time.sleep(0.05)
				
				self.AccelAngle = 1
				self.AccelMotorSpeed = 1
				self.Accel.data = [self.AccelAngle, self.AccelMotorSpeed]
				self.AccelPub.publish(self.Accel)
				time.sleep(0.05)
				return 0
	
			elif(self.ReferenceSpeed <= 0):
				self.BrakeAngle = self.BrakeMax
				self.BrakeMotorSpeed = 1
				self.Brake.data = [self.BrakeAngle, self.BrakeMotorSpeed]
				self.BrakePub.publish(self.Brake)
				time.sleep(0.05)
				self.AccelAngle = 1
				self.AccelMotorSpeed = 1
				self.Accel.data = [self.AccelAngle, self.AccelMotorSpeed]
				self.AccelPub.publish(self.Accel)
				time.sleep(0.05)

			else:
				self.AccelPID.setpoint = self.ReferenceSpeed
				self.BrakePID.setpoint = self.ReferenceSpeed			

				if(1):
					print("PID working")
					
					self.AccelPID.auto_mode = True
					self.BrakePID.auto_mode = True

					self.AccelAngle = 50 + self.AccelPID(self.CurrentVelocity)					
					if(self.AccelAngle > self.AccelMax):
						self.AccelAngle = self.AccelMax
					self.AccelMotorSpeed = 1					
					self.Accel.data = [self.AccelAngle, self.AccelMotorSpeed]
					self.AccelPub.publish(self.Accel)
					time.sleep(0.05)

					self.BrakeAngle = 0 + self.BrakePID(self.CurrentVelocity)					
					if(self.BrakeAngle > self.BrakeMax):
						self.BrakeAngle = self.BrakeMax
					self.BrakeMotorSpeed = 1					
					self.Brake.data = [self.BrakeAngle, self.BrakeMotorSpeed]
					self.BrakePub.publish(self.Brake)
					time.sleep(0.05)
		

def main(args):
	try:
		control = Controls()
		while (not rospy.is_shutdown()):
			control.cruizer()
			# control.convertor()
			#control.errorcheck()
			#time.sleep(0.05)
	except KeyboardInterrupt:
		self.Brake.data = [1, 1]
		self.BrakePub.publish(self.Brake)
		time.sleep(0.05)
		self.Accel.data = [1, 1]
		self.AccelPub.publish(self.Accel)
		time.sleep(0.05)

		print("Shutting down..")
		rospy.shutdown()
		sys.exit()

if __name__ == '__main__':
	main(sys.argv)