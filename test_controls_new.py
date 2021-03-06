#!/usr/bin/env python
'''
We will see

'''

import rospy
import array
from std_msgs.msg import Int64, Float64MultiArray, Float64, Float32MultiArray, Bool

from simple_pid import PID

# from camera_radar_msg.msg import fused_data
# from camera_radar_msg.msg import fused_data_
import sys
import time

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
		
		#Objects for PID class
		# self.LK2 = PID(self.Accel2Fact, self.Accel2Param, self.Accel2Constrain)
		# self.steer = PID(self.SteerFact,self.SteerParam, self.SteerConstrain)
		# self.Brake1 = PID(self.Brake1Fact, self.Brake1Param, self.Brake1Constrain)
		
		#Motor Speed 
		self.AccelMotorSpeed  = 1
		self.BrakeMotorSpeed = 2
		self.SteerMotorSpeed = 1
		self.ClutchMotorSpeed = 1
		
		#Speed related 
		self.ObjectPresent = False
		self.ReferenceSpeed = 0
		self.CurrentSum = 0
		self.index = 0
		self.CurrentReadings = [0,0,0,0,0,0,0,0,0,0,0]
		self.CurrentValue = 0


		self.BrakeAngle = 0.0
		self.ClutchAngle = 0.0
		self.CurrentVelocity = 0.0
		self.EmergencyBrakeMotorSpeed = 0.05
		self.EmergencyBrakeAngle = 129
		# self.STOPBoardDistance = 15.0

		#Limiting Variables
		self.AccelMax = 70
		self.BrakeMax = 135		#100 
		self.ClutchMax = 330
		self.halfClutch = 210 	#210-200

		self.NormalAccel = 1
		self.ThrottlePos = 1
		
		self.Brake = Float32MultiArray()
		self.Accel = Float32MultiArray()
		self.Steer = Float32MultiArray()
		self.Clutch = Float32MultiArray()
		
		self.BrakeEmergency = Float32MultiArray()

		self.FBSteeringAngle = 0
		self.CurrentSteeringAngle = 0
		self.i =0
		self.CurrentSteeringAngleBuf = []
		for self.i in range(0,100):
			self.CurrentSteeringAngleBuf.append(0)
		self.i = 0

		#Accel PID Parameters
		self.AccelKp = 6.0
		self.AccelKi = 1.2
		self.AccelKd = 1.5
		self.Accelsetpoint = self.ReferenceSpeed

		self.AccelPID = PID(Kp = self.AccelKp, Ki = self.AccelKi, Kd = self.AccelKd, setpoint = self.Accelsetpoint)
		self.AccelPID.output_limits = (0, 50)

		#Brake PID Parameters
		self.BrakeKp = -8.0 
		self.BrakeKi = -0.0
		self.BrakeKd = -5.0
		self.Brakesetpoint = self.ReferenceSpeed

		self.BrakePID = PID(Kp = self.BrakeKp, Ki = self.BrakeKi, Kd = self.BrakeKd, setpoint = self.Brakesetpoint)
		self.BrakePID.output_limits = (0, self.BrakeMax)
		# self.BrakePID.proportional_on_measurement = True


		if not rospy.is_shutdown():
			#Subscribers
			# self.OffsetValue = rospy.Subscriber("Vision/Lane_Offset", Int64, self.cal_angle)
			rospy.Subscriber("Input/Inputs", Float64MultiArray, self.get_data , queue_size=1)
			# rospy.Subscriber("/fused_data", fused_data_, self.update_dist)
			rospy.Subscriber("/manual_speed", Float64, self.call_manualSPeed)
			# rospy.Subscriber("/object_dis", Float64, self.get_obj_dis)
			# rospy.Subscriber("/speedCommand", Float64, self.get_MPCspeed, queue_size=1)
			# rospy.Subscriber("/steerCommand", Float64, self.get_MPCsteer, queue_size=1)
			# rospy.Subscriber("/planning/gideon/emergencyFlag", Bool, self.get_emergency, queue_size=1)
			#rospy.Subscriber("/Controls/debug", , self.get_emergency, queue_size=1)

			#Publisher
			self.SteerPub = rospy.Publisher('/Output/Steering_Angle', Float32MultiArray, queue_size=1)   # For steering motor
			self.BrakePub = rospy.Publisher('/Output/Brake_Angle', Float32MultiArray, queue_size=1)       # For brake motor
			self.ClutchPub = rospy.Publisher('/Output/Clutch_Angle', Float32MultiArray, queue_size=1)       # For brake motor			
			self.AccelPub = rospy.Publisher('/Output/Speed_Angle', Float32MultiArray, queue_size=1)   # For accelerator motor
			self.SpeedPub = rospy.Publisher('/OBD', Float64, queue_size=1)   # For accelerator motor


	def call_manualSPeed(self, data):
		self.ReferenceSpeed = data.data
		print("manual_speed",self.ReferenceSpeed)

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
		# self.CurrentVelocity = data.data[5]
		self.CurrentSum = self.CurrentSum - self.CurrentReadings[self.index]
		self.CurrentValue = data.data[5]
		self.CurrentReadings[self.index] = self.CurrentValue
		self.CurrentSum = self.CurrentSum + self.CurrentValue
		self.index = (self.index+1)%10
		self.CurrentVelocity = (self.CurrentSum/10)/2

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

		print(self.ObjectPresent,self.CurrentVelocity,self.ReferenceSpeed,self.NormalAccel,int(self.BrakeAngle),self.ClutchAngle)
		if(self.ObjectPresent):
			print("Going to Emergency ")
			self.EmergencyBraking()
			
		else:
			if(self.ReferenceSpeed >= 100):	
				self.BrakeAngle = 1
				self.BrakeMotorSpeed = 1
				self.Brake.data = [self.BrakeAngle, self.BrakeMotorSpeed]
				self.BrakePub.publish(self.Brake)
				time.sleep(0.05)
				self.NormalAccel = 1
				self.AccelMotorSpeed = 1
				self.Accel.data = [self.NormalAccel, self.AccelMotorSpeed]
				self.AccelPub.publish(self.Accel)
				time.sleep(0.05)
				self.ClutchAngle = 1
				self.ClutchMotorSpeed = 1
				self.Clutch.data = [self.ClutchAngle, self.ClutchMotorSpeed]
				self.ClutchPub.publish(self.Clutch)
				time.sleep(0.05)
				return 0
	
			elif(self.ReferenceSpeed <= 0):
				self.BrakeAngle = self.BrakeMax
				self.BrakeMotorSpeed = 1
				self.Brake.data = [self.BrakeAngle, self.BrakeMotorSpeed]
				self.BrakePub.publish(self.Brake)
				time.sleep(0.05)
				self.NormalAccel = 1
				self.AccelMotorSpeed = 1
				self.Accel.data = [self.NormalAccel, self.AccelMotorSpeed]
				self.AccelPub.publish(self.Accel)
				time.sleep(0.05)
				self.ClutchAngle = self.ClutchMax
				self.ClutchMotorSpeed = 1
				self.Clutch.data = [self.ClutchAngle, self.ClutchMotorSpeed]
				self.ClutchPub.publish(self.Clutch)
				time.sleep(0.05)
				
			else:
				self.BrakePID.auto_mode = True
				self.AccelPID.auto_mode = True
					
				self.AccelPID.setpoint = self.ReferenceSpeed
				self.BrakePID.setpoint = self.ReferenceSpeed
			
				if(1):
					if(self.CurrentVelocity < 8):
						self.AccelPID.output_limits = (0, 30)
						self.BrakePID.output_limits = (0, self.BrakeMax)		
						self.ClutchAngle = self.halfClutch
						self.ClutchMotorSpeed = 1
						self.Clutch.data = [self.ClutchAngle, self.ClutchMotorSpeed]
						self.ClutchPub.publish(self.Clutch)
				
					else:
						self.AccelPID.output_limits = (0, 60)
						self.ClutchAngle = 130
						self.ClutchMotorSpeed = 1
						self.Clutch.data = [self.ClutchAngle, self.ClutchMotorSpeed]
						self.ClutchPub.publish(self.Clutch)
					print("Accelerating", self.NormalAccel)
					
					# self.NormalAccel = 17 + self.AccelPID(self.CurrentVelocity)					
					# if(self.NormalAccel > self.AccelMax):
					# 	self.NormalAccel = self.AccelMax
					# self.AccelMotorSpeed = 1					
					# self.Accel.data = [self.NormalAccel, self.AccelMotorSpeed]
					# self.AccelPub.publish(self.Accel)
					# time.sleep(0.05)
					
					
					self.BrakeAngle = 100 + self.BrakePID(self.CurrentVelocity)					
					if(self.BrakeAngle > self.BrakeMax):
						self.BrakeAngle = self.BrakeMax
					self.BrakeMotorSpeed = 1					
					self.Brake.data = [int(self.BrakeAngle), self.BrakeMotorSpeed]
					self.BrakePub.publish(self.Brake)
					time.sleep(0.05)									

def main(args):
	try:
		control = Controls()
		# rate = rospy.Rate(10)
		while (not rospy.is_shutdown()):
			# rate.sleep()
			control.cruizer()
			
	except KeyboardInterrupt():
		print("Shutting down..")
		rospy.shutdown()

if __name__ == '__main__':
	main(sys.argv)