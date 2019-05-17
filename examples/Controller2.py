

import PID

import time
import TrajectoryPlanning as tp
import collections

import Geometry
from multiprocessing import Process, Pipe, Value, Lock
import SPOKe_IO


GANTRY_ROBOT = 1
RING_ROBOT = 2


def PID_to_control_input(pid_output):
	if pid_output >= 0:
		direction = 1
	else:
		direction = -1
	pid_output = abs(pid_output)/20
	return [direction, min(pid_output, 1)]



def reactToError(state, control_instance, buttonPipe, stopButtonPressed, graphPipe, graphPipeSize, graphLock, newButtonData):
	if (stopButtonPressed.value == 1):
		control_instance.stop()
		control_instance.dataBuffer[5] = 100
		sendData(control_instance, graphPipe, graphPipeSize, graphLock)
		print("In error state button pressed")
		time.sleep(4)
		control_instance.waitForStartSignal(buttonPipe, newButtonData, stopButtonPressed)
		stopButtonPressed.value = 0
		return True
	elif ((state == 3 or state == 6) and control_instance.isStuck()):
		print("We are in state ", state, "and are stuck. Proceeding to the state")
		control_instance.stop()
		control_instance.motor_control.openGrip()
		return True
#	elif (control_instance.isStuck()):
#		print("The robot is stuck. Stopping all motion")
#		control_instance.stop()
#		control_instance.dataBuffer[5] = 50
#		sendData(control_instance, graphPipe, graphPipeSize, graphLock)
#		print("In error stuck")
#		time.sleep(4)
#		control_instance.waitForStartSignal(buttonPipe, newButtonData)
#		while(1):
#			print("In error, robot stuck detected")
#			time.sleep(4)
#		return True
#	elif(control_instance.ls_instance.anyActive()):
#		print("The robot has touched limit switch. Stopping all motion")
#		control_instance.stop()
#		while(1):
#			print("Limit switch active: ", int(control_instance.ls_instance.active(1)), int(control_instance.ls_instance.active(2)), int(control_instance.ls_instance.active(3)), int(control_instance.ls_instance.active(4)))
#			time.sleep(4)
#		return True
	return False



def getTf(state, timeMultiplier):
	# timeMultiplier in range [0.5, 1.5]
	# Want low value to represent low velocity -> high tf
	if (state == 1 or state == 4):
		return 20 *abs(timeMultiplier.value -2) 
	elif (state == 2):
		return 7 *abs(timeMultiplier.value -2)
	elif (state == 5):
		return  4 *abs(timeMultiplier.value -2)
	elif (state == 3 or state == 6):
		return -1

def sendData(data_storage, graphPipe,graphPipeSize, graphLock):
	graphLock.acquire()
	graphPipe.send(data_storage.dataBuffer)
	graphPipeSize.value = graphPipeSize.value + 1
	graphLock.release()

def main(graphPipe, graphPipeReceiver, buttonPipe, graphPipeSize, graphLock, stopButtonPressed, newButtonData, operatingTimeConstant):
	# Initializing the robot, guaranteeing a safe starting position
	run_start_time = round(time.time(),2)
	control_instance = controller(run_start_time)
	control_instance.waitForInitSignal(buttonPipe)

	#control_instance.dataBuffer[5] = -1
	#sendData(control_instance, graphPipe, graphPipeSize, graphLock)
	#time.sleep(4)
	#control_instance.encoder_instance.reset_counter(1)
	#control_instance.encoder_instance.reset_counter(2)
	#control_instance.dataBuffer[5] = 0
	#sendData(control_instance, graphPipe, graphPipeSize, graphLock)
	#buttonPipe.send("Init finished")
	#newButtonData.value += 1


	if (control_instance.initialize(buttonPipe, newButtonData, stopButtonPressed, graphPipe, graphPipeSize, graphLock) == False):
		print("Stop button is pressed during init, looping forever")
		while (True):
			time.sleep(0.5)
	
	control_instance.waitForStartSignal(buttonPipe, newButtonData, stopButtonPressed)
	control_instance.run_start_time = round(time.time(),2)
	state = 1
	continuing = False

	while(True):
		t0 = 0
		tf = getTf(state, operatingTimeConstant)
		if (not continuing):
			if ( not control_instance.getNextTheta4d(state) ):
				# In case next desired angle is outside working area
				break
		continuing = False
		control_instance.initNewState(t0, tf, state)
		i = 0 

		while ((not control_instance.timeout) and (stopButtonPressed.value == 0)): # and (not control_instance.isStuck())):# and (not control_instance.ls_instance.anyActive())): # and control_instance.theta4_e > 0.017 and control_instance.r2_e > 0.02): 
			# Only check time when testing while the trajectory is still moving, theta4_e < 1 deg, r2_e < 2 cm.

			control_instance.updateTrajectory(state)
			control_instance.updatePosition()
			control_instance.updatePID(state)
			control_instance.storeData()

			if (i == 15):
				print("r2 value = ", control_instance.r2, " | theta4 value = ", control_instance.theta4)
				if (graphPipeSize.value == 0):
					control_instance.eraseBufferData()
				control_instance.bufferData()
				control_instance.eraseData()
				graphCommunication = Process(target=sendData, args=(control_instance, graphPipe, graphPipeSize, graphLock))
				graphCommunication.start()
				i = 0
			i += 1
			time.sleep(0.02)

		if (reactToError(state, control_instance, buttonPipe, stopButtonPressed, graphPipe, graphPipeSize, graphLock, newButtonData)):
			if (state != 3 or state != 6):
				state -= 1
			continuing = True
		if (state < 6):
			state += 1
		else: 
			state = 1
	control_instance.stop()



class controller:
	def __init__(self, time_value):
		self.run_start_time = time_value
		self.time_list = [0, 0]
		self.measurement_list_gantry = [0, 0]  
		self.reference_list_gantry = [0, 0]    
		self.measurement_list_ring = [0, 0]
		self.reference_list_ring = [0, 0]
		self.dataBuffer = [self.time_list[:], self.measurement_list_gantry[:], self.reference_list_gantry[:], self.measurement_list_ring[:], self.reference_list_ring[:], -1]

		self.encoder_instance = SPOKe_IO.Encoder_input()
		self.motor_control = SPOKe_IO.Motor_output()
		self.ls_instance = SPOKe_IO.LimitSwitch()			

		self.dimensions = Geometry.Dimensions()
		#self.tstart
		#self.tf
		#self.timeout 
		#self.pid_gantry
		#self.pid_ring

		# Trajectory parameters:
		#self.A0_gantry
		#self.A1_gantry
		#self.A2_gantry
		#self.tb_gantry
		#self.A0_ring
		#self.A1_ring
		#self.A2_ring
		#self.tb_ring

		# Position parameters
		#self.theta4
		self.theta4d = self.dimensions.theta4Min
		self.theta4_ref = 0
		#self.theta4_e
		#self.r2
		self.r2_max = self.dimensions.r2Max
		self.r2_min = self.dimensions.r2Min
		self.r2_ref = 0
		#self.r2_e

		self.timeDiffBuffer = collections.deque(maxlen=5)
		self.tickDiffBuffer1 = collections.deque(maxlen=5)
		self.tickDiffBuffer2 = collections.deque(maxlen=5)


		# Storage lists
	def initialize(self, buttonPipe, newButtonData,  stopButtonPressed, graphPipe, graphPipeSize, graphLock):
		
		self.dataBuffer[5] = -1
		sendData(self, graphPipe, graphPipeSize, graphLock)

		self.motor_control.setMotorDirection(GANTRY_ROBOT, -1)
		self.motor_control.setMotorDirection(RING_ROBOT, -1)
		initVelocity = 0.5

		while( not ( self.ls_instance.active(1) or self.ls_instance.active(2) or self.ls_instance.active(3) or self.ls_instance.active(4) or stopButtonPressed.value )):
			self.motor_control.setMotorSpeed(GANTRY_ROBOT, initVelocity)
			self.motor_control.setMotorSpeed(RING_ROBOT, initVelocity)
			time.sleep(0.05)
		self.stop()
		if (stopButtonPressed.value):
			return False

#		while ( not ( self.ls_instance.active(1) or self.ls_instance.active(2) or stopButtonPressed.value )):
#			self.motor_control.setMotorSpeed(RING_ROBOT, initVelocity)
#			time.sleep(0.05)
#		self.stop()
#		if (stopButtonPressed.value):
#			return False

		while ( not ( self.ls_instance.active(3) or self.ls_instance.active(4) or stopButtonPressed.value )):
			self.motor_control.setMotorSpeed(GANTRY_ROBOT, initVelocity)
			time.sleep(0.05)
		self.stop()
		if (stopButtonPressed.value):
			return False

		self.encoder_instance.reset_counter(1)
		self.encoder_instance.reset_counter(2)
		self.dataBuffer[5] = 0
		sendData(self, graphPipe, graphPipeSize, graphLock)
		buttonPipe.send("Init finished")
		newButtonData.value += 1
		return True

		
	def initNewState(self, t0, tf, state):
		# Fixing time
		self.tstart = round(time.time(),2)
		self.tf = tf
		self.t0 = t0
		self.timeout = False
		
		# Controller initialization
		if (state == 1 or state == 4):
			[P_g, I_g, D_g] = [10, 1, 0.2]
			[P_r, I_r, D_r] = [10, 1, 0.2]
		elif(state == 2 or state ==5):
			[P_g, I_g, D_g] = [10, 1, 0.2]
			[P_r, I_r, D_r] = [10, 1, 0.2]	
		if (state == 3 or state == 6):
			[P_g, I_g, D_g] = [0, 0, 0]
			[P_r, I_r, D_r] = [10, 1, 0.2]

		self.pid_gantry = PID.PID(P_g, I_g, D_g)
		self.pid_gantry.setSampleTime(0.1)
		self.pid_ring = PID.PID(P_r, I_r, D_r)
		self.pid_ring.setSampleTime(0.1)
		print(self.encoder_instance.read_counter_deg(GANTRY_ROBOT))
		print(self.encoder_instance.read_counter_deg(RING_ROBOT))
		#self.theta4 = self.encoder_instance.read_counter_deg(RING_ROBOT)
		#self.r2 = self.encoder_instance.read_counter_deg(GANTRY_ROBOT)
		self.theta4 = self.theta4_ref					# Only for simulation
		self.r2 = self.r2_ref 							# Only for simulation
		
		if (state == 1 or state == 4):
			self.theta4_ref = self.theta4d
			if (state == 1):
				self.r2_ref = self.r2_max
				velocityDir = 1
			elif (state == 4):
				self.r2_ref = self.r2_min
				velocityDir = -1
			velocity = tp.getLSPB_velocity(self.r2, self.r2_ref, self.t0, self.tf, 0.5)
			[self.A0_gantry, self.A1_gantry, self.A2_gantry, self.tb_gantry] = tp.LSPB(velocity*velocityDir, [self.r2, 0, self.r2_ref, 0], [self.t0, self.tf])

		elif (state == 2 or state == 5):
			velocity = tp.getLSPB_velocity(self.theta4, self.theta4d, self.t0, self.tf, 0.2)
			[self.A0_ring, self.A1_ring, self.A2_ring, self.tb_ring] = tp.LSPB(velocity, [self.theta4, 0, self.theta4d, 0], [t0, tf])
			if (state == 2):
				self.r2_ref = self.r2_max
			elif (state == 5):
				self.r2_ref = self.r2_min

		#elif (state == 3 or state == 6):
			# don't care about this

		self.dataBuffer[5] = state
		

	def waitForInitSignal(self, buttonPipe):
		while(True):
			b = buttonPipe.recv()
			if (b == "INIT"):
				break
			elif (b == "STOP"):
				self.stop()

	def waitForStartSignal(self, buttonPipe, newButtonData, stopButtonPressed):
		while(True):
			b = buttonPipe.recv()
			if (b == "START"):
				buttonPipe.send("Starting")
				newButtonData.value += 1
				stopButtonPressed.value = 0
				break
			elif (b == "STOP"):
				self.stop()

	def getNextTheta4d(self, state):
		if (state == 2):
			self.theta4d = self.theta4d + self.dimensions.alpha1
			if (self.theta4d > self.dimensions.theta4Max):
				return False
			return self.theta4d
		elif (state == 5):
			self.theta4d = self.theta4d + self.dimensions.alpha2
			if (self.theta4d > self.dimensions.theta4Max):
				return False
			return self.theta4d
		elif (state == 1 or state == 3 or state == 4 or state == 6):
			return self.theta4d
		else: 
			return False

	def updateTrajectory(self, state):
		operation_time = (round(time.time(),2) - self.tstart)
		if (operation_time > self.tf - 0.1):
			self.timeout = True
			return True
		if (state == 1 or state == 4):
			self.r2_ref = tp.getLSPB_position(self.A0_gantry, self.A1_gantry, self.A2_gantry, self.tb_gantry, self.tf, operation_time)
			return True
		elif (state == 2 or state == 5):
			self.theta4_ref = tp.getLSPB_position(self.A0_ring, self.A1_ring, self.A2_ring, self.tb_ring, self.tf, operation_time)
			return True
		elif (state ==3 or state == 6):
			return True
		return False

	def updatePosition(self):
		# Possible to make this return True or False?  
		self.r2 = Geometry.rad2r2(self.encoder_instance.read_counter_rad(GANTRY_ROBOT))
		self.theta4 = Geometry.rad2theta4(self.encoder_instance.read_counter_rad(RING_ROBOT))

	def updatePID(self, state):
    	# Necessary to make this return True or False?
		if (state == 3 or state == 6):
			self.r2_e = 0
			self.pid_ring.SetPoint = self.theta4_ref 
			self.pid_ring.update(self.theta4)
			self.theta4_e = self.theta4_ref - self.theta4
			self.pid_gantry.SetPoint = self.r2_ref
			return True
		elif (state == 1 or state == 2 or state == 4 or state == 5):
			self.pid_gantry.SetPoint = self.r2_ref
			self.pid_gantry.update(self.r2)
			self.r2_e = self.r2_ref - self.r2
			self.pid_ring.SetPoint = self.theta4_ref 
			self.pid_ring.update(self.theta4)
			self.theta4_e = self.theta4_ref - self.theta4
			return True
		return False

	def setOutput(self, state):
		# Possible to make this return True or False?
		[direction_gantry, PWM_signal_strength_gantry] = PID_to_control_input(self.pid_gantry.output)
		self.motor_control.setMotorDirection(GANTRY_ROBOT, direction_gantry)
		self.motor_control.setMotorSpeed(GANTRY_ROBOT, PWM_signal_strength_gantry)

		if (state == 3):
			self.motor_control.setMotorDirection(RING_ROBOT, -1)
			self.motor_control.setMotorSpeed(RING_ROBOT, 0.4)
			return True
		elif (state == 6):
			self.motor_control.setMotorDirection(RING_ROBOT, 1)
			self.motor_control.setMotorSpeed(RING_ROBOT, 0.4)
			return True
		
		[direction_ring, PWM_signal_strength_ring] = PID_to_control_input(self.pid_ring.output)
		self.motor_control.setMotorDirection(RING_ROBOT, direction_ring)
		self.motor_control.setMotorSpeed(RING_ROBOT, PWM_signal_strength_ring)

	# MUST BE TESTED BEFORE FIRST RUN: is PWM == 0 full throttle or full stop? 
	def stop(self):
		self.motor_control.setMotorSpeed(GANTRY_ROBOT, 0)
		self.motor_control.setMotorSpeed(RING_ROBOT, 0)
	
	# Could buffer PID signal also
	def isStuck(self):
		StuckThreshold = 129/0.5 # = 258 [tick/(s*PWM)]
		# 129 ticks/s at PWM = 0.5 gives a speed of 258 

		if (len(self.time_list) > 2):
			self.timeDiffBuffer.append(self.time_list[-1] - self.time_list[-2])
		else:
			# In the case where the data is erased before 
			self.timeDiffBuffer.append(self.timeDiffBuffer[-1])
		self.tickDiffBuffer1.append(self.encoder_instance.last_tick_diff1)
		self.tickDiffBuffer1.append(self.encoder_instance.last_tick_diff2)

		return False # Must be removed when testing the stuck function.

		if (len(self.timeDiffBuffer) == 5):
			velocityEstimate1 = sum(self.tickDiffBuffer1) / sum(self.timeDiffBuffer) # [tick/s]
			GantryPWMSignal = PID_to_control_input(self.pid_gantry.output)[1]
			if (GantryPWMSignal == 0): 
				return False
			if (velocityEstimate1 / GantryPWMSignal < StuckThreshold ): 
				return True

			velocityEstimate2 = sum(self.tickDiffBuffer2) / sum(self.timeDiffBuffer)
			RingPWMSignal = PID_to_control_input(self.pid_ring.output)[1]
			if (RingPWMSignal == 0):
				return False
			if (velocityEstimate2 / RingPWMSignal < StuckThreshold ): 
				return True

		return False 


	def bufferData(self):
		self.dataBuffer[0].extend(self.time_list[:])
		self.dataBuffer[1].extend(self.measurement_list_gantry[:])
		self.dataBuffer[2].extend(self.reference_list_gantry[:])
		self.dataBuffer[3].extend(self.measurement_list_ring[:])
		self.dataBuffer[4].extend(self.reference_list_ring[:])

	def eraseBufferData(self):
		self.dataBuffer = [[], [], [], [], [], self.dataBuffer[5]]

	def storeData(self):
		self.time_list.append((round(time.time(),2) - self.run_start_time))
		self.measurement_list_gantry.append(self.r2)
		self.reference_list_gantry.append(self.pid_gantry.SetPoint)
		self.measurement_list_ring.append(self.theta4)
		self.reference_list_ring.append(self.pid_ring.SetPoint)

	def eraseData(self):
		self.time_list = []
		self.measurement_list_gantry = []
		self.reference_list_gantry = []
		self.measurement_list_ring = []
		self.reference_list_ring = []


import matplotlib as plt

#test = 0
#if test == 1:
#	main()
#	# PLOT: 
#	plt.figure()
#	plt.plot(time_list, measurement_list_gantry, 'b')
#	plt.plot(time_list, reference_list_gantry, 'r')
#	plt.show()
#
#	plt.figure()
#	plt.plot(time_list, measurement_list_ring, 'b')
#	plt.plot(time_list, reference_list_ring, 'r')
#	plt.show()
	
	
