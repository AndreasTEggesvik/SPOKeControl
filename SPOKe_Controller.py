

import PID

import time
import Trajectory_Planning as tp
import collections

import SPOKe_Geometry
from multiprocessing import Process, Pipe, Value, Lock
import SPOKe_IO
import math
from collections import deque


GANTRY_ROBOT = 1
RING_ROBOT = 2

# State = [init, deploymentDone stopButtonPressed, errorLimitSwitch, stuck, moveAlongRing, moveAlongRingBack moveInwards, moveOutwards, tightenRopeInwards, tightenRopeOutwards]
#stateToRevertBackTo = [moveAlongRing, moveInwards, moveOutwards, tightenRopeInwards, tightenRopeOutwards]
state = "init"
stateToRevertBackTo = "moveAlongRing" # Only used when in an error state
powerThrust = 100
mode = "Deploy"

def PID_to_control_input(pid_output, motor, encoder):
	global powerThrust
	if pid_output >= 0:
		direction = 1
	else:
		direction = -1
	pid_output = abs(pid_output)
	if (pid_output < 15):
		pid_output = 0
	elif (motor == RING_ROBOT and abs(encoder.last_tick_diff2) < 2):
		pid_output = powerThrust
	elif (motor == GANTRY_ROBOT and abs(encoder.last_tick_diff1) < 2):
		pid_output = powerThrust
	else: 
		pid_output = 15 + 6 * math.sqrt(pid_output-15)
	return [direction, min(pid_output, 100)]

def main(graphPipe, graphPipeReceiver, buttonPipe, graphPipeSize, graphLock, stopButtonPressed, newButtonData, operatingTimeConstant):
	# Initializing the robot, guaranteeing a safe starting position
	global mode
	global state
	
	run_start_time = round(time.time(),2)
	control_instance = controller(run_start_time)

	while(mode == "Deploy" or mode == "Detatch"):
		if mode == "Deploy":
			control_instance.waitForInitSignal(buttonPipe)

			if (control_instance.initialize(buttonPipe, newButtonData, stopButtonPressed, graphPipe, graphPipeSize, graphLock, "Deploy") == False):
				print("Stop button is pressed during init, looping forever")
				while (True):
					time.sleep(0.5)
			
			control_instance.waitForStartSignal(buttonPipe, newButtonData, stopButtonPressed)
			control_instance.run_start_time = round(time.time(),2)
			continuing = False
			state = "moveOutwards"

			control_instance.theta4d = control_instance.dimensions.theta4Min
			control_instance.r2_ref = control_instance.dimensions.r2Min

		elif (mode == "Detatch"):
			print("Deployment finished!! ")
			control_instance.waitForInitSignal(buttonPipe)

			if (control_instance.initialize(buttonPipe, newButtonData, stopButtonPressed, graphPipe, graphPipeSize, graphLock, "Detatch") == False):
				print("Stop button is pressed during init, looping forever")
				while (True):
					time.sleep(0.5)
			
			control_instance.waitForStartSignal(buttonPipe, newButtonData, stopButtonPressed)
			control_instance.run_start_time = round(time.time(),2)
			continuing = False
			state = "moveAlongRingBack"
			control_instance.theta4d = control_instance.dimensions.LS4Position
			control_instance.r2_ref = control_instance.dimensions.r2Max

		print("Ready for while loop")
		# Main loop:
		while(True):
			print("initializing state", state)
			t0 = 0
			tf = getTf(state, operatingTimeConstant)
			if (not continuing):
				if ( not control_instance.getNextTheta4d(state, mode) ):
					# In case next desired angle is outside working area, breaking the while loop
					print ("We will break because we received ", control_instance.getNextTheta4d(state, mode))
					break
			continuing = False
			control_instance.initNewState(t0, tf, state)
			i = 0 
			print("ready for while loop. Timeout: ", control_instance.timeout)
			while ((not control_instance.timeout or (abs(control_instance.theta4_e) > 0.01 or abs(control_instance.r2_e) > 0.007)) and (stopButtonPressed.value == 0) and (not control_instance.ls_instance.anyActive()) and (not control_instance.isStuck())): 
				# Only continue when the trajectory is still moving, theta4_e < 0.57 deg, r2_e < 7 mm and no stop button or limit switch is hit.

				control_instance.updateTrajectory(state)
				control_instance.updatePosition()
				control_instance.updatePID(state)
				control_instance.setOutput(state)
				control_instance.storeData()

				if (i == 15):

					[direction_ring, PWM_signal_strength_ring] = PID_to_control_input(control_instance.pid_ring.output, 2, control_instance.encoder_instance)
					print("Motor control signals: DIRECTION = ", direction_ring, " | POWER = ", PWM_signal_strength_ring, " (",control_instance.pid_ring.output, ")" )

					if (graphPipeSize.value == 0):
						control_instance.eraseBufferData()
					control_instance.bufferData()
					control_instance.eraseData()
					graphCommunication = Process(target=sendData, args=(control_instance, graphPipe, graphPipeSize, graphLock))
					graphCommunication.start()
					i = 0
				i += 1
				time.sleep(0.02)
			print("Transitioning from state ", state)
			state = transitionStateDeployment(state, control_instance, stopButtonPressed, mode)
			print("To state ", state)
			if (state == "stopButtonPressed" or state == "errorLimitSwitch" or state == "stuck"):
				[state, mode] = reactToError(state, control_instance, buttonPipe, stopButtonPressed, graphPipe, graphPipeSize, graphLock, newButtonData, mode)
				if (state == False):
					break
				else:
					continuing = True	

			elif (state == "deploymentDone"):
				mode = 	"Detatch"
			elif (state == "detatchmentDone"):
				mode = "Deploy"
	print("Finished")
	control_instance.stop()



class controller:
	def __init__(self, time_value):
		self.run_start_time = time_value
		self.time_list = [0, 0]
		self.measurement_list_gantry = [0, 0]  
		self.reference_list_gantry = [0, 0]    
		self.pid_list_gantry = [0, 0]
		self.measurement_list_ring = [0, 0]
		self.reference_list_ring = [0, 0]
		self.pid_list_ring = [0, 0]
		self.dataBuffer = [self.time_list[:], self.measurement_list_gantry[:], self.reference_list_gantry[:], self.pid_list_gantry[:], self.measurement_list_ring[:], self.reference_list_ring[:], self.pid_list_ring[:], -1]
		self.powerBuffer = [0, deque(maxlen=10), deque(maxlen=10)] # Length of 10 -> 0.02s * 10 = 0.2 s
		self.powerBuffer[GANTRY_ROBOT].append(1)
		self.powerBuffer[RING_ROBOT].append(1)
		import pymonarco_hat as plc
		lib_path = '../pymonarco-hat/monarco-c/libmonarco.so'
		self.plc_handler = plc.Monarco(lib_path, debug_flag=plc.MONARCO_DPF_WRITE | plc.MONARCO_DPF_WARNING)	

		self.encoder_instance = SPOKe_IO.Encoder_input(self.plc_handler)
		self.motor_control = SPOKe_IO.Motor_output(self.plc_handler)
		self.ls_instance = SPOKe_IO.LimitSwitch()			

		self.dimensions = SPOKe_Geometry.Dimensions()

		#####################################################
		# Other local variables for the class 'controller':	#
		#	tstart											#
		#	tf												#
		#	timeout 										#
		#	pid_gantry										#
		#	pid_ring										#
		#	A0_gantry										#
		#	A1_gantry										#
		#	A2_gantry										#
		#	tb_gantry										#
		#	A0_ring											#
		#	A1_ring											#
		#	A2_ring											#
		#	tb_ring											#
		#	theta4											#
		#	r2												#
		#####################################################

		# Position parameters
		self.theta4d = self.dimensions.theta4Min
		self.previous_theta4d = self.theta4d
		self.theta4_ref = self.dimensions.theta4Min
		self.theta4_e = 0

		self.r2_max = self.dimensions.r2Max
		self.r2_min = self.dimensions.r2Min
		self.r2_ref = 0
		self.r2_e = 0

		self.timeDiffBuffer = collections.deque(maxlen=5)
		self.tickDiffBuffer1 = collections.deque(maxlen=5)
		self.tickDiffBuffer2 = collections.deque(maxlen=5)


	def initializeEncoder(self, motorNumber, direction, stopButtonPressed):
		
		motorDirectionValue = 0
		motorSpeed = 0
		limitSwitchNumber = 0
		positionOfLimitSwitch = 0
		if (motorNumber == RING_ROBOT):
			if (direction == "right"):
				motorDirectionValue = -1
				limitSwitchNumber = 2
				positionOfLimitSwitch = 0
			elif (direction == "left"):
				motorDirectionValue = 1
				limitSwitchNumber = 4
				positionOfLimitSwitch = self.dimensions.LS4Position # Should be changed
		elif (motorNumber == GANTRY_ROBOT):
			if (direction == "in"):
				motorDirectionValue = -1
				limitSwitchNumber = 3
				positionOfLimitSwitch = 0
			elif (direction == "out"):
				motorDirectionValue = 1
				limitSwitchNumber = 1
				positionOfLimitSwitch = self.dimensions.r2Max
		self.motor_control.setMotorDirection(motorNumber, motorDirectionValue)
		self.encoder_instance.reset_counter(motorNumber)
		self.updatePosition()

		while(not ( self.ls_instance.active(limitSwitchNumber) or stopButtonPressed.value)):
			self.updatePosition()
			tickDiff = abs(self.encoder_instance.getTickDiff(motorNumber))
			if (tickDiff < 40):
				motorSpeed = 90
			elif (tickDiff < 200):
				motorSpeed = 50
			else: 
				motorSpeed = 20
			self.motor_control.setMotorSpeed(motorNumber, motorSpeed)
			time.sleep(0.05)
		self.stop()
		self.encoder_instance.set_position(motorNumber, positionOfLimitSwitch)
		print("Encoder set for motor number", motorNumber)
		self.motor_control.setMotorDirection(motorNumber, motorDirectionValue * -1)
		while(self.ls_instance.active(limitSwitchNumber)):
			self.motor_control.setMotorSpeed(motorNumber, 30)
			time.sleep(0.05)
		self.stop()





		# Storage lists
	def initialize(self, buttonPipe, newButtonData,  stopButtonPressed, graphPipe, graphPipeSize, graphLock, operation):
		self.dataBuffer[7] = -1
		sendData(self, graphPipe, graphPipeSize, graphLock)
		if (operation == "Deploy"):
			self.initializeEncoder(RING_ROBOT, "right", stopButtonPressed)
			self.initializeEncoder(GANTRY_ROBOT, "in", stopButtonPressed)

		elif (operation == "Detatch"):
			self.initializeEncoder(RING_ROBOT, "left", stopButtonPressed)
			self.initializeEncoder(GANTRY_ROBOT, "out", stopButtonPressed)

		if (stopButtonPressed.value):
			return False
		self.dataBuffer[7] = 0
		sendData(self, graphPipe, graphPipeSize, graphLock)
		buttonPipe.send("Init finished")
		newButtonData.value += 1
		return True
		
	def initNewState(self, t0, tf, state):
		print("Initializing state: ", state)
		self.tstart = round(time.time(),2)
		self.tf = tf
		self.t0 = t0
		self.timeout = False
		
		# Enables possibility of different PID control for different states
		if (state == "moveInwards" or state == "moveOutwards"):
			[P_g, I_g, D_g] = [950, 90, 94]
			[P_r, I_r, D_r] = [800, 600, 350]
		elif(state == "moveAlongRing" or state == "moveAlongRingBack"):
			[P_g, I_g, D_g] = [400, 100, 74]
			[P_r, I_r, D_r] = [900, 470, 350]	
		if (state == "tightenRopeInwards" or state == "tightenRopeOutwards"):
			[P_g, I_g, D_g] = [450, 80, 94]			# PID controller is not used for gantry in these states
			[P_r, I_r, D_r] = [900, 470, 350]
			self.motor_control.closeGrip()
			time.sleep(1.5)
		

		self.pid_gantry = PID.PID(P_g, I_g, D_g)
		self.pid_gantry.setSampleTime(0.02)
		self.pid_ring = PID.PID(P_r, I_r, D_r)
		self.pid_ring.setSampleTime(0.02)
		
		self.theta4 = SPOKe_Geometry.rad2theta4(self.encoder_instance.read_counter_rad(RING_ROBOT))
		self.r2 = SPOKe_Geometry.rad2r2(self.encoder_instance.read_counter_rad(GANTRY_ROBOT))

#		self.theta4 = self.theta4_ref					# Only for simulation, these valuas represents position if control is perfect
#		self.r2 = self.r2_ref 							# Only for simulation
		
		self.calculateTrajectory(state)
		self.dataBuffer[7] = state

	def calculateTrajectory(self, state):
		sign = lambda x: math.copysign(1, x)
		if (state == "moveOutwards" or state == "moveInwards"):
			if (state == "moveOutwards"):
				self.r2_ref = self.r2_max
			elif (state == "moveInwards"):
				self.r2_ref = self.r2_min
			velocity = tp.getLSPB_velocity(self.r2, self.r2_ref, self.t0, self.tf, 0.15) 
			velocityDir = sign(self.r2_ref - self.r2)
			print("Calculating trajectory for gantry robot with r2 = ", self.r2, " | r2_ref = ", self.r2_ref, " | velocity = ", velocity)
			[self.A0_gantry, self.A1_gantry, self.A2_gantry, self.tb_gantry] = tp.LSPB(velocity*velocityDir, [self.r2, 0, self.r2_ref, 0], [self.t0, self.tf])
			# We want the angle to move as the middle third of the movement:
			
			stateRunTime = self.tf - self.t0
			
			if (mode == "Deploy" and self.theta4d == self.dimensions.theta4Min):
				# If statement is true the first time state "moveOutwards" is run
				[self.A0_ring, self.A1_ring, self.A2_ring, self.tb_ring] = [0, 0, 0, 0]
			else:
				velocityAngular = tp.getLSPB_velocity(self.previous_theta4d, self.theta4d, self.t0 + stateRunTime/3, self.tf - stateRunTime/3, 0.3)
				velocityDir = sign(self.theta4d - self.previous_theta4d)
				print("Calculating trajectory for ring with theta4 = ", self.previous_theta4d, " | theta4d = ", self.theta4d, " | velocity = ", velocityAngular)
				[self.A0_ring, self.A1_ring, self.A2_ring, self.tb_ring] = tp.LSPB(velocityAngular* velocityDir, [self.previous_theta4d, 0, self.theta4d, 0], [0, stateRunTime/3])

			
		elif (state == "moveAlongRing" or state == "moveAlongRingBack"):
			#if (state == "moveAlongRing"):
			#	velocityDir = 1
			#elif (state == "moveAlongRingBack"):
			#	velocityDir = -1
			velocity = tp.getLSPB_velocity(self.theta4, self.theta4d, self.t0, self.tf, 0.5)
			velocityDir = sign(self.theta4d - self.theta4)
			print("Calculating trajectory for ring with theta4 = ", self.theta4, " | theta4d = ", self.theta4d, " | velocity = ", velocity*velocityDir)
			[self.A0_ring, self.A1_ring, self.A2_ring, self.tb_ring] = tp.LSPB(velocity*velocityDir, [self.theta4, 0, self.theta4d, 0], [self.t0, self.tf])


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

	# Used to set the next reference point for theta_4, based on geometry of the SPOKe cleats
	def getNextTheta4d(self, _state, mode):
		global state
		self.previous_theta4d = self.theta4d
		if (state == "moveInwards" or state == "moveOutwards"):
			if (self.theta4d == self.dimensions.theta4Min):
				self.theta4d = self.dimensions.theta4Min
			elif (mode == "Deploy"): 
				self.theta4d += self.dimensions.angularMovementState_1_4
			elif (mode == "Detatch"):
				self.theta4d -= self.dimensions.angularMovementState_1_4
			return self.theta4d

		elif (state == "moveAlongRing"): 
			if (self.theta4d == self.dimensions.theta4Min):
				self.theta4d += self.dimensions.initialAngularMovement
			else:
				self.theta4d += self.dimensions.angularMovementState_2_5
			if (self.theta4d > self.dimensions.theta4Max):
				state = "deploymentDone"
				return False
			return self.theta4d
		elif (state  == "moveAlongRingBack"):
			if (self.theta4d == self.dimensions.LS4Position): # Should be changed
				print("Ready for initial movement back!")
				self.theta4d -= self.dimensions.initialAngularMovementBack
			else:
				self.theta4d -= self.dimensions.angularMovementState_2_5
			if (self.theta4d < self.dimensions.theta4Min):
				state = "detatchmentDone"
				return False
			return self.theta4d


		elif (state == "tightenRopeInwards" or state == "tightenRopeOutwards"):
			return self.theta4d
		else: 
			return False

		

	def updateTrajectory(self, state):
		operation_time = (round(time.time(),2) - self.tstart)
		if (operation_time > self.tf - 0.1):
			self.timeout = True
			return True
		if (state == "moveOutwards" or state == "moveInwards"):
			self.r2_ref = tp.getLSPB_position(self.A0_gantry, self.A1_gantry, self.A2_gantry, self.t0, self.tb_gantry, self.tf, operation_time)

			stateRunTime = self.tf - self.t0
			if (self.theta4d == self.dimensions.theta4Min): # For the first time state "moveOutwards" is excecuted
				self.theta4_ref = self.theta4d
				return True
			elif (operation_time < stateRunTime/5):
				# This is a way to have constant desired theta4 until the trajectory is supposed to begin
				self.theta4_ref = tp.getLSPB_position(self.A0_ring, self.A1_ring, self.A2_ring, self.t0, self.tb_ring, stateRunTime/3, 0)
				return True
			else: 
				self.theta4_ref = tp.getLSPB_position(self.A0_ring, self.A1_ring, self.A2_ring, self.t0, self.tb_ring, stateRunTime/3, operation_time - stateRunTime/5)
			return True
		elif (state == "moveAlongRing" or state == "moveAlongRingBack"):
			self.theta4_ref = tp.getLSPB_position(self.A0_ring, self.A1_ring, self.A2_ring, self.t0,  self.tb_ring, self.tf, operation_time)
			return True

		elif (state == "tightenRopeInwards" or state == "tightenRopeOutwards"):
			return True
		return False

	def updatePosition(self):
		self.r2 = SPOKe_Geometry.rad2r2(self.encoder_instance.read_counter_rad(GANTRY_ROBOT))
		self.theta4 = SPOKe_Geometry.rad2theta4(self.encoder_instance.read_counter_rad(RING_ROBOT))

		#self.r2 = self.r2_ref # For simulation only, represent perfect control
		#self.theta4 = self.theta4_ref

	def updatePID(self, state):
		if (state == "tightenRopeInwards" or state == "tightenRopeOutwards"):
			self.r2_e = 0
			self.pid_ring.SetPoint = self.theta4_ref 
			self.pid_ring.update(self.theta4)
			self.theta4_e = self.theta4_ref - self.theta4
			self.pid_gantry.SetPoint = self.r2_ref
			return True
		elif (state == "moveInwards" or state == "moveOutwards" or state == "moveAlongRing" or state =="moveAlongRingBack"):
			self.r2_e = self.r2_ref - self.r2

			# To eliminate windup effecting us badly: 
			if (abs(self.r2_e) < 0.001): # 1 mm
				self.pid_gantry.setWindup(0)
			else: 
				self.pid_gantry.setWindup(20)

			self.pid_gantry.SetPoint = self.r2_ref
			self.pid_gantry.update(self.r2)

			self.theta4_e = self.theta4_ref - self.theta4
			# To eliminate windup effecting us badly: 
#			if (abs(self.theta4_e) < 0.2*3.14/180): # 0.2 deg
#				self.pid_ring.setWindup(0)
#			else: 
#				self.pid_ring.setWindup(20)
			
			self.pid_ring.SetPoint = self.theta4_ref 
			self.pid_ring.update(self.theta4)
			return True
		return False

	def setOutput(self, state):
		# This function updates output for both motors based on PID controller
		[direction_ring, PWM_signal_strength_ring] = PID_to_control_input(self.pid_ring.output, RING_ROBOT, self.encoder_instance)
		self.motor_control.setMotorDirection(RING_ROBOT, direction_ring)
		self.motor_control.setMotorSpeed(RING_ROBOT, PWM_signal_strength_ring)
		self.powerBuffer[RING_ROBOT].append(PWM_signal_strength_ring)
		#self.motor_control.setMotorSpeed(2, 0.3)
		

		if (state == "tightenRopeInwards"):
			if (self.r2_max * (1/3) < self.r2 < self.r2_max * (2/3)):
				# Protection in case we have too much rope to tighten
				self.motor_control.setMotorSpeed(GANTRY_ROBOT, 0)
			else:
				self.motor_control.setMotorDirection(GANTRY_ROBOT, -1)
				self.motor_control.setMotorSpeed(GANTRY_ROBOT, 50)
			return True
		elif (state == "tightenRopeOutwards"):
			if (self.r2_max * (1/3) < self.r2 < self.r2_max * (2/3)):
				self.motor_control.setMotorSpeed(GANTRY_ROBOT, 0)
			else:
				self.motor_control.setMotorDirection(GANTRY_ROBOT, 1)
				self.motor_control.setMotorSpeed(GANTRY_ROBOT, 50)
			return True
		
		[direction_gantry, PWM_signal_strength_gantry] = PID_to_control_input(self.pid_gantry.output, GANTRY_ROBOT, self.encoder_instance)
		self.motor_control.setMotorDirection(GANTRY_ROBOT, direction_gantry)
		self.motor_control.setMotorSpeed(GANTRY_ROBOT, PWM_signal_strength_gantry)
		self.powerBuffer[GANTRY_ROBOT].append(PWM_signal_strength_gantry)
		return True
	def isStuck(self):
		global powerThrust
		if (sum(self.powerBuffer[RING_ROBOT])/len(self.powerBuffer[RING_ROBOT]) == powerThrust ):
			print("Ring movment is stuck!!")
			return True
		elif (sum(self.powerBuffer[GANTRY_ROBOT])/len(self.powerBuffer[GANTRY_ROBOT]) == powerThrust):
			print("Gantry movement is stuck!!")
			return True
		return False
		

	def stop(self):
		self.motor_control.setMotorSpeed(GANTRY_ROBOT, 0)
		self.motor_control.setMotorSpeed(RING_ROBOT, 0)
		return True
	
	def bufferData(self):
		self.dataBuffer[0].extend(self.time_list[:])
		self.dataBuffer[1].extend(self.measurement_list_gantry[:])
		self.dataBuffer[2].extend(self.reference_list_gantry[:])
		self.dataBuffer[3].extend(self.pid_list_gantry[:])
		self.dataBuffer[4].extend(self.measurement_list_ring[:])
		self.dataBuffer[5].extend(self.reference_list_ring[:])
		self.dataBuffer[6].extend(self.pid_list_ring[:])

	def eraseBufferData(self):
		self.dataBuffer = [[], [], [], [], [], [], [], self.dataBuffer[7]]

	def storeData(self):
		self.time_list.append((round(time.time(),2) - self.run_start_time))
		self.measurement_list_gantry.append(self.r2)
		self.reference_list_gantry.append(self.pid_gantry.SetPoint)
		self.pid_list_gantry.append(self.pid_gantry.output)
		self.measurement_list_ring.append(self.theta4)
		self.reference_list_ring.append(self.pid_ring.SetPoint)
		self.pid_list_ring.append(self.pid_ring.output)

	def eraseData(self):
		self.time_list = []
		self.measurement_list_gantry = []
		self.reference_list_gantry = []
		self.pid_list_gantry = []
		self.measurement_list_ring = []
		self.reference_list_ring = []
		self.pid_list_ring = []



def reactToError(state, control_instance, buttonPipe, stopButtonPressed, graphPipe, graphPipeSize, graphLock, newButtonData, mode):
	global stateToRevertBackTo
	if (state == "stopButtonPressed"):
		control_instance.stop()
		control_instance.dataBuffer[7] = 100
		sendData(control_instance, graphPipe, graphPipeSize, graphLock)
		print("In error state button pressed")
		time.sleep(4)
		control_instance.waitForStartSignal(buttonPipe, newButtonData, stopButtonPressed)
		stopButtonPressed.value = 0
		return [stateToRevertBackTo, mode]
	elif(state == "errorLimitSwitch"):
		control_instance.dataBuffer[7] = 51
		sendData(control_instance, graphPipe, graphPipeSize, graphLock)
		print("The robot has touched limit switch. Stopping all motion")
		print("theta4 = ", control_instance.theta4, " | counter 2 = ", control_instance.encoder_instance.local_counter2)
		control_instance.stop()
		print("Limit switch active: ", int(control_instance.ls_instance.active(1)), int(control_instance.ls_instance.active(2)), int(control_instance.ls_instance.active(3)), int(control_instance.ls_instance.active(4)))
		buttonPipe.send("Ready to continue")
		newButtonData.value += 1
		time.sleep(4)
		control_instance.waitForStartSignal(buttonPipe, newButtonData, stopButtonPressed)
		return [stateToRevertBackTo, mode]
	elif (state == "stuck"):
		print("The robot is stuck. Stopping all motion")
		control_instance.stop()
		control_instance.dataBuffer[7] = 50
		sendData(control_instance, graphPipe, graphPipeSize, graphLock)
		print("Terminating due to stuckness")
		return [False, False]
	return False



def getTf(state, timeMultiplier):
	# timeMultiplier in range [0.5, 1.5], received from the slider on the touch screen
	# Want low value to represent low velocity -> high tf
	if (state == "moveInwards" or state == "moveOutwards"):
		return 20 *abs(timeMultiplier.value -2) 
	elif (state == "moveAlongRing" or state =="moveAlongRingBack"):
		return 7 *abs(timeMultiplier.value -2)
	elif (state == "tightenRopeInwards" or state == "tightenRopeOutwards"):
		# return -1 # This should be used when the system has a stuck detection, as the state 3 and 6 don't have a trajectory to follow
		return 3

def sendData(data_storage, graphPipe,graphPipeSize, graphLock):
	graphLock.acquire()
	graphPipe.send(data_storage.dataBuffer)
	graphPipeSize.value = graphPipeSize.value + 1
	graphLock.release()



def transitionStateDeployment(state, control_instance, stopButtonPressed, mode):
	global stateToRevertBackTo

	if (stopButtonPressed.value):
		stateToRevertBackTo = state
		return "stopButtonPressed"
	elif (control_instance.isStuck()):
		return "stuck"
	elif ((state == "moveInwards" and control_instance.ls_instance.active(3)) or (state == "moveOutwards" and control_instance.ls_instance.active(1))):
		if (state == "moveInwards"):
			control_instance.motor_control.setMotorDirection(1,1)
		else: 
			control_instance.motor_control.setMotorDirection(1,-1)
		print("Hit limit switch, moving away from it.")
		while (control_instance.ls_instance.active(1) or control_instance.ls_instance.active(3)):
			control_instance.updatePosition()
			if (abs(control_instance.encoder_instance.last_tick_diff1) < 60):
                                control_instance.motor_control.setMotorSpeed(GANTRY_ROBOT, 60)
			elif (abs(control_instance.encoder_instance.last_tick_diff1) < 500):
				control_instance.motor_control.setMotorSpeed(GANTRY_ROBOT, 35)
			else:
				control_instance.motor_control.setMotorSpeed(GANTRY_ROBOT, 20)
			time.sleep(0.05)
		control_instance.stop()		

		if (state == "moveInwards"):
			control_instance.r2 = control_instance.r2_min
			control_instance.encoder_instance.set_position(GANTRY_ROBOT, control_instance.r2_min)
		else: 
			control_instance.r2 = control_instance.r2_max
			control_instance.encoder_instance.set_position(GANTRY_ROBOT, control_instance.r2_max)
		if (mode == "Deploy"):
			return "moveAlongRing"
		elif (mode == "Detatch"):
			return "moveAlongRingBack"

	elif (control_instance.ls_instance.anyActive()):
		stateToRevertBackTo = state
		return "errorLimitSwitch"
	
	elif (state == "moveInwards" or state == "moveOutwards"):
		if (mode == "Deploy"):
			return "moveAlongRing"
		elif (mode == "Detatch"):
			return "moveAlongRingBack"

	elif (state == "moveAlongRing"):
		if (abs(control_instance.r2_ref - control_instance.r2_max) < 0.2):
			return "tightenRopeInwards"
		elif (abs(control_instance.r2_ref - control_instance.r2_min) < 0.2):
			return "tightenRopeOutwards"
		else:
			print("Reference: ", control_instance.r2_ref)
			return False
	
	elif (state == "moveAlongRingBack"):
		if (abs(control_instance.r2_ref - control_instance.r2_max) < 0.2):
			return "moveInwards"
		elif (abs(control_instance.r2_ref - control_instance.r2_min) < 0.2):
			return "moveOutwards"
		else:
			print("Reference: ", control_instance.r2_ref)
			return False

	elif (state == "tightenRopeInwards"):
		print("We are in state ", state, "and are stuck. Proceeding to the next state")
		control_instance.stop()
		control_instance.motor_control.openGrip()
		time.sleep(1.5)
		return "moveInwards"
	
	elif (state == "tightenRopeOutwards"):
		print("We are in state ", state, "and are stuck. Proceeding to the next state")
		control_instance.stop()
		control_instance.motor_control.openGrip()
		time.sleep(1.5)
		return "moveOutwards"
	
	
	
