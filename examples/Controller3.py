

import PID

import time
import TrajectoryPlanning as tp
import collections

import Geometry
from multiprocessing import Process, Pipe, Value, Lock
import SPOKe_IO


GANTRY_ROBOT = 1
RING_ROBOT = 2




def main(graphPipe, graphPipeReceiver, buttonPipe, graphPipeSize, graphLock, stopButtonPressed, newButtonData, operatingTimeConstant):
	# Initializing the robot, guaranteeing a safe starting position
	run_start_time = round(time.time(),2)
	control_instance = Controller(run_start_time)

	state = 1
	i = 0

	t0 = 0
	tf = getTf(state, operatingTimeConstant)
	if (not continuing):
		if ( not control_instance.getNextTheta4d(state) ):
			# In case next desired angle is outside working area
			break
	continuing = False
	control_instance.initNewState(t0, tf, state)
	i = 0 
	#while ((not control_instance.timeout) and (stopButtonPressed.value == 0)): # and (not control_instance.isStuck())):# and (not control_instance.ls_instance.anyActive())): # and control_instance.theta4_e > 0.017 and control_instance.r2_e > 0.02): 
		# Only check time when testing while the trajectory is still moving, theta4_e < 1 deg, r2_e < 2 cm.
	while(1):
		control_instance.updateTrajectory(state)
#		control_instance.updatePosition()
		control_instance.updatePID(state)
		control_instance.storeData()
		if (i == 15):
			#print("r2 value = ", control_instance.r2, " | theta4 value = ", control_instance.theta4)
			print("Degrees: ", control_instance.encoder_instance.read_counter_deg(1, control_instance.plc_handler), ' ( ', control_instance.encoder_instance.local_counter1, ')' )
			#print("Ticks: ", control_instance.encoder_instance.readCounterValue(1, control_instance.plc_handler))
			if (graphPipeSize.value == 0):
				control_instance.eraseBufferData()
			control_instance.bufferData()
			control_instance.eraseData()
#			graphCommunication = Process(target=sendData, args=(control_instance, graphPipe, graphPipeSize, graphLock))
#			graphCommunication.start()
			i = 0
		i += 1
		time.sleep(0.02)



class Controller:
	def __init__(self, time_value):
		self.run_start_time = time_value
		self.time_list = [0, 0]
		self.measurement_list_gantry = [0, 0]  
		self.reference_list_gantry = [0, 0]    
		self.measurement_list_ring = [0, 0]
		self.reference_list_ring = [0, 0]
		self.dataBuffer = [self.time_list[:], self.measurement_list_gantry[:], self.reference_list_gantry[:], self.measurement_list_ring[:], self.reference_list_ring[:], -1]

		import pymonarco_hat as plc
		lib_path = '../../../pymonarco-hat/monarco-c/libmonarco.so'
		self.plc_handler = plc.Monarco(lib_path, debug_flag=plc.MONARCO_DPF_WRITE | plc.MONARCO_DPF_WARNING)	

		self.encoder_instance = SPOKe_IO.Encoder_input(self.plc_handler)
		self.motor_control = SPOKe_IO.Motor_output(self.plc_handler)
		self.ls_instance = SPOKe_IO.LimitSwitch()			

		self.dimensions = Geometry.Dimensions()
		self.tstart = round(time.time(),2)
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
	#	self.motor_control.setMotorSpeed(GANTRY_ROBOT, PWM_signal_strength_gantry)

		if (state == 3):
			self.motor_control.setMotorDirection(RING_ROBOT, -1)
	#		self.motor_control.setMotorSpeed(RING_ROBOT, 0.4)
			return True
		elif (state == 6):
			self.motor_control.setMotorDirection(RING_ROBOT, 1)
	#		self.motor_control.setMotorSpeed(RING_ROBOT, 0.4)
			return True
		
		[direction_ring, PWM_signal_strength_ring] = PID_to_control_input(self.pid_ring.output)
		self.motor_control.setMotorDirection(RING_ROBOT, direction_ring)
	#	self.motor_control.setMotorSpeed(RING_ROBOT, PWM_signal_strength_ring)

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

			
	# MUST BE TESTED BEFORE FIRST RUN: is PWM == 0 full throttle or full stop? 
	def stop(self):
		return True
	#	self.motor_control.setMotorSpeed(GANTRY_ROBOT, 0)
	#	self.motor_control.setMotorSpeed(RING_ROBOT, 0)

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