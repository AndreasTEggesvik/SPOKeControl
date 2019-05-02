

import PID

import time
import TrajectoryPlanning as tp

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

def invert_PWM(pwm_in):
	return abs(pwm_in - 1)

def next_theta4(theta4):
	# This function must correspond to controller. Should be in geometry? 
	return theta4 + 10 * 3.14/180 # 10 deg increase
def current_theta4(theta4):
    # This function should maybe be in geometry? Or should current and next be stored in memory
	# and not be functions of current value, but of some 
	return theta4

def sendData(data_storage, graphPipe,graphPipeSize, graphLock):
	graphLock.acquire()
	print("Sending data of size: ", len(data_storage.dataBuffer[0]))
	graphPipe.send(data_storage.dataBuffer)
	graphPipeSize.value = graphPipeSize.value + 1
	graphLock.release()
	
def main_test(graphPipe, graphPipeReceiver, buttonPipe, graphPipeSize, graphLock, stopButtonPressed, newButtonData):
    
	# Initializing the robot, guaranteeing a safe starting position
	run_start_time = round(time.time(),2)
	control_instance = controller(run_start_time)
	control_instance.waitForInitSignal(buttonPipe)
	if (control_instance.initialize(stopButtonPressed) == False):
		while (True):
			time.sleep(0.5)
	buttonPipe.send("Init finished")
	newButtonData.value += 1


	# Start the normal procedure
	control_instance.waitForStartSignal(buttonPipe)
	buttonPipe.send("Starting")
	newButtonData.value += 1
	control_instance.run_start_time = round(time.time(),2)


	# State 1:
	[t0, tf, state, theta4_next] = [0, 20, 1, 0]
	control_instance.initNewState(t0, tf, state, theta4_next) # (t0, tf, state, theta4_next)

	i = 0
	while (not control_instance.timeout): # and control_instance.theta4_e < 0.017 and control_instance.r2_e < 0.02): # Only check time when testing
    	# While the trajectory is still moving, theta4_e < 1 deg, r2_e < 2 cm.
		control_instance.updateTrajectory(state)
		control_instance.updatePosition()
		control_instance.updatePID(state)
		control_instance.storeData()

		if (i == 15):
			if (graphPipeSize.value == 0):
				control_instance.eraseBufferData()
			control_instance.bufferData()
			control_instance.eraseData()
			graphCommunication = Process(target=sendData, args=(control_instance, graphPipe, graphPipeSize, graphLock))
			graphCommunication.start()
			i = 0
		i += 1

		time.sleep(0.02)
	print("Done with mode 1")

	# State 2:
	[t0, tf, state, theta4_next] = [0, 10, 2, 10 * 3.14/180] # 10 deg increase
	control_instance.initNewState(t0, tf, state, theta4_next) #Does the PID reset? 
	while (not control_instance.timeout):# and control_instance.theta4_e < 0.017 and control_instance.r2_e < 0.02): # Only check time when testing
    	# While the trajectory is still moving, theta4_e < 1 deg, r2_e < 2 cm.
		control_instance.updateTrajectory(state)
		control_instance.updatePosition()
		control_instance.updatePID(state) 
		control_instance.storeData()
		
		if (i == 15):
			if (graphPipeSize.value == 0):
				control_instance.eraseBufferData()
			control_instance.bufferData()
			control_instance.eraseData()
			graphCommunication = Process(target=sendData, args=(control_instance, graphPipe, graphPipeSize, graphLock))
			graphCommunication.start()
			i = 0
		i += 1

		time.sleep(0.02)
	print("Done with mode 2")

	# State 3: 
	control_instance.motor_control.closeGrip()
	[t0, tf, state, theta4_next] = [0, 0, 3, "don't care"]
	control_instance.initNewState(t0, tf, state, theta4_next)
	while (False):
		control_instance.updateTrajectory(state)
		control_instance.updatePosition()
		control_instance.updatePID(state)
		control_instance.storeData()
	
		if (i == 15):
			if (graphPipeSize.value == 0):
				control_instance.eraseBufferData()
			control_instance.bufferData()
			control_instance.eraseData()
			graphCommunication = Process(target=sendData, args=(control_instance, graphPipe, graphPipeSize, graphLock))
			graphCommunication.start()
			i = 0
		i += 1

		time.sleep(0.02)
	control_instance.motor_control.openGrip()

	print("Done with mode 3")
	#print("r2: ", round(r2, 4), " | co: ", pid_gantry.output, " | ", direction, " | ", round(PWM_signal_strength_gantry, 4), " | reference: ", pid_gantry.SetPoint)
	graphCommunication.terminate()
	

class controller:
	def __init__(self, time_value):
		self.run_start_time = time_value
		self.time_list = []
		self.measurement_list_gantry = []  
		self.reference_list_gantry = []    
		self.measurement_list_ring = []
		self.reference_list_ring = []
		self.dataBuffer = [self.time_list[:], self.measurement_list_gantry[:], self.reference_list_gantry[:], self.measurement_list_ring[:], self.reference_list_ring[:]]

		self.encoder_instance = SPOKe_IO.Encoder_input()
		self.motor_control = SPOKe_IO.Motor_output()
		self.ls_instance = SPOKe_IO.LimitSwitch()			

		#self.tstart
		#self.op_time
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
		#self.theta4_ref
		#self.theta4_e
		#self.r2
		self.r2_max = 1.5# Geometry.r2_max
		self.r2_min = 0# Geometry.r2_min
		#self.r2_ref
		#self.r2_e

		# Storage lists
	def initialize(self, stopButtonPressed):
		
		self.motor_control.setMotorDirection(GANTRY_ROBOT, -1)
		self.motor_control.setMotorDirection(RING_ROBOT, -1)
		initVelocity = 0.5

		while( not ( self.ls_instance.active(1) or self.ls_instance.active(2) or self.ls_instance.active(3) or self.ls_instance.active(4) or stopButtonPressed.value )):
			self.motor_control.setMotorSpeed(GANTRY_ROBOT, initVelocity)
			self.motor_control.setMotorSpeed(RING_ROBOT, initVelocity)
			time.sleep(0.05)
		self.stop()
		if (stopButtonPressed.value):
			print("Stop button is pressed, looping forever")
			return False

		while ( not ( self.ls_instance.active(1) or self.ls_instance.active(2) or stopButtonPressed.value )):
			self.motor_control.setMotorSpeed(RING_ROBOT, initVelocity)
			time.sleep(0.05)
		self.stop()
		if (stopButtonPressed.value):
			print("Stop button is pressed, looping forever")
			return False

		while ( not ( self.ls_instance.active(3) or self.ls_instance.active(4) or stopButtonPressed.value )):
			self.motor_control.setMotorSpeed(GANTRY_ROBOT, initVelocity)
			time.sleep(0.05)
		self.stop()
		if (stopButtonPressed.value):
			print("Stop button is pressed, looping forever")
			return False

		self.encoder_instance.reset_counter(1)
		self.encoder_instance.reset_counter(2)
		return True

		
	def initNewState(self, t0, tf, state, theta4_next):
		# Fixing time
		self.tstart = round(time.time(),2)
		self.op_time = (round(time.time(),2) - self.tstart)
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
		
		self.theta4 = self.encoder_instance.read_counter_deg(RING_ROBOT)
		self.r2 = self.encoder_instance.read_counter_deg(GANTRY_ROBOT)
		print("Theta4 at initiation: ", self.theta4)
		print("r2 at initiation: ", self.r2)

		# Pass på å ha kode som senere kan kontrollere to vinkler samtidig. 
		if (state == 1 or state == 4):
			self.theta4_ref = current_theta4(self.theta4) # constant
			if (state == 1):
				self.r2_ref = self.r2_max
			elif (state == 4):
				self.r2_ref = self.r2_min
			[self.A0_gantry, self.A1_gantry, self.A2_gantry, self.tb_gantry] = tp.LSPB(0.1, [self.r2, 0, self.r2_max, 0], [self.t0, self.tf])
		elif (state == 2 or state == 5):
			self.theta4_ref = next_theta4(self.theta4)
			[self.A0_ring, self.A1_ring, self.A2_ring, self.tb_ring] = tp.LSPB(0.02, [self.theta4, 0, theta4_next, 0], [t0, tf])
			if (state == 2):
				self.r2_ref = self.r2_max
			elif (state == 5):
				self.r2_ref = self.r2_min
		#elif (state == 3 or state == 6):
			# don't care about this
	def waitForInitSignal(self, buttonPipe):
		while(True):
			b = buttonPipe.recv()
			if (b == "INIT"):
				print("Initializing robot")
				break
			elif (b == "STOP"):
				print("Controller is being asked to stop. Has not yet started")
				self.stop()
			elif (b == "START"):
				print("Controller can not start, has not been initiated")
			print("Should we sleep now? ")

	def waitForStartSignal(self, buttonPipe):
		while(True):
			b = buttonPipe.recv()
			if (b == "START"):
				print("Starting controller")
				break
			elif (b == "STOP"):
				self.stop()
				print("Stopping, has not yet finished initializing")
			print("Should we sleep now? ")

	def updateTrajectory(self, state):
		self.op_time = (round(time.time(),2) - self.tstart)
		if (self.op_time > self.tf - 0.1):
			self.timeout = True
			return True
		if (state == 1 or state == 4):
			self.r2_ref = tp.getLSPB_position(self.A0_gantry, self.A1_gantry, self.A2_gantry, self.tb_gantry, self.tf, self.op_time)
			self.theta4_ref = self.theta4_ref # constant
			return True
		elif (state == 2 or state == 5):
			self.r2_ref = self.r2_ref # constant
			self.theta4_ref = tp.getLSPB_position(self.A0_ring, self.A1_ring, self.A2_ring, self.tb_ring, self.tf, self.op_time)
			return True
		elif (state ==3 or state == 6):
			self.r2_ref = self.r2
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

	def bufferData(self):
		self.dataBuffer[0].extend(self.time_list[:])
		self.dataBuffer[1].extend(self.measurement_list_gantry[:])
		self.dataBuffer[2].extend(self.reference_list_gantry[:])
		self.dataBuffer[3].extend(self.measurement_list_ring[:])
		self.dataBuffer[4].extend(self.reference_list_ring[:])

	def eraseBufferData(self):
		self.dataBuffer = [[], [], [], [], []]

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

test = 0
if test == 1:
	main_test()
	# PLOT: 
	plt.figure()
	plt.plot(time_list, measurement_list_gantry, 'b')
	plt.plot(time_list, reference_list_gantry, 'r')
	plt.show()

	plt.figure()
	plt.plot(time_list, measurement_list_ring, 'b')
	plt.plot(time_list, reference_list_ring, 'r')
	plt.show()
	
	
