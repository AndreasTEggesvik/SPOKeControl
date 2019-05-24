

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
	continuing = False

	t0 = 0
	i = 0 


	#while ((not control_instance.timeout) and (stopButtonPressed.value == 0)): # and (not control_instance.isStuck())):# and (not control_instance.ls_instance.anyActive())): # and control_instance.theta4_e > 0.017 and control_instance.r2_e > 0.02): 
		# Only check time when testing while the trajectory is still moving, theta4_e < 1 deg, r2_e < 2 cm.
	while(1):
#		control_instance.updatePosition()
		if (i == 15):
			#print("r2 value = ", control_instance.r2, " | theta4 value = ", control_instance.theta4)
			print("Degrees: ", control_instance.encoder_instance.read_counter_deg(1, control_instance.plc_handler), ' ( ', control_instance.encoder_instance.local_counter1, ')' )
			#print("Ticks: ", control_instance.encoder_instance.readCounterValue(1, control_instance.plc_handler))

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
		self.theta4_ref = 0
		#self.theta4_e
		#self.r2
#		self.r2_max = self.dimensions.r2Max
#		self.r2_min = self.dimensions.r2Min
		self.r2_ref = 0
		#self.r2_e

		self.timeDiffBuffer = collections.deque(maxlen=5)
		self.tickDiffBuffer1 = collections.deque(maxlen=5)
		self.tickDiffBuffer2 = collections.deque(maxlen=5)







	# MUST BE TESTED BEFORE FIRST RUN: is PWM == 0 full throttle or full stop? 
	def stop(self):
		return True
	#	self.motor_control.setMotorSpeed(GANTRY_ROBOT, 0)
	#	self.motor_control.setMotorSpeed(RING_ROBOT, 0)