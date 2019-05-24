

import PID

import time
import TrajectoryPlanning as tp
import collections

import Geometry
from multiprocessing import Process, Pipe, Value, Lock
import SPOKe_IO


def main(graphPipe, graphPipeReceiver, buttonPipe, graphPipeSize, graphLock, stopButtonPressed, newButtonData, operatingTimeConstant):
	# Initializing the robot, guaranteeing a safe starting position
	control_instance = Controller()

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
	def __init__(self):
		import pymonarco_hat as plc
		lib_path = '../../../pymonarco-hat/monarco-c/libmonarco.so'
		self.plc_handler = plc.Monarco(lib_path, debug_flag=plc.MONARCO_DPF_WRITE | plc.MONARCO_DPF_WARNING)	

		self.encoder_instance = SPOKe_IO.Encoder_input(self.plc_handler)
		self.motor_control = SPOKe_IO.Motor_output(self.plc_handler)
		self.ls_instance = SPOKe_IO.LimitSwitch()			