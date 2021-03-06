

#########################################################################
# Stay close to power switch when running this script, and start with 	#
# 	the motor far from the edges										#
#																		#
# This script is made to achieve standing oscillations, and are have	#
# 	not implemented the limit switches. 								#
#																		#	
#																		#
#########################################################################

#### Changable Variables ####
							#
[P, I, D] = [120, 60, 74.4]		#
motorNumber = 1				#
reference = 0.10 				# 
							#
#############################

import PID
import time
import SPOKe_Geometry
import SPOKe_IO
import csv 
import math

def PID_to_control_input(pid_output, encoder):
	if pid_output >= 0:
		direction = 1
	else:
		direction = -1
	pid_output = abs(pid_output)
	if (pid_output < 15):
		pid_output = 0
	elif (abs(encoder.last_tick_diff1) < 2):
		pid_output = 80
	else:
		pid_output = 22 + 6*math.sqrt(max(pid_output-20, 0))
	return [direction, min(pid_output, 100)]


def main():
	startTime = round(time.time(),2)
	global motorNumber, reference
	control_instance = controller(reference, startTime)
	i = 0
#	kickStartMovement(control_instance.motor_control, motorNumber, 20)
	while ((round(time.time(),2) - startTime) < 10):
		control_instance.updatePosition(motorNumber)
		control_instance.updatePID()
		control_instance.setOutput(motorNumber)
		control_instance.storeData()

		if (i == 15):
			[d, pidPower] = PID_to_control_input(control_instance.pid.output, control_instance.encoder_instance)
			print("Position: ", control_instance.position, " | Reference: ", control_instance.reference ," | PID output: ", d* pidPower, " ( ", control_instance.pid.output, " )" )
			i = 0
		i += 1

		time.sleep(0.02)

def kickStartMovement(motor_control, motorNumber, control_signal):
	if (control_signal > 0):
		motor_control.setMotorDirection(motorNumber, 1)
	if (control_signal <= 0):
		motor_control.setMotorDirection(motorNumber, -1)	

	motor_control.setMotorSpeed(motorNumber, 80)
	time.sleep(0.1)
	motor_control.setMotorSpeed(motorNumber, 0)	



class controller:
	def __init__(self, reference, t0):
		global P, I, D
		import pymonarco_hat as plc
		lib_path = '../pymonarco-hat/monarco-c/libmonarco.so'
		self.plc_handler = plc.Monarco(lib_path, debug_flag=plc.MONARCO_DPF_WRITE | plc.MONARCO_DPF_WARNING)	

		self.encoder_instance = SPOKe_IO.Encoder_input(self.plc_handler)
		self.motor_control = SPOKe_IO.Motor_output(self.plc_handler)


		self.startTime = t0
		self.reference = reference
		self.time_list = [0, 0]
		self.measurement_list = [0, 0]  
		self.reference_list = [0, 0]    
		self.pid_list = [0, 0]

		

		self.pid = PID.PID(P, I, D)
		self.pid.setSampleTime(0.02)
		self.encoder_instance.reset_counter(1)
		self.encoder_instance.reset_counter(2)
		
	def __del__(self):
		with open('ProportionalWaves.csv', 'a') as csvFile:
			writer = csv.writer(csvFile)
			for i in range( len(self.time_list)  ):
				writer.writerow([self.time_list[i], self.measurement_list[i], self.reference_list[i], self.pid_list[i]])
		csvFile.close

		

	def updatePosition(self, motorNumber):
		if (motorNumber == 1):
			self.position = SPOKe_Geometry.rad2r2(self.encoder_instance.read_counter_rad(motorNumber))
		elif (motorNumber == 2):
			self.position = SPOKe_Geometry.rad2theta4(self.encoder_instance.read_counter_rad(motorNumber))
	
	def updatePID(self):
		if (abs(self.reference - self.position) < 0.005): # 5 mm
			self.pid.setWindup(0)
		else:
			self.pid.setWindup(20)
		self.pid.SetPoint = self.reference
		self.pid.update(self.position)

	def setOutput(self, motorNumber):
		[direction_ring, PWM_signal_strength_ring] = PID_to_control_input(self.pid.output, self.encoder_instance)
		self.motor_control.setMotorDirection(motorNumber, direction_ring)
		self.motor_control.setMotorSpeed(motorNumber, PWM_signal_strength_ring)
			
	def storeData(self):
		self.time_list.append((round(time.time(),2) - self.startTime))
		self.measurement_list.append(self.position)
		self.reference_list.append(self.reference)
		self.pid_list.append(self.pid.output)

main()
