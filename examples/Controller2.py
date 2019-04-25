
import PID

import time
import TrajectoryPlanning as tp
import matplotlib.pyplot as plt
import Geometry

#import Encoder
#import MotorControl
import SPOKe_IO



GANTRY_ROBOT = 1
RING_ROBOT = 2

encoder_instance = SPOKe_IO.Encoder_input()
encoder_instance.reset_counter(1) 
encoder_instance.update_counter(1)

r2 = encoder_instance.read_counter_deg(GANTRY_ROBOT)		# Don't need?
theta4 = encoder_instance.read_counter_deg(RING_ROBOT)		# Don't need?


motor_control = SPOKe_IO.Motor_output()		# Instance for motor control



# For plots
time_list = []
measurement_list_gantry = []  
reference_list_gantry = []    
measurement_list_ring = []
reference_list_ring = []

def PID_to_control_input(pid_output):
	if pid_output >= 0:
		direction = 1
	else:
		direction = -1
	pid_output = abs(pid_output)/20
	return [direction, min(pid_output, 1)]

def invert_PWM(pwm_in):
	return abs(pwm_in - 1)

def mode1():
	# Fixing time
	tstart = round(time.time(),2)
	op_time = (round(time.time(),2) - tstart)


	# Controller initialization
	P = 10
	I = 1
	D = 0.2

	pid_gantry = PID.PID(P, I, D)
	pid_gantry.setSampleTime(0.1)
	
	pid_ring = PID.PID(P, I, D)
	pid_ring.setSampleTime(0.1)

	# Trajectory variables:
	t0 = 0
	tf = 20
	r2_max = 1.5
	r2 = encoder_instance.read_counter_deg(GANTRY_ROBOT) # Because we don't know where to start
	[A0, A1, A2, tb] = tp.LSPB(0.1, [r2, 0, r2_max, 0], [t0, tf])
	
	theta4_ref = 0
	
	while(1):
		op_time = (round(time.time(),2) - tstart)
		
		# Update gantry
		encoder_instance.update_counter(GANTRY_ROBOT)
		r2 = Geometry.rad2r2(encoder_instance.read_counter_rad(GANTRY_ROBOT))
		pid_gantry.SetPoint = tp.getLSPB_position(A0, A1, A2, tb, tf, op_time)
		pid_gantry.update(r2)
		[direction, PWM_signal_strength_gantry] = PID_to_control_input(pid_gantry.output)
		motor_control.setMotorDirection(GANTRY_ROBOT, direction)
		motor_control.setMotorSpeed(GANTRY_ROBOT, PWM_signal_strength_gantry)
		
		# Update ring
		encoder_instance.update_counter(RING_ROBOT)
		theta4 = Geometry.rad2theta4(encoder_instance.read_counter_rad(RING_ROBOT))
		pid_ring.SetPoint = theta4_ref # Constant
		pid_gantry.update(theta4)
		[direction, PWM_signal_strength_ring] = PID_to_control_input(pid_ring.output)
		motor_control.setMotorDirection(RING_ROBOT, direction)
		motor_control.setMotorSpeed(RING_ROBOT, PWM_signal_strength_ring)
		
		# History storage
		print("r2: ", round(r2, 4), " | co: ", pid_gantry.output, " | ", direction, " | ", round(PWM_signal_strength_gantry, 4), " | reference: ", pid_gantry.SetPoint)
		time_list.append(op_time)
		measurement_list_gantry.append(r2)
		reference_list_gantry.append(pid_gantry.SetPoint)
		measurement_list_ring.append(theta4)
		reference_list_ring.append(pid_ring.SetPoint)
		
		# Ending
		if (op_time > tf - 0.4):
				return True
		time.sleep(0.1)

def next_theta4(theta4):
	# This function must correspond to controller. Should be in geometry? 
	return theta4 + 10 * 3.14/180 # 10 deg increase
def current_theta4(theta4):
    # This function should maybe be in geometry? Or should current and next be stored in memory
	# and not be functions of current value, but of some 
	return theta4

def main_test():
    # Initialize the trajectory and controller parameters
	control_instance = controller()

	run_start_time = round(time.time(),2)
	#plot_time = (round(time.time(),2) - tstart)

	# State 1:
	[t0, tf, state, theta4_next] = [0, 20, 1, 0]
	control_instance.initNewState(t0, tf, state, theta4_next) # (t0, tf, state, theta4_next)
	while (not control_instance.timeout): # and control_instance.theta4_e < 0.017 and control_instance.r2_e < 0.02): # Only check time when testing
    	# While the trajectory is still moving, theta4_e < 1 deg, r2_e < 2 cm.
		control_instance.updateTrajectory(state)
		control_instance.updatePosition()
		control_instance.updatePID()

		# For plots sake
		time_list.append((round(time.time(),2) - run_start_time))
		measurement_list_gantry.append(control_instance.r2)
		reference_list_gantry.append(control_instance.pid_gantry.SetPoint)
		measurement_list_ring.append(control_instance.theta4)
		reference_list_ring.append(control_instance.pid_ring.SetPoint)
		print(control_instance.pid_gantry.SetPoint)

	print("Done with mode 1")
	# State 2:
	[t0, tf, state, theta4_next] = [0, 10, 2, 10 * 3.14/180] # 10 deg increase
	control_instance.initNewState(t0, tf, state, theta4_next) #Does the PID reset? 
	while (not control_instance.timeout):# and control_instance.theta4_e < 0.017 and control_instance.r2_e < 0.02): # Only check time when testing
    	# While the trajectory is still moving, theta4_e < 1 deg, r2_e < 2 cm.
		control_instance.updateTrajectory(state)
		control_instance.updatePosition()
		control_instance.updatePID() 

		# For plots sake
		time_list.append((round(time.time(),2) - run_start_time))
		measurement_list_gantry.append(control_instance.r2)
		reference_list_gantry.append(control_instance.pid_gantry.SetPoint)
		measurement_list_ring.append(control_instance.theta4)
		reference_list_ring.append(control_instance.pid_ring.SetPoint)
		print(control_instance.pid_ring.SetPoint)
	print("Done with mode 2")
	#print("r2: ", round(r2, 4), " | co: ", pid_gantry.output, " | ", direction, " | ", round(PWM_signal_strength_gantry, 4), " | reference: ", pid_gantry.SetPoint)
	

	
	limit_switch = 0
	#while (not limit_switch):
		# Loop prosessen
	#	controller_process(state) 

class controller:
	def __init__(self):
		#self.tstart
		#self.op_time
		#self.tf
		#self.timeout 
		#self.pid_gantry #init the PID controller here? 
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

	def initNewState(self, t0, tf, state, theta4_next):
		# Fixing time
		self.tstart = round(time.time(),2)
		self.op_time = (round(time.time(),2) - self.tstart)
		self.tf = tf
		self.t0 = 0
		self.timeout = False
		
		# Controller initialization
		if (state == 1 or state == 4):
			[P_g, I_g, D_g] = [10, 1, 0.2]
			[P_r, I_r, D_r] = [10, 1, 0.2]
		elif(state == 2 or state ==5):
			[P_g, I_g, D_g] = [10, 1, 0.2]
			[P_r, I_r, D_r] = [10, 1, 0.2]	

		self.pid_gantry = PID.PID(P_g, I_g, D_g)
		self.pid_gantry.setSampleTime(0.1)
		self.pid_ring = PID.PID(P_r, I_r, D_r)
		self.pid_ring.setSampleTime(0.1)

		
		self.theta4 = encoder_instance.read_counter_deg(RING_ROBOT)
		self.r2 = encoder_instance.read_counter_deg(GANTRY_ROBOT)

		# Pass på å ha kode som senere kan kontrollere to vinkler samtidig. 
		if (state == 1 or state == 4):
			self.theta4_ref = current_theta4(self.theta4) # constant
			if (state == 1):
				self.r2_ref = self.r2_max
			elif (state == 4):
				self.r2_ref = self.r2_min
			[self.A0_gantry, self.A1_gantry, self.A2_gantry, self.tb_gantry] = tp.LSPB(0.1, [self.r2, 0, self.r2_max, 0], [self.t0, self.tf])
		elif (state ==2 or state == 5):
			self.theta4_ref = next_theta4(self.theta4)
			[self.A0_ring, self.A1_ring, self.A2_ring, self.tb_ring] = tp.LSPB(0.02, [theta4, 0, theta4_next, 0], [t0, tf])
			if (state ==2):
				self.r2_ref = self.r2_max
			elif (state ==5):
				self.r2_ref = self.r2_min

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
		return False

	def updatePosition(self):
		# Possible to make this return True or False?  
		encoder_instance.update_counter(GANTRY_ROBOT)
		self.r2 = Geometry.rad2r2(encoder_instance.read_counter_rad(GANTRY_ROBOT))
		encoder_instance.update_counter(RING_ROBOT)
		self.theta4 = Geometry.rad2theta4(encoder_instance.read_counter_rad(RING_ROBOT))

	def updatePID(self):
    	# Necessary to make this return True or False?
		self.pid_gantry.SetPoint = self.r2_ref
		self.pid_gantry.update(self.r2)
		self.r2_e = self.r2_ref - self.r2
		self.pid_ring.SetPoint = self.theta4_ref 
		self.pid_ring.update(self.theta4)
		self.theta4_e = self.theta4_ref - self.theta4

	def setOutput(self):
		# Possible to make this return True or False?
		[direction_gantry, PWM_signal_strength_gantry] = PID_to_control_input(self.pid_gantry.output)
		motor_control.setMotorDirection(GANTRY_ROBOT, direction_gantry)
		motor_control.setMotorSpeed(GANTRY_ROBOT, PWM_signal_strength_gantry)

		[direction_ring, PWM_signal_strength_ring] = PID_to_control_input(self.pid_ring.output)
		motor_control.setMotorDirection(RING_ROBOT, direction_ring)
		motor_control.setMotorSpeed(RING_ROBOT, PWM_signal_strength_ring)


		

def mode2():
	# Fixing time
	tstart = round(time.time(),2)
	op_time = (round(time.time(),2) - tstart)
	
	# Controller initialization
	P = 10
	I = 1
	D = 0.2

	pid_gantry = PID.PID(P, I, D)
	pid_gantry.setSampleTime(0.1)
	
	pid_ring = PID.PID(P, I, D)
	pid_ring.setSampleTime(0.1)

	# Trajectory variables, these should be parameters:
	# t0, tf, state, 
	t0 = 0
	tf = 10
	theta4 = encoder_instance.read_counter_deg(RING_ROBOT)
	theta4_next = next_theta4(theta4)
	
	[A0, A1, A2, tb] = tp.LSPB(0.02, [theta4, 0, theta4_next, 0], [t0, tf])
	
	r2_max = 1.5
	r2_ref = r2_max # 1.5
	
	while(1):
		op_time = (round(time.time(),2) - tstart)
		
		# Update gantry
		encoder_instance.update_counter(GANTRY_ROBOT)
		r2 = Geometry.rad2r2(encoder_instance.read_counter_rad(GANTRY_ROBOT))
		pid_gantry.SetPoint = r2_ref # constant
		pid_gantry.update(r2)
		[direction, PWM_signal_strength_gantry] = PID_to_control_input(pid_gantry.output)
		motor_control.setMotorDirection(GANTRY_ROBOT, direction)
		motor_control.setMotorSpeed(GANTRY_ROBOT, PWM_signal_strength_gantry)
		
		# Update ring
		encoder_instance.update_counter(RING_ROBOT)
		theta4 = Geometry.rad2theta4(encoder_instance.read_counter_rad(RING_ROBOT))
		pid_ring.SetPoint = tp.getLSPB_position(A0, A1, A2, tb, tf, op_time) 
		pid_gantry.update(theta4)
		[direction, PWM_signal_strength_ring] = PID_to_control_input(pid_ring.output)
		motor_control.setMotorDirection(RING_ROBOT, direction)
		motor_control.setMotorSpeed(RING_ROBOT, PWM_signal_strength_ring)
		
		
		
		
		# History storage
		print("Deg: ", round(theta4, 4), " | co: ", pid_ring.output, " | ", direction, " | ", round(PWM_signal_strength_ring, 4), " | reference: ", pid_ring.SetPoint)
		time_list.append(op_time)
		measurement_list_gantry.append(r2)
		reference_list_gantry.append(pid_gantry.SetPoint)
		measurement_list_ring.append(theta4)
		reference_list_ring.append(pid_ring.SetPoint)
		
		# Ending
		if (op_time > tf - 0.4):
				return True
		time.sleep(0.1)

#if (mode1()):
#	print("Mode 1 time run out")
	
#if (mode2()):
#	print("Mode 2 time run out")

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
	
	
	
	
	
	
