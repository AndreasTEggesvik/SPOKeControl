
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

	# Trajectory variables:
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

if (mode1()):
	print("Mode 1 time run out")
	
if (mode2()):
	print("Mode 2 time run out")
	

motor_control.setMotorDirection(1, 0)
motor_control.setMotorSpeed(1, 1)
	
# PLOT: 
plt.figure()
plt.plot(time_list, measurement_list_gantry, 'b')
plt.plot(time_list, reference_list_gantry, 'r')
plt.show()

plt.figure()
plt.plot(time_list, measurement_list_ring, 'b')
plt.plot(time_list, reference_list_ring, 'r')
plt.show()
	
	
	
	
	
	
