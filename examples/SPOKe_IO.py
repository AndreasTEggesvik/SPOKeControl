
import pymonarco_hat as plc
import RPi.GPIO as GPIO
import time


lib_path = '../../pymonarco-hat/monarco-c/libmonarco.so'
plc_handler = plc.Monarco(lib_path, debug_flag=plc.MONARCO_DPF_WRITE | plc.MONARCO_DPF_VERB | plc.MONARCO_DPF_ERROR | plc.MONARCO_DPF_WARNING)
		
		
class Encoder_input: 
	# Encoder readings
	def __init__(self):
		#lib_path = '../monarco-c/libmonarco.so'
		#lib_path = 'monarco-c/libmonarco.so'
		#self.plc_handler = plc.Monarco(lib_path, debug_flag=plc.MONARCO_DPF_WRITE | plc.MONARCO_DPF_VERB | plc.MONARCO_DPF_ERROR | plc.MONARCO_DPF_WARNING)
		# Constants: 
		self.gear_reduction = 92 # Don't really know
		self.encoder_precision = 500 # Think we know

		# (counter_identifier, mode, edge_count)
		plc_handler.initiate_counter(2, 'QUAD', 'NONE')
		plc_handler.initiate_counter(1, 'QUAD', 'NONE')
		
		# Turning on Encoder power
		plc_handler.set_digital_out(plc.DOUT1, plc.LOW)

		self.local_counter1 = 0
		self.last_received1 = 0

		self.local_counter2 = 0
		self.last_received2 = 0




		#The monarco counter can only count to 65,536


		# Not finished
	def update_counter(self, counter_identifier):
		if (counter_identifier == 1):
			#self.local_counter1
			#self.last_received1
			
			new_value = plc_handler.read_counter(1)
			
			
			if abs(new_value - self.last_received1) < 45000:
				# We have moved a reasonable length (just over 2 rotations)
				self.local_counter1 +=  - self.last_received1 + new_value
			elif new_value < self.last_received1:
				# We have probably passed the storage limit
				self.local_counter1 += 65536 - self.last_received1 + new_value
			elif new_value < self.last_received1:
				# We have probably went backwards passt zero
				self.local_counter1 += new_value - self.last_received1 - 65536
		
			#local_counter1 +=  - last_received1 + new_value
			self.last_received1 = new_value
					
		elif (counter_identifier == 2):
			#global local_counter2
			#global last_received2
			
			new_value = plc_handler.read_counter(2)
			
			
			if abs(new_value - self.last_received2) < 45000:
				# We have moved a reasonable length (just over 2 rotations)
				self.local_counter2 +=  - self.last_received2 + new_value
			elif new_value < self.last_received2:
				# We have probably passed the storage  limit
				self.local_counter2 += 65536 - self.last_received2 + new_value
			elif new_value < self.last_received2:
				# We have probably went backwards passt zero
				self.local_counter2 += new_value - self.last_received2 - 65536
			
			self.last_received2 = new_value
	

	def reset_counter(self, counter_identifier):
		self.update_counter(counter_identifier)
		
		if (counter_identifier == 1):
			self.local_counter1 = 0
		elif (counter_identifier == 2):
			self.local_counter2 = 0
			
	def read_counter_rad(self, counter_identifier):
		self.update_counter(counter_identifier)
		if (counter_identifier == 1):
			return self.local_counter1 * 2 * 3.14 / (self.gear_reduction * self.encoder_precision)
		elif (counter_identifier == 2):
			return self.local_counter2 * 2 * 3.14 / (self.gear_reduction * self.encoder_precision)
		
	def read_counter_deg(self, counter_identifier):
		self.update_counter(counter_identifier)
		if (counter_identifier == 1):
			return self.local_counter1 * 360 / (self.gear_reduction * self.encoder_precision)
		elif (counter_identifier == 2):
			return self.local_counter2 * 360 / (self.gear_reduction * self.encoder_precision)







# Robot definition
ringRobot = 1
gantryRobot = 2

# Define the pins
motor1Pin1 = 15
motor1Pin2 = 22
motor2Pin1 = 13
motor2Pin2 = 11

class Motor_output:
	def __init__(self):
		GPIO.setmode(GPIO.BOARD) # Use the physical naming convention
		#lib_path = '../monarco-c/libmonarco.so'
		#self.plc_handler = plc.Monarco(lib_path, debug_flag=plc.MONARCO_DPF_WRITE | plc.MONARCO_DPF_VERB | plc.MONARCO_DPF_ERROR | plc.MONARCO_DPF_WARNING)

		# Initialize Raspberry pins
		GPIO.setup(motor1Pin1, GPIO.OUT, initial = 0)
		GPIO.setup(motor1Pin2, GPIO.OUT, initial = 0)
		GPIO.setup(motor2Pin1, GPIO.OUT, initial = 0)
		GPIO.setup(motor2Pin2, GPIO.OUT, initial = 0)
		
		# Set voltage switching ground and 3.7V
		groundPin = 32
		GPIO.setup(groundPin, GPIO.OUT, initial = 0)
		GPIO.output(groundPin, GPIO.LOW)
		referenceVoltagePin = 35
		GPIO.setup(referenceVoltagePin, GPIO.OUT, initial = 0)
		GPIO.output(referenceVoltagePin, GPIO.HIGH)
		
		
		
		# Initialize pwm channels
		plc_handler.set_pwm_frequency(plc.PWM_CHANNEL1, 1000)
		plc_handler.set_pwm_out(plc.DOUT2, 1)
		

		
		
	# motor number [1,2], direction [-1, 0, 1]
	def setMotorDirection(self, motorNumber, direction):
		if (motorNumber == ringRobot):
			pin1 = motor1Pin1
			pin2 = motor1Pin2
		elif (motorNumber == ringRobot):
			pin1 = motor2Pin1
			pin2 = motor2Pin2
		else:
			return False
		
		if (direction == 1):
			GPIO.output(pin1, GPIO.HIGH)
			GPIO.output(pin2, GPIO.LOW)
		elif (direction == -1):
			GPIO.output(pin1, GPIO.LOW)
			GPIO.output(pin2, GPIO.HIGH)
		else:
			GPIO.output(pin1, GPIO.LOW)
			GPIO.output(pin2, GPIO.LOW)
			
		return True;
		
		
	# motor number [1,2], speedValue in range [-1, 1]
	def setMotorSpeed(self, motorNumber, speedValue):
		if (1 < speedValue or speedValue < -1):
			return False 
			
		if (motorNumber == 1):
			plc_handler.set_pwm_out(plc.DOUT2, speedValue)
		elif (motorNumber == 2):
			plc_handler.set_pwm_out(plc.DOUT4, speedValue)
		else:
			return False
		
	def closeGrip(self):
		return False

	def openGrip(self):
		return False




###################### TESTS ###########################
import Geometry as geo


test = 0

if (test == 1):
	# Reads counter2 and angle in rad
	encoder_instance = Encoder_input()
	while (1):
		print("DI1-4:", plc_handler.get_digital_in(plc.DIN1), " | Counter 2 [deg]:", encoder_instance.read_counter_deg(2), " | R2 [m]: ", geo.rad2r2(encoder_instance.read_counter_rad(2)))
		time.sleep(1)
	
elif (test == 2):
	# Reads Counter 1 and Counter 2 and angle in deg, resets every 20 count
	encoder_instance = Encoder_input()
	while (1):
		for i in range (0,20):
			encoder_instance.update_counter(1)
			encoder_instance.update_counter(2)
			print("DI1-4:", plc_handler.get_digital_in(plc.DIN1), plc_handler.get_digital_in(plc.DIN2), plc_handler.get_digital_in(plc.DIN3), plc_handler.get_digital_in(plc.DIN4) ,
			" |  Counter 1:", plc_handler.read_counter(1), " (", encoder_instance.read_counter_deg(1), ") |  Counter 2:", plc_handler.read_counter(2), " (", encoder_instance.read_counter_deg(2), ")")
			time.sleep(1)
		encoder_instance.reset_counter(1)
		encoder_instance.reset_counter(2)
		
elif (test == 3):
	# Testing motor direction
	iterator = 0.25
	while 1:
		# Run forwars
		print("Running forwards")
		setMotorDirection(ringRobot, 1)
		time.sleep(1)
	 
		
		# Stop
		print("Stopping")
		setMotorDirection(ringRobot, 0)
		time.sleep(1)
	  
		
		# Run Backwards
		print("Running backwards")
		setMotorDirection(ringRobot, -1)
		time.sleep(1)
