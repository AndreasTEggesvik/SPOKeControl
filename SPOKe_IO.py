import pymonarco_hat as plc
import RPi.GPIO as GPIO
import SPOKe_Geometry

import time

GPIO.setwarnings(False)

		
class Encoder_input: 
	# Encoder readings
	def __init__(self, plc_handler):
		self.plc_handler = plc_handler
		# Constants: 
		self.gear_reduction = 23 
		self.encoder_precision = 500 
		self.tickMultiplier = 4 

		self.local_counter1 = 0
		self.last_received1 = 0
		self.last_tick_diff1 = 0
		self.last_counter1 = 0

		self.local_counter2 = 0
		self.last_received2 = 0
		self.last_tick_diff2 = 0
		self.last_counter2 = 0

#############################################################
#	The monarco counter can only count to 65,535			#
#	We can count a total of 46 000 ticks for each rotation.	#
# 	This gives a precision of 0.0078 degrees				#
#	By only counting every 46th tick, we count 1000 tick	#
#		per rotation, a precision of 0.36 deg.				#
# 	I was however not capable of doing that					#
#############################################################

		self.counterDownScalingFactor = 46
		self.counterScalingRest1 = 0
		self.counterScalingRest2 = 0

		GPIO.setmode(GPIO.BOARD)
		
		self.reset_counter(1)
		self.index1SignalPort = 7
		GPIO.setup(self.index1SignalPort, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
		self.firstZTickValue1 = 0
		self.ZCount1 = 0

		self.reset_counter(2)
		self.index2SignalPort = 12
		GPIO.setup(self.index2SignalPort, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
		self.firstZTickValue2 = 0
		self.ZCount2 = 0



	def update_counter(self, counter_identifier):
		if (counter_identifier == 1):
			new_value = self.plc_handler.read_counter(1)
			if abs(new_value - self.last_received1) < 23000: 
				# We have moved a reasonable length (half a rotation)
				increase = new_value - self.last_received1
				#print('Increase in counter 2 is ', increase)
			elif new_value < self.last_received1:
				# We have probably passed the storage  limit
				print('Probably moved past storage limit ring')
				increase = new_value - self.last_received1 + 65535
			elif new_value > self.last_received1:
				# We have probably went backwards past zero
				print('Probably moved backwards past zero ring')
				increase = new_value - self.last_received1 - 65535
			self.last_tick_diff1 = increase
			self.local_counter1 +=  increase
			self.last_received1 = new_value

		elif (counter_identifier == 2):
			new_value = self.plc_handler.read_counter(2)
			if abs(new_value - self.last_received2) < 23000: 
				# We have moved a reasonable length (half a rotation)
				increase = new_value - self.last_received2
				#print('Increase in counter 2 is ', increase)
			elif new_value < self.last_received2:
				# We have probably passed the storage  limit
				print('Probably moved past storage limit ring')
				increase = new_value - self.last_received2 + 65535
			elif new_value > self.last_received2:
				# We have probably went backwards past zero
				print('Probably moved backwards past zero ring')
				increase = new_value - self.last_received2 - 65535
			self.last_tick_diff2 = increase
			self.local_counter2 +=  increase
			self.last_received2 = new_value

	def readCounterValue(self,counter_identifier):
		return self.plc_handler.read_counter(counter_identifier)

	def reset_counter(self, counter_identifier):
		self.update_counter(counter_identifier)
		if (counter_identifier == 1):
			self.local_counter1 = 0
		elif (counter_identifier == 2):
			self.local_counter2 = 0

	def set_position(self, counter_identifier, value):
		if (counter_identifier == 1):
#			self.local_counter1 = SPOKe_Geometry.r2TOrad(value) * self.gear_reduction * self.encoder_precision * self.tickMultiplier / (2 * 3.14 *self.counterDownScalingFactor)
			self.local_counter1 = SPOKe_Geometry.r2TOrad(value) * self.gear_reduction * self.encoder_precision * self.tickMultiplier / (2 * 3.14 )
		elif (counter_identifier == 2):
			self.local_counter2 = SPOKe_Geometry.theta4TOrad(value) * self.gear_reduction * self.encoder_precision * self.tickMultiplier / (2*3.14)
			
	def read_counter_rad(self, counter_identifier):
		self.update_counter(counter_identifier)
		if (counter_identifier == 1):
			return self.local_counter1 * 2 * 3.14  / (self.gear_reduction * self.encoder_precision * self.tickMultiplier)
		elif (counter_identifier == 2):
			return self.local_counter2 * 2 * 3.14  / (self.gear_reduction * self.encoder_precision * self.tickMultiplier)
		
	def read_counter_deg(self, counter_identifier):
		self.update_counter(counter_identifier)
		if (counter_identifier == 1):
			return self.local_counter1 * 360  / (self.gear_reduction * self.encoder_precision * self.tickMultiplier)
		elif (counter_identifier == 2):
			return self.local_counter2  * 360 / (self.gear_reduction * self.encoder_precision * self.tickMultiplier)
			
	def getTickDiff(self, counter_identifier):
		if (counter_identifier == 1):
			return self.last_tick_diff1
		elif (counter_identifier == 2):
			return self.last_tick_diff2


# Robot definition
gantryRobot = 1
ringRobot = 2

# Define the pins
motor1Pin1 = 15
motor1Pin2 = 22
motor2Pin1 = 13
motor2Pin2 = 11

limitSwitchPin1 = 32
limitSwitchPin2 = 33
limitSwitchPin3 = 36
limitSwitchPin4 = 35

motor1pwmPin = 18
motor2pwmPin = 16

class Motor_output:
	def __init__(self, plc_handler):
		self.plc_handler = plc_handler
		GPIO.setmode(GPIO.BOARD) # Use the physical naming convention
		
		# Initialize Raspberry pins
		GPIO.setup(motor1Pin1, GPIO.OUT, initial = 0)
		GPIO.setup(motor1Pin2, GPIO.OUT, initial = 0)
		GPIO.setup(motor2Pin1, GPIO.OUT, initial = 0)
		GPIO.setup(motor2Pin2, GPIO.OUT, initial = 0)
		
		# Initialize pwm channels, frequency 100, duty cycle 0
		GPIO.setup(motor1pwmPin, GPIO.OUT, initial = 0)
		GPIO.setup(motor2pwmPin, GPIO.OUT, initial = 0)
		self.M1 = GPIO.PWM(motor1pwmPin, 100) 
		self.M2 = GPIO.PWM(motor2pwmPin, 100)
		self.M1.start(0) 		
		self.M2.start(0) 
		self.plc_handler.set_pwm_frequency(plc.PWM_CHANNEL1, 50)
		self.openGrip()

		
		
	# motor number [1,2], direction [-1, 0, 1]
	def setMotorDirection(self, motorNumber, direction):
		if (motorNumber == gantryRobot):
			pin1 = motor1Pin2
			pin2 = motor1Pin1
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
		return True
		
		
	# motor number [1,2], speedValue in range [0, 100]
	def setMotorSpeed(self, motorNumber, speedValue):
		if (100 < speedValue or speedValue < 0):
			return False 
		if (motorNumber == 1):
			self.M1.ChangeDutyCycle(speedValue)
		elif (motorNumber == 2):
			self.M2.ChangeDutyCycle(speedValue)
		else:
			return False
		
	def closeGrip(self):
		self.plc_handler.set_pwm_out(plc.DOUT3, 0.98)
		return True

	def openGrip(self):
		self.plc_handler.set_pwm_out(plc.DOUT3, 0.95)
		return True

class LimitSwitch():
	def __init__(self):
		GPIO.setmode(GPIO.BOARD) # Use the physical naming convention
		GPIO.setup(limitSwitchPin1, GPIO.IN, pull_up_down = GPIO.PUD_UP) 
		GPIO.setup(limitSwitchPin2, GPIO.IN, pull_up_down = GPIO.PUD_UP)
		GPIO.setup(limitSwitchPin3, GPIO.IN, pull_up_down = GPIO.PUD_UP)
		GPIO.setup(limitSwitchPin4, GPIO.IN, pull_up_down = GPIO.PUD_UP)

	def active(self, switchNumber):
		# Returns true if the limit switch is active
		pin = -1
		if switchNumber == 1:
			pin = 32
		elif switchNumber == 2:
			pin = 33
		elif switchNumber == 3:
			pin = 35
		elif switchNumber == 4:
			pin = 36
		else:
			return True
		return not bool(GPIO.input(pin))
		
	def anyActive(self):
		return not bool(GPIO.input(32) and GPIO.input(33) and GPIO.input(35) and GPIO.input(36)) 


# Only needed if usin pwm pins from Monarco
def invert_PWM(pwm_in):
	# Required as pwm = 1 is off, while pwm = 0 is off, and we want the opposite
	return abs(pwm_in - 1)
