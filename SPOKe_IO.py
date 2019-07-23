import pymonarco_hat as plc
import RPi.GPIO as GPIO
import time
import SPOKe_Geometry


		
class Encoder_input: 
	# Encoder readings
	def __init__(self, plc_handler):
		self.plc_handler = plc_handler
		# Constants: 
		self.gear_reduction = 23 
		self.encoder_precision = 500 
		self.tickMultiplier = 4 

		dimensions = SPOKe_Geometry.Dimensions()

		self.gear_radius1 = dimensions.r_m1
		self.gear_radius2 = dimensions.r_m2

		self.local_counter1 = 0
		self.last_received1 = 0
		self.last_tick_diff1 = 0
		self.last_counter1 = 0

		self.local_counter2 = 0
		self.last_received2 = 0
		self.last_tick_diff2 = 0
		self.last_counter2 = 0

		#The monarco counter can only count to 65,535
		# We can count a total of 46 000 ticks for each rotation.
		# This gives a precision of 0.0078 degrees
		# We only need a precision of 0.36 degrees 
		# We therefore only want to count 1000
		# We therefore only count every 46th tick.
		self.counterDownScalingFactor = 46
		self.counterScalingRest1 = 0
		self.counterScalingRest2 = 0

		# (counter_identifier, mode, edge_count)
		GPIO.setmode(GPIO.BOARD)
		
		plc_handler.initiate_counter(1, 'QUAD', 'NONE') # Test to write "RISE"
		self.reset_counter(1)
		self.index1SignalPort = 7
		GPIO.setup(self.index1SignalPort, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
		self.firstZTickValue1 = 0
		self.ZCount1 = 0


		plc_handler.initiate_counter(2, 'QUAD', 'RISE') # Test to write "RISE"
		self.reset_counter(2)
		self.index2SignalPort = 12
		GPIO.setup(self.index2SignalPort, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
		self.firstZTickValue2 = 0
		self.ZCount2 = 0

	def findFirstZ(self, counter_identifier):
		if (counter_identifier == 1):
			if (GPIO.wait_for_edge(self.index1SignalPort, GPIO.RISING, timeout=10000) is not None):
				GPIO.remove_event_detect(self.index1SignalPort)
				self.update_counter(counter_identifier)
				self.firstZTickValue1 = self.local_counter1
				GPIO.add_event_detect(self.index1SignalPort, GPIO.RISING, callback=self.receivedIndex1Value, bouncetime=2)
				print('FOUND: ', self.firstZTickValue1)
				return True
		
		elif (counter_identifier == 2):
			if (GPIO.wait_for_edge(self.index2SignalPort, GPIO.RISING, timeout=10000) is not None):
				GPIO.remove_event_detect(self.index2SignalPort)
				self.update_counter(counter_identifier)
				self.firstZTickValue2 = self.local_counter2
				GPIO.add_event_detect(self.index2SignalPort, GPIO.RISING, callback=self.receivedIndex2Value, bouncetime=2)
				print('FOUND: ', self.firstZTickValue2)
				return True
		return False
		

	def receivedIndex1Value(self, channel):
		self.update_counter(1)
		diff = self.local_counter1 - self.firstZTickValue1 - self.ZCount1*46000/(self.counterDownScalingFactor * self.gear_reduction)
		#print('Diff = ', self.local_counter1, ' - ', self.firstZTickValue1, ' - ', self.ZCount1*46000/(self.counterDownScalingFactor * self.gear_reduction), ' = ', diff)

		if (diff > 700/self.gear_reduction):
#			print('Rotation Forwards')
			self.ZCount1 +=1
		elif (diff < -700/self.gear_reduction):
#			print("Back to last position")
			self.ZCount1 -=1
#		else: 
#			print('Back to earlier position')
		self.local_counter1 =  self.firstZTickValue1 + self.ZCount1*46000/(self.counterDownScalingFactor * self.gear_reduction)
		self.counterScalingRest1 = 0

	def receivedIndex2Value(self, channel):
		self.update_counter(2)
		diff = self.local_counter2 - self.firstZTickValue2 - self.ZCount2*46000/(self.counterDownScalingFactor * self.gear_reduction)
		print('Diff = ', self.local_counter2, ' - ', self.firstZTickValue2, ' - ', self.ZCount2*46000/(self.counterDownScalingFactor * self.gear_reduction), ' = ', diff)

		if (diff > 700/self.gear_reduction):
#			print('Rotation Forwards')
			self.ZCount2 +=1
		elif (diff < -700/self.gear_reduction):
#			print("Rotation Backwards")
			self.ZCount2 -=1
#		else:
#			print("Back to last position")
		self.local_counter2 =  self.firstZTickValue2 + self.ZCount2*46000/(self.counterDownScalingFactor * self.gear_reduction)
		self.counterScalingRest2 = 0


	def update_counter(self, counter_identifier):
		if (counter_identifier == 1):
			new_value = self.plc_handler.read_counter(1)
			if abs(new_value - self.last_received1) < 23000:
				# We have moved a reasonable length (a half rotation)
				increase = new_value - self.last_received1
			elif new_value < self.last_received1:
				# We have probably passed the storage limit
				print('Probably moved past storage limit gantry')
				increase = new_value - self.last_received1 + 65536
				print(new_value, ' - ', self.last_received1, ' + ', 65536, ' = ', increase)

			elif new_value > self.last_received1:
				# We have probably went backwards past zero
				print('Probably moved backwards past zero gantry')
				increase = new_value - self.last_received1 - 65536
				print(new_value, ' - ', self.last_received1, ' - ', 65536, ' = ', increase)
				
			self.counterScalingRest1 += increase % self.counterDownScalingFactor
			restOverflow = self.counterScalingRest1 // self.counterDownScalingFactor
			increase += restOverflow
			self.last_tick_diff1 = increase
			self.counterScalingRest1 -= restOverflow * self.counterDownScalingFactor
			self.local_counter1 +=  increase // self.counterDownScalingFactor
			self.last_received1 = new_value
					
		elif (counter_identifier == 2):
			new_value = self.plc_handler.read_counter(2)			
			if abs(new_value - self.last_received2) < 23000: 
				# We have moved a reasonable length (half a rotation)
				increase = new_value - self.last_received2
			elif new_value < self.last_received2:
				# We have probably passed the storage  limit
				print('Probably moved past storage limit ring')
				increase = new_value - self.last_received2 + 65535
			elif new_value > self.last_received2:
				# We have probably went backwards past zero
				print('Probably moved backwards past zero ring')
				increase = new_value - self.last_received2 - 65535

			self.counterScalingRest2 += increase % self.counterDownScalingFactor
			restOverflow = self.counterScalingRest2 // self.counterDownScalingFactor
			increase += restOverflow
			self.last_tick_diff2 = increase
			self.counterScalingRest2 -= restOverflow * self.counterDownScalingFactor
			self.local_counter2 +=  increase // self.counterDownScalingFactor
			self.last_received2 = new_value

	def readCounterValue(self,counter_identifier):
		return self.plc_handler.read_counter(counter_identifier)

	def reset_counter(self, counter_identifier):
		self.update_counter(counter_identifier)
		if (counter_identifier == 1):
			self.local_counter1 = 0
		elif (counter_identifier == 2):
			self.local_counter2 = 0
			
	def read_counter_rad(self, counter_identifier):
		self.update_counter(counter_identifier)
		if (counter_identifier == 1):
			return self.local_counter1 * 2 * 3.14 *self.counterDownScalingFactor / (self.gear_reduction * self.encoder_precision * self.tickMultiplier)
		elif (counter_identifier == 2):
			return self.local_counter2 * 2 * 3.14 *self.counterDownScalingFactor / (self.gear_reduction * self.encoder_precision * self.tickMultiplier)
		
	def read_counter_deg(self, counter_identifier):
		self.update_counter(counter_identifier)
		if (counter_identifier == 1):
			return self.local_counter1 * 360 * self.counterDownScalingFactor / (self.gear_reduction * self.encoder_precision * self.tickMultiplier)
		elif (counter_identifier == 2):
			return self.local_counter2  * 360 * self.counterDownScalingFactor / (self.gear_reduction * self.encoder_precision * self.tickMultiplier)


# Robot definition
ringRobot = 1
gantryRobot = 2

# Define the pins
motor1Pin1 = 15
motor1Pin2 = 22
motor2Pin1 = 13
motor2Pin2 = 11

limitSwitchPin1 = 32
limitSwitchPin2 = 33
limitSwitchPin3 = 36
limitSwitchPin4 = 35

class Motor_output:
	def __init__(self, plc_handler):
		self.plc_handler = plc_handler
		GPIO.setmode(GPIO.BOARD) # Use the physical naming convention
		
		# Initialize Raspberry pins
		GPIO.setup(motor1Pin1, GPIO.OUT, initial = 0)
		GPIO.setup(motor1Pin2, GPIO.OUT, initial = 0)
		GPIO.setup(motor2Pin1, GPIO.OUT, initial = 0)
		GPIO.setup(motor2Pin2, GPIO.OUT, initial = 0)
		
		# Initialize pwm channels
		self.plc_handler.set_pwm_frequency(plc.PWM_CHANNEL1, 1000)
		self.plc_handler.set_pwm_frequency(plc.PWM_CHANNEL2, 50)
		self.plc_handler.set_pwm_out(plc.DOUT2, 1) #Assuming 1 is off
		self.plc_handler.set_pwm_out(plc.DOUT1, 1)
		self.openGrip()

		
		
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
		speedValue = invert_PWM(speedValue)

		if (motorNumber == 1):
			self.plc_handler.set_pwm_out(plc.DOUT1, speedValue)
		elif (motorNumber == 2):
			self.plc_handler.set_pwm_out(plc.DOUT2, speedValue)
		else:
			return False
		
	def closeGrip(self):
		self.plc_handler.set_pwm_out(plc.DOUT4, 0.98)
		return False

	def openGrip(self):
		self.plc_handler.set_pwm_out(plc.DOUT4, 0.95)
		return False

def invert_PWM(pwm_in):
	# Required as pwm = 1 is off, while pwm = 0 is off, and we want the opposite
	return abs(pwm_in - 1)

class LimitSwitch():

	# Assuming low represents active switch
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
