import pymonarco_hat as plc
import RPi.GPIO as GPIO
import time


#import sys # Needed only for the path to libmonarco
#lib_path = sys.argv[1]

GPIO.setmode(GPIO.BOARD) # Use the physical naming convention
lib_path = '../monarco-c/libmonarco.so'

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
		
		self.plc_handler = plc.Monarco(lib_path, debug_flag=plc.MONARCO_DPF_WRITE | plc.MONARCO_DPF_VERB | plc.MONARCO_DPF_ERROR | plc.MONARCO_DPF_WARNING)

		# Initialize the pins
		GPIO.setup(motor1Pin1, GPIO.OUT, initial = 0)
		GPIO.setup(motor1Pin2, GPIO.OUT, initial = 0)
		GPIO.setup(motor2Pin1, GPIO.OUT, initial = 0)
		GPIO.setup(motor2Pin2, GPIO.OUT, initial = 0)


		self.plc_handler.set_pwm_frequency(plc.PWM_CHANNEL1, 1000)
		self.plc_handler.set_pwm_out(plc.DOUT2, 1)


	# motor number [1,2], direction [-1, 0, 1]
	def setMotorDirection(self, motorNumber, direction):
		Success = True
		if (motorNumber == ringRobot):
			pin1 = motor1Pin1
			pin2 = motor1Pin2
		elif (motorNumber == ringRobot):
			pin1 = motor2Pin1
			pin2 = motor2Pin2
		else:
			Success = False
		
		if (direction == 1):
			GPIO.output(pin1, GPIO.HIGH)
			GPIO.output(pin2, GPIO.LOW)
		elif (direction == -1):
			GPIO.output(pin1, GPIO.LOW)
			GPIO.output(pin2, GPIO.HIGH)
		else:
			GPIO.output(pin1, GPIO.LOW)
			GPIO.output(pin2, GPIO.LOW)
			
		return Success;
		
		
	# motor number [1,2], speedValue in range [-1, 1]
	def setMotorSpeed(self, motorNumber, speedValue):
		if (1 < speedValue or speedValue < -1):
			return False 
			
		if (motorNumber == 1):
			self.plc_handler.set_pwm_out(plc.DOUT2, speedValue)
		elif (motorNumber == 2):
			self.plc_handler.set_pwm_out(plc.DOUT4, speedValue)
		else:
			return False
		
	def closeGrip(self):
		return False

	def openGrip(self):
		return False

iterator = 0.25
test = 0
if (test == 1):
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

	




	
