
import math
# Constants, lengths are in metres, angles in rads
 
r4 = 1.966 
l_total = 1.56
r_gear_gantry = 26.1 / 1000 # 2.86 cm is an estimate based on test
r_gear_rotary = 20 / 1000


########### Helper functions ############

def rad2theta4(encoder_angle_rad):
	global r_gear_rotary
	return encoder_angle_rad * r_gear_rotary / r4
	
def rad2r2(encoder_angle_rad):
	global r_gear_gantry
	return encoder_angle_rad * r_gear_gantry

def theta4TOrad(theta4):
	return theta4 * r4 /r_gear_rotary

def r2TOrad(r2):
	return r2 /r_gear_gantry
	

class Dimensions:
	# Dimensions given in metres and radians
	def __init__(self):
		self.r3 = 0.519 
		self.r4 = 2.01
		self.l_total = 1.565
		self.l_rail = 1.711

		self.phi4 = math.acos((self.r3**2 + self.r4**2 - self.l_total**2) / (2*self.r3*self.r4))
		self.phi1 = math.pi - self.phi4

		self.r2Min = 0
		#self.r2Max = self.r3*math.cos(self.phi1)*math.sqrt(self.r3**2*math.cos(self.phi1)**2 - r3**2 + r4**2)
		# self.r2Max = 1.5
		self.r2Max = 1.63 
		self.theta4Min = 0.0087 # 0.5 deg
		self.theta4Max = 8 * 0.1588

		# Motor gears: 
		self.r_m1 = 0.10 
		self.r_m2 = 0.10

		# Rope connection point dimensions:
		self.l1 = 0.156 
		self.l2 = 0.059
		self.alpha1 = self.l1/self.r4 # Read thise values in Controller
		self.alpha2 = self.l2/self.r4

		self.angularMovementState_1_4 = - 0.5 * 0.1588
		self.angularMovementState_2_5 = 0.1588 # Distance from one clam to the next
		self.initialAngularMovement = 0.1348 #0.1358 # Distance to move from the limit switch to the middle of the first two clams
		
