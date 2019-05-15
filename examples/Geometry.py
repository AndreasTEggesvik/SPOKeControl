
import math
# Constants, lengths are in metres, angles in rads

r3 = 0.5 
r4 = 2 
l_total = 1.64 # This value is too low
r_gear_gantry = 19 / 1000
r_gear_rotary = 19 / 1000

phi4 = math.acos((r3**2 + r4**2 - l_total**2) / (2*r3*r4))
phi1 = math.pi - phi4


########### Helper functions ############

def rad2theta4(encoder_angle_rad):
	global r_gear_rotary
	return encoder_angle_rad * r_gear_rotary / r4
	
def rad2r2(encoder_angle_rad):
	global r_gear_rotary
	return encoder_angle_rad * r_gear_rotary
	
def phi3(r2):
	global r1, r3
	return math.acos((r1**2 + r3**2 - r2**2)/(2*r1*r3))


def theta1(theta_4, r2):
	global phi4
	return theta_4 + phi3(r2) - phi4
	
def r1(r2):
	global r3, phi1
	return math.sqrt(r2**2 + r3**2 -2*r2*r3*math.cos(phi1))
	
def r2(r1):
	global r3, phi1
	return r3 * math.cos(phi1) + math.sqrt(r1**2 - r3**2 + r3**2 * math.cos(phi1)**2)
	
def theta4(theta1):
	global phi4
	return theta1 + phi3 - phi3(r2)
	
def cartesian2polar(cartesianInput):
	[x,y] = cartesianInput
	return [math.atan2(y,x), math.sqrt(x**2 + y**2)]
	
def polar2cartesian(polarInput):
	[r, theta] = polarInput
	return [r*math.cos(theta), r*math.sin(theta)]

class Dimensions:
	# Dimensions given in metres and radians
	def __init__(self):
		self.r3 = 0.519 
		self.r4 = 2.01
		self.l_total = 1.620
		self.l_rail = 1.711

		self.phi4 = math.acos((self.r3**2 + self.r4**2 - self.l_total**2) / (2*self.r3*self.r4))
		self.phi1 = math.pi - self.phi4

		self.r2Min = 0
		#self.r2Max = self.r3*math.cos(self.phi1)*math.sqrt(self.r3**2*math.cos(self.phi1)**2 - r3**2 + r4**2)
		self.r2Max = 1.5
		self.theta4Min = 0.000001
		self.theta4Max = math.pi/2

		# Motor gears: 
		r_m1 = 0.10 
		r_m2 = 0.10

		# Rope connection point dimensions:
		self.l1 = 0.156 
		self.l2 = 0.059
		self.alpha1 = self.l1/self.r4 # Read thise values in Controller
		self.alpha2 = self.l2/self.r4

	
##########################################

def getSPOKeCoordinates(encoderInput):
	[encoder_angle_rad_rotary, encoder_angle_rad_gantry] = encoderInput
	return [rad2theta4(encoder_angle_rad_rotary), rad2r2(encoder_angle_rad_gantry)]
	
def getPolarCoordinates(encoderInput):
	[encoder_angle_rad_rotary, encoder_angle_rad_gantry] = encoderInput
	theta4 = rad2theta4(encoder_angle_rad_rotary)
	r2 = rad2r2(encoder_angle_rad_gantry)
	return [theta1(theta4, r2), r1(r2)]
	