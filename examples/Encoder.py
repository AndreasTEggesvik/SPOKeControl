

import pymonarco_hat as plc
import RPi.GPIO as GPIO
import time

import Geometry as geo

class Encoder_input: 
	# Encoder readings
	def __init__(self):
		lib_path = '../monarco-c/libmonarco.so'
		#lib_path = 'monarco-c/libmonarco.so'
		self.plc_handler = plc.Monarco(lib_path, debug_flag=plc.MONARCO_DPF_WRITE | plc.MONARCO_DPF_VERB | plc.MONARCO_DPF_ERROR | plc.MONARCO_DPF_WARNING)
		# Constants: 
		self.gear_reduction = 92 # Don't really know
		self.encoder_precision = 500 # Think we know

		# (counter_identifier, mode, edge_count)
		self.plc_handler.initiate_counter(2, 'QUAD', 'NONE')
		self.plc_handler.initiate_counter(1, 'QUAD', 'NONE')


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
			
			new_value = self.plc_handler.read_counter(1)
			
			
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
			
			new_value = self.plc_handler.read_counter(2)
			
			
			if abs(new_value - self.last_received2) < 45000:
				# We have moved a reasonable length (just over 2 rotations)
				self.local_counter2 +=  - self.last_received2 + new_value
			elif new_value < self.last_received2:
				# We have probably passed the storage  limit
				self.local_counter2 += 65536 - self.last_received2 + new_value
			elif new_value < last_received2:
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
		

		
		
# Lock memory allocations for realtime performance
#mlockall(MCL_CURRENT|MCL_FUTURE)

test = 0
if (test == 1):
	encoder_instance = Encoder_input()
	while (1):
		print("DI1-4:", encoder_instance.plc_handler.get_digital_in(plc.DIN1), " | Counter 2 [deg]:", encoder_instance.read_counter_deg(2), " | R2 [m]: ", geo.rad2r2(encoder_instance.read_counter_rad(2)))
		time.sleep(1)
	
if (test == 2):
	encoder_instance = Encoder_input()
	while (1):
		for i in range (0,20):
			encoder_instance.update_counter(1)
			encoder_instance.update_counter(2)
			print("DI1-4:", encoder_instance.plc_handler.get_digital_in(plc.DIN1), encoder_instance.plc_handler.get_digital_in(plc.DIN2), encoder_instance.plc_handler.get_digital_in(plc.DIN3), encoder_instance.plc_handler.get_digital_in(plc.DIN4) ,
			" |  Counter 1:", encoder_instance.plc_handler.read_counter(1), " (", encoder_instance.read_counter_deg(1), ") |  Counter 2:", encoder_instance.plc_handler.read_counter(2), " (", encoder_instance.read_counter_deg(2), ")")
			time.sleep(1)
		encoder_instance.reset_counter(1)
		encoder_instance.reset_counter(2)
	


#static monarco_cxt_t cxt;

#cxt = plc.monarco_cxt_t()

#sdc_item_cnt2_mode = &cxt.sdc_items[i]
