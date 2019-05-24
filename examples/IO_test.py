

import SPOKe_IO
import Geometry as geo
import time
from multiprocessing import Process, Pipe, Value, Lock

import pymonarco_hat as plc
#plc_handler = plc.Monarco(lib_path, debug_flag=plc.MONARCO_DPF_WRITE) # | plc.MONARCO_DPF_VERB | plc.MONARCO_DPF_WARNING)

#motor_instance = SPOKe_IO.Motor_output(plc_handler)
#ls = SPOKe_IO.LimitSwitch()

#encoder_instance = SPOKe_IO.Encoder_input(plc_handler)
#motor_instance = SPOKe_IO.Motor_output(plc_handler)
#ls = SPOKe_IO.LimitSwitch()
def main():
#	plc_handler = plc.Monarco(lib_path, debug_flag=plc.MONARCO_DPF_WRITE)
#	encoder_instance = SPOKe_IO.Encoder_input(plc_handler)
#	motor_instance = SPOKe_IO.Motor_output(plc_handler)
#	ls = SPOKe_IO.LimitSwitch()
	c = Controller()
	i = 9
#	time.sleep(0.2)
	while (1):
		if (i == 9):
#		for i in range (0,100):
#			c.motor_instance.setMotorDirection(1, -1)
#			c.motor_instance.setMotorSpeed(1,1)
#			c.encoder_instance.update_counter(1, c.plc_handler)
#			c.encoder_instance.update_counter(2, c.plc_handler)
#			if (not i % 10):
#				print("Ticks: ", encoder_instance.readCounterValue(1, plc_handler))
			print("Counter 1:", c.encoder_instance.read_counter_deg(1, c.plc_handler), "(", c.encoder_instance.local_counter1, ")") 
			i = 0
			time.sleep(0.2)
		i +=1
		time.sleep(0.02)
#		c.encoder_instance.reset_counter(1, c.plc_handler)
#		c.encoder_instance.reset_counter(2, c.plc_handler)

class Controller:
	def __init__(self):
		lib_path = '../../../pymonarco-hat/monarco-c/libmonarco.so'
		self.plc_handler = plc.Monarco(lib_path, debug_flag=plc.MONARCO_DPF_WRITE)
		self.encoder_instance = SPOKe_IO.Encoder_input(self.plc_handler)
		self.motor_instance = SPOKe_IO.Motor_output(self.plc_handler)
		self.ls = SPOKe_IO.LimitSwitch()

#main()
