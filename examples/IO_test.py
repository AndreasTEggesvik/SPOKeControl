

import SPOKe_IO
import Geometry as geo
import time


import pymonarco_hat as plc
lib_path = '../../../pymonarco-hat/monarco-c/libmonarco.so'
plc_handler = plc.Monarco(lib_path, debug_flag=plc.MONARCO_DPF_WRITE | plc.MONARCO_DPF_VERB | plc.MONARCO_DPF_WARNING)

encoder_instance = SPOKe_IO.Encoder_input(plc_handler)

while (1):
#	encoder_instance.reset_counter(1)
#	encoder_instance.reset_counter(2)
	for i in range (0,1000):
#		encoder_instance.update_counter(1)
#		encoder_instance.update_counter(2)
		if (not i % 10):
			print("Ticks: ", encoder_instance.readCounterValue(1, plc_handler))
			print("Counter 1:", geo.rad2r2(encoder_instance.read_counter_rad(1, plc_handler)), "(", encoder_instance.read_counter_deg(1, plc_handler), ")",  " |  Counter 2:", geo.rad2theta4(encoder_instance.read_counter_rad(2, plc_handler)))
		time.sleep(0.1)
	encoder_instance.reset_counter(1, plc_handler)
	encoder_instance.reset_counter(2, plc_handler)
