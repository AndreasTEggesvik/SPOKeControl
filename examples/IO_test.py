

import SPOKe_IO
import Geometry as geo
import time

encoder_instance = SPOKe_IO.Encoder_input()
while (1):
	for i in range (0,1000):
#		encoder_instance.update_counter(1)
#		encoder_instance.update_counter(2)
		if (not i % 10):
			print("Counter 1:", geo.rad2r2(encoder_instance.read_counter_rad(1)), "(", encoder_instance.read_counter_rad(1), ")",  " |  Counter 2:", geo.rad2theta4(encoder_instance.read_counter_rad(2)))
		time.sleep(0.1)
	encoder_instance.reset_counter(1)
	encoder_instance.reset_counter(2)
