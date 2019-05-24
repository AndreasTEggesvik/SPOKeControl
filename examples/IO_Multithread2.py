import SPOKe_IO

import pymonarco_hat as plc
import IO_test

lib_path = '../../../pymonarco-hat/monarco-c/libmonarco.so'
plc_handler = plc.Monarco(lib_path, debug_flag=plc.MONARCO_DPF_WRITE) # | plc.MONARCO_DPF_VERB | plc.MONARCO_DPF_WARNI$
encoder_instance = SPOKe_IO.Encoder_input(plc_handler)
#motor_instance = SPOKe_IO.Motor_output(plc_handler)
#ls = SPOKe_IO.LimitSwitch()
IO_test2(plc_handler)
