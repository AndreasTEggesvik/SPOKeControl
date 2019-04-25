import pymonarco_hat as plc
#import sys
import time

#lib_path = sys.argv[1]
lib_path = '../monarco-c/libmonarco.so'
plc_handler = plc.Monarco(lib_path, debug_flag=plc.MONARCO_DPF_WRITE | plc.MONARCO_DPF_VERB | plc.MONARCO_DPF_ERROR | plc.MONARCO_DPF_WARNING)

plc_handler.set_analog_out(plc.AOUT1, 5.0)

direction = 1

plc_handler.set_digital_out(plc.DOUT1, plc.LOW)
plc_handler.set_digital_out(plc.DOUT2, plc.HIGH)

test = 0
if test ==1:
    plc_handler.set_pwm_frequency(plc.PWM_CHANNEL1, 1000)
    plc_handler.set_pwm_out(plc.DOUT2, 0.75)
    pwm_const = 0.75
    while 1:
        plc_handler.set_digital_out(plc.DOUT4, plc.LOW)
        plc_handler.set_pwm_out(plc.DOUT2, pwm_const)
        time.sleep(0.2)
        #print("DIN4:", plc_handler.get_digital_in(plc.DIN4))
        #print("AIN1:", plc_handler.get_analog_in(plc.AIN1))
        
        #plc_handler.set_digital_out(plc.DOUT1, plc.LOW)
        
        if (pwm_const > 0.94):
            direction = -1
        elif (pwm_const < 0.05):
            direction = 1
        pwm_const += 0.01*direction

        print(pwm_const)
        #print("AIN1:", plc_handler.get_analog_in(plc.AIN1))




