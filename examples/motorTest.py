
import pymonarco_hat as plc
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)
lib_path = '../monarco-c/libmonarco.so'
plc_handler = plc.Monarco(lib_path, debug_flag=plc.MONARCO_DPF_WRITE | plc.MONARCO_DPF_VERB | plc.MONARCO_DPF_ERROR | plc.MONARCO_DPF_WARNING)

GPIO.setup(11, GPIO.OUT, initial = 0)
GPIO.setup(13, GPIO.OUT, initial = 0)
GPIO.setup(15, GPIO.OUT, initial = 0)

GPIO.output(11, GPIO.LOW)
GPIO.output(13, GPIO.HIGH)
GPIO.output(15, GPIO.HIGH)


GPIO.setup(35, GPIO.OUT, initial = 0)
GPIO.output(35, GPIO.LOW)

GPIO.setup(22, GPIO.OUT, initial = 0)
GPIO.output(22, GPIO.LOW)

GPIO.setup(32, GPIO.OUT, initial = 0)
GPIO.output(32, GPIO.LOW)


plc_handler.set_digital_out(plc.DOUT2, plc.HIGH)
plc_handler.set_digital_out(plc.DOUT1, plc.LOW)

print("AIN2:", plc_handler.get_analog_in(plc.AIN2))
