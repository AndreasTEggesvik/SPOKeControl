

import kivy 
kivy.require('1.11.0') # replace with your current kivy version !

import os
os.environ['KIVY_GL_BACKEND'] = 'gl'

import pymonarco_hat as plc
lib_path = '../../pymonarco-hat/monarco-c/libmonarco.so'
plc_handler = plc.Monarco(lib_path, debug_flag=plc.MONARCO_DPF_WRITE | plc.MONARCO_DPF_VERB | plc.MONARCO_DPF_ERROR | plc.MONARCO_DPF_WARNING)
plc_handler.set_pwm_frequency(plc.PWM_CHANNEL1, 1000)



from kivy.uix.button import Button
from kivy.uix.button import Label
from kivy.uix.togglebutton import ToggleButton
from kivy.uix.gridlayout import GridLayout
from kivy.uix.image import Image
from kivy.uix.slider import Slider
from kivy.clock import Clock
from kivy.graphics import Color, Rectangle, Ellipse

# Graph:
from kivy.garden.matplotlib.backend_kivyagg import FigureCanvasKivyAgg
from kivy.uix.boxlayout import BoxLayout
from kivy.lang import Builder
import matplotlib.pyplot as plt
from kivy.app import App
from functools import partial
import time


  

#plt.plot([1, 23, 2, 4])
#plt.ylabel('some numbers')

# Thread: 
import threading




speed = 1
def press_callback(obj):
	print("Button pressed,", obj.text)
	if obj.text == 'BEEP!':
		# turn on the beeper:
		plc_handler.set_digital_out(plc.DOUT1, plc.HIGH)
		print('Digital 3 input: ', plc_handler.get_digital_in(plc.DIN3))
		print('Digital 4 input: ', plc_handler.get_digital_in(plc.DIN4))
		# schedule it to turn off:
		Clock.schedule_once(buzzer_off, .1)

		global speed
		plc_handler.set_pwm_out(plc.DOUT2, speed)
		print('PWM signal: ', speed)
	elif obj.text == 'INIT':
		obj.text = 'START'
	elif obj.text == 'START':
		print("Start button pressed")
	elif obj.text == 'STOP':
		print("Stop button pressed")

def buzzer_off(dt):
	plc_handler.set_digital_out(plc.DOUT1, plc.LOW)

time_list = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
measurement_list = [0, 0.1, 0.2, 0.1, 0.15, 0.4]
tstart = round(time.time(),2)

# This is called when the slider is updated:

def update_speed(obj, value):
	global speed, time_list, measurement_list, tstart
	print("Updating speed to:" + str(obj.value))
	speed = obj.value
	time_list.append((round(time.time(),2) - tstart))
	measurement_list.append(speed)

# Modify the Button Class to update according to GPIO input:
class InputButton(Button):
	def update(self, dt):
		if plc_handler.get_digital_in(plc.DIN4) > 0:
			self.state = 'normal'
		else:
			self.state = 'down'

			
class StateLabel(Label):
	def update(self, dt):
		textInput = 'Digital input 3: ' + str(plc_handler.get_digital_in(plc.DIN3)) + '\n' 
		textInput += 'Digital input 4: ' + str(plc_handler.get_digital_in(plc.DIN4))
		self.text = textInput
plt.plot(time_list, measurement_list, 'r')
graph = FigureCanvasKivyAgg(plt.gcf())

def UpdateGraph(dt, not_time_list, data_list):
	# Clear existing figure and re-use it
	plt.clf()
	global time_list
	global measurement_list
	#plt.plot([0.1, 0.2, 0.3, 0.4, 0.5, 0.6], [0, 0.1, 0.2, 0.1, 0.15, 0.4], 'r')
	plt.plot(time_list, measurement_list, 'r')
	#graph = FigureCanvasKivyAgg(plt.gcf)
	graph.draw()

from multiprocessing import Process,Pipe
def HelloWorld(child_conn):
	child_conn.send("Hello world")
	#child_conn.close()

def Calculation(child_conn):
	A = child_conn.recv()
	child_conn.send(A[0] + A[1])
	child_conn.close()

class MyApp(App):

	def build(self):
	
		# Create the rest of the UI objects (and bind them to callbacks, if necessary):
		
		beepButton = Button(text="BEEP!")
		beepButton.bind(on_press=press_callback)

		stateLabel = StateLabel(text = 'Digital input 3 \nDigital input 4')
		Clock.schedule_interval(stateLabel.update, 1.0/10.0)
		global time_list, measurement_list
		Clock.schedule_interval(partial(UpdateGraph, time_list, measurement_list), 0.2)

		startButton = Button(text = "INIT")
		startButton.background_normal = ''
		startButton.background_color = [0, 0.7, 0, 1]
		startButton.bind(on_press=press_callback)
		
		stopButton = Button(text = "STOP")
		stopButton.background_normal = ''
		stopButton.background_color = [0.7, 0, 0, 1]
		stopButton.bind(on_press=press_callback)
		

		wimg = Image(source='Prototype1.png')
		speedSlider = Slider(orientation='vertical', min=0, max=1, value=speed)
		speedSlider.bind(on_touch_move=update_speed)
		speedSlider.size_hint_x=(0.2)
		# on_touch_down=update_speed,
		
		superBox = BoxLayout()

		verticalTextBox1 = BoxLayout()
		verticalTextBox1.orientation = 'vertical'
		verticalTextBox1.add_widget(stateLabel)
		verticalTextBox1.add_widget(beepButton)
		verticalTextBox1.add_widget(wimg)
		verticalTextBox1.size_hint_x=(0.3)

		verticalTextBox2 = BoxLayout()
		verticalTextBox2.orientation = 'vertical'
		verticalTextBox2.add_widget(startButton)
		verticalTextBox2.add_widget(stopButton)
		verticalTextBox2.size_hint_x=(0.3)

		#superBox.add_widget(FigureCanvasKivyAgg(plt.gcf()))
		global graph
		superBox.add_widget(graph)
		superBox.add_widget(verticalTextBox1)
		superBox.add_widget(verticalTextBox2)

		superBox.add_widget(speedSlider)

		return superBox


class MyPlot(App):
	
	def build(self):
		box = BoxLayout()
		box.add_widget(FigureCanvasKivyAgg(plt.gcf()))
		return box

class BoxLayoutTest(App):
	def build(self):
		superBox = BoxLayout(orientation='horizontal')

		verticalTextBox = BoxLayout(orientation='vertical')
		verticalTextBox.add_widget(inputDisplay)
		verticalTextBox.add_widget(beepButton)
		verticalTextBox.add_widget(wimg)
		
		superBox.add_widget(FigureCanvasKivyAgg(plt.gcf))
		superBox.add_widget(verticalTextBox)
		superBox.add_widget(startButton)
		superBox.add_widget(speedSlider)
		return superBox


#if __name__ == '__main__':
#	MyApp().run()
#MyPlot().run()

