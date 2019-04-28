

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
plt.plot([1, 23, 2, 4])
plt.ylabel('some numbers')

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
		obj.background_color = [0, 0.7, 0, 1]
	elif obj.text == 'START':
		obj.text = 'STOP'
		obj.background_color = [0.7, 0, 0, 1]
	elif obj.text == 'STOP':
		obj.text = 'INIT'
		obj.background_color = [0, 0.7, 0, 1]

def buzzer_off(dt):
	plc_handler.set_digital_out(plc.DOUT1, plc.LOW)

# This is called when the slider is updated:

def update_speed(obj, value):
	global speed
	print("Updating speed to:" + str(obj.value))
	speed = obj.value

# Modify the Button Class to update according to GPIO input:
class InputButton(Button):
	def update(self, dt):
		if plc_handler.get_digital_in(plc.DIN4) > 0:
			self.state = 'normal'
		else:
			self.state = 'down'

class StartButton(Button):
	def __init__(self):
		self.background_normal = ''
		self.background_color = [0, 0.7, 0, 1]
		self.text = "INIT" 
			
class StateLabel(Label):
	def update(self, dt):
		textInput = 'Digital input 3: ' + str(plc_handler.get_digital_in(plc.DIN3)) + '\n' 
		textInput += 'Digital input 4: ' + str(plc_handler.get_digital_in(plc.DIN4))
		self.text = textInput

class MyApp(App):

	def build(self):
	

		# Instantiate the first UI object (the GPIO input indicator):
		#inputDisplay = InputButton(text="Input")
		# Schedule the update of the state of the GPIO input button:
		#Clock.schedule_interval(inputDisplay.update, 1.0/10.0)
		


		# Create the rest of the UI objects (and bind them to callbacks, if necessary):
		
		beepButton = Button(text="BEEP!")
		beepButton.bind(on_press=press_callback)

		#startButton = StartButton()
		#startButton.bind(on_press=press_callback)

		stateLabel = StateLabel(text = 'Digital input 3 \nDigital input 4')
		Clock.schedule_interval(stateLabel.update, 1.0/10.0)

		startButton = Button(text = "INIT")
		startButton.background_normal = ''
		startButton.background_color = [0.7, 0, 0, 1]
		startButton.bind(on_press=press_callback)
		

		wimg = Image(source='Prototype1.png')
		speedSlider = Slider(orientation='vertical', min=0, max=1, value=speed)
		speedSlider.bind(on_touch_move=update_speed)
		# on_touch_down=update_speed,
		
		superBox = BoxLayout()

		verticalTextBox = BoxLayout()
		verticalTextBox.orientation = 'vertical'
		#verticalTextBox.add_widget(inputDisplay)
		verticalTextBox.add_widget(stateLabel)
		
		verticalTextBox.add_widget(beepButton)
		verticalTextBox.add_widget(wimg)

		superBox.add_widget(FigureCanvasKivyAgg(plt.gcf()))
		superBox.add_widget(verticalTextBox)
		superBox.add_widget(startButton)
		superBox.add_widget(speedSlider)

		return superBox

		# Add the UI elements to the layout:
		#layout.add_widget(wimg)
		#layout.add_widget(inputDisplay)
		#layout.add_widget(beepButton)
		#layout.add_widget(startButton)
		#layout.add_widget(speedSlider)
		#return layout

class MyPlot(App):
	
	def build(self):
		box = BoxLayout()
		box.add_widget(FigureCanvasKivyAgg(plt.gcf()))
		return box
	# don't need to define run(), as it is a part of App

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



MyApp().run()
#MyPlot().run()

