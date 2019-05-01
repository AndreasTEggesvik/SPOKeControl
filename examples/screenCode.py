# In order to fix a "bug" in kivy
import os
os.environ['KIVY_GL_BACKEND'] = 'gl'

import kivy
from kivy.uix.button import Button, Label
from kivy.uix.boxlayout import BoxLayout
from kivy.lang import Builder
from kivy.app import App
from kivy.uix.image import Image
from kivy.uix.slider import Slider
from kivy.clock import Clock
from functools import partial

# Graph:
from kivy.garden.matplotlib.backend_kivyagg import FigureCanvasKivyAgg
import matplotlib.pyplot as plt

# Multi processing
from multiprocessing import Process, Pipe, Value, Lock
import Multi_process_one


speed = 1
time_list = []
measurement_list = []
plt.plot(time_list, measurement_list, 'r')
graph = FigureCanvasKivyAgg(plt.gcf())

def press_callback(obj):
	print("Button pressed,", obj.text)
	if obj.text == 'BEEP!':
		# turn on the beeper:
		
		# schedule it to turn off:
		Clock.schedule_once(buzzer_off, .1)

	elif obj.text == 'INIT':
		obj.text = 'START'
	elif obj.text == 'START':
		print("Start button pressed")
	elif obj.text == 'STOP':
		print("Stop button pressed")

def buzzer_off(dt):
	print("After button-press")

# This is called when the slider is updated:
def update_speed(obj, value):
	global speed
	print("Updating speed to:" + str(obj.value))
	speed = obj.value

class StateLabel(Label):
	def update(self, dt):
		textInput = 'Digital input 3: ' + '\n' 
		textInput += 'Digital input 4: ')
		self.text = textInput

def UpdateGraph(graphPipeParent, graphPipeSize, graphLock, dt):
	global time_list
	global measurement_list

	graphLock.acquire()
	if (graphPipeSize.value > 0):
		print("Updating graph: ", graphPipeSize.value)
		[timeD, measurementD] = graphPipeParent.recv()
		graphPipeSize.value = graphPipeSize.value - 1 # Indicating that the data is read, pipe is cleared
		time_list.extend(timeD)
		measurement_list.extend(measurementD)
		plt.clf()
		plt.plot(time_list, measurement_list, 'r')
		graph.draw()
	graphLock.release()


class MyApp(App):
	def build(self):
	
		# Create the rest of the UI objects (and bind them to callbacks, if necessary):
		graphPipeParent, graphPipeChild = Pipe()
		buttonPipeParent, buttonPipeChild = Pipe()
		graphLock = Lock()
		
		# Variables shared with the controller. 
		graphPipeSize = Value('i', 0)
		stopButtonPressed = Value('i', 0)
		newButtonData = Value('i', 0)

		#controllerSimulator(graphPipe, graphPipeReceier, buttonPipe, graphPipeSize, graphLock, stopButtonPressed, newButtonData)

		p = Process(target=Multi_process_one.controllerSimulator, args=(graphPipeChild, graphPipeParent, buttonPipeChild, graphPipeSize, graphLock, stopButtonPressed, newButtonData))
		p.start()

		beepButton = Button(text="BEEP!")
		beepButton.bind(on_press=press_callback)

		stateLabel = StateLabel(text = 'Digital input 3 \nDigital input 4')
		Clock.schedule_interval(stateLabel.update, 1.0/10.0)
		global time_list, measurement_list
		#Clock.schedule_interval(partial(UpdateGraph, time_list, measurement_list), 0.2)
		Clock.schedule_interval(partial(UpdateGraph, graphPipeParent, graphPipeSize, graphLock), 0.6)

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

if __name__ == '__main__':
	MyApp().run()
#MyPlot().run()

