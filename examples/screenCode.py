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

import csv

# Graph:
from kivy.garden.matplotlib.backend_kivyagg import FigureCanvasKivyAgg
import matplotlib.pyplot as plt

# Multi processing
from multiprocessing import Process, Pipe, Value, Array, Lock

# Other programs
import Multi_process_one
import Controller2

speed = 1
state = -1
time_list = [0]
measurement_list = []

gantry_val_list = [0]
gantry_ref_list = [0]
ring_val_list = [0]
ring_ref_list = [0]

plt.plot(time_list, gantry_ref_list, 'r')
graph = FigureCanvasKivyAgg(plt.gcf())

def press_callback(startButton, buttonPipeParent, stopButtonPressed, newButtonData, obj):
	if obj.text == 'STOP':
		buttonPipeParent.send("STOP")
		stopButtonPressed.value = 1
		print("Stop button pressed")

		startButton.text = "START"
		startButton.background_color = [0, 0.7, 0, 1]


class StartButton(Button):
	def buttonPressed(self, buttonPipeParent, obj):
		print(type(self), " | ", type(obj)," | ", type(buttonPipeParent))
		if self.text == 'INIT':
			buttonPipeParent.send("INIT")
			self.background_color = [0, 0.3, 0, 1]
		elif self.text == 'START':
			buttonPipeParent.send("START")

	def updateStartButton(self, buttonPipeParent, newButtonData, dt):
		if (newButtonData.value):
			b = buttonPipeParent.recv()
			if (b == "Init finished"):
				self.text = "START"
				self.background_color = [0, 0.7, 0, 1]
				newButtonData.value -= 1
			elif (b == "Starting"):
				self.background_color = [0, 0.3, 0, 1]
				newButtonData.value -= 1
			print("Received message: ", b)

# This is called when the slider is updated:
def update_speed(operatingTimeConstant, obj, value):
	operatingTimeConstant.value = obj.value
	print("Updating speed to:" + str(operatingTimeConstant.value))
#	print("Should be: " + str(obj.value))
#	print("Or: " + str(value))
	global speed
	speed = obj.value

class ValueLabel(Label):
	def update(self, dt):
		global ring_val_list, gantry_val_list
		textInput = 'Theta4: ' + str(round(ring_ref_list[-1]*180/3.14, 2)) + 'deg' '\n' 
		textInput += 'r2: ' + str(round(gantry_ref_list[-1], 2)) + 'm'
		self.text = textInput

class StateLabel(Label):
	def update(self, dt):
		global state
		if (state == -1):
			self.text = "State: \nPre-initialization"
		elif (state == 0):
			self.text = "State: \nInitialized"
		elif (state == 1):
			self.text = "State: 1"
		elif (state == 2):
			self.text = "State: 2"
		elif (state == 3):
			self.text = "State: 3"
		elif (state == 4):
			self.text = "State: 4"
		elif (state == 5):
			self.text = "State: 5"
		elif (state == 6):
			self.text = "State: 6"
		elif (state == 50):
			self.text = "State: \nStuck"
		elif (state == 100):
			self.text = "State: \nStop button pressed"


def UpdateGraph(graphPipeParent, graphPipeSize, graphLock, dt):
	global time_list, measurement_list, gantry_val_list, gantry_ref_list, ring_val_list, ring_ref_list, state

	graphLock.acquire()
	if (graphPipeSize.value > 1):
		# If message was not read, clear the old messages in pipe
		while(graphPipeSize.value > 1):
			graphPipeParent.recv()
			graphPipeSize.value = graphPipeSize.value - 1

	if (graphPipeSize.value == 1):
		# [timeD, measurementD] = graphPipeParent.recv() 					# Compatible with Multi_process_one.py
		[timeD, gantryM, gantryR, ringM, ringR, systemState] = graphPipeParent.recv() 	# Compatible with Controller2.py
		state = systemState
		print("The graph pipe is being read")
		print("Size of time list: ", len(timeD))

		graphPipeSize.value = graphPipeSize.value - 1 # Indicating that the data is read, pipe is cleared

		time_list.extend(timeD)
		#measurement_list.extend(measurementD) 								# Compatible with Multi_process_one.py
		gantry_val_list.extend(gantryM)										# Compatible with Controller2.py
		gantry_ref_list.extend(gantryR)										# Compatible with Controller2.py
		ring_val_list.extend(ringM)											# Compatible with Controller2.py
		ring_ref_list.extend(ringR)											# Compatible with Controller2.py
		
		plt.clf()
		#plt.plot(time_list, measurement_list, 'r') 						# Compatible with Multi_process_one.py
		plotLen = min(len(time_list), 2000)
		plt.plot(time_list[-plotLen:], gantry_ref_list[-plotLen:], 'r', time_list[-plotLen:], ring_ref_list[-plotLen:], 'b')
		graph.draw()
		if (len(time_list) > 5000):
			with open('SPOKeRunData.csv', 'a') as csvFile:
				writer = csv.writer(csvFile)
				for i in range( len(time_list) - 2000 ):
					writer.writerow([time_list[i], gantry_val_list[i], gantry_ref_list[i], ring_val_list[i], ring_ref_list[i]])
			csvFile.close

			time_list = time_list[-2000:]
			gantry_val_list = gantry_val_list[-2000:]
			gantry_ref_list = gantry_ref_list[-2000:]
			ring_val_list = ring_val_list[-2000:]
			ring_ref_list = ring_ref_list[-2000:]
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
		operatingTimeConstant = Value('d', 1.0)

		#controllerSimulator(graphPipe, graphPipeReceier, buttonPipe, graphPipeSize, graphLock, stopButtonPressed, newButtonData)
		#self.p = Process(target=Multi_process_one.controllerSimulator, args=(graphPipeChild, graphPipeParent, buttonPipeChild, graphPipeSize, graphLock, stopButtonPressed, newButtonData))
		self.p = Process(target=Controller2.main, args=(graphPipeChild, graphPipeParent, buttonPipeChild, graphPipeSize, graphLock, stopButtonPressed, newButtonData, operatingTimeConstant))
		self.p.start()

		#beepButton = Button(text="BEEP!")
		#beepButton.bind(on_press=press_callback)

		stateLabel = StateLabel(text = 'State: \n Pre-initialization')
		Clock.schedule_interval(stateLabel.update, 0.5)


		valueLabel = ValueLabel(text = 'Theta4: -1 \nr2: -1')
		Clock.schedule_interval(valueLabel.update, 0.5)
		Clock.schedule_interval(partial(UpdateGraph, graphPipeParent, graphPipeSize, graphLock), 0.2)

		startButton = StartButton(text = "INIT")
		startButton.background_normal = ''
		startButton.background_color = [0, 0.7, 0, 1]
		startButton.bind(on_press=(partial(startButton.buttonPressed, buttonPipeParent)))
		Clock.schedule_interval(partial(startButton.updateStartButton, buttonPipeParent, newButtonData), 0.6)

		
		stopButton = Button(text = "STOP")
		stopButton.background_normal = ''
		stopButton.background_color = [0.7, 0, 0, 1]
		stopButton.bind(on_press=(partial(press_callback, startButton, buttonPipeParent, stopButtonPressed, newButtonData)))

		wimg = Image(source='Prototype1.png')

		speedSlider = Slider(orientation='vertical', min=0.5, max=1.5, value=speed)
		#speedSlider.bind(on_touch_move=update_speed)
		speedSlider.bind(on_touch_move=partial(update_speed, operatingTimeConstant))
		speedSlider.size_hint_x=(0.2)
		# on_touch_down=update_speed,
		
		superBox = BoxLayout()

		verticalTextBox1 = BoxLayout()
		verticalTextBox1.orientation = 'vertical'
		verticalTextBox1.add_widget(valueLabel)
		verticalTextBox1.add_widget(stateLabel)
		verticalTextBox1.add_widget(wimg)
		verticalTextBox1.size_hint_x=(0.3)

		verticalTextBox2 = BoxLayout()
		verticalTextBox2.orientation = 'vertical'
		verticalTextBox2.add_widget(startButton)
		verticalTextBox2.add_widget(stopButton)
		verticalTextBox2.size_hint_x=(0.3)

		global graph
		superBox.add_widget(graph)
		superBox.add_widget(verticalTextBox1)
		superBox.add_widget(verticalTextBox2)

		superBox.add_widget(speedSlider)

		return superBox
	def on_stop(self):
		self.p.terminate()
	

if __name__ == '__main__':
	MyApp().run()
