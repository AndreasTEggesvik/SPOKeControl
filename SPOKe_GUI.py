#####################################################################################################
#   																								#
#   Start the program using the command 'sudo python3 SPOKe_GUI' 									#
#   This code runs the main loop, and starts the main loop in 'Controller.py'						#
#   The two loops run simultaneously and communicates over shared variables and pipes.				#
#  																									#
#   TEMP FIX: In order to receive data from the encoders, the Monarco must be initialized. 			#
#             This is done by running the code sudo ./monarco-complex-demo which can be found in 	#
#             https://github.com/monarco/monarco-hat-driver-c/tree/master/examples 					#
#             after running the make file. This fix is needed as the pymonarco library				#
# 			  currently does not initiade quadrature encoding.										#
#																									#
#####################################################################################################



import os
os.environ['KIVY_GL_BACKEND'] = 'gl' # In order to fix error message 'Minnesegmentsfeil' in kivy

import kivy
from kivy.uix.button import Button, Label
from kivy.uix.boxlayout import BoxLayout
from kivy.lang import Builder
from kivy.app import App
from kivy.uix.image import Image
from kivy.uix.slider import Slider
from kivy.clock import Clock
from functools import partial
from kivy.graphics import Color, Rectangle

import csv # Used to write data to file

import matplotlib.pyplot as plt
from kivy.garden.matplotlib.backend_kivyagg import FigureCanvasKivyAgg

from multiprocessing import Process, Pipe, Value, Array, Lock
import SPOKe_Controller



speed = 1 					# Used to modify the speed of the trajectories for the controller
state = -1 					# Used to display the state of the controller
storeData = False 			# Variable to enable writing to file SPOKeRunData.csv

time_list = [0]				
measurement_list = [0]		
gantry_val_list = [0]	
gantry_ref_list = [0]	
gantry_pid_list	= [0]
ring_val_list = [0]		
ring_ref_list = [0]	
ring_pid_list = [0]

plt.plot(time_list, gantry_ref_list, 'r')
graph = FigureCanvasKivyAgg(plt.gcf())

def screenPress(data, obj):
	if obj.text == 'STOP':
		[startButton, buttonPipeParent, stopButtonPressed, newButtonData] = data
		buttonPipeParent.send("STOP")
		stopButtonPressed.value = 1
		print("Stop button pressed")
		startButton.text = "START"
		startButton.background_color = [0, 0.7, 0, 1]

	if obj.text == 'INIT':
		buttonPipeParent = data
		buttonPipeParent.send("INIT")
		obj.background_color = [0, 0.3, 0, 1]

	elif obj.text == 'START':
		buttonPipeParent = data
		buttonPipeParent.send("START")


class StartButton(Button):
		# Code for taking screenshot
		#if (self.i > 100):
		#	superBox.export_to_png("Screenshot.png")
		#	print("Took screenshot")
		#	self.i = 0
		#self.i += 1
	def updateStartButton(self, buttonPipeParent, newButtonData, superBox, dt):
		if (newButtonData.value):
			b = buttonPipeParent.recv()
			if (b == "Init finished"):
				self.text = "START"
				self.background_color = [0, 0.7, 0, 1]
				newButtonData.value -= 1
			elif (b == "Ready to continue"):
				self.text = "START"
				self.background_color = [0, 0.7, 0, 1]
				newButtonData.value -= 1
			elif (b == "Ready to init again"):
				self.text = "INIT"
				self.background_color = [0, 0.7, 0, 1]
				newButtonData.value -= 1
			elif (b == "Starting"):
				self.background_color = [0, 0.3, 0, 1]
				newButtonData.value -= 1
			print("Received message: ", b)

# This is called when the slider is updated:
def screenSwipe(operatingTimeConstant, obj, value):
	operatingTimeConstant.value = obj.value
	print("Speed is now:" + str(operatingTimeConstant.value))
#	print("Should be: " + str(obj.value))
#	print("Or: " + str(value))
	global speed
	speed = obj.value

class ValueLabel(Label):
	def update(self, dt):
		global ring_val_list, gantry_val_list
		textInput = 'Theta4: ' + str(round(ring_val_list[-1]*180/3.14, 2)) + 'deg' '\n' 
		textInput += 'r2: ' + str(round(gantry_val_list[-1], 2)) + 'm'
		self.text = textInput

class StateLabel(Label):
	def update(self, dt):
		global state
		if (state == -2):
			self.text = "State: \nPre-initialization"
		elif (state == -1):
			self.text = "State: \nInitializing"
		elif (state == 0):
			self.text = "State: \nInitialized"
		elif (state == 1):
			self.text = "State: 1 \nMove radially \nout"
		elif (state == 2):
			self.text = "State: 2 \nMove angular \nouter"
		elif (state == 3):
			self.text = "State: 3 \nTighten rope"
		elif (state == 4):
			self.text = "State: 4 \nMove radially \nin"
		elif (state == 5):
			self.text = "State: 5 \nMove angular \ninner"
		elif (state == 6):
			self.text = "State: 6 \nTighten rope"
		elif (state == 50):
			self.text = "State: \nStuck"
		elif (state == 51):
			self.text = "State: \nLimit switch hit"
		elif (state == 100):
			self.text = "State: \nStop button pressed"
		elif (state == "moveInwards"):
			self.text =  "State: \nMove radially \nin"
		elif (state == "moveOutwards"):
                        self.text = "State: \nMove radially \nout"
		elif (state == "moveAlongRing"):
                        self.text = "State: \nMove angular"
		elif (state == "tightenRopeInwards" or state == "tightenRopeOutwards"):
                        self.text ="State: \nTighten rope"


#################################################################################
# This function updates the graph with data received from the controller. 		#
# It also recveives and updates the state, which is used by class StateLabel	#
#################################################################################
def UpdateGraph(graphPipeParent, graphPipeSize, graphLock, dt):
	global time_list, measurement_list, gantry_val_list, gantry_ref_list, gantry_pid_list, ring_val_list, ring_ref_list, ring_pid_list, state, storeData

	graphLock.acquire() 
	if (graphPipeSize.value > 1):
		# Pipe is FIFO, if the pipe has more items than one, the last one carries most up to date information
		# We therefore only care about the last item in the pipe
		while(graphPipeSize.value > 1):
			graphPipeParent.recv()
			graphPipeSize.value -= 1

	if (graphPipeSize.value == 1):
		[timeD, gantryM, gantryR, gantryPID, ringM, ringR, ringPID, systemState] = graphPipeParent.recv()  
		state = systemState

		graphPipeSize.value -= 1

		time_list.extend(timeD)
		gantry_val_list.extend(gantryM)										
		gantry_ref_list.extend(gantryR)
		gantry_pid_list.extend(gantryPID)						

		ring_val_list.extend(ringM)											
		ring_ref_list.extend(ringR)
		ring_pid_list.extend(ringPID)											
		
		plt.clf()
		plotLen = min(len(time_list), 2000)
		plt.plot(time_list[-plotLen:], gantry_ref_list[-plotLen:], '--r', time_list[-plotLen:], gantry_val_list[-plotLen:], 'r', time_list[-plotLen:], ring_ref_list[-plotLen:], '--b', time_list[-plotLen:], ring_val_list[-plotLen:], 'b')
		graph.draw()

		# In order to not run out of memory, arrays are reduced to only the latest data once they reach size of 5000
		# Before the size is reduced, the data about to be deleted is added to the text file 'SPOKeRunData.csv'
		if (len(time_list) > 5000):
			if (storeData):
				with open('SPOKeRunData.csv', 'a') as csvFile:
					writer = csv.writer(csvFile)
					for i in range( len(time_list) - 2000 ):
						writer.writerow([time_list[i], gantry_val_list[i], gantry_ref_list[i], gantry_pid_list[i] ,ring_val_list[i], ring_ref_list[i], ring_pid_list[i] ])
				csvFile.close
			time_list = time_list[-2000:]
			gantry_val_list = gantry_val_list[-2000:]
			gantry_ref_list = gantry_ref_list[-2000:]
			gantry_pid_list = gantry_pid_list[-2000:]
			ring_val_list = ring_val_list[-2000:]
			ring_ref_list = ring_ref_list[-2000:]
			ring_pid_list = ring_pid_list[-2000:]
	graphLock.release()


class DisplayApp(App):
	def build(self):
	
		# Pipes and variables used for communication with the process running in 'Controller.py'
		graphPipeParent, graphPipeChild = Pipe()
		buttonPipeParent, buttonPipeChild = Pipe()
		graphLock = Lock()
		graphPipeSize = Value('i', 0)
		stopButtonPressed = Value('i', 0)
		newButtonData = Value('i', 0)
		operatingTimeConstant = Value('d', 1.0)

		# Running the main looop  'Controller.py' as a separate process
		self.p = Process(target=SPOKe_Controller.main, args=(graphPipeChild, graphPipeParent, buttonPipeChild, graphPipeSize, graphLock, stopButtonPressed, newButtonData, operatingTimeConstant))
		self.p.start()
		
		superBox = BoxLayout()

		stateLabel = StateLabel(text = 'State: \n Pre-initialization')
		Clock.schedule_interval(stateLabel.update, 0.5)

		valueLabel = ValueLabel(text = 'Theta4: -1 \nr2: -1')
		Clock.schedule_interval(valueLabel.update, 0.5)
		Clock.schedule_interval(partial(UpdateGraph, graphPipeParent, graphPipeSize, graphLock), 0.2)

		startButton = StartButton(text = "INIT")
		startButton.background_normal = ''
		startButton.background_color = [0, 0.7, 0, 1]
		startButton.bind(on_press=(partial(screenPress, buttonPipeParent)))
		Clock.schedule_interval(partial(startButton.updateStartButton, buttonPipeParent, newButtonData, superBox), 0.6)
		
		stopButton = Button(text = "STOP")
		stopButton.background_normal = ''
		stopButton.background_color = [0.7, 0, 0, 1]
		stopButton.bind(on_press=(partial(screenPress, [startButton, buttonPipeParent, stopButtonPressed, newButtonData])))

		wimg = Image(source='Prototype1.png')

		speedSlider = Slider(orientation='vertical', min=0.5, max=1.5, value=speed)
		speedSlider.bind(on_touch_move=partial(screenSwipe, operatingTimeConstant))
		speedSlider.size_hint_x=(0.2)
		
		# Add gray color to the background, useful when taking screenshots
		with superBox.canvas.before:
			Color(.2,.2,.2,1)
			self.rect = Rectangle(size=(800,600), pos=superBox.pos)
	
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
	DisplayApp().run()
