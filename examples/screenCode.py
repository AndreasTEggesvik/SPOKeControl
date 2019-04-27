

import kivy 
kivy.require('1.11.0') # replace with your current kivy version !

import pymonarco_hat as plc
lib_path = '../../pymonarco-hat/monarco-c/libmonarco.so'
plc_handler = plc.Monarco(lib_path, debug_flag=plc.MONARCO_DPF_WRITE | plc.MONARCO_DPF_VERB | plc.MONARCO_DPF_ERROR | plc.MONARCO_DPF_WARNING)



from kivy.app import App
from kivy.uix.button import Button
from kivy.uix.togglebutton import ToggleButton
from kivy.uix.gridlayout import GridLayout
from kivy.uix.image import Image
from kivy.uix.slider import Slider
from kivy.clock import Clock
from kivy.graphics import Color, Rectangle, Ellipse

from kivy.properties import StringProperty, ListProperty


speed = 1
def press_callback(obj):
	print("Button pressed,", obj.text)
	if obj.text == 'BEEP!':
		# turn on the beeper:
		plc_handler.set_digital_out(plc.DOUT1, plc.HIGH)
		# schedule it to turn off:
		Clock.schedule_once(buzzer_off, .1)

def buzzer_off(dt):
	plc_handler.set_digital_out(plc.DOUT1, plc.LOW)


# Modify the Button Class to update according to GPIO input:
class InputButton(Button):
	def update(self, dt):
		if plc_handler.get_digital_in(plc.DIN4) == True:
			self.state = 'normal'
		else:
			self.state = 'down'

# This is called when the slider is updated:

def update_speed(obj, value):
	global speed
	print("Updating speed to:" + str(obj.value))
	speed = obj.value


class MyApp(App):

	def build(self):
		# Set up the layout:
		layout = GridLayout(cols=5, spacing=30, padding=30, row_default_height=150)


		# Make the background gray:
		with layout.canvas.before:
			Color(.2,.2,.2,1)
			self.rect = Rectangle(size=(800,600), pos=layout.pos)


		# Instantiate the first UI object (the GPIO input indicator):
		inputDisplay = InputButton(text="Input")


		# Schedule the update of the state of the GPIO input button:
		Clock.schedule_interval(inputDisplay.update, 1.0/10.0)


		# Create the rest of the UI objects (and bind them to callbacks, if necessary):
		
		beepButton = Button(text="BEEP!")
		beepButton.bind(on_press=press_callback)

		startButton = Button(text = "START")
		startButton.background_normal = ''
		#startButton.bind(on_press=press_callback)
		#startButton.background_color = ListProperty([0.7, 0.5, 0, 1])

		wimg = Image(source='Prototype1.png')
		speedSlider = Slider(orientation='vertical', min=1, max=30, value=speed)
		speedSlider.bind(on_touch_down=update_speed, on_touch_move=update_speed)

		# Add the UI elements to the layout:
		layout.add_widget(wimg)
		layout.add_widget(inputDisplay)
		layout.add_widget(beepButton)
		layout.add_widget(startButton)
		layout.add_widget(speedSlider)


		return layout

MyApp().run()

if __name__ == '__main__':
	MyApp().run()
