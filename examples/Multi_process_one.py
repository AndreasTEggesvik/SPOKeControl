
from multiprocessing import Process,Queue, Pipe, Condition, Value, Lock

import screenCode
import time
from math import sin


def controllerSimulator(graphPipe, graphPipeReceier, buttonPipe, graphPipeSize, graphLock, stopButtonPressed, newButtonData):
	time_list = []
	measurement_list = []
	tstart = round(time.time(),2)

	dataBuffer = [time_list, measurement_list]

	i = 0
	while (True):
#		if not (round( (time.time() - tstart) * 10 , 2) % 1):
		time_list.append((round(time.time(),2) - tstart))
		measurement_list.append(sin((time.time() - tstart))*3.14/180)

		#if not (round( (time.time() - tstart)*100 , 0) % 100): 
			#Every second, send data
		if (i == 10):
			graphLock.acquire()
			if (graphPipeSize.value == 0):
				# Only erase buffered data if the last message is read
				dataBuffer = [[],[]]
			elif (graphPipeSize.value > 0):
				# If message was not read, clear the pipe
				while(graphPipeSize.value != 0):
					graphPipeReceier.recv()
					graphPipeSize.value = graphPipeSize.value - 1
			dataBuffer[0].extend(time_list)
			dataBuffer[1].extend(measurement_list)
			time_list = []
			measurement_list = []
			graphPipe.send(dataBuffer)
			graphPipeSize.value = graphPipeSize.value + 1
			graphLock.release()
			i = 0
		time.sleep(0.1)
		i += 1
				# Should also write to file for storage
#		time.sleep(0.11)
			

test = 0
if test == 1:
	parent_conn,child_conn = Pipe()
	p = Process(target=screenCode.HelloWorld, args=(child_conn,))
	p.start()
	print(parent_conn.recv())   # prints "Hello world"
	#p.start()
	print(parent_conn.recv())

elif test == 2:
	parent_conn,child_conn = Pipe()
	Parameter1 = [4,5]
	p = Process(target=screenCode.Calculation, args=(child_conn,Parameter1))
	p.start()
	parent_conn.send(Parameter1)
	print(parent_conn.recv())
	print(parent_conn.recv())
	p.join()
	print(Parameter1)
