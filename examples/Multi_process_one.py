
from multiprocessing import Process,Queue,Pipe

import screenCode



test = 2
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
