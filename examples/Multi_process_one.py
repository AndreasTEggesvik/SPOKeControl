
from multiprocessing import Process,Queue,Pipe, Condition

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
	p = Process(target=screenCode.Calculation, args=(child_conn,))
	p.start()
	parent_conn.send([4,5])
	print(parent_conn.recv())
