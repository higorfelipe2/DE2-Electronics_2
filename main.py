'''
-------------------------------------------------------
Name: main
Creator:  Higor Alves
-------------------------------------------------------

'''	

import pyb
from pyb import Pin, LED

#  Configure X2:4, X7 as setting input pin - pull_up to receive switch settings
s0=Pin('Y3',pyb.Pin.IN,pyb.Pin.PULL_UP)
s1=Pin('X6',pyb.Pin.IN,pyb.Pin.PULL_UP)
r_LED = LED(1)
g_LED = LED(2)
y_LED = LED(3)
b_LED = LED(4)
'''
Define various test functions
'''
def read_sw():
	value = 3 - (s0.value() + 2*s1.value())
	if (not s0.value()):
		y_LED.on()
	if (not s1.value()):
		g_LED.on()
	return value

if read_sw() == 0:
	print('Running Milestone 1: BLE Control')
	execfile('milestone1.py')	
elif read_sw() == 1:
	print('Running Milestone 3: Dancing to beat')
	execfile('milestone3.py')	
elif read_sw() == 2:
	print('Running Milestone 4: Self balancing')
	execfile('milestone4.py')	
elif read_sw() == 3:
	print('Running User Python Program')
	execfile('pybench_test.py')	
	