'''    Marks the start of comment section
-------------------------------------------------------
Name: Electronics 2 Project: Milestone 4
Creator:  Higor, Ben, Esther, Imogen
Revision:  1.0
-------------------------------------------------------
Balances the segway using a PID controller
-------------------------------------------------------
'''   

import pyb
from pyb import Pin, ADC, Timer
import time
from oled_938 import OLED_938
from mpu6050 import MPU6050

import micropython
micropython.alloc_emergency_exception_buf(100)

# Use OLED to display mode
oled = OLED_938(pinout={'sda': 'Y10', 'scl': 'Y9', 'res': 'Y8'}, height=64, external_vcc=False, i2c_devid=61)
oled.poweron()
oled.init_display()
oled.draw_text(0,0, 'Group 8')
oled.draw_text(0, 10, 'Milestone 4: Balance')
oled.draw_text(0, 20, 'Press USR button')
oled.display()

print('Performing Milestone 4')
print('Waiting for USR to be pressed')
trigger = pyb.Switch()		# Create trigger object
while not trigger():		# Wait for trigger to be pressed
	time.sleep(0.001)
while trigger():
	pass			# Wait for trigger to be released
print('Button pressed - Running')

# Set up
# Define Potentiometer
pot = ADC(Pin('X11'))
imu = MPU6050(1, False)
# PID controller tuning
ScaleP = 10
ScaleD = 1
ScaleI = 1
while not trigger():	# Wait to tune Kp
	time.sleep(0.001)
	#K_p = pot.read() * ScaleP / 4095		# Use pot to set up the Kp
	K_p = 5.5
	oled.draw_text(0, 30, 'Kp={:5.3f}'.format(K_p))	# Displays live value on the oled display
	oled.display()
while trigger(): pass

while not trigger():	# Wait to tune Kd
	time.sleep(0.001)
	#K_d = pot.read() * ScaleD / 4095		# Use pot to set up the Ki
	K_d = 0.5
	oled.draw_text(60, 30, 'Kd={:5.3f}'.format(K_d))	# Displays live value on the oled display
	oled.display()
while trigger(): pass

while not trigger():	# Wait to tune Ki
	time.sleep(0.001)
	#K_i = pot.read() * ScaleI / 4095		# Use pot to set up the Ki
	K_i = 0.35
	oled.draw_text(0, 40, 'Ki={:5.3f}'.format(K_i))	# Displays live value on the oled display
	oled.display()
while trigger(): pass

# Tuning the angle offset for slanted floors
ScaleR = 1
while not trigger():	# Wait to tune the floor slant offset, r 
	time.sleep(0.001)
	#r = pot.read() * ScaleR / 4095		# Use pot to set up the offset for floor slant, r
	r = 2.9
	oled.draw_text(60, 40, 'r={:5.3f}'.format(r))	# Displays live value on the oled display
	oled.display()
while trigger(): pass

#   Motor
class MOTOR(object):

	def __init__(self):
		# set up motor with PWM and timer control
		self.A1 = Pin('X3',Pin.OUT_PP)	# A is right motor
		self.A2 = Pin('X4',Pin.OUT_PP)
		self.B1 = Pin('X7',Pin.OUT_PP)	# B is left motor
		self.B2 = Pin('X8',Pin.OUT_PP)
		self.PWMA = Pin('X1')			
		self.PWMB = Pin('X2')
		
		# Configure timer to provide PWM signal
		self.tim = Timer(2, freq = 10000)
		self.motorA = self.tim.channel(1, Timer.PWM, pin = self.PWMA)
		self.motorB = self.tim.channel(2, Timer.PWM, pin = self.PWMB)
		
		# initialise variables for motor drive strength (PWM values)
		self.Aspeed = 0			# PWM value for motorA
		self.Bspeed = 0			# PWM value for motorB
	
	def drive(self):	# drove motors at Aspeed and Bspeed
		if self.Aspeed > 0:
			self.A_forward(self.Aspeed)
		else:
			self.A_back(-self.Aspeed)
		if self.Bspeed > 0:
			self.B_forward(self.Bspeed)
		else:
			self.B_back(-self.Bspeed)					
		
	def up_Aspeed(self,value):	# increase motor A speed by value
		self.Aspeed = min(100, self.Aspeed + value)
		drive()
		
	def up_Bspeed(self,value):	# increase motor B speed by value
		self.Bspeed = min(100, self.Bspeed + value)
		drive()

	def dn_Aspeed(self,value):	# decrease motor A speed by value
		self.Aspeed = max(-100, self.Aspeed - value)
		drive()
		
	def dn_Bspeed(self,value):	# decrease motor B speed by value	
		self.Bspeed = max(-100, self.Bspeed - value)
		drive()

	def A_forward(self,value):	# Drive motor A forward by value
		self.Aspeed = min(100, value)
		self.A2.high()
		self.A1.low()
		self.motorA.pulse_width_percent(abs(value))
		
	def A_back(self,value):		# Drive motor A backward by value
		self.Aspeed = min(100, value)
		self.A2.low()
		self.A1.high()
		self.motorA.pulse_width_percent(value)
		
	def B_forward(self,value):	# Drive motor B forward by value
		self.Bspeed = min(100, value)
		self.B1.high()
		self.B2.low()
		self.motorB.pulse_width_percent(abs(value))
		
	def B_back(self,value):		# Drive motor B backward by value
		self.Aspeed = min(100, value)
		self.B1.low()
		self.B2.high()
		self.motorB.pulse_width_percent(abs(value))
		
	def A_stop(self):			# stop motor A
		self.Aspeed = 0
		self.motorA.pulse_width_percent(0)
		
	def B_stop(self):			# stop motor B
		self.Bspeed = 0
		self.motorB.pulse_width_percent(0)


# Complementary filter for pitch angle calculation
def pitch_estimation(pitch, dt, alpha):
    theta = imu.pitch()
    pitch_dot = imu.get_gy()
    pitch = alpha*(pitch + pitch_dot*dt*0.000001) + (1-alpha)*theta
    print(pitch)
    return (pitch, pitch_dot)

alpha = 0.95
pitch = 0
e_int = 0
w = 0
MotorA = MOTOR()
MotorB = MOTOR()

# Main while loop
tic = pyb.micros()
while True:
    dt = pyb.micros()-tic
    
    if dt > 5000:     #loop period is 5msec
        pitch, pitch_dot = pitch_estimation(pitch, dt, alpha)
        # PID controller
        e = pitch - r
        w = (K_p*e + K_i*e_int + K_d*pitch_dot)
        e_int += e

        if w > 0:
            MotorA.A_back(w)
            MotorA.B_back(w)
        if w < 0:
            MotorB.A_forward(w)
            MotorB.B_forward(w)

        tic = pyb.micros()