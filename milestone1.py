'''    Marks the start of comment section
-------------------------------------------------------
Name: Electronics 2 Project: Milestone 1
Creator:  Higor, Ben, Esther, Imogen
Revision:  1.0
-------------------------------------------------------
Drive motor via BlueFruit UART Friend
-------------------------------------------------------
'''    
import pyb
from pyb import Pin, Timer, ADC, UART
from oled_938 import OLED_938

# Define pins to control motor
A1 = Pin('X3', Pin.OUT_PP)		# Control direction of motor A
A2 = Pin('X4', Pin.OUT_PP)
PWMA = Pin('X1')				# Control speed of motor A
B1 = Pin('X7', Pin.OUT_PP)		# Control direction of motor B
B2 = Pin('X8', Pin.OUT_PP)
PWMB = Pin('X2')				# Control speed of motor B

# Use OLED to display mode
oled = OLED_938(pinout={'sda': 'Y10', 'scl': 'Y9', 'res': 'Y8'}, height=64, external_vcc=False, i2c_devid=61)
oled.poweron()
oled.init_display()
oled.draw_text(0,0, 'Group 8')
oled.draw_text(0, 10, 'Milestone 1: BLE Control')
oled.display()


# Configure timer 2 to produce 1KHz clock for PWM control
tim = Timer(2, freq = 1000)
motorA = tim.channel (1, Timer.PWM, pin = PWMA)
motorB = tim.channel (2, Timer.PWM, pin = PWMB)

# Define 5k Potentiometer
pot = pyb.ADC(Pin('X11'))

#initialise UART communication
uart = UART(6)
uart.init(9600, bits=8, parity = None, stop = 2)


def A_forward(value):
	A1.low()
	A2.high()
	motorA.pulse_width_percent(value)

def A_back(value):
	A2.low()
	A1.high()
	motorA.pulse_width_percent(value)
	
def A_stop():
	A1.high()
	A2.high()
	
def B_forward(value):
	B2.low()
	B1.high()
	motorB.pulse_width_percent(value)

def B_back(value):
	B1.low()
	B2.high()
	motorB.pulse_width_percent(value)
	
def B_stop():
	B1.high()
	B2.high()
	
# Use keypad U and D keys to control speed
DEADZONE = 5
speed = 0


while True:				# loop forever until CTRL-C
    while (uart.any()!=10):    # wait for 10 chars
        pass
    command = uart.read(10)
    if command[2]==ord('5'):
        print('forwar')
        A_forward(40)
        B_forward(40)
    if command[2]==ord('6'):
        print('forwar')
        A_back(40)
        B_back(40)

