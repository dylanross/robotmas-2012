import hardware_interface as hi

'''
Controls motors via an ArduinoLAB abstraction layer. Mainly defines motor actions involving more than one
arduino pin and allows access to these via the command function.

See arduinolab source for more information on arduino configuration. 
'''

'''
Global variables.
'''

initialised = False

ARDUINO = False
BRICKPI = True
h = None

arduino_config_file = 'arduino_config.txt'
brick_pi_config_file = None

def initialise() :
	'''
	Function to be called before use.
	'''

	global initialised, h
        if ARDUINO :
            h = hi.Arduino(arduino_config_file)
        elif BRICKPI :
            h = hi.BrickPi(brick_pi_config_file)

	initialised = True						# update initialised flag

'''
Functions mediating useful motor actions.
'''

'''
Functions using multiple arduino pins.
'''

def drive_l(value) :
	'''
	Controls speed and direction of the left drive using a single -1..1 value.
	'''

	speed, direction = motor_format(value)				# produce spd and dir from -1 .. 1 value
	h.command(('drive_dir_l', direction))				# set motor direction pin value
	h.command(('drive_spd_l', speed))				# set motor speed pin value

def drive_r(value) :
	'''
	Controls speed and direction of the right drive using a single -1..1 value.
	'''

	speed, direction = motor_format(value)				# produce spd and dir from -1 .. 1 value
	h.command(('drive_spd_r', speed))				# set motor speed pin value
	h.command(('drive_dir_r', direction))				# set motor speed pin value

def drive_lr(value) :
	'''
	Controls speed and direction of both drives simultaneously.
	'''

	drive_l(value)							# set left drive spd / dir
	drive_r(value)							# set right drive spd / dir

def drive(value) :
	'''
	Controls speed and direction of both drives from a tuple of two -1..1 values.
	'''

	drive_l(value[0])						# set left drive spd / dir
	drive_r(value[1])						# set right drive spd / dir

def stop(value=1) :
	'''
	Puts speed of both motors to zero. 
	Arg value does nothing - only serves to maintain command : value syntax.
	'''

	drive_lr(0)							# set both drive speeds to 0

'''
Functions for dealing with incoming commands.
'''

def command((command, value)) :
	'''
	Takes an incoming command and maps it to a motor function.
	'''

	eval(command)(value)						# attempt to find the command in this 
									# module and pass value to it

'''
Utility functions.
'''

def motor_format(value) :
	'''
	Takes a single -1 .. 1 value and returns the direction and speed values necessary for motor control.
	'''

	value = hi.force_range(value, -1, 1)				# force value to be between -1 and 1
	if value < 0 :							# if value is negative
		direction = 0						# use backwards direction
		speed = 0 - value					# make value positive for PWM / speed
	else :								# if value is positive
		direction = 1						# use forwards direction
		speed = value						# use value as is for PWM / speed
	return speed, direction

if __name__ == '__main__' :
	initialise()
