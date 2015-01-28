import time
import csv
import numpy as np
import pyfirmata as pf
from serial import SerialException

"""
Controls motors via an arduino. Arduino pins should be defined using a config file, which is
arduino_config.txt by default. See initialise() docstring for config file requirements.
"""

"""
Global variables.
"""

initialised = False

arduino = None								# will hold Arduino object
it = None
func_to_pin = None							# will hold function name -> pin dict

def initialise(config_filename) :
    """Parse an arduino configuration file and store Arduino object with correctly configured pins and a
    dictionary mapping function names (e.g. drive_spd_l) to pins. The arduino config file should be in the
    following format (see arduino_config.txt for an example) :

	line number == 1 | serial address <tab> serial address <tab> etc.
	line number != 1 | function name <tab> pin config
	
    Pin configs should be in pyfirmata format (e.g. d:12:o means set digital pin 12 to output).
    """

    global initialised, arduino, it, func_to_pin			# use these global variables
	
    config_file = open(config_filename, 'rb')			        # open the data file
    config_lines = list(csv.reader(config_file, delimiter='\t'))	# read as csv file with tab delimiters

    for addr in config_lines[0] :					# read serial addresses in sequence
	try :
	    arduino = pf.Arduino(addr)			                # try to connect at each address
            it = pf.util.Iterator(arduino)
            it.start()
	    print 'Connected to arduino at', addr		        # report connection made
	    break						        # break loop if successful
	except SerialException :				        # if connection fails :
	    print 'Couldn\'t connect to arduino at', addr	        # print error report
    if arduino is None :						# ensure an arduino was found
	raise ValueError('Couldn\'t connect to an arduino at ' +
			 'any of the serial addresses provided '+
			 'in ' + config_filename)

    func_to_pin = [[func, arduino.get_pin(pin_conf)] 		        # form table of 'func' -> pin
		    for func, pin_conf in config_lines[1:]]		# from 'func' and pin config data
    func_to_pin = dict(np.array(func_to_pin))			        # transform to dictionary

    initialised = True						        # update initialised flag


def terminate() :
    global initialised, arduino
    arduino.exit()
    initialised = False


"""
Functions for dealing with incoming commands.
"""

def command((func_name, value)) :
    """Takes command : value pair and uses to control relevant pin.
    Command should be the func_name.
    """
    global func_to_pin

    pin = func_to_pin[func_name]
    if pin.mode is pf.INPUT :					        # if input pin
	raise IOError('Pin referred to by ' + func_name + 	        # raise exception
		      'is set to input mode.')
    if pin.mode is pf.OUTPUT :					        # if output pin
	pin.write(force_boolean(value))				        # force 0 or 1 value
    if pin.mode is pf.ANALOG :					        # if analog pin
	raise IOError('Pin referred to by ' + func_name +	        # raise exception
		      'is set to analog mode.')
    if pin.mode is pf.PWM :						# if PWM pin
	pin.write(force_range(value))				        # force value between 0 and 1 
    if pin.mode is pf.SERVO :					        # if servo pin
	pin.write(value)					        # write unadulterated value


def read(func_name) :
    """Return the current value of the pin responsible for the named function.
    """
    global func_to_pin
    return func_to_pin[func_name].read()

"""
Utility functions.
"""

def force_boolean(value) :
    """Force an input value to take boolean 0 or 1. Useful for correct formatting of outputs to arduino.
    Return 1 if input is >=0.5, and 0 for all other cases.
    """

    if value is not 0 and value is not 1 :				# if not already bool
	if type(value) is type(int()) or type(value) is type(float()) :	# if int or float
	    if value >= 0.5 :					        # >0.5 -> 1
		value = 1					
	    else :							# <0.5 -> 0
		value = 0
	else :								# if not int or float
	    value = 0						        # -> 0
    return value							# return boolean value


def force_range(value, min=0, max=1) :
    """Clip an input value to lie between minimum / maximum limits given.
    """

    if value > max:							# if greater than max
	value = max							# set to max
    if value < min:							# if less than min
	value = min							# set to min
    return value							# return clipped value


def pin_test(func_name, delay_length=0.5, repetitions=5, high=1, low=0) :
    """Switch the state of the pin specified by func_name between high and low values repeatedly.  """

    if repetitions <= 0 :
	while True :
	    command((func_name, high))
	    time.sleep(delay_length)
	    command((func_name, low))
	    time.sleep(delay_length)
    else :
	for rep in range(repetitions) :
	    command((func_name, high))
	    time.sleep(delay_length)
	    command((func_name, low))
	    time.sleep(delay_length)

if __name__ == '__main__' :						# if ran as main program
    initialise('arduino_config.txt')				# attempt initialisation
