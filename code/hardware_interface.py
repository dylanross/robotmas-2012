#! /usr/bin/env python

#import csv
import time
import threading
import BrickPi as bp
import arduinolab as al

class HardwareInterface(object) :
    def __init__(self) :
        pass

    def command(self, (func_name, value)) :
        pass
    
    def read(self, func_name) :
        pass


class Arduino(HardwareInterface) :
    def __init__(self, config_filename=None) :
        if config_filename is None : config_filename = 'arduino_config.txt'
        al.initialise(config_filename)

    def command(self, (func_name, value)) :
        return al.command((func_name, value))

    def read(self, func_name) :
        return al.read(func_name)


class BrickPi(HardwareInterface, threading.Thread) :
    def __init__(self, config_filename=None) :
        bp.BrickPiSetup()

        self.MOTOR_PORTS = [bp.PORT_A, bp.PORT_B, bp.PORT_C, bp.PORT_D]
        for port in self.MOTOR_PORTS :
            bp.BrickPi.MotorEnable[port] = 1
            bp.BrickPi.MotorSpeed[port] = 0
        self.drive_dir_l = 1
        self.drive_dir_r = 1

        self.func_name_to_port = {'drive_spd_l':bp.PORT_A, 
                                  'drive_spd_r':bp.PORT_D}

        bp.BrickPiSetupSensors()
        threading.Thread.__init__(self)
        self.start()

    def command(self, (func_name, value)) :
        if func_name == 'drive_spd_l' :
            spd = int(force_range(value) * 255)
            bp.BrickPi.MotorSpeed[self.func_name_to_port[func_name]] = spd * self.drive_dir_l

        elif func_name == 'drive_spd_r' :
            spd = int(force_range(value) * 255)
            bp.BrickPi.MotorSpeed[self.func_name_to_port[func_name]] = spd * self.drive_dir_r

        elif func_name == 'drive_dir_l' :
            self.drive_dir_l = int((force_boolean(value) - 0.5) * 2)

        elif func_name == 'drive_dir_r' :
            self.drive_dir_r = int((force_boolean(value) - 0.5) * 2)

        else :
            print 'sorry! function not yet supported :' + str(func_name)

        bp.BrickPiUpdateValues()

    def read(self, func_name) :
        pass

    def run(self) :
        self._stop = False
        while self._stop is not True :
            bp.BrickPiUpdateValues()
            time.sleep(0.01)

    def stop(self) :
        self._stop = True


"""
Utility Functions
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


