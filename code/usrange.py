#!/usr/bin/python

import time
import numpy as np
import arduinolab as al
al.initialise('arduino_config.txt')

SOS = 340.29 * 100  # speed of sound in cm s^-1

print('Finding pins...')
TRIG = al.func_to_pin['rng_trig']
ECHO = al.func_to_pin['rng_echo']

print('Settling...')
TRIG.write(0)
time.sleep(0.5)

def pulse(dur=0.00001) :
	TRIG.write(1)
	time.sleep(dur)
	TRIG.write(0)

def grab() :
    def f(dummy) :
        t = time.time()
        v = ECHO.read()
        return [t, v]

    axis = range(2000)
    pulse()
    t, v = np.array(map(f, axis)).T

    if np.sum(v == True) > 1 :
        elapsed = t[v == True][-1] - t[v == True][0]
        distance = 0.5*SOS*elapsed
        return distance
    else :
        return np.NaN

#al.terminate()
