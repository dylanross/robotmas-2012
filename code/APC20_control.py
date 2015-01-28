#! /usr/bin/env/python

import datatransceivercore as DTC
import pygame.midi as midi

ROBO_IP = '172.20.113.237'
ROBO_PORT = 5005


midi.init()
APC20 = midi.Input(3)

print 'READING APC20 COMMANDS!'

while True :
    if APC20.poll() :
        input = APC20.read(1)
        print input[0][0][0]
        if input[0][0][0] == 176 : command = 'drive_l'
        elif input[0][0][0] == 177 : command = 'drive_r'
        else : command = 'stop'
        value = (input [0][0][2] - (127/2.))/63.5
        print command, value
        DTC.quick_send(command + ' ' + str(value), ip=ROBO_IP, port=ROBO_PORT)
