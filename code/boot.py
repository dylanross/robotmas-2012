import time
import picamera
import motorcommandcore as MCC
import datatransceivercore as DTC
#import intelligentprocessingcore as IPC

def drive_test() :
    MCC.drive_l(0.05); MCC.drive_r(0.05); time.sleep(0.2)
    MCC.drive_l(0.10); MCC.drive_r(0.10); time.sleep(0.2)
    MCC.drive_l(0.20); MCC.drive_r(0.20); time.sleep(0.2)
    MCC.drive_l(0.30); MCC.drive_r(0.30); time.sleep(0.2)
    MCC.stop()


def cam_test() :
    with picamera.PiCamera() as cam :
	for i in range(3) :
	    cam.led = False; time.sleep(0.1)
	    cam.led = True; time.sleep(0.1)
	for i in range(3) :
	    cam.led = False; time.sleep(0.05)
	    cam.led = True; time.sleep(0.05)


def terminate() :
	#if IPC.initialised is True :
	#	IPC.terminate()
	if DTC.initialised is True :
		DTC.terminate()

print 'Initialising Motor Command Core (MCC)...'
MCC.initialise()							# initialise MCC
print 'Testing communication with MCC...'
drive_test()								# test MCC by sending drive commands
print 'MCC is operational'

print 'Initialising Data Transceiver Core (DTC)...'
#DTC.initialise(effector=MCC.command)					# initialise DTC - dev machine version
DTC.initialise(ip='172.20.112.39', effector=MCC.command)			# initialise DTC - RPi version
print 'DTC is operational'

print 'Testing communication with RPi Camera Board...'
cam_test()								# test camera by blinking LED
print 'Camera Board is operational'

print 'IPC is loaded and may be initialised by calling IPC.initialise()'
print 'Remember to call terminate() before exit'			# reminder to terminate()
