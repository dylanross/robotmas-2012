Code from robotmas 2012. Our task was to create a simple, wheeled robot capable
of taking remote commands and autonomously charging at red objects.


HARDWARE
--------

We used a Raspberry Pi Model B and an Arduino Uno with Motor Shield to drive 5V
lego motors in a simple differential wheeled robot design.


SOFTWARE DEPENDENCIES
---------------------

The StandardFirmata sketch must be loaded onto the Arduino Uno.

The following python modules must be installed on the RasPi :

	NumPy
	SciPy
	matplotlib
	pyFirmata (requires pySerial)
	PyBrain
	PIL
	OpenCV (w/ python bindings)


Commands can be issued to the robot over a network connection from a terminal
on another computer, or from TouchOSC running on a mobile device.

CONTACT INFORMATION
-------------------

email : dylan.martin.ross@gmail.com
