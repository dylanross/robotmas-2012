import threading
import os
import io
import time
import csv
import cv2
import arduinolab as al
import motorcommandcore as MCC
import numpy as np
import matplotlib.pyplot as plt

from pybrain.structure import FeedForwardNetwork, SigmoidLayer, LinearLayer, FullConnection #, RecurrentNetwork
from pybrain.tools.xml.networkwriter import NetworkWriter
from pybrain.tools.xml.networkreader import NetworkReader
from pybrain.datasets import SupervisedDataSet
from pybrain.supervised.trainers import BackpropTrainer

# TODO check/write docstrings - include arguments and return values

"""
Global variables.
"""

initialised = False
verbose = True

"""
Classes.
"""

class SensorInput(threading.Thread) :
    """Repeatedly grabs a 'snapshot' of a sensor's state and passes this to another object for processing.
    Should be extended by other classes for dealing with specific inputs. All that should need to be
    changed is __init__() (if other parameters e.g. resolution are needed) and grab().

    See WebcamInput for an example of how to extend this class.
    """

    def __init__(self, refresh=1, output=None, verbose=False) :
	"""Calculate and store delay needed to produce given refresh rate.
	output is checked to ensure it has a process() function that this object can call.
		
	Typically, the input device will be configured in this method.
	"""

	self._verbose=verbose						# store verbose flag

	self._refresh_delay = int(round(1./refresh*1000))		# set delay between frames in ms

	if output is not None :						# if an output has been set
	    try :
		output.process					        # try to find process attr
		self._output = output				        # if found, store output obj
	    except AttributeError :					# if output is inappropriate
		raise TypeError('output must be Processor, ' + 
				'sublass Processor, ', 
				'or be None')			        # raise exception
	else :								# if an output is not set
	    self._output = Processor()				        # use blank Processor

	threading.Thread.__init__(self)					# call Thread superconstructor

    def grab(self) :
	"""This is the single most important method of a SensorInput object, and defines how to take
	data from the sensor. The method should take a single 'snapshot' of the input, and return it.
	This method is repeatedly called from within the main run() loop.
	"""

    def run(self) :
	"""Grab image from webcam and pass resultant array to specified function for processing.
	Loops until stop() is called or self._stop is otherwise set to True.
	"""

	self._stop = False						# set stop flag to False
	self._pause = False						# set pause flag to False
	while self._stop is not True :					# if stop flag is False
	    if self._pause : time.sleep(1)				# if paused, wait 1 second
	    else :							# if not paused
		data = self.grab()				        # grab data from sensor
		if self._verbose : print data			        # if verbose, print captured data
		self._output.process(data)			        # pass data to processing function
		#cv2.waitKey(self._refresh_delay)		        # wait according to refresh rate
                cv2.waitKey(self._refresh_delay)

    def stop(self) :
	"""Halt the main run() loop.
	"""
	self._stop = True						# break run() loop

    def pause(self) :
	"""Pause the main run() loop.
	"""
	self._pause = True						# postpone run() loop

    def resume(self) :
	"""Resume the main run() loop.
	"""
	self._pause = False						# continue run() loop


    def set_refresh(self, refresh) :
	"""Calculate and store delay needed between grab() calls to produce given refresh rate.
	"""
	self._refresh_delay = int(round(1./refresh*1000))		# set delay between frames in ms


class HCSR04Input() :
    def __init__(self) :
        self._SOS = 340.29 * 100
        self.TRIG = al.func_to_pin['rng_trig']
        self.ECHO = al.func_to_pin['rng_echo']
        self.TRIG.write(1)
        cv2.waitKey(500)

    def pulse(self, dur=0.00001) :
        self.TRIG.write(1)
        cv2.waitKey(dur)
        self.TRIG.write(0)

    def read(self, dummy) :
        return [time.time(), self.ECHO.read()]

    def grab(self) :
        axis = range(2000)
        self.pulse()
        t, v = np.array(map(self.read, axis)).T
        if np.sum(v == True) > 1 :
            elapsed = t[v == True][-1] - t[v == True][0]
            distance = 0.5*self._SOS*elapsed
            return distance
        else :
            return np.NaN


class WebcamInput(SensorInput) :
    """Thread takes input from a webcam and passes it to an image processing function.
    """

    def __init__(self, resolution=(10, 10), refresh=1, output=None, index=0, verbose=False) :
	"""Store arguments and produce opencv VideoCapture object with correct resolution etc.
	"""

	self._camcapture = cv2.VideoCapture(index)		        # create and store camera capture object

	if not self._camcapture :				        # if camera capture couldn't be created	
	    raise IOError('Couldn\'t connect to a webcam')	        # print error message

	self.res = resolution					        # store intended resolution
	self._camcapture.set(3, resolution[0])			        # set width of frames to be captured
	self._camcapture.set(4, resolution[1])			        # set height of frames to be captured

	SensorInput.__init__(self, refresh=refresh, 
				    output=output,
				    verbose=verbose)		        # call SensorInput superconstructor

    def grab(self) :
	"""Grabs a single frame from the webcam and returns it as a numpy array.
	"""

	img = self._camcapture.read()[1]			        # capture frame from webcam
	res = np.shape(img)					        # store image's resolution
	if res[0] != self.res[1] or res[1] != self.res[0] :	        # check img has correct resolution
	    img = cv2.resize(img, (self.res[0], self.res[1]),           # resize to correct resolution
				interpolation=cv2.INTER_AREA)
        return img[:, :, ::-1]						# return the image


class RPiCamInput(SensorInput) :
    """Thread takes input from raspberry pi camera module and passes to an image processing function.
    """
    # FIXME 10x10 image -> green mess! use larger resolution and then downsize using sp.misc.imresize

    def __init__(self, resolution=(10, 10), refresh=1, output=None, verbose=False) :
        """Store resolution and produce camera object for later use.
        """
        import picamera
        self._cam = picamera.PiCamera()
        self.res = resolution
        SensorInput.__init__(self, refresh=refresh, 
                                    output=output, 
                                    verbose=verbose)

    def grab(self) :
        """Grab a single frame from the RPi camera and return as a numpy array.
        """
        stream = io.BytesIO()
        self._cam.capture(stream, format='jpeg', use_video_port=True, resize=self.res)
        stream.seek(0)
        data = np.fromstring(stream.getvalue(), dtype=np.uint8)
        img = cv2.imdecode(data, 1)
        stream.close()
        return img[:, :, ::-1]

    def stop(self) :
	"""Halt the main run() loop.
	"""

	self._stop = True						# break run() loop
        self._cam.close()


class ANN :
    """Processes data using an ANN. The ANN can either be built from a configuration file containing training
    data or restored from a persisted network.  
    """

    def __init__(self, filename=None) :
        """Builds or restores a network and provides a simple interface for using the net to process data.
        See build() and load() methods for details.  
	"""

	self.filename = filename					# store filename
	if filename is not None :
	    filename, extension = os.path.splitext(filename)	        # retrieve extension
	
	    if extension == '.txt' :				        # if given a config file
		self.build()					        # call build method
	
	    if extension == '.xml' :				        # if given persisted net
		self.load()					        # call load method

	
    def build(self, filename=None) :
        """Build and train a network according to a supplied configuration file. The config file should be in
        the following format :

	    line number == 1 | input layer size <tab> hidden layer size <tab> output layer size 
	    line number == 2 | acceptable error
	    line number <= 3 | input 1 <tab> input 2 <tab> | <tab> output 1 <tab> output 2

	<tab> : tab separator

        If multiple hidden layer sizes are specified on the first line , multiple hidden layers will be
        included in the built network. E.g. 5 <tab> 4 <tab> 3 <tab> 2 <tab> will built a net with 4 neurons at
        the input layer, 4 at the first hidden layer, 3 at the second hidden layer, and 2 at the output layer.

        Multiple input and output states can be specified in the training data in a similar way.
		
        Blank lines in the data set will be ignored. This is useful for separating training data for
        readability.

	See red_net_config.txt for an example network config file.
	"""
		
	if filename is None :						# if no filename is supplied
	    filename = self.filename				        # use filename given at init

	if filename != self.filename :					# if new filename has been given
	    self.filename = filename				        # store new filename

	config_file = open(filename, 'rb')				# open the data file
	config_lines = list(csv.reader(config_file, delimiter='\t'))	# read as csv with tab delim's
	layers = [int(size) for size in config_lines[0]]		# split off layer config
	acceptable_error = float(config_lines[1][0])			# split off acceptable error
	dataset = config_lines[2:]					# split off data set

	# build empty network TODO variable network types
	self.network = FeedForwardNetwork()				# build the net

	# form layers TODO variable layer types
	input_size = layers[0]
	output_size = layers[-1]
	input_layer = LinearLayer(input_size)				# form input layer
	hidden_layers = [SigmoidLayer(size) for size in layers[1:-1]]	# form hidden layers
	output_layer = SigmoidLayer(output_size)			# form output layer

	# add layers to network
	self.network.addInputModule(input_layer)			# add input layer
	[self.network.addModule(layer) for layer in hidden_layers]	# add hidden layers
	self.network.addOutputModule(output_layer)			# add output layer

	# form connections TODO variable connection types and topologies
	in_to_h = FullConnection(input_layer, hidden_layers[0])		# form input -> first hidden
	h_to_out = FullConnection(hidden_layers[-1], output_layer)	# form last hidden -> output
	h_to_h = []							# list for hidden conn's
	for x in range(len(hidden_layers)) :				# count through hidden layers
	    if x is not len(hidden_layers) - 1 :			# if not at last hidden layer
		hh_conn = FullConnection(hidden_layers[x], 	        # form hidden n -> 
					    hidden_layers[x+1])	        # hidden n + 1 connection
		h_to_h.append(hh_conn)				        # add to list of hidden conn's

	# add connections to network
	self.network.addConnection(in_to_h)				# add input -> first hidden
	self.network.addConnection(h_to_out)				# add last hidden -> output
	[self.network.addConnection(hh_conn) for hh_conn in h_to_h]	# add hidden n -> hidden n + 1
		
	# solidify network
	self.network.sortModules()					# sort network topology

	# train network TODO variable trainer types
	self.dataset = SupervisedDataSet(input_size, output_size)	# form data set 
	for mapping in dataset :
	    input_data = tuple([float(input_state) for input_state in mapping[0:input_size]])
	    output_data = tuple(float(output_state) for output_state in mapping[0 - output_size:])
	    if input_data is not tuple([]) :
		self.dataset.addSample(input_data, output_data)
	trainer = BackpropTrainer(self.network, self.dataset)		# form trainer

	trained = False							# set trained flag to False
	epoch = 0							# set epoch to 0
	self._pause = False
	while trained is False :					# as long as net isn't trained
	    if self._pause : time.sleep(1)				# if paused, wait a second
	    else :
		epoch += 1					        # increment epoch counter
		error = trainer.train()				        # reduce the error
		print('epoch : %i error : %f' % (epoch, error))	        # print current error
		if error < acceptable_error :			        # if error is acceptable
		    trained = True				        # set trained flag to True
	

    def load(self, filename=None) :
        """Loads a persisted network from XML file. Cannot load pickled networks yet.
	"""

	if filename is None :						# if not given a filename
	    filename = self.filename				        # use filename given at init

	if filename != self.filename :					# if new filename has been given
	    self.filename = filename				        # store new filename

	self.network = NetworkReader.readFrom(filename)			# load the network


    def save(self, filename) :
        """Saves a network to XML file. Cannot pickle networks yet.
	"""

	NetworkWriter.writeToFile(self.network, filename)		# save the network
	

    def process(self, data, axis=2) :
        """Activates the network using data in a numpy array and stores output as numpy array. Axis defaults
        to 2 because this allows quick application of a net to an image's RGB values.
	"""

	if type(data) is not type(np.array([])) :			# if not given a numpy array
		data = np.array(data)					# convert to numpy array

	processed_data = np.apply_along_axis(self.network.activate, 	# activate net using data
						axis, data)		# and store net's output
	return processed_data						# return output of net


class Processor() :
    """ Class to be extended by all classes for processing 'event-driven' input streams such as that from a
    repeatedly refreshed webcam input. The process() function will be called repeatedly as new input data is
    grabbed. By setting other Processors as outputs, Processors may be formed into branching linked list-like
    processing chains.
    """

    def __init__(self, output=None, bypass=False, verbose=False) :
	"""Store the given parameters. 
	"""

	if output is not None :						# if an output has been set
	    try :
		output.process					        # try to find process attr
		self._output = output				        # if found, store output obj
	    except AttributeError :					# if output is inappropriate
		raise TypeError('output must be Processor, ' + 
				'sublass Processor, ', 
				'or be None')			        # raise exception

	self._bypass = bypass						# set bypass flag
	self._verbose = verbose						# set verbose flag
	

    def process(self, data) :
        """Deal with data in some way. In this case, data is simply printed (so long as verbose is True and
        the Processor hasn't been told to bypass processing).
	"""

	if self._bypass is False :					# if not told to bypass
	    output_data = self.run(data)				# process data with run method
	else :								# if bypass is active
	    if self._output is not None :				# check output object is set
		self._output.process(output_data)		        # pass data to output object


    def run(self, data) :
	"""Deal with data in some way. This is called from process() once bypass has been checked.
	"""

	if self._verbose :						# if verbose mode is on
	    print data						        # print the data
	return data							# return data as is


    def set_bypass(self, bypass) :
	"""Tell process() function to bypass processing step and pass data straight to output.
	"""

	if bypass == True or bypass == False :				# if value is boolean
	    self._bypass = bypass					# set bypass value


class MotorVectorProcessor(Processor) :
    """Passes an input motor vector to the MCC to cause movement.
    """
	
    def run(self, mvec) :
        """Passes mvec to MCC and then returns mvec without further processing. mvec should be indexable. The
        first two elements will be used as left and right drive commands, respectively.
        """

	if MCC.initialised :						# if MCC is initialised
	    MCC.drive((mvec[0], mvec[1]))				# send motor vector to MCC

	    if verbose :						# if verbose flag is True
		print('Motor Vector : %s' % (str(mvec)))	        # report the motor vector
	else :								# if MCC is not initialised
	    if verbose :						# if verbose flag is True
		print('Motor Vector : MCC not initialised!')	        # report MCC is not initialised

	return mvec							# return motor vector


class BullProcessor(Processor) :
    """Implements a behaviour in which the robot will charge towards red areas of its visual field.
    """
	
    def __init__(self, output=None, bypass=False, verbose=False) :
        """Produce a webcam input object, a neural net for detecting redness, and a neural net for producing
        motor output vectors.
	"""

	self._red_net = ANN('red_net.xml')				# load red detecting net
	self._move_net = ANN('move_net.xml')				# load motor output net

	Processor.__init__(self, output=output, 
				 bypass=bypass, 
				 verbose=verbose)			# call Processor superconstructor

    def run(self, img, gui=False, amp=1) :
	"""Method for performing the behaviour. 
	"""

        if gui :
	    plt.ion()
    	    plt.show()
			
	if self._bypass is False :
	    img = img / 255.					        # scale frame to 0..1 
	    img[:, :, [0, 2]] = img[:, :, [2, 0]]			# convert to RGB 
				
	    red_frm = self._red_net.process(img)[:, :, 0]		# use red detecting net
            av_frm = np.average(red_frm.T, 1)                           # average red values for columns

	    m_vec = self._move_net.process(av_frm, axis=0)		# produce motor vector
	    m_vec = (m_vec * 2) - 1					# normalise motor vector
	    m_vec = m_vec * amp					        # amplify for testing

            if gui :
	        self.plot(av_frm=av_frm, mvec=m_vec)			# plot the motor vector

	    if self._verbose :					        # if verbose flag is set to True
		print('Motor Vector : %s' % (str(m_vec)))	        # report the motor vector

	return m_vec							# return the motor vector


    def plot(self, av_frm=None, mvec=None) :
        """Plots a motor vector, corrected to look nice on a graph (i.e. forward-backward axis = y-axis of
        graph).
	"""

	theta = 45. * (np.pi / 180.)					# 45 degrees -> radians
	rot_mat = np.array([[np.cos(theta), -np.sin(theta)], 		# matrix to rotate 2D vector 
			    [np.sin(theta), np.cos(theta)]])		# by theta angle
	rot_mvec = np.dot(rot_mat, mvec)				# rotate motor vector

	x = np.arange(-1.4, 1.5, 0.3) + 0.05				# x axis values
	plt.plot(x, av_frm * 1.5, c='r', linewidth=0.3)			# plot col-averaged red frame
	plt.scatter(rot_mvec[0], rot_mvec[1], s=10.3)			# plot motor vector
	plt.xlim(x[0], x[-1])						# set range of x axis
	plt.ylim(x[0], x[-1])						# set range of y axis
	plt.grid(True)							# show a grid


"""
Methods.
"""

cam = None
def initialise() :
    """Implements a basic behaviour - the robot charges at red areas in its visual field. This will ordinarily
    be called during execution of boot.py, and requires the MCC to be initialised.
    """

    global initialised, cam, verbose

    if initialised is False :						# check IPC is not yet initialised
	bull = BullProcessor(verbose=verbose)				# create bull-like behaviour object
	printer = Processor(verbose=verbose)
	#cam = WebcamInput(output=printer, refresh=0.1)			# process webcam input with bull
        cam = RPiCamInput(output=printer, refresh=0.1)               # process webcam input with bull
	cam.start()							# start camera capture
	initialised = True						# update initialised flag
    else :								# if already initialised
	print 'IPC has already been initialised!'			# tell user


def terminate() :
    """Stops the main loop of the camera used by initialise() method.
    """

    global initialised, cam

    if initialised :							# check IPC is initialised
	cam.stop()							# break cam's main loop
	initialised = False						# update initialised flag
    else :								# if not initialised
	print 'IPC has not been initialised!'				# tell user

if __name__ == '__main__' :						# if run as a script
    initialise()							# initialise
