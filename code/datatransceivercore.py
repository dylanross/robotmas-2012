import threading
import socket
import struct
import time

"""
Deals with all sending and receiving of data over the network. E.g. used to receive and parse OSC motor
commands from an iOS TouchOSC interface.  
"""

"""
Global variables
"""

initialised = False

host_ip = socket.gethostbyname(socket.gethostname())			# store the host's ip address
host_ip = '172.20.114.27' # store the host's ip address
default_port = 5005							# default port for UDP communication

"""
Classes.
"""

class Parser :
    """Class to be extended by classes for parsing specific formats of input.
    """

    def __init__(self) :
	"""Do nothing."""

    def parse(self, data) :
	"""Parse an input."""
	return data


class OSCParser(Parser) :
    """Parser for OSC input. Mediates between messages received by receiver classes (e.g.
    UDPReceiver) and the commands available in the CommandCore class(es).
    """

    __name__ = 'OSCParser'

    def __init__(self) :
	Parser.__init__(self)

    def parse(self, data) :
	"""Parses an input in OSC format.
	"""
		
	try :
	    p_data = data.rsplit(',', 2)			        # split message / value pair
	    value = self.convert_byte(p_data[1])		        # process value
	    command = p_data[0].rsplit('/', 1)[1]		        # split command from message
	    command = command.split(b'\x00', 1)[0]		        # clean \x00 bytes from command
	    return (command, value)				        # return the c/v tuple
        except :
	    print 'OSCParser couldn\'t parse data :', data	        # print data that couldn't be parsed

    def convert_byte(self, data) :
        """ Converts byte representation to another data type.  First character determines the data
        type (e.g. inputting f?N?? would lead to ?N?? being processed as a float).
	"""

	if type(data) is type(str()) :				        # check data is a string
	    data_values = [chr(byte) for byte in bytearray(data)[::-1][:4]]	
	    data_value = ''.join(data_values)
	    output = struct.unpack('f', data_value)[0]	                # unpack last 4 bytes to float
	else :
	    raise TypeError('Byte must be provided as string!')

	return output						        # return parsed value


class StringCommandParser(Parser) :
    def __init__(self) :
        Parser.__init__(self)

    def parse(self, data) :
        try :
            command, value = data.lower().split(' ')
            value = eval(value)
            return (command, value)
	except :
	    print 'StringCommandParser couldn\'t parse data :', data	# print data that couldn't be parsed


class UDPReceiver(threading.Thread) :
    """Receives UDP messages and passes them to an input parser for further processing.

    The following 'hidden' attributes exist. Use at own risk.
		
	_sock		socket for UDP communication
	_ip		        ip to which sock is currently bound
	_port		port to which sock is currently bound
	_parser		parser for dealing with incoming messages
	_listen		setting this to anything but True will cause the run() loop to end
    """
	
    def __init__(self, ip=host_ip, port=default_port, parser=None, effector=None) :
        """Optionally pass an IP, port, parser, and effector to be used. Defaults are host ip and
        default port (see global variables, above), and the do-nothing Parser parser.  
	"""
	if type(ip) is not type(str()) :			        # check ip is given as a string
	    raise TypeError('udp_ip must be a string')	                # raise exception if it isn't
	if type(port) is not type(int()) :			        # check port is given as an int
	    raise TypeError('udp_port must be an int')	                # raise exception if it isn't
	if type(parser) is not type(Parser()) :			        # check parser is a Parser
	    if type(parser) is type(None) :			        # if None, default to OSCParser
		parser = OSCParser()
	    else :						        # raise exception if not None or OSC.
		raise TypeError('parser must be a Parser')
	if type(effector) is None :				        # if no effector set, tell user
	    print('No effector set for UDPReceiver -',
		    'received instructions will be printed.')
		
	self._ip = ip						        # store ip 
	self._port = port					        # store port 
	self._parser = parser					        # store parser
	self._effector = effector				        # store effector

	self._sock = socket.socket(socket.AF_INET, 		        # communicate over internet
				   socket.SOCK_DGRAM)		        # using UDP protocol

	threading.Thread.__init__(self)				        # run Thread superconstructor

    def run(self) :
        """Starts the socket listening for messages, and does SOMETHING when a message is received.
	"""

	self.bind(self._ip, self._port)				        # bind socket to ip and port
	self._listen = True					        # set listen flag to True

	while self._listen is True :				        # continuous while loop :
	    data, addr = self._sock.recvfrom(9600)		        # get data and sender's address
	    if self._listen is not False :			        # check hasn't been told to die
		instruction = self._parser.parse(data)		        # use parser to deal with data
		if instruction is not None :			        # check instruction exists
		    if self._effector is None :		                # if no effector set
			print instruction		                # print the parsed instruction
		    else :					        # if an effector is set
			self._effector(instruction)	                # pass instruction to effector
		time.sleep(0.01)
	    else :						        # if thread told to die
		break						        # break from the while loop

    def stop(self) :
	"""Tell core while loop of run() method to stop.
	"""

	self._listen = False					        # set listen to false
	quick_send('quit')					        # send quit message

    def bind(self, ip, port) :
	"""Bind the internal socket to a new ip / port combination.
	"""

	self._ip = ip						        # store new ip
	self._port = port					        # store new port
	self._sock.bind((self._ip, self._port))			        # bind socket to ip / port
		
    def get_ip(self) :
	"""Return the currently bound ip address.
	"""

	return self._ip						        # return currently bound ip
	
    def get_port(self) :
	"""Return the currently bound port.
	"""

	return self._port					        # return currently bound port

"""
Methods.
"""

_txt_r = None
_osc_r = None
def initialise(ip=None, txt_port=None, osc_port=None, effector=None) :
    """Initialise the data transceiver core by starting OSC parsing and plain text parsing UDP servers.
    """
	
    global initialised, _txt_r, _osc_r				        # use global variables
	
    if ip is None : ip = host_ip
    if txt_port is None : txt_port = default_port
    if osc_port is None : osc_port = default_port + 1

    _txt_r = UDPReceiver(ip=ip, port=txt_port, 
            parser=StringCommandParser(), effector=effector)	        # create UDPReceiver paired to effector
    _txt_r.start()							# start the UDPReceiver thread
    print 'Receiving at', _txt_r._ip, ':', _txt_r._port 		# report start of thread
    print 'Using', _txt_r._parser.__class__.__name__, 'parser'	        # report parser used

    _osc_r = UDPReceiver(ip=ip, port=osc_port, 
            parser=OSCParser(), effector=effector)	                # create UDPReceiver paired to effector
    _osc_r.start()							# start the UDPReceiver thread
    print 'Receiving at', _osc_r._ip, ':', _osc_r._port 		# report start of thread
    print 'Using', _osc_r._parser.__class__.__name__, 'parser'	        # report parser used

    initialised = True						        # update initialised flag


def terminate() :
    """Stop initialised UDPReceiver threads. MUST be called before quitting if initialise() has been
    called.
    """

    global initialised, _txt_r, _osc_r				        # use global variables

    _txt_r.stop()
    _osc_r.stop()							# stop the UDPReceiver thread

    initialised = False						        # update initialised flag


def quick_send(message, ip=host_ip, port=default_port) :
    """
    Sends a message to the specified ip and port.
    Default port and host's ip are used by default for quick testing.
    """

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)		# create new socket at ip / port
    sock.sendto(message, (ip, port))				        # send message to ip / port
    sock.close()							# close socket


if __name__ == '__main__' :						# if run as a script
    initialise()							# run initialise method
