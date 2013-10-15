#--IMPORTS--
from gestalt import nodes
from gestalt import utilities
from gestalt.utilities import notice
from gestalt import functions as functions
from gestalt import packets as packets
import math
import time
#define rs232 messages
ENQ = '\x05'
STX = '\x02'
ETX = '\x03'
ACK = '\x06'
NAK = '\x15'
EOT = '\x04'

class virtualNode(nodes.baseSoloIndependentNode):

	def init(self):
		self.interface.setTimeout(0.5)	#sets a general timeout of 0.5 seconds for the serial port
	
	def calc_chksum(self, packet):
	# calculate the total and transmit last 2 hex numbers as strings for the efd checksum
		total = 0
		for i in packet:
			total += ord(i)
		chksum = 0-total
		chksum = chksum & 0xFF
		high = str(hex(chksum))[2]
		low = str(hex(chksum))[3]
		return (high, low)
	
	def dispense(self):
	# toggles the dispensing
		self.interface.transmit(ENQ)
		time.sleep(.001) #whatever ack
		#start transmission
		self.interface.transmit(STX)
		packet = '03' + 'DI ' #3 bytes, start stop dispensing
		#send packet
		for i in packet:
			self.interface.transmit(i)
		#send chksum
		(high, low) = self.calc_chksum(packet)
		self.interface.transmit(high)
		self.interface.transmit(low)
		#end transmission
		self.interface.transmit(ETX)

	def set_pressure(self, pressure):
		self.interface.transmit(ENQ)
		time.sleep(0.001)
		self.interface.transmit(STX)
		packet = '08' + 'PS  ' +'0' + str(pressure) + '0'
		for i in packet:
			self.interface.transmit(i)
		(high, low) = self.calc_chksum(packet)
		self.interface.transmit(high)
		self.interface.transmit(low)
		self.interface.transmit(ETX)

