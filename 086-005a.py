from gestalt import nodes
from gestalt import utilities
from gestalt.utilities import notice
from gestalt import functions as functions
from gestalt import packets as packets
import time

class virtualNode(nodes.baseNetworkedGestaltNode):
	def init(self, **kwargs):
		pass
	
	def initParameters(self):
		self.clockFrequency 	= 18432000.0	#Hz
		self.ADCRefVoltage = 5.0	#voltage divider to ref pin is 5V -> 10K -> REF -> 5K -> GND
		self.motorSenseResistor = 0.1	#ohms
		self.timerTop 			= 921.0	#clock cycles
		self.timePeriod 		= self.timerTop / self.clockFrequency	#seconds

	def initFunctions(self):
		pass
		
	def initPackets(self):
		self.enableRequestPacket = packets.packet(template = [])
		self.enableResponsePacket = packets.packet(template = [])
		
		self.disableRequestPacket = packets.packet(template = [])
		self.disableResponsePacket = packets.packet(template = [])
		
		self.stepConfigRequestPacket = packets.packet(template = [packets.pInteger('direction',1),
														packets.pInteger('steps',2)])
		self.stepConfigResponsePacket = packets.packet(template = [])
		
		self.stepSyncPacket = packets.packet(template = [packets.pInteger('majorSteps',2),
													packets.pInteger('minVelocity',2),
													packets.pInteger('maxVelocity',2),
													packets.pInteger('acceleration',1)])
		
		self.getMoveStatusRequestPacket = packets.packet(template = [])		
		self.getMoveStatusResponsePacket = packets.packet(template = [packets.pInteger('status', 2)])
		
		self.getReferenceVoltageRequestPacket = packets.packet(template = [])
		self.getReferenceVoltageResponsePacket = packets.packet(template = [packets.pInteger('voltage',2)])
		
		
	def initPorts(self):
		#enable drivers
		self.bindPort(port = 10, outboundFunction = self.enableRequest, outboundPacket = self.enableRequestPacket,
								inboundFunction = self.enableResponse, inboundPacket = self.enableResponsePacket)

		#disable drivers
		self.bindPort(port = 11, outboundFunction = self.disableRequest, outboundPacket = self.disableRequestPacket,
								inboundFunction = self.disableResponse, inboundPacket = self.disableResponsePacket)
		
		#config step move
		self.bindPort(port = 12, outboundFunction = self.stepConfigRequest, outboundPacket = self.stepConfigRequestPacket,
								inboundFunction = self.stepConfigResponse, inboundPacket = self.stepConfigResponsePacket)
		
		#execute step move
		self.bindPort(port = 13, outboundFunction = self.stepSync, outboundPacket = self.stepSyncPacket)	#no inbound packet because sync gets no response
		
		#get status
		self.bindPort(port = 15, outboundFunction = self.getMoveStatusRequest, outboundPacket = self.getMoveStatusRequestPacket,
								inboundFunction = self.getMoveStatusResponse, inboundPacket = self.getMoveStatusResponsePacket)
		
		self.bindPort(port = 20, outboundFunction = self.getReferenceVoltageRequest, outboundPacket = self.getReferenceVoltageRequestPacket,
								inboundFunction = self.getReferenceVoltageResponse, inboundPacket = self.getReferenceVoltageResponsePacket)
	

#----- API FUNCTIONS --------------------
	def setMotorCurrent(self, setCurrent):
		if setCurrent < 0 or setCurrent > 2.0:
			notice(self, "Motor current must be between 0 and 2.0 amps. " + str(setCurrent) + " was requested.")
		setCurrent = round(float(setCurrent), 1)
		while True:
			motorVoltage = self.getReferenceVoltage()
			motorCurrent = round(motorVoltage /(8.0*self.motorSenseResistor),2)	#amps
			if round(motorCurrent,1) > setCurrent:
				notice(self, "Motor current: " + str(motorCurrent) + "A / " + "Desired: " + str(setCurrent) + "A. Turn trimmer CW.")
			elif round(motorCurrent,1) < setCurrent:
				notice(self, "Motor current: " + str(motorCurrent) + "A / " + "Desired: " + str(setCurrent) + "A. Turn trimmer CCW.")
			else:
				notice(self, "Motor current set to: " + str(motorCurrent) + "A")
				break;
			time.sleep(0.5)
		

	def getReferenceVoltage(self):
		ADCReading = self.getReferenceVoltageRequest()
		if ADCReading:
			return self.ADCRefVoltage * ADCReading / 1024.0
		else:
			return False
	
	def enableDriver(self):
		return self.enableRequest()
	
	def disableDriver(self):
		return self.disableRequest()
	
#----- SERVICE ROUTINES -----------------
	class enableRequest(functions.gFunction):
		class gFunctionCore(functions.gFunctionObject):
			def init(self):
				self.updatePacketSet({})
				self.transmit('unicast')
				if self.waitForResponse(0.2): 
					notice(self.virtualNode, "Stepper motor enabled.")
					return True
				else: 
					notice(self.virtualNode, "Stepper motor could not be enabled.")
					return False
				
	class enableResponse(functions.gFunction):
		pass
	
	class disableRequest(functions.gFunction):
		class gFunctionCore(functions.gFunctionObject):
			def init(self):
				self.updatePacketSet({})
				self.transmit('unicast')
				if self.waitForResponse(0.2): 
					notice(self.virtualNode, "Stepper motor disabled.")
					return True
				else: 
					notice(self.virtualNode, "Stepper motor could not be disabled.")
					return False
				
	class disableResponse(functions.gFunction):
		pass

	class stepConfigRequest(functions.gFunction):
		class gFunctionCore(functions.gFunctionObject):
			def init(self, direction, steps):
				#	direction:	0/1
				#	steps: driver steps

				self.updatePacketSet({'direction':direction, 'steps': int(steps)})
				self.transmit('unicast')
				if self.waitForResponse(0.2): return True
				else: return False

	class stepConfigResponse(functions.gFunction):
		pass

	class stepSync(functions.gFunction):
		class gFunctionCore(functions.gFunctionObject):
			def init(self, majorSteps, maxVelocity, minVelocity = 0, acceleration = 0):
				#	maxVelocity: steps/sec
				#	minVelocity: steps/sec
				#	acceleration: steps/sec^2
				
				uMaxVelocity = maxVelocity * 1048576.0 * self.virtualNode.timePeriod	#uSteps/timePeriod
				uMinVelocity = minVelocity * 1048576.0 * self.virtualNode.timePeriod	#uSteps/timePeriod
				uAcceleration = acceleration * 1048576.0 * self.virtualNode.timePeriod * self.virtualNode.timePeriod #uSteps/timePeriod^2
				
				if uMaxVelocity > 1048576.0:
					notice(self.virtualNode, 'Request exceeds maximum velocity limit! No move initiated.')
					return False
				print int(uAcceleration)
				self.updatePacketSet({'majorSteps': int(majorSteps), 'minVelocity': int(uMinVelocity / 1024.0),
									  'maxVelocity': int(uMaxVelocity / 1024.0), 'acceleration': int(uAcceleration)})
				self.transmit('multicast')
				return

	class getReferenceVoltageRequest(functions.gFunction):
		class gFunctionCore(functions.gFunctionObject):
			def init(self):
				self.updatePacketSet({})
				self.transmit('unicast')
				if self.waitForResponse(0.2): return self.getPacket()['voltage']
				else: return False
				
	class getReferenceVoltageResponse(functions.gFunction):
		pass
	
	class getMoveStatusRequest(functions.gFunction):
		class gFunctionCore(functions.gFunctionObject):
			def init(self):
				self.updatePacketSet({})
				self.transmit('unicast')
				if self.waitForResponse(0.2): return self.getPacket()['status']
				else: return False
				
	class getMoveStatusResponse(functions.gFunction):
		pass	