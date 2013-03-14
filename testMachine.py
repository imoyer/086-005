from gestalt.Nodes import printrboard
from gestalt import nodes
from gestalt.Nodes import dummyNetworkedNode
from gestalt import interfaces
import time
import math

class axis(object):
	def __init__(self, virtualNode, lead = 2.54, stepsPerRev = 200.0, microSteps = 1.0, direction = 1):
		self.virtualNode = virtualNode
		self.lead = lead
		self.stepsPerRev = stepsPerRev
		self.microSteps = microSteps
		self.direction = direction
	
	def loadProgram(self, programName):
		self.virtualNode.loadProgram(programName)
		
	def setMotorCurrent(self, motorCurrent):
		self.virtualNode.setMotorCurrent(motorCurrent)
		
	def getSteps(self, delta):
		return round(delta*self.stepsPerRev*self.microSteps*self.direction/self.lead,0)
	
	def getDelta(self, steps):
		return self.lead / (float(steps) * self.stepsPerRev * self.microSteps)


class virtualMachine(object):
	def __init__(self):
		self.fabnet = interfaces.gestaltInterface('FABNET', interfaces.serialInterface(baudRate = 115200, interfaceType = 'ftdi', portName = '/dev/tty.usbserial-FTVG67VT'))
		self.xAxis = axis(nodes.networkedGestaltNode('X Axis', self.fabnet, filename = '086-005a.py'))
		self.yAxis = axis(nodes.networkedGestaltNode('Y Axis', self.fabnet, filename = '086-005a.py'))
		self.zAxis = axis(nodes.networkedGestaltNode('Z Axis', self.fabnet, filename = '086-005a.py'))	

		self.motorCurrent = 1.0	#amps
		self.motorsInitialized = False	#gets set to true once motors have been run once.
		
		self.machinePosition = [0.0, 0.0, 0.0]
		
		
	def loadProgram(self, programName):
		self.xAxis.loadProgram(programName)
		self.yAxis.loadProgram(programName)
		self.zAxis.loadProgram(programName)

	def setMotorCurrents(self, motorCurrent):
		self.xAxis.setMotorCurrent(self.motorCurrent)
		self.yAxis.setMotorCurrent(self.motorCurrent)
		self.zAxis.setMotorCurrent(self.motorCurrent)
		
	def move(self, x = None, y = None, z = None, speed = 20):	#speed in mm/sec
		if x: xDelta = x - self.machinePosition[0]
		else: xDelta = 0
		if y: yDelta = y - self.machinePosition[1]
		else: yDelta = 0
		if z: zDelta = z - self.machinePosition[2]
		else: zDelta = 0
		
		def direction(value):
			if value>=0: return 1
			else: return 0
		
		diagonalDelta = math.sqrt(math.pow(xDelta,2)+math.pow(yDelta,2)+math.pow(zDelta,2))
		moveTime = diagonalDelta/float(speed)	#mm/sec
		
		xSteps = self.xAxis.getSteps(xDelta)
		ySteps = self.yAxis.getSteps(yDelta)
		zSteps = self.zAxis.getSteps(zDelta)
		
		maxSteps = max(abs(xSteps), abs(ySteps), abs(zSteps))
		velocity = int(float(maxSteps)/float(moveTime))		#steps/sec
		
		
		self.xAxis.virtualNode.stepConfigRequest(direction(xSteps), abs(xSteps))
		self.yAxis.virtualNode.stepConfigRequest(direction(ySteps), abs(ySteps))
		self.zAxis.virtualNode.stepConfigRequest(direction(zSteps), abs(zSteps))
		
		while self.xAxis.virtualNode.getMoveStatusRequest() != 0:	#prior move still pending
			time.sleep(0.05)
		
		self.xAxis.virtualNode.stepSync(maxSteps, velocity, velocity, 0)	#start move, no acceleration
		
		#update machine position
		self.machinePosition[0] += self.xAxis.getDelta(xSteps)
		self.machinePosition[1] += self.yAxis.getDelta(ySteps)
		self.machinePosition[2] += self.zAxis.getDelta(zSteps)
		





EFD = nodes.soloIndependentNode(name = 'EFD', interface = interfaces.serialInterface(baudRate = 115200, interfaceType = 'lufa',
																								portName = "/dev/tty.PL2303-000012FD"),
									filename = 'ultimus.py')

#myMachine = virtualMachine()
#myMachine.move(25, 25, 25, 50)
#
#EFD.dispense()
#time.sleep(5)

fabnet = interfaces.gestaltInterface('FABNET', interfaces.serialInterface(baudRate = 115200, interfaceType = 'ftdi', portName = '/dev/tty.usbserial-FTVG67VT'))
myNode = nodes.networkedGestaltNode('myNode', fabnet, filename = '086-005a.py')
#myNode.loadProgram('086-005a.hex')
myNode.setMotorCurrent(0.5)
myNode.stepConfigRequest(1, 3000)
myNode.stepSync(10000,1000,0,500)
for i in range(12):
	print myNode.getMoveStatusRequest()
	time.sleep(1)
myNode.disableDriver()