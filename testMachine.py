from gestalt.Nodes import printrboard
from gestalt import nodes
from gestalt.Nodes import dummyNetworkedNode
from gestalt import interfaces
import time

myInterface = interfaces.gestaltInterface('myInterface', interfaces.serialInterface(baudRate = 115200, interfaceType = 'ftdi', portName = '/dev/tty.usbserial-FTVG67VT'))
myFabUnit = nodes.networkedGestaltNode('myFabUnit', myInterface, filename = '086-005a.py')

#myFabUnit.loadProgram('086-005a.hex')
#myFabUnit.identifyRequest()
#print myFabUnit.urlRequest()

myFabUnit.setMotorCurrent(0.5)
print myFabUnit.stepConfigRequest(0, 1000)
print myFabUnit.stepSync(10000, 1000, 0, 500)

for i in range(13):
	print myFabUnit.getMoveStatusRequest()
	time.sleep(1)


myFabUnit.disableDriver()