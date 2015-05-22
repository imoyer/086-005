#include <gestalt.h>
#include <avr/io.h>
#include <avr/delay.h>
#include <avr/interrupt.h>

//STEPPER CONTROL PORT/PIN ASSIGNEMNTS
#define motorPORT   PORTC
#define motorDDR    DDRC
#define motorPIN    PINC
#define motorStep   PC0
#define motorDir    PC1
#define motorReset  PC2
#define motorMS1    PC3
#define motorMS0    PC4
#define motorEnable PC5
//Motor Current REF is on ADC7


//DEFINE NETWORK PORTS
const uint8_t getVrefPort         = 20; //reports back the current motor reference voltage
const uint8_t enableDriverPort    = 21; //enables the drivers
const uint8_t disableDriverPort   = 22; //disables the drivers
const uint8_t movePort            = 23; //move
const uint8_t setVelocityPort     = 24; //sets current velocity
const uint8_t spinStatusPort      = 26; //get spin status

const uint8_t syncPort            = 30; //triggers a sync. This is a soft proxy for the sync control line.

//Move Packet Definition
#define moveMajorSteps  0
#define moveDirections  1
#define moveSteps       2
#define moveAccel       3
#define moveAccelSteps  4
#define moveDeccelSteps 5
#define moveSync        6

//--Move Status Definition
#define statusCode            0
#define statusCurrentKey      1
#define statusStepsRemaining  2
#define statusReadPosition    3
#define statusWritePosition   4

#define velocity0       0
#define velocity1       1


//CONFIGURE URL
char myurl[] = "http://www.fabunit.com/vn/086-005a.py";

//STEP CONTROL VARIABLE AND MEMORY

struct moveSegment{ //8 bytes total
  volatile uint8_t segmentKey; //keeps track of segment number for status reports, pausing, etc.
  volatile uint8_t majorSteps;  //virtual major steps to take
  volatile uint8_t directions; //0b0000000a
  volatile uint8_t steps; //number of steps to take
  volatile uint8_t accel; //accel/decel rate for this segment
  volatile uint8_t accelSteps;  //number of steps over which to accelerate
  volatile uint8_t decelSteps; //number of steps over which to decelerate
  volatile uint8_t waitForSync;  //if>0 then this move is waiting for a synchronization packet.
};

const uint8_t bufferLength = 64;  //512 bytes, on an atmega32x total memory is 2K.
struct moveSegment moveBuffer[bufferLength];  //stores all buffered moves

//CIRCULAR BUFFER INDEXES
//When a new packet comes in, the write buffer position gets incremented and then that location is written to.
//The main process (or maybe the step generator process) detects that the write buffer is ahead of
//the read buffer, and increments the read buffer position and then reads that location into the step generator.
volatile uint8_t readPosition = 0; //gets incremented and then read, so reflects location that was last read
volatile uint8_t writePosition = 0; //gets incremented and then written, so reflects location that was last written to.
volatile uint8_t segmentKeyCounter = 255; //used to pull new segment keys. The counter is incremented, and then a key is pulled.
volatile uint8_t syncSearchPosition = 0; //the last buffer location where a search for a sync packet has been conducted.


//DIRECTION MASKS
const uint8_t aDirectionMask = 1; //only one axis.

//BRESENHAM VARIABLES
//Default units for packets are 1/4 steps, but system is operating at 1/16 steps.
volatile uint16_t majorSteps;
volatile uint16_t majorError; //majorError = majorSteps/2
volatile uint16_t majorStepsRemaining;
volatile int16_t aSteps;
volatile int16_t aError;

//STEP GENERATOR VARIABLES
volatile uint8_t microstepping = 2; //microstep resolution is 4*2^microstepping (base is 1/4 step, this variable is the bit shift)
volatile uint16_t accelSteps = 0; //current segment accel steps
volatile uint16_t decelSteps = 0; //current segment Decel steps
volatile uint8_t uAccel = 0; //current segment acceleration
volatile uint32_t uVelocity = 0;
volatile uint32_t uPosition = 0;  //uSteps
const uint32_t uSteps = 1048576;  //uSteps per step (2^20)
volatile uint8_t waitingForSync = 0;  //this will get set to 1 when the move interrupt is waiting on a sync.
volatile uint8_t antiLockout = 0; //if 1, indicates that velocity should be returned to zero at end of move.


//USER SETUP
void userSetup(){
  setURL(&myurl[0], sizeof(myurl));

  //PIN AND PORT CONFIGURATION FOR SINGLE STEPPER
  //LED
  IO_ledPORT = &PORTB;
  IO_ledDDR = &DDRB;
  IO_ledPIN = &PINB;
  IO_ledPin = 1<<3;

  //FABNET BUTTON
  IO_buttonPORT = &PORTB;
  IO_buttonDDR = &DDRB;
  IO_buttonPIN = &PINB;
  IO_buttonPin = 1<<2;

  //FABNET TXRX
  IO_txrxPORT = &PORTD;
  IO_txrxDDR = &DDRD;
  IO_rxPin = 1<<0;
  IO_txPin = 1<<1;

  //FABNET TX ENABLE
  IO_txEnablePORT = &PORTD;
  IO_txEnableDDR = &DDRD;
  IO_txEnablePin = 1<<2;

  ADMUX = (0<<REFS1)|(1<<REFS0)|(0<<ADLAR)|(0<<MUX3)|(1<<MUX2)|(1<<MUX1)|(1<<MUX0);   //External Reference, Right Adjusted, External Reference
  ADCSRA = (1<<ADEN)|(0<<ADSC)|(0<<ADATE)|(0<<ADIF)|(0<<ADIE)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);  //Enable ADC, clock source is CLK/128

  motorDDR |= (1<<motorStep)|(1<<motorDir)|(1<<motorReset)|(1<<motorMS1)|(1<<motorMS0)|(1<<motorEnable);  //set all motor pins to outputs
  motorPORT |= (1<<motorReset)|(1<<motorEnable)|(1<<motorMS0)|(1<<motorMS1); //start with motor disabled, not in reset, 1/16 stepping
  motorPORT &= ~(1<<motorDir)|(1<<motorStep); //dir in reverse, step low, (note ~)

  //CONFIGURE TIMER 1 FOR STEP GENERATION
  TCCR1A = (0<<COM1A1)|(0<<COM1A0)|(0<<COM1B1)|(0<<COM1B0)|(0<<WGM11)|(0<<WGM10);  //CTC on OCR1A
  TCCR1B = (0<<ICNC1)|(0<<ICES1)|(0<<WGM13)|(1<<WGM12)|(0<<CS12)|(0<<CS11)|(1<<CS10);  //CTC on OCR1A, CLK/1
  OCR1A = 921 ;  //921 clock ticks per stepper routine, time unit = 50uS ~= 20KHz
  TIMSK1 = (1<<OCIE1A);  //timer interrupt on
  sei();
}


//USER LOOP
void userLoop(){
};





//--UTILITY FUNCTIONS----
void enableDriver(){
  motorPORT &= ~(_BV(motorEnable));
}

void disableDriver(){
  motorPORT |= _BV(motorEnable);
}

void setForward(){
  motorPORT |= _BV(motorDir);  //pin should be pin# in port
}

void setReverse(){
  motorPORT &= ~(_BV(motorDir));
}

//--STEP GENERATOR--
//--INTERRUPT ROUTINES----
ISR(TIMER1_COMPA_vect){
   //check for steps to move
  if(majorStepsRemaining>0){

    //ACCEL/DECCEL
    if(accelSteps > (majorSteps-majorStepsRemaining)){  //accelerating, changed from >= to >, should examine implications more
      uVelocity += uAccel;  //accelerate
    }else if(decelSteps >= majorStepsRemaining){  //deccelerating
      if (uVelocity > uAccel){ //make sure not to decelerate below zero
        uVelocity -= uAccel;  //deccelerate
      }else{  //velocity goes to zero
        //uVelocity = 0;
        antiLockout = 1;  //indicate that velocity should be returned to zero at end of move.
        *IO_ledPORT |= IO_ledPin;
      }
    }

    //MODIFY uPOSITION
    uPosition += uVelocity;

    //MAJOR AXIS STEP AND BRESENHAM ALGORITHM
    if(uPosition>uSteps){
      majorStepsRemaining --; //take a step in the virtual major axis
      uPosition -= uSteps;

      aError += aSteps;

      if(aError > int16_t(majorError)){ //take step in A
        motorPORT |= _BV(motorStep);  //rather than use a delay, step line gets turned off
        aError -= int16_t(majorSteps);
      }
      _delay_us(1);
      motorPORT &= ~(_BV(motorStep));  // turn off step line

    }
  }else{  //check for new packet to load

      if(writePosition!=readPosition){
      
      if (antiLockout>0){ //anti-lockout feature prevents the step velocity from hitting zero.
        uVelocity = 0;  //flag was set, so return velocity to zero AFTER move.
        antiLockout = 0;
        *IO_ledPORT &= ~IO_ledPin;
      }

        if(moveBuffer[readPosition + 1].waitForSync == 0){
          waitingForSync = 0;  //indicate to external processes that not waiting for sync.
          *IO_ledPORT &= ~IO_ledPin; //REMOVE turn off led
          // ENABLE AXES
          enableDriver();

          if(syncSearchPosition == readPosition){ //prevent syncSearch from searching moves that have already been read.
            syncSearchPosition++;
          }
          if(syncSearchPosition == bufferLength){
            syncSearchPosition = 0;
          }

          readPosition ++;
          if(readPosition == bufferLength){ //wrap-around
            readPosition = 0;
          }

          //LOAD MAJOR STEP VARIABLES
          majorSteps = uint16_t(moveBuffer[readPosition].majorSteps)<<microstepping;
          majorError = majorSteps>>1;
          majorStepsRemaining = majorSteps;

          //SET MOTOR DIRECTIONS
          if(moveBuffer[readPosition].directions & aDirectionMask){
            setForward();
          }else{
            setReverse();
          }

          //SET AXIS STEPS
          aSteps = int16_t(uint16_t(moveBuffer[readPosition].steps)<<microstepping);
          aError = 0;

          //SET ACCEL/DECCEL PARAMETERS
          accelSteps = (uint16_t(moveBuffer[readPosition].accelSteps)<<microstepping);
          decelSteps = (uint16_t(moveBuffer[readPosition].decelSteps)<<microstepping);
          uAccel = (uint16_t(moveBuffer[readPosition].accel)<<microstepping);  //might need to shift here by microstepping. Remember that accel is per cycle, not step.
        
          //CLEAR uPOSITION
          uPosition = 0; //this way acceleration will be more consistent in the face of rounding errors.
        }else{
          waitingForSync=1;
          *IO_ledPORT |= IO_ledPin;
        }
      }
  }
}

//SERVICE ROUTINES--
void svcReadMotorRef(){
  ADCSRA |= (1<<ADSC);  //start conversion
  while(ADCSRA & (1<<ADSC)){  //wait for conversion to complete
  }

  uint16_t ADCResult = ADC; //conversion result

  txBuffer[payloadLocation] = ADCResult & 255;  //low byte
  txBuffer[payloadLocation + 1] = (ADCResult>>8); //high byte
  transmitUnicastPacket(getVrefPort, 2);
}

void svcEnableDriver(){
  enableDriver();
  transmitUnicastPacket(enableDriverPort, 0);
  
}

void svcDisableDriver(){
  disableDriver();
  transmitUnicastPacket(disableDriverPort, 0);
}

void svcSync(){
  if(waitingForSync){
    TCNT1 = 0;  //clear counter to synchronize clocks off of sync move.
  } //this also gives 921 clock cycles to complete this routine.
  uint8_t newSyncSearchPosition = syncSearchPosition;

  do{
    if (newSyncSearchPosition == writePosition){   //have already searched to the current write position
      syncSearchPosition = newSyncSearchPosition; //record that have searched to here.
      return;
    }
    newSyncSearchPosition++; //increment sync search position
    if (newSyncSearchPosition == bufferLength){  //wrap-around
      newSyncSearchPosition = 0;
    }
  }while(moveBuffer[newSyncSearchPosition].waitForSync != 1);
  syncSearchPosition = newSyncSearchPosition; //commit changes to sync write position.
  moveBuffer[newSyncSearchPosition].waitForSync = 0; //move is now ready to be run
}

void svcMove(){
  //NEED TO ADD PROVISION FOR HANDLING SYNC WRITE POSITION
  uint8_t newWritePosition = writePosition + 1; //check for buffer full condition before overwriting buffer position
  if(newWritePosition==bufferLength){ //wrap-around
    newWritePosition = 0;
  } 
  if(newWritePosition == readPosition){ //buffer full
    txBuffer[payloadLocation + statusCode] = 0;
    txBuffer[payloadLocation + statusCurrentKey] = moveBuffer[readPosition].segmentKey;
    txBuffer[payloadLocation + statusStepsRemaining] = majorStepsRemaining>>microstepping;
    txBuffer[payloadLocation + statusReadPosition] = readPosition;
    txBuffer[payloadLocation + statusWritePosition] = writePosition;
    transmitUnicastPacket(movePort, 5); //5 payload bytes in packet
    return;
  }

  //fill segment parameters
  segmentKeyCounter ++; //increment segment key counter before pulling a key
  moveBuffer[newWritePosition].segmentKey = segmentKeyCounter;
  moveBuffer[newWritePosition].majorSteps = rxBuffer[payloadLocation + moveMajorSteps];
  moveBuffer[newWritePosition].directions = rxBuffer[payloadLocation + moveDirections];
  moveBuffer[newWritePosition].steps = rxBuffer[payloadLocation + moveSteps];
  moveBuffer[newWritePosition].accel = rxBuffer[payloadLocation + moveAccel];
  moveBuffer[newWritePosition].accelSteps = rxBuffer[payloadLocation + moveAccelSteps];
  moveBuffer[newWritePosition].decelSteps = rxBuffer[payloadLocation + moveDeccelSteps];
  moveBuffer[newWritePosition].waitForSync = rxBuffer[payloadLocation + moveSync];

  //transmit a response
  txBuffer[payloadLocation + statusCode] = 1;
  txBuffer[payloadLocation + statusCurrentKey] = moveBuffer[readPosition].segmentKey;
  txBuffer[payloadLocation + statusStepsRemaining] = majorStepsRemaining>>microstepping;
  txBuffer[payloadLocation + statusReadPosition] = readPosition;
  txBuffer[payloadLocation + statusWritePosition] = newWritePosition;
  transmitUnicastPacket(movePort, 5); //5 payload bytes in packet

  //increment write buffer position (this will trigger a read if idle)
  writePosition = newWritePosition;
  return;
}

void svcSpinStatus(){
  //transmit a response
  txBuffer[payloadLocation + statusCode] = 1;
  txBuffer[payloadLocation + statusCurrentKey] = moveBuffer[readPosition].segmentKey;
  txBuffer[payloadLocation + statusStepsRemaining] = majorStepsRemaining>>microstepping;
  txBuffer[payloadLocation + statusReadPosition] = readPosition;
  txBuffer[payloadLocation + statusWritePosition] = writePosition;
  transmitUnicastPacket(spinStatusPort, 5); //5 payload bytes in packet  
}

void svcSetVelocity(){
  //incoming packet represents velocity in a 16 bit word. The max value of velocity is 2^20, so need to shift by 4.
  //Also the incoming value is in steps/timebase, but need to convert into usteps/timebase
  uVelocity = uint32_t(uint16_t(rxBuffer[payloadLocation+velocity0]) + uint16_t(rxBuffer[payloadLocation+velocity1])<<8)<<(4 + microstepping);
  transmitUnicastPacket(setVelocityPort, 0);
}
//PACKET ROUTER
void userPacketRouter(uint8_t destinationPort){

  switch(destinationPort){
    case getVrefPort:  //status request
      svcReadMotorRef();
      break;
    case enableDriverPort: //enable drivers request
      svcEnableDriver();
      break;
    case disableDriverPort: //disable drivers
      svcDisableDriver();
      break;
    case movePort: //configure a move
      svcMove();
      break;
    case setVelocityPort: //set velocity
      svcSetVelocity();
      break;
    case spinStatusPort:
      svcSpinStatus();
      break;
    case syncPort:
      svcSync();
      break;
  };
};


