#include <gestalt.h>
#include <avr/io.h>
#include <avr/delay.h>
#include <avr/interrupt.h>

//CONFIGURE URL
char myurl[] = "http://www.fabunit.com/vn/086-005a.py";

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
const uint8_t enableDriverPort    = 10;
const uint8_t disableDriverPort   = 11;
const uint8_t stepConfigPort      = 12;
const uint8_t stepSyncPort        = 13;
const uint8_t getStatusPort       = 15;
const uint8_t readMotorRefPort    = 20;


//--CONFIG PACKET DEFINITION--
#define direction       0
#define axisStep0       1
#define axisStep1       2

//--SYNC PACKET DEFINITION--
#define majorSteps0     0
#define majorSteps1     1
#define minVelocity0    2
#define minVelocity1    3
#define maxVelocity0    4
#define maxVelocity1    5
#define moveAccel       6




//STEPPER VARIABLES
volatile uint16_t axisSteps = 0;  //axis steps to move
volatile uint16_t axisStepsBuffer = 0;  //buffers axis steps because config can occur while stepping
volatile int32_t stepError = 0;  //used for bresenham algorithm

volatile uint16_t stepsToMove = 0;  //virtual major axis steps
volatile uint16_t stepsMiddle = 0;  //keeps track of halfway 
volatile uint16_t accelSteps = 0;  //stores how may steps are needed to finish accel move

volatile uint8_t accelFlag = 0;  //when !=0, currently accelerating
volatile uint8_t deccelFlag = 0;  //when !=0, currently deccelerating

volatile uint8_t directionBuffer = 0; //used to buffer direction

//1 step = 1048576 uSteps (2^20)
volatile uint8_t uAcceleration = 0;  // uSteps/timeunit^2
volatile uint32_t uVelocity = 0;  // uSteps/timeunit
volatile uint32_t uVelocityMax = 0;
volatile uint32_t uPosition = 0;  //uSteps
const uint32_t uSteps = 1048576;  //uSteps per step (2^20)



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
  motorPORT |= (1<<motorReset)|(1<<motorEnable); //start with motor disabled, not in reset
  motorPORT &= ~(1<<motorDir)|(1<<motorStep)|(1<<motorMS0)|(1<<motorMS1); //dir in reverse, step low, full stepping (note ~)

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

//--STEP GENERATOR--
//--INTERRUPT ROUTINES----
ISR(TIMER1_COMPA_vect){
  //check if steps to move
  if(stepsToMove > 0){
    if(accelFlag == 1){
      uVelocity += uAcceleration;  //accelerate by uAcceleration
    }else if(deccelFlag == 1){
      uVelocity -= uAcceleration; //deccelerates by uAcceleration
    }else if(stepsToMove <= accelSteps){
      deccelFlag = 1;  //begin decceleration
    }
    
    uPosition += uVelocity;
    if(uPosition>uSteps){
      //take a step in major axis. test if step in minor axis.
      stepError += int32_t(axisSteps);
      if(stepError>int32_t(stepsMiddle)){
        motorPORT |= _BV(motorStep);  //rather than use a delay, step line gets turned off after a few more commands.
        stepError -= int32_t(stepsMiddle<<1);
      }
      stepsToMove --;  //decrement x steps counter
      if(accelFlag == 1){
        accelSteps ++;
        if(stepsToMove == stepsMiddle){
          //start decel immediately, because still accelerating past mid-way point
          accelFlag = 0;
          deccelFlag = 1;
        }
      }else{
        _delay_us(1);
      }
      motorPORT &= ~(_BV(motorStep));  // turn off step line
      uPosition -= uSteps;  //keeps uPosition bounded to max value of 1048576
    }
    
    //stop accelerating if max velocity is reached
    if(uVelocity >= uVelocityMax){
      accelFlag = 0;
    }     
  }
}

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

//SERVICE ROUTINES--
void svcReadMotorRef(){
  ADCSRA |= (1<<ADSC);  //start conversion
  while(ADCSRA & (1<<ADSC)){  //wait for conversion to complete
  }

  uint16_t ADCResult = ADC; //conversion result

  txBuffer[payloadLocation] = ADCResult & 255;  //low byte
  txBuffer[payloadLocation + 1] = (ADCResult>>8); //high byte
  transmitUnicastPacket(readMotorRefPort, 2);
}

void svcEnableDriver(){
  enableDriver();
  transmitUnicastPacket(enableDriverPort, 0);
  
}

void svcDisableDriver(){
  disableDriver();
  transmitUnicastPacket(disableDriverPort, 0);
}

void svcStepConfig(){
  //load buffers for move to come
  directionBuffer = rxBuffer[payloadLocation + direction];
  axisStepsBuffer = uint16_t(rxBuffer[payloadLocation + axisStep1])<<8;
  axisStepsBuffer += uint16_t(rxBuffer[payloadLocation + axisStep0]);
  transmitUnicastPacket(stepConfigPort, 0);
}

void svcStepSync(){
  enableDriver();
  //load memory locations and then begin move
  //set minimum and maximum velocities
  //word value is multiplied by 2^10 or 1024
  uVelocity = uint32_t(rxBuffer[payloadLocation + minVelocity0])<<10;
  uVelocity += uint32_t(rxBuffer[payloadLocation + minVelocity1])<<18;
  uVelocityMax = uint32_t(rxBuffer[payloadLocation + maxVelocity0])<<10;
  uVelocityMax += uint32_t(rxBuffer[payloadLocation + maxVelocity1])<<18;
  
  //set acceleration
  uAcceleration = rxBuffer[payloadLocation + moveAccel];
  
  //set initial state
  uPosition = 0;  //clear position
  accelFlag = 1;  //move is an acceleration move
  deccelFlag = 0;
  accelSteps = 0; //clear accel steps

  stepError = 0;  //initialize bresenham generator
  axisSteps = axisStepsBuffer;  //load steps to take
  if(directionBuffer>0){  //set direction
    setForward();
  }else{
    setReverse();
  }
  
  //set stepsToMove, starting with larger byte (so that move doesn't end prematurely, although this won't take long)
  cli();
  stepsToMove = uint16_t(rxBuffer[payloadLocation + majorSteps1])<<8;
  stepsToMove += uint16_t(rxBuffer[payloadLocation + majorSteps0]);
  stepsMiddle = stepsToMove>>1;  //set number of steps before decelleration should commence. Also used for bresenham algorithm
  sei();

  //at this point, since steps have been loaded into stepsToMove, motion should commence within the ISR
}

void svcGetStatus(){
  txBuffer[payloadLocation] = stepsToMove & 0x000000FF;
  txBuffer[payloadLocation + 1] = (stepsToMove & 0x0000FF00)>>8;
  transmitUnicastPacket(getStatusPort, 2);  
}

//PACKET ROUTER
void userPacketRouter(uint8_t destinationPort){

    switch(destinationPort){
      case readMotorRefPort:  //status request
        svcReadMotorRef();
        break;
    case enableDriverPort: //enable drivers request
      svcEnableDriver();
      break;
    case disableDriverPort: //disable drivers
      svcDisableDriver();
      break;
    case stepConfigPort:
      svcStepConfig();
      break;
    case stepSyncPort:  //step synchronization packet
      svcStepSync();
      break;
    case getStatusPort:
      svcGetStatus();
      break;
    }
};


