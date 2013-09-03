#include <gestalt.h>
#include <avr/io.h>

//CONFIGURE URL
char myurl[] = "http://www.fabunit.com/vn/086-005a_boot.py";

//USER SETUP
void userSetup(){
  setURL(&myurl[0], sizeof(myurl));
  //PIN AND PORT CONFIGURATION FOR SINGLE STEPPER
  IO_ledPORT = &PORTB;
  IO_ledDDR = &DDRB;
  IO_ledPIN = &PINB;
  IO_ledPin = 1<<3;

  IO_buttonPORT = &PORTB;
  IO_buttonDDR = &DDRB;
  IO_buttonPIN = &PINB;
  IO_buttonPin = 1<<2;

  IO_txrxPORT = &PORTD;
  IO_txrxDDR = &DDRD;
  IO_rxPin = 1<<0;
  IO_txPin = 1<<1;

  IO_txEnablePORT = &PORTD;
  IO_txEnableDDR = &DDRD;
  IO_txEnablePin = 1<<2;
}

void userLoop(){
};

//PACKET ROUTER
void userPacketRouter(uint8_t destinationPort){

};

