//Sets up to become the CANOpen master node on CAN0
// Required libraries
#include "variant.h"
#include <due_can.h>
#include <due_canopen.h>

//Leave defined if you use native port, comment if using programming port
#define Serial SerialUSB

void onChangedState(CANOPEN_OPSTATE state)
{
	Serial.print("New operating state: ");
	Serial.println((int)state);
}

void onPDOReceive(CAN_FRAME *frame)
{
	Serial.print("Got PDO frame with ID ");
	Serial.println(frame->id);
	Serial.print("Data: ");
	for (int x = 0; x < frame->length; x++) Serial.print(frame->data.byte[x]);
	Serial.println();
}

void onSDOReceive(CAN_FRAME *frame)
{
	Serial.println("Got SDO frame.");
	Serial.print("Operation byte: ");
	Serial.println(frame->data.byte[0]);
	Serial.print("Index byte 1: ");
	Serial.println(frame->data.byte[1]);
	Serial.print("Index Byte 2: ");
	Serial.println(frame->data.byte[2]);
	Serial.print("Subindex Byte: ");
	Serial.println(frame->data.byte[3]);
	Serial.print("Data: ");
	for (int x = 0; x < 4; x++) Serial.print(frame->data.byte[4+x]);
	Serial.println();
}


void setup()
{

  Serial.begin(115200);
  
  CanOpen0.setMasterMode();
  CanOpen0.begin(250000, 0x10);
  
  CanOpen0.setStateChangeCallback(onChangedState);
  CanOpen0.setPDOCallback(onPDOReceive);
  CanOpen0.setSDOCallback(onSDOReceive);
  
  CanOpen0.sendNodeReset(0); //reset all nodes
  delay(2000);
  CanOpen0.sendNodeStart(0); //make all connected nodes go active
  
}

void loop(){
   CanOpen0.loop();
   static unsigned char buff[5];
   buff[0] += 10;
   buff[1] += 20;
   buff[2] += 30;
   buff[3] += 40;
   buff[4] += 50;
  delay(2000);
  CanOpen0.sendPDOMessage(0x200, 5, buff);
}


