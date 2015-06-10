/*
  Copyright (c) 2015 Collin Kidder.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef _CANOPEN_LIBRARY_
#define _CANOPEN_LIBRARY_

#include <Arduino.h>
#include <due_can.h>  //relies on due_can for hardware access

const int MAX_DEVICES = 6;

enum CANOPEN_OPSTATE
{
	BOOTUP,
	PREOP,	
	OPERATIONAL,
	STOPPED
};

enum SDO_COMMAND
{
	SDO_WRITE = 0x20,
	SDO_READ = 0x40,
	SDO_WRITEACK = 0x60,
};

struct SDO_FRAME
{
	uint8_t nodeID;
	SDO_COMMAND cmd;
	uint16_t index;
	uint8_t subIndex;
	uint8_t dataLength;
	uint8_t data[4];
};

//CANOpen has slaves and a master. We've got to pick, are we the slave or the master.

class CANOPEN
{
public:
	CANOPEN(int);
	void setMasterMode();
	void setSlaveMode();
	void setHeartbeatInterval(uint32_t);
	bool isInitialized();
	void begin(int, int);
	void sendNodeStart(int);
	void sendNodePreop(int);
	void sendNodeReset(int);
	void sendNodeStop(int);
	void sendPDOMessage(int, int, unsigned char *);
	void sendSDORequest(SDO_FRAME *frame);
	void sendSDOResponse(SDO_FRAME *frame);
	void sendHeartbeat();
	void receiveFrame(CAN_FRAME *);
	void loop();
	void setStateChangeCallback(void (*cb)(CANOPEN_OPSTATE));
	void setPDOCallback(void (*cb)(CAN_FRAME *));
	void setSDOReqCallback(void (*cb)(SDO_FRAME *));
	void setSDOReplyCallback(void (*cb)(SDO_FRAME *));

protected:
private:
	bool isMaster;
	bool bInitialized; //has a device already got this channel set up?
	CANOPEN_OPSTATE opState;
	int nodeID; //our ID
	CANRaw *bus;
	void (*cbStateChange[MAX_DEVICES])(CANOPEN_OPSTATE newState); //callback used when the network state changes
	void (*cbGotPDOMsg[MAX_DEVICES])(CAN_FRAME *); //callback used when we get a PDO request addressed to us
	void (*cbGotSDOReq[MAX_DEVICES])(SDO_FRAME *); //callback used when we get a SDO request addressed to us.
	void (*cbGotSDOReply[MAX_DEVICES])(SDO_FRAME *); //callback used when we get a SDO request addressed to us.
	void sendStateChange(CANOPEN_OPSTATE newState);
	void sendGotPDOMsg(CAN_FRAME *);
	void sendGotSDOReq(SDO_FRAME *);
	void sendGotSDOReply(SDO_FRAME *);

	uint32_t heartbeatInterval; //in milliseconds
	uint32_t lastHeartbeat;

	void sendNMTMsg(int, int);
};

extern CANOPEN CanOpen0;
extern CANOPEN CanOpen1;
#endif // _CANOPEN_LIBRARY_
