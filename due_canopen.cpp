/*
  Copyright (c) 2013 Arduino.  All right reserved.

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


#include "due_canopen.h"

void trampolineCAN0(CAN_FRAME *frame)
{
	CanOpen0.receiveFrame(frame);
}

void trampolineCAN1(CAN_FRAME *frame)
{
	CanOpen1.receiveFrame(frame);
}

CANOPEN::CANOPEN(int busNum)
{
	isMaster = false;
	opState = BOOTUP;
	nodeID = 0x5F;
	if (busNum == 0)
	{
		bus = &Can0;
		Can0.attachCANInterrupt(trampolineCAN0); //automatically route all frames through this library
	}
	else 
	{
		bus = &Can1;
		Can1.attachCANInterrupt(trampolineCAN1);
	}
}

void CANOPEN::setMasterMode()
{
	isMaster = true;
}

void CANOPEN::setSlaveMode()
{
	isMaster = false;
}

void CANOPEN::begin(int speed = 250000, int id = 0x5F)
{
	nodeID = id;

	bus->begin(speed);

	if (isMaster)
	{
		opState = OPERATIONAL; //as the master we automatically become operational
		bus->watchFor(); //accept all frames if we're the master
	}
	else
	{
		opState = BOOTUP;
		bus->watchFor(id, 0x7F); //only match against our actual ID - but allow all the frame types in upper 4 bits
		bus->watchFor(0, 0x7F); //Also allow through any message to ID 0 which is broadcast (allows all upper bit functions)
		sendHeartbeat();
	}

}

void CANOPEN::sendNodeStart(int id = 0)
{
	sendNMTMsg(id, 1);
}

void CANOPEN::sendNodePreop(int id = 0)
{
	sendNMTMsg(id, 0x80);
}

void CANOPEN::sendNodeReset(int id = 0)
{
	sendNMTMsg(id, 0x81);
}

void CANOPEN::sendNodeStop(int id = 0)
{
	sendNMTMsg(id, 2);
}

void CANOPEN::sendNMTMsg(int id, int cmd)
{
	if (!isMaster) return; //only the master sends NMT commands
	id &= 0x7F;
	CAN_FRAME frame;
	frame.id = 0;
	frame.length = 8; //might want to set length to 2. Some specs seem to indicate this value
	frame.data.byte[0] = cmd;
	frame.data.byte[1] = id;
	//the rest don't matter
	bus->sendFrame(frame);
}

void CANOPEN::sendPDOMessage(int id, int length, unsigned char *data)
{
	if (id > 0x57F) return; //invalid ID for a PDO message
	if (id < 0x180) return; //invalid ID for a PDO message
	if (length > 8 || length < 0) return; //invalid length
	CAN_FRAME frame;
	frame.id = id;
	frame.length = length;
	for (int x = 0; x < length; x++) frame.data.byte[x] = data[x];
	bus->sendFrame(frame);
}

void CANOPEN::sendSDOResponse(SDO_FRAME *sframe)
{
	sframe->nodeID &= 0x7f;
	CAN_FRAME frame;
	frame.length = 8;
	frame.id = 0x580 + sframe->nodeID;
	if (sframe->dataLength <= 4)
	{
		frame.data.byte[0] = sframe->cmd;
		if (sframe->dataLength > 0) //responding with data
		{
			frame.data.byte[0] |= 0x0F - ((sframe->dataLength - 1) * 4); 
		}
		frame.data.byte[1] = sframe->index & 0xFF;
		frame.data.byte[2] = sframe->index >> 8;
		frame.data.byte[3] = sframe->subIndex;
		for (int x = 0; x < sframe->dataLength; x++) frame.data.byte[4 + x] = sframe->data[x];
		bus->sendFrame(frame);
	}
}

void CANOPEN::sendSDORequest(SDO_FRAME *sframe)
{
	sframe->nodeID &= 0x7F;
	CAN_FRAME frame;
	frame.length = 8;
	frame.id = 0x600 + sframe->nodeID;
	if (sframe->dataLength <= 4)
	{
		frame.data.byte[0] = sframe->cmd;
		if (sframe->dataLength > 0) //request to write data
		{
			frame.data.byte[0] |= 0x0F - ((sframe->dataLength - 1) * 4); //kind of dumb the way this works...
		}
		frame.data.byte[1] = sframe->index & 0xFF;
		frame.data.byte[2] = sframe->index >> 8;
		frame.data.byte[3] = sframe->subIndex;
		for (int x = 0; x < sframe->dataLength; x++) frame.data.byte[4 + x] = sframe->data[x];
		bus->sendFrame(frame);
	}
}

void CANOPEN::receiveFrame(CAN_FRAME *frame)
{
	SDO_FRAME sframe;
	if (frame->id == 0) //NMT message
	{
		if (frame->data.byte[1] != nodeID && frame->data.byte[1] != 0) return; //not for us.
		switch (frame->data.byte[0])
		{
		case 1: //start this node
			opState = OPERATIONAL;
			break;
		case 2: //stop this node
			opState = STOPPED;
			break;
		case 0x80: //enter pre-op state
			opState = PREOP;
			break;
		case 0x81: //reset this node
			opState = PREOP; 
			break;
		}
		if (cbStateChange != NULL) cbStateChange(opState);
	}
	if (frame->id > 0x17F && frame->id < 0x580)
	{
		if (cbGotPDOMsg != NULL) cbGotPDOMsg(frame);
	}
	if (frame->id == 0x600 + nodeID) //SDO request targetted to our ID
	{
		if (cbGotSDOReq != NULL) 
		{
			sframe.nodeID = nodeID;
			sframe.index = frame->data.byte[1] + (frame->data.byte[2] * 256);
			sframe.subIndex = frame->data.byte[3];
			sframe.cmd = (SDO_COMMAND)(frame->data.byte[0] & 0xF0);
			
			if ((frame->data.byte[0] != 0x40) && (frame->data.byte[0] != 0x60))
			{
				sframe.dataLength = (3 - ((frame->data.byte[0] & 0xC) >> 2)) + 1;			
			}
			else sframe.dataLength = 0;

			for (int x = 0; x < sframe.dataLength; x++) sframe.data[x] = frame->data.byte[4 + x];
			
			cbGotSDOReq(&sframe);
		}
	}
	if (frame->id == 0x580 + nodeID) //SDO reply to our ID
	{
		if (cbGotSDOReply != NULL)
		{
			sframe.nodeID = nodeID;
			sframe.index = frame->data.byte[1] + (frame->data.byte[2] * 256);
			sframe.subIndex = frame->data.byte[3];
			sframe.cmd = (SDO_COMMAND)(frame->data.byte[0] & 0xF0);
			
			if ((frame->data.byte[0] != 0x40) && (frame->data.byte[0] != 0x60))
			{
				sframe.dataLength = (3 - ((frame->data.byte[0] & 0xC) >> 2)) + 1;			
			}
			else sframe.dataLength = 0;

			for (int x = 0; x < sframe.dataLength; x++) sframe.data[x] = frame->data.byte[4 + x];

			cbGotSDOReply(&sframe);
		}
	}
}

void CANOPEN::setStateChangeCallback(void (*cb)(CANOPEN_OPSTATE))
{
	cbStateChange = cb;
}

void CANOPEN::setPDOCallback(void (*cb)(CAN_FRAME *))
{
	cbGotPDOMsg = cb;
}

void CANOPEN::setSDOReqCallback(void (*cb)(SDO_FRAME *))
{
	cbGotSDOReq = cb;
}

void CANOPEN::setSDOReplyCallback(void (*cb)(SDO_FRAME *))
{
	cbGotSDOReply = cb;
}

void CANOPEN::setHeartbeatInterval(uint32_t interval)
{
	heartbeatInterval = interval;
}

void CANOPEN::loop()
{
	if ((millis() - lastHeartbeat) >= heartbeatInterval)
	{
		sendHeartbeat();
	}
	if (opState == BOOTUP)
	{
		opState = PREOP;
	}
}

void CANOPEN::sendHeartbeat()
{
	CAN_FRAME frame;
	frame.id = 0x700 + nodeID;
	frame.length = 1;
	switch (opState)
	{
	case OPERATIONAL:
		frame.data.byte[0] = 5;
		break;
	case STOPPED:
		frame.data.byte[0] = 4;
		break;
	case PREOP:
		frame.data.byte[0] = 0x7F;
		break;
	case BOOTUP:
		frame.data.byte[0] = 0;
		break;
	}
	
	bus->sendFrame(frame);
	lastHeartbeat = millis();
}

CANOPEN CanOpen0(0);
CANOPEN CanOpen1(1);