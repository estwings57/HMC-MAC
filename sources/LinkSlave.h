/*********************************************************************************
*  HMC-MAC v1.0 - 2017.02.14
*  Processing-in Memory Architecture for Multiply-Accumulate Operations with Hybrid Memory Cube
*
*  Copyright (c) 2017, Dong-Ik Jeon
*                      Ki-Seok Chung
*                      Hanyang University
*                      estwings57 [at] gmail [dot] com
*  All rights reserved.
*********************************************************************************/

#ifndef LINKSLAVE_H
#define LINKSLAVE_H

//LinkSlave.h

#include <vector>		//vector

#include "SingleVectorObject.h"
#include "DualVectorObject.h"
#include "SimConfig.h"
#include "Packet.h"

using namespace std;

namespace CasHMC
{
//forward declaration
class LinkMaster;
class LinkSlave : public SingleVectorObject<Packet>
{
public:
	//
	//Functions
	//
	LinkSlave(ofstream &debugOut_, ofstream &stateOut_, unsigned id, bool down);
	virtual ~LinkSlave();
	void CallbackReceive(Packet *packet, bool chkReceive);
	void Update();
	bool CheckNoError(Packet *chkPacket);
	void PrintState();

	//
	//Fields
	//
	unsigned linkSlaveID;
	DualVectorObject<Packet, Packet> *downBufferDest;
	DualVectorObject<Transaction, Packet> *upBufferDest;
	LinkMaster *localLinkMaster;
	
	int slaveSEQ;
	int countdownCRC;
	bool startCRC;
};

}

#endif