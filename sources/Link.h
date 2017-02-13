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

#ifndef LINK_H
#define LINK_H

//Link.h

#include <stdint.h>		//uint64_t

#include "SingleVectorObject.h"
#include "SimConfig.h"

using namespace std;

namespace CasHMC
{
//forward declaration
class LinkMaster;
class Link : public SimulatorObject
{
public:
	//
	//Functions
	//
	Link(ofstream &debugOut_, ofstream &stateOut_, unsigned id, bool down, TranStatistic *statisP);
	virtual ~Link();
	void Update() {};
	void Update(bool lastUpdate);
	void UpdateStatistic(Packet *packet);
	void NoisePacket(Packet *packet);
	void PrintState();

	//
	//Fields
	//
	unsigned linkID;
	bool downstream;
	unsigned errorProba;
	LinkMaster *linkMasterP;
	TranStatistic *statis;
	SingleVectorObject<Packet> *linkSlaveP;
	
	//Currently transmitting packet through link
	Packet *inFlightPacket;
	unsigned inFlightCountdown;
};

}

#endif