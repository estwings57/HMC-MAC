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

#ifndef HMCCONTROLLER_H
#define HMCCONTROLLER_H

//HMCController.h

#include <vector>		//vector

#include "DualVectorObject.h"
#include "SimConfig.h"
#include "LinkMaster.h"
#include "LinkSlave.h"

using namespace std;

namespace CasHMC
{
	
class HMCController : public DualVectorObject<Transaction, Packet>
{
public:
	//
	//Functions
	//
	HMCController(ofstream &debugOut_, ofstream &stateOut_);
	virtual ~HMCController();
	void CallbackReceiveDown(Transaction *downEle, bool chkReceive);
	void CallbackReceiveUp(Packet *upEle, bool chkReceive);
	void Update();
	Packet *ConvTranIntoPacket(Transaction *tran);
	void PrintState();
	
	//
	//Fields
	//
	vector<LinkMaster *> downLinkMasters;
	vector<LinkSlave *> upLinkSlaves;
	int inServiceLink;	
};

}

#endif