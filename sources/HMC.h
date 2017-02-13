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

#ifndef HMC_H
#define HMC_H

//HMC.h

#include <vector>		//vector

#include "SimulatorObject.h"
#include "SimConfig.h"
#include "DRAMConfig.h"
#include "LinkSlave.h"
#include "LinkMaster.h"
#include "CrossbarSwitch.h"
#include "VaultController.h"
#include "DRAM.h"

using namespace std;

namespace CasHMC
{
	
class HMC : public SimulatorObject
{
public:
	//
	//Functions
	//
	HMC(ofstream &debugOut_, ofstream &stateOut_);
	virtual ~HMC();
	void Update();
	void PrintState();

	//
	//Fields
	//
	uint64_t clockTuner;
	
	vector<LinkSlave *> downLinkSlaves;
	vector<LinkMaster *> upLinkMasters;
	CrossbarSwitch *crossbarSwitch;
	vector<VaultController *> vaultControllers;
	vector<DRAM *> drams;
};

}

#endif