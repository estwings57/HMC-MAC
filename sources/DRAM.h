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

#ifndef DRAM_H
#define DRAM_H

//DRAM.h

#include <vector>		//vector
#include <stdint.h>		//uint64_t

#include "SimulatorObject.h"
#include "SimConfig.h"
#include "DRAMConfig.h"
#include "DRAMCommand.h"
#include "BankState.h"

using namespace std;

namespace CasHMC
{
//forward declaration
class VaultController;
class DRAM : public SimulatorObject
{
public:
	//
	//Functions
	//
	DRAM(ofstream &debugOut_, ofstream &stateOut_, unsigned id, VaultController *contP);
	~DRAM();
	void receiveCMD(DRAMCommand *recvCMD);
	bool powerDown();
	void powerUp();
	void Update();
	void UpdateState();
	void PrintState();

	//
	//Fields
	//
	unsigned DRAMID;
	VaultController *vaultContP;
	vector<BankState *> bankStates;
	vector<BankStateType> previousBankState;
	
	//To be returned command	
	DRAMCommand *readData;
	unsigned dataCyclesLeft;
	vector<DRAMCommand *> readReturnDATA;
	vector<unsigned> readReturnCountdown;
};

}

#endif