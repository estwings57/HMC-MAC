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

#ifndef BANKSTATE_H
#define BANKSTATE_H

//BankState.h

#include <stdint.h>		//uint64_t
#include <stdlib.h>		//exit(0)
#include <iomanip>		//setw()
#include <iostream> 	//ostream

#include "SimConfig.h"
#include "DRAMCommand.h"

using namespace std;

namespace CasHMC
{
enum BankStateType
{
	IDLE,
	ROW_ACTIVE,
	PRECHARGING,
	REFRESHING,
	POWERDOWN,
	AWAKING
};	
	
class BankState
{
public:
	//
	//Functions
	//
	BankState(unsigned id);
	void UpdateStateChange();

	//
	//Fields
	//
	unsigned bankID;
	
	BankStateType currentBankState;
	unsigned openRowAddress;
	uint64_t nextActivate;
	uint64_t nextRead;
	uint64_t nextWrite;
	uint64_t nextPrecharge;
	uint64_t nextPowerUp;

	DRAMCommandType lastCommand;
	unsigned stateChangeCountdown;
};

ostream &operator<<(ostream &out, const BankState &bs);
}

#endif