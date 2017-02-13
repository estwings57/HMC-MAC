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

#include "BankState.h"

namespace CasHMC
{
	
BankState::BankState(unsigned id):
	bankID(id)
{
	currentBankState = IDLE;
	openRowAddress = 0;
	nextActivate = 0;
	nextRead = 0;
	nextWrite = 0;
	nextPrecharge = 0;
	nextPowerUp = 0;
	lastCommand = PRECHARGE;
	stateChangeCountdown = 0;
}

void BankState::UpdateStateChange()
{

}

//
//Defines "<<" operation for printing
//
ostream &operator<<(ostream &out, const BankState &bs)
{
	switch(bs.currentBankState) {
		case IDLE:
			out<<"Idle";
			break;
		case ROW_ACTIVE:
			out<<"Actv";
			break;
		case PRECHARGING:
			out<<"Prch";
			break;
		case REFRESHING:
			out<<"Refr";
			break;
		case POWERDOWN:
			out<<"Pwdw";
			break;
		case AWAKING:
			out<<"Awak";
			break;
		default:
			ERROR(" (BS) == Error - Trying to print unknown kind of bank state");
			ERROR("         Type : "<<bs.currentBankState);
			exit(0);
	}
	return out;
}

} //namespace CasHMC