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

#ifndef TRANSACTION_H
#define TRANSACTION_H

//Transaction.h

#include <stdint.h>		//uint64_t
#include <stdlib.h>		//exit(0)
#include <iomanip>		//setw()
#include <iostream> 	//ostream
#include <sstream>		//stringstream

#include "SimConfig.h"
#include "TranTrace.h"
#include "TranStatistic.h"

using namespace std;

namespace CasHMC
{

static unsigned tranGlobalID=0;

enum TransactionType
{
	DATA_READ,
	DATA_WRITE,
	RETURN_DATA,
	//ATOMICS commands for HMC
	ATM_2ADD8, ATM_ADD16, ATM_P_2ADD8, ATM_P_ADD16, ATM_2ADDS8R, ATM_ADDS16R, ATM_INC8, ATM_P_INC8, //ARITHMETIC ATOMICS
	ATM_XOR16, ATM_OR16, ATM_NOR16, ATM_AND16, ATM_NAND16, 											//BOOLEAN ATOMICS
	ATM_CASGT8, ATM_CASLT8, ATM_CASGT16, ATM_CASLT16, ATM_CASEQ8, ATM_CASZERO16, ATM_EQ16, ATM_EQ8, //COMPARISON ATOMICS
#ifdef HMC_MAC
	//HMC_MAC ATOMICS
	MAC_X4B, MAC_X8B, MAC_X16B, MAC_L4B, MAC_L8B, MAC_L16B,
#endif
	ATM_BWR, ATM_P_BWR, ATM_BWR8R, ATM_SWAP16 														//BITWISE ATOMICS
};

class Transaction
{
public:
	//
	//Functions
	//
	Transaction(TransactionType tranType, uint64_t addr, unsigned size, TranStatistic *statis);
	virtual ~Transaction();
	void ReductGlobalID();

	//
	//Fields
	//
	TranTrace *trace;
	TransactionType transactionType;	//Type of transaction (defined above)
	uint64_t address;					//Physical address of request
	unsigned dataSize;					//[byte] Size of data
#ifdef HMC_MAC
	unsigned macSize;
#endif
	unsigned transactionID;				//Unique identifier
	unsigned LNG;
};

ostream& operator<<(ostream &out, const Transaction &t);
}

#endif