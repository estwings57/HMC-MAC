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

#include "Transaction.h"

using namespace std;

namespace CasHMC
{
	
Transaction::Transaction(TransactionType tranType, uint64_t addr, unsigned size, TranStatistic *statis):
	transactionType(tranType),
	address(addr),
	dataSize(size)
{
#ifdef HMC_MAC
	macSize = 0;
#endif
	LNG = 1;
	transactionID = tranGlobalID++;
	trace = new TranTrace(statis);
}

Transaction::~Transaction()
{
}

//
//Reduction tranGlobalID
//
void Transaction::ReductGlobalID()
{
	tranGlobalID--;
}	

//
//Defines "<<" operation for printing
//
ostream& operator<<(ostream &out, const Transaction &t)
{
	string header;
	stringstream id;
	id << t.transactionID;

	switch(t.transactionType) {
		case DATA_READ:		header = "[T" + id.str() + "-Read]";	break;
		case DATA_WRITE:	header = "[T" + id.str() + "-Write]";	break;
		case RETURN_DATA:	header = "[T" + id.str() + "-Data]";	break;
		//ATOMICS commands for HMC
		case ATM_2ADD8:		header = "[T" + id.str() + "-2ADD8]";	break;		case ATM_ADD16:		header = "[T" + id.str() + "-ADD16]";	break;
		case ATM_P_2ADD8:	header = "[T" + id.str() + "-P_2ADD8]";	break;		case ATM_P_ADD16:	header = "[T" + id.str() + "-P_ADD16]";	break;
		case ATM_2ADDS8R:	header = "[T" + id.str() + "-2ADDS8R]";	break;		case ATM_ADDS16R:	header = "[T" + id.str() + "-ADDS16R]";	break;
		case ATM_INC8:		header = "[T" + id.str() + "-INC8]";	break;		case ATM_P_INC8:	header = "[T" + id.str() + "-P_INC8]";	break;
		case ATM_XOR16:		header = "[T" + id.str() + "-XOR16]";	break;		case ATM_OR16:		header = "[T" + id.str() + "-OR16]";	break;
		case ATM_NOR16:		header = "[T" + id.str() + "-NOR16]";	break;		case ATM_AND16:		header = "[T" + id.str() + "-AND16]";	break;
		case ATM_NAND16:	header = "[T" + id.str() + "-NAND16]";	break;		case ATM_CASGT8:	header = "[T" + id.str() + "-CASGT8]";	break;
		case ATM_CASLT8:	header = "[T" + id.str() + "-CASLT8]";	break;		case ATM_CASGT16:	header = "[T" + id.str() + "-CASGT16]";	break;
		case ATM_CASLT16:	header = "[T" + id.str() + "-CASLT16]";	break;		case ATM_CASEQ8:	header = "[T" + id.str() + "-CASEQ8]";	break;
		case ATM_CASZERO16:	header = "[T" + id.str() + "-CASZR16]";	break;		case ATM_EQ16:		header = "[T" + id.str() + "-EQ16]";	break;
		case ATM_EQ8:		header = "[T" + id.str() + "-EQ8]";		break;		case ATM_BWR:		header = "[T" + id.str() + "-BWR]";		break;
		case ATM_P_BWR:		header = "[T" + id.str() + "-P_BWR]";	break;		case ATM_BWR8R:		header = "[T" + id.str() + "-BWR8R]";	break;
		case ATM_SWAP16:	header = "[T" + id.str() + "-SWAP16]";	break;
#ifdef HMC_MAC
		case MAC_X4B:	header = "[T" + id.str() + "-M_X4B]";		break;
		case MAC_X8B:	header = "[T" + id.str() + "-M_X8B]";		break;
		case MAC_X16B:	header = "[T" + id.str() + "-M_X16B]";		break;
		case MAC_L4B:	header = "[T" + id.str() + "-M_L4B]";		break;
		case MAC_L8B:	header = "[T" + id.str() + "-M_L8B]";		break;
		case MAC_L16B:	header = "[T" + id.str() + "-M_L16B]";		break;
#endif
		default:
			ERROR(" (TS) == Error - Trying to print unknown kind of transaction type");
			ERROR("         T"<<t.transactionID<<" [?"<<t.transactionType<<"?] [0x"<<hex<<setw(16)<<setfill('0')<<t.address<<dec<<"]");
			exit(0);
	}
	out<<header;
	return out;
}

} //namespace CasHMC