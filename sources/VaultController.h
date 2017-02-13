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

#ifndef VAULTCONTROLLER_H
#define VAULTCONTROLLER_H

//VaultController.h

#include <stdint.h>		//uint64_t
#include <vector>		//vector
#include <math.h>		//ceil()

#include "DualVectorObject.h"
#include "SimConfig.h"
#include "DRAMConfig.h"
#include "DRAMCommand.h"
#include "CommandQueue.h"
#include "DRAM.h" 
using namespace std;

namespace CasHMC
{
	
class VaultController : public DualVectorObject<Packet, Packet>
{
public:
	//
	//Functions
	//
	VaultController(ofstream &debugOut_, ofstream &stateOut_, unsigned id);
	virtual ~VaultController();
	void CallbackReceiveDown(Packet *packet, bool chkReceive);
	void CallbackReceiveUp(Packet *packet, bool chkReceive);
	void ReturnCommand(DRAMCommand *retRead);
	void MakeRespondPacket(DRAMCommand *retCMD);
	void Update();
	void UpdateCountdown();
	bool ConvPacketIntoCMDs(Packet *packet);
	void AddressMapping(uint64_t physicalAddress, unsigned &bankAdd, unsigned &colAdd, unsigned &rowAdd);
	void EnablePowerdown();
	void PrintState();

	//
	//Fields
	//
	unsigned vaultContID;
	unsigned refreshCountdown;
	bool powerDown;
	
	DRAM *dramP;
	CommandQueue *commandQueue;
	DRAMCommand *poppedCMD;
	DRAMCommand *atomicCMD;
#ifdef HMC_MAC
	vector<DRAMCommand *> pendingAtomicCMD;	
#endif
	unsigned atomicOperLeft;
	unsigned pendingDataSize;
	vector<unsigned> pendingReadData;	//Store Read packet TAG for return data
	DualVectorObject<Packet, Packet> *upBufferDest;
	
	//Command and data to be transmitted to DRAM I/O
	DRAMCommand *cmdBus;
	unsigned cmdCyclesLeft;
	DRAMCommand *dataBus;
	unsigned dataCyclesLeft;
	vector<DRAMCommand *> writeDataToSend;
	vector<unsigned> writeDataCountdown;	
};

}

#endif