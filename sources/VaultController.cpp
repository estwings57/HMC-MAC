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

#include "VaultController.h"

namespace CasHMC
{
	
VaultController::VaultController(ofstream &debugOut_, ofstream &stateOut_, unsigned id):
	DualVectorObject<Packet, Packet>(debugOut_, stateOut_, MAX_VLT_BUF, MAX_VLT_BUF),
	vaultContID(id)
{	
	classID << vaultContID;
	header = "        (VC_" + classID.str() + ")";
	
	refreshCountdown = 0;
	powerDown = false;
	poppedCMD = NULL;
	atomicCMD = NULL;
	atomicOperLeft = 0;
	pendingDataSize = 0;
	
	cmdBus = NULL;
	cmdCyclesLeft = 0;
	dataBus = NULL;
	dataCyclesLeft = 0;
	
	//Make class objects
	commandQueue = new CommandQueue(debugOut, stateOut, this, vaultContID);
}

VaultController::~VaultController()
{
#ifdef HMC_MAC
	pendingAtomicCMD.clear();
#endif
	pendingReadData.clear(); 
	writeDataToSend.clear(); 
	writeDataCountdown.clear(); 

	delete commandQueue;
}

//
//Callback receiving packet result
//
void VaultController::CallbackReceiveDown(Packet *downEle, bool chkReceive)
{
/*	if(chkReceive) {
		DEBUG(ALI(18)<<header<<ALI(15)<<*downEle<<"Down) RECEIVING packet");
	}
	else {
		DEBUG(ALI(18)<<header<<ALI(15)<<*downEle<<"Down) packet buffer FULL");
	}*/
}

void VaultController::CallbackReceiveUp(Packet *upEle, bool chkReceive)
{
	if(chkReceive) {
		switch(upEle->CMD) {
			case RD_RS:	DE_CR(ALI(18)<<header<<ALI(15)<<*upEle<<"Up)   RETURNING read data response packet");	break;
			case WR_RS:	DE_CR(ALI(18)<<header<<ALI(15)<<*upEle<<"Up)   RETURNING write response packet");		break;
			default:
				ERROR(header<<"  == Error - WRONG response packet command type  "<<*upEle<<"  (CurrentClock : "<<currentClockCycle<<")");
				exit(0);
				break;
		}
	}
	else {
		ERROR(header<<"  == Error - Vault controller upstream packet buffer FULL  "<<*upEle<<"  (CurrentClock : "<<currentClockCycle<<")");
		ERROR(header<<"             Vault buffer max size : "<<upBufferMax<<", current size : "<<upBuffers.size()<<", "<<*upEle<<" size : "<<upEle->LNG);
		exit(0);
	}
}	

//
//Return read commands from DRAM
//
void VaultController::ReturnCommand(DRAMCommand *retCMD)
{
	//Read and write data commands share one data bus
	if(dataBus != NULL) {
		ERROR(header<<"  == Error - Data Bus Collision  (dataBus:"<<*dataBus<<", retCMD:"<<*retCMD<<")  (CurrentClock : "<<currentClockCycle<<")");
		exit(0);
	}
	
	bool foundMatch = false;
	for(int i=0; i<pendingReadData.size(); i++) {
		if(retCMD->packetTAG == pendingReadData[i]) {
			if(retCMD->lastCMD == true) {
				if(retCMD->atomic) {
#ifdef HMC_MAC
					bool lastMACcmd = true;
					for(int j=i+1; j<pendingReadData.size(); j++) {
						if(retCMD->packetTAG == pendingReadData[j]) {
							lastMACcmd = false;
						}
					}
					if(lastMACcmd)		retCMD->lastMAC = true;
	
					if(atomicCMD == NULL) {
						atomicCMD = retCMD;
						if(atomicCMD->packetCMD >= MX4B && atomicCMD->packetCMD <= MX16B) {	//Fixed point multiplication
							atomicOperLeft = atmMUL + 1;	//MAC operation time (multiplication + addition)
						}
						else if(atomicCMD->packetCMD >= ML4B && atomicCMD->packetCMD <= ML16B) {	//Floating point multiplication
							atomicOperLeft = atmFMUL + 1;	//MAC operation time (multiplication + addition)
						}
						else {
							atomicOperLeft = 1;
						}
						DE_CR(ALI(18)<<header<<ALI(15)<<*atomicCMD<<"Up)   RETURNING ATOMIC read data");
					}
					else {
						pendingAtomicCMD.push_back(retCMD);
						DEBUG(ALI(18)<<header<<ALI(15)<<*retCMD<<"Up)   ATOMIC read data is WAITING ["<<pendingAtomicCMD.size()<<"]");
					}
#else
					atomicCMD = retCMD;
					atomicOperLeft = 1;
					DE_CR(ALI(18)<<header<<ALI(15)<<*retCMD<<"Up)   RETURNING ATOMIC read data");
#endif
				}
				else {
					MakeRespondPacket(retCMD);
					delete retCMD;
				}
			}
			else {
				DEBUG(ALI(18)<<header<<ALI(15)<<*retCMD<<"Up)   RETURNING read data");
				delete retCMD;
			}
			pendingReadData.erase(pendingReadData.begin()+i);
			foundMatch = true;
			break;
		}
	}
	if(!foundMatch) {
		ERROR(header<<"  == Error - Can't find a matching transaction  "<<*retCMD<<" 0x"<<hex<<retCMD->packetTAG<<dec<<"  (CurrentClock : "<<currentClockCycle<<")");
		exit(0);
	}
}

//
//Make response packet from request
//
void VaultController::MakeRespondPacket(DRAMCommand *retCMD)
{
	if(retCMD->trace != NULL) {
		retCMD->trace->vaultFullLat = currentClockCycle - retCMD->trace->vaultIssueTime;
	}
	Packet *newPacket;
	if(retCMD->atomic) {
		if(retCMD->packetCMD == _2ADD8 || retCMD->packetCMD == ADD16 || retCMD->packetCMD == INC8
		|| retCMD->packetCMD == EQ8 || retCMD->packetCMD == EQ16 || retCMD->packetCMD == BWR) {
			newPacket = new Packet(RESPONSE, WR_RS, retCMD->packetTAG, 1, retCMD->trace);
			pendingDataSize -= 1;
		}
		else {
			newPacket = new Packet(RESPONSE, RD_RS, retCMD->packetTAG, 2, retCMD->trace);
			pendingDataSize -= 2;
		}
#ifdef HMC_MAC
		if(retCMD->packetCMD >= MX4B && retCMD->packetCMD <= ML16B) {
			newPacket->pktMAC = true;
		}
#endif
	}
	else {
		if(retCMD->commandType == WRITE_DATA) {
			//packet, cmd, tag, lng, *lat
			newPacket = new Packet(RESPONSE, WR_RS, retCMD->packetTAG, 1, retCMD->trace);
			pendingDataSize -= 1;
			//DEBUG(ALI(18)<<header<<ALI(15)<<*retCMD<<"Up)   pendingDataSize 1 decreased   (current pendingDataSize : "<<pendingDataSize<<")");
		}
		else if(retCMD->commandType == READ_DATA) {
			//packet, cmd, tag, lng, *lat
			newPacket = new Packet(RESPONSE, RD_RS, retCMD->packetTAG, (retCMD->dataSize/16)+1, retCMD->trace);
			pendingDataSize -= (retCMD->dataSize/16)+1;
			//DEBUG(ALI(18)<<header<<ALI(15)<<*retCMD<<"Up)   pendingDataSize "<<(retCMD->dataSize/16)+1<<" decreased   (current pendingDataSize : "<<pendingDataSize<<")");
		}
		else {
			ERROR(header<<"  == Error - Unknown response packet command  cmd : "<<retCMD->commandType<<"  (CurrentClock : "<<currentClockCycle<<")");
			exit(0);
		}
	}
	newPacket->segment = retCMD->segment;
	ReceiveUp(newPacket);
}

//
//Update the state of vault controller
//
void VaultController::Update()
{
	//Update DRAM state and various countdown
	UpdateCountdown();

	//Convert request packet into DRAM commands
	if(bufPopDelay == 0) {
		for(int i=0; i<downBuffers.size(); i++) {
			//Make sure that buffer[0] is not virtual tail packet.
			if(downBuffers[i] != NULL) {
				if(ConvPacketIntoCMDs(downBuffers[i])) {
					int tempLNG = downBuffers[i]->LNG;
					delete downBuffers[i];
					downBuffers.erase(downBuffers.begin()+i, downBuffers.begin()+i+tempLNG);
				}
			}
		}
	}
	
	//Send response packet to crossbar switch
	if(upBuffers.size() > 0) {
		//Make sure that buffer[0] is not virtual tail packet.
		if(upBuffers[0] == NULL) {
			ERROR(header<<"  == Error - Vault controller up buffer[0] is NULL (It could be one of virtual tail packet occupying packet length  (CurrentClock : "<<currentClockCycle<<")");
			exit(0);
		}
		else{
			if(upBufferDest->ReceiveUp(upBuffers[0])) {
				DEBUG(ALI(18)<<header<<ALI(15)<<*upBuffers[0]<<"Up)   SENDING packet to crossbar switch (CS)");
				upBuffers.erase(upBuffers.begin(), upBuffers.begin()+upBuffers[0]->LNG);
			}
			else {
				//DEBUG(ALI(18)<<header<<ALI(15)<<*upBuffers[0]<<"Up)   Crossbar switch buffer FULL");	
			}
		}
	}
	
	//Pop command from command queue
	if(commandQueue->CmdPop(&poppedCMD)) {
		//Write data command will be issued after countdown
		if(poppedCMD->commandType == WRITE || poppedCMD->commandType == WRITE_P) {
			DRAMCommand *writeData = new DRAMCommand(*poppedCMD);
			writeData->commandType = WRITE_DATA;
			writeDataToSend.push_back(writeData);
			writeDataCountdown.push_back(WL);
		}
		
		if(poppedCMD->lastCMD == true) {
			if(!poppedCMD->posted) {
				if(poppedCMD->commandType == WRITE || poppedCMD->commandType == WRITE_P) {
					if(!poppedCMD->atomic) {
						pendingDataSize += 1;
						//DEBUG(ALI(18)<<header<<ALI(15)<<*poppedCMD<<"Down) pendingDataSize 1 increased   (current pendingDataSize : "<<pendingDataSize<<")");
					}
				}
				else if(poppedCMD->commandType == READ || poppedCMD->commandType == READ_P) {
					if(!poppedCMD->atomic) {
						pendingDataSize += (poppedCMD->dataSize/16)+1;
						//DEBUG(ALI(18)<<header<<ALI(15)<<*poppedCMD<<"Down) pendingDataSize "<<(poppedCMD->dataSize/16)+1<<" increased   (current pendingDataSize : "<<pendingDataSize<<")");
					}
					else {
						if(poppedCMD->packetCMD == _2ADD8 || poppedCMD->packetCMD == ADD16 || poppedCMD->packetCMD == INC8
						|| poppedCMD->packetCMD == EQ8 || poppedCMD->packetCMD == EQ16 || poppedCMD->packetCMD == BWR) {
							pendingDataSize += 1;
						}
						else {
							pendingDataSize += 2;
						}
					}
				}
			}
		}
		
		//Check for collision on bus
		if(cmdBus != NULL) {
			ERROR(header<<"  == Error - Command bus collision  (cmdBus:"<<*cmdBus<<", poppedCMD:"<<*poppedCMD<<")  (CurrentClock : "<<currentClockCycle<<")");
			exit(0);
		}
		cmdBus = poppedCMD;
		cmdCyclesLeft = tCMD;
		poppedCMD = NULL;
	}
	
	//Atomic command operation (assume that all operations consume one clock cycle)
	if(atomicCMD != NULL) {
		if(atomicOperLeft == 0) {
#ifdef HMC_MAC
			if(atomicCMD->packetCMD >= MX4B && atomicCMD->packetCMD <= ML16B) {
				if(!atomicCMD->lastMAC) {
					pendingDataSize -= 2;
				}
				else {
					DRAMCommand *atmRstActCMD = new DRAMCommand(*atomicCMD);
					atmRstActCMD->commandType = ACTIVATE;
					atmRstActCMD->dataSize = 0;
					DRAMCommand *atmRstCMD = new DRAMCommand(*atomicCMD);
					atmRstCMD->commandType = (OPEN_PAGE ? WRITE : WRITE_P);;
					atmRstCMD->dataSize = 32;
					atmRstCMD->lastCMD = true;
					if(!atmRstCMD->posted) {
						atmRstCMD->trace = NULL;
					}
					if(QUE_PER_BANK) {
						commandQueue->queue[atomicCMD->bank].insert(commandQueue->queue[atomicCMD->bank].begin(), atmRstCMD);
						commandQueue->queue[atomicCMD->bank].insert(commandQueue->queue[atomicCMD->bank].begin(), atmRstActCMD);
						classID.str( string() );	classID.clear();
						classID << atomicCMD->bank;
						DE_CR(ALI(18)<<(commandQueue->header+"-"+classID.str()+")")<<ALI(15)<<*atmRstCMD<<"Down) PUSHING atomic result command into command queue with "<<*atmRstActCMD);
					}
					else {
						commandQueue->queue[0].insert(commandQueue->queue[0].begin(), atmRstCMD);
						commandQueue->queue[0].insert(commandQueue->queue[0].begin(), atmRstActCMD);
						DE_CR(ALI(18)<<(commandQueue->header+")")<<ALI(15)<<*atmRstCMD<<"Down) PUSHING atomic result command into command queue with "<<*atmRstActCMD);
					}
					
					if(!atomicCMD->posted) {
						MakeRespondPacket(atomicCMD);
					}
					else {
						pendingDataSize -= 2;
					}
				}
			}
			else {
				//The results are written back to DRAM, overwriting the original memory operands
				if(atomicCMD->packetCMD != EQ16 && atomicCMD->packetCMD != EQ8) {
					DRAMCommand *atmRstCMD = new DRAMCommand(*atomicCMD);
					atmRstCMD->commandType = (OPEN_PAGE ? WRITE : WRITE_P);
					atmRstCMD->dataSize = 16;
					atmRstCMD->lastCMD = true;
					if(!atmRstCMD->posted) {
						atmRstCMD->trace = NULL;
					}
					if(QUE_PER_BANK) {
						commandQueue->queue[atomicCMD->bank].insert(commandQueue->queue[atomicCMD->bank].begin(), atmRstCMD);
						classID.str( string() );	classID.clear();
						classID << atomicCMD->bank;
						DE_CR(ALI(18)<<(commandQueue->header+"-"+classID.str()+")")<<ALI(15)<<*atmRstCMD<<"Down) PUSHING atomic result command into command queue");
					}
					else {
						commandQueue->queue[0].insert(commandQueue->queue[0].begin(), atmRstCMD);
						DE_CR(ALI(18)<<(commandQueue->header+")")<<ALI(15)<<*atmRstCMD<<"Down) PUSHING atomic result command into command queue");
					}
				}
				
				if(!atomicCMD->posted) {
					MakeRespondPacket(atomicCMD);
				}
			}
			
			delete atomicCMD;
			atomicCMD = NULL;
			
			if(pendingAtomicCMD.size()>0) {
				atomicCMD = pendingAtomicCMD[0];
				pendingAtomicCMD.erase(pendingAtomicCMD.begin());
				
				if(atomicCMD->packetCMD >= MX4B && atomicCMD->packetCMD <= MX16B) {	//Fixed point multiplication
					atomicOperLeft = atmMUL + 1;	//MAC operation time (multiplication + addition)
				}
				else if(atomicCMD->packetCMD >= ML4B && atomicCMD->packetCMD <= ML16B) {	//Floating point multiplication
					atomicOperLeft = atmFMUL + 1;	//MAC operation time (multiplication + addition)
				}
				else {
					atomicOperLeft = 1;
				}
				DE_CR(ALI(18)<<header<<ALI(15)<<*atomicCMD<<"Up)   RETURNING ATOMIC read data");
			}
#else
			//The results are written back to DRAM, overwriting the original memory operands
			if(atomicCMD->packetCMD != EQ16 && atomicCMD->packetCMD != EQ8) {
				DRAMCommand *atmRstCMD = new DRAMCommand(*atomicCMD);
				atmRstCMD->commandType = (OPEN_PAGE ? WRITE : WRITE_P);
				atmRstCMD->dataSize = 16;
				atmRstCMD->lastCMD = true;
				if(!atmRstCMD->posted) {
					atmRstCMD->trace = NULL;
				}
				if(QUE_PER_BANK) {
					commandQueue->queue[atomicCMD->bank].insert(commandQueue->queue[atomicCMD->bank].begin(), atmRstCMD);
					classID.str( string() );	classID.clear();
					classID << atomicCMD->bank;
					DE_CR(ALI(18)<<(commandQueue->header+"-"+classID.str()+")")<<ALI(15)<<*atmRstCMD<<"Down) PUSHING atomic result command into command queue");
				}
				else {
					commandQueue->queue[0].insert(commandQueue->queue[0].begin(), atmRstCMD);
					DE_CR(ALI(18)<<(commandQueue->header+")")<<ALI(15)<<*atmRstCMD<<"Down) PUSHING atomic result command into command queue");
				}
			}
			
			if(!atomicCMD->posted) {
				MakeRespondPacket(atomicCMD);
			}
			delete atomicCMD;
			atomicCMD = NULL;			
#endif	
		}
		else {
			DEBUG(ALI(18)<<(commandQueue->header+")")<<ALI(15)<<*atomicCMD<<"Down) Atomic command is now operating ["<<atomicOperLeft<<" clk]");
			atomicOperLeft--;
		}
	}

	//Power-down mode setting
	EnablePowerdown();

	commandQueue->Update();
	Step();
}

//
//Update DRAM state and various countdown
//
void VaultController::UpdateCountdown()
{
	//Check for outgoing command and handle countdowns
	if(cmdBus != NULL) {
		cmdCyclesLeft--;
		if(cmdCyclesLeft == 0) {	//command is ready to be transmitted
			DE_CR(ALI(18)<<header<<ALI(15)<<*cmdBus<<"Down) ISSUING command to DRAM");
			if(cmdBus->trace != NULL && cmdBus->trace->vaultIssueTime == 0) {
				cmdBus->trace->vaultIssueTime = currentClockCycle;
			}
			dramP->receiveCMD(cmdBus);
			cmdBus = NULL;
		}
	}
	//Check for outgoing write data packets and handle countdowns
	if(dataBus != NULL) {
		dataCyclesLeft--;
		if(dataCyclesLeft == 0) {
			DE_CR(ALI(18)<<header<<ALI(15)<<*dataBus<<"Down) ISSUING data corresponding to previous write command");
			if(dataBus->lastCMD) {
				if(!dataBus->atomic && !dataBus->posted) {
					MakeRespondPacket(dataBus);
				}
				else if(dataBus->trace != NULL && dataBus->posted) {
					dataBus->trace->tranFullLat = ceil(currentClockCycle * (double)tCK/CPU_CLK_PERIOD) - dataBus->trace->tranTransmitTime;
					dataBus->trace->linkFullLat = ceil(currentClockCycle * (double)tCK/CPU_CLK_PERIOD) - dataBus->trace->linkTransmitTime;
					dataBus->trace->vaultFullLat = currentClockCycle - dataBus->trace->vaultIssueTime;
					delete dataBus->trace;
				}
			}
			dramP->receiveCMD(dataBus);
			dataBus = NULL;
		}
	}
	//Check write data to be sent to DRAM and handle countdowns
	if(writeDataCountdown.size() > 0) {
		if(writeDataCountdown[0] == 0) {
			if(dataBus != NULL) {
				ERROR(header<<"   == Error - Data Bus Collision  "<<*dataBus<<"  (CurrentClock : "<<currentClockCycle<<")");
				exit(0);
			}
			dataBus = writeDataToSend[0];
			dataCyclesLeft = BL;	//block size according to address mapping / DATA_WIDTH

			writeDataCountdown.erase(writeDataCountdown.begin());
			writeDataToSend.erase(writeDataToSend.begin());
		}
		
		for(int i=0; i<writeDataCountdown.size(); i++) {
			writeDataCountdown[i]--;
		}
	}
	
	//Time for a refresh issue a refresh
	refreshCountdown--;
	if(refreshCountdown == 0) {
		commandQueue->refreshWaiting = true;
		refreshCountdown = REFRESH_PERIOD/tCK;
		DEBUG(ALI(39)<<header<<"REFRESH countdown is over");
	}
	//If a rank is powered down, make sure we power it up in time for a refresh
	else if(powerDown && refreshCountdown<=tXP)	{
	}
}

//
//Convert packet into DRAM commands
//
bool VaultController::ConvPacketIntoCMDs(Packet *packet)
{
	unsigned bankAdd, colAdd, rowAdd;
	AddressMapping(packet->ADRS, bankAdd, colAdd, rowAdd);
	
	DRAMCommandType tempCMD;
	bool tempPosted = false;
	bool atomic = false;
#ifdef HMC_MAC
	double reqSize = packet->reqDataSize;
	double macSize = packet->macSize;
	int macCNT;
#endif
	switch(packet->CMD) {
		//Write
		case WR16:		tempCMD = OPEN_PAGE ? WRITE : WRITE_P;	break;
		case WR32:		tempCMD = OPEN_PAGE ? WRITE : WRITE_P;	break;
		case WR48:		tempCMD = OPEN_PAGE ? WRITE : WRITE_P;	break;
		case WR64:		tempCMD = OPEN_PAGE ? WRITE : WRITE_P;	break;
		case WR80:		tempCMD = OPEN_PAGE ? WRITE : WRITE_P;	break;
		case WR96:		tempCMD = OPEN_PAGE ? WRITE : WRITE_P;	break;
		case WR112:		tempCMD = OPEN_PAGE ? WRITE : WRITE_P;	break;
		case WR128:		tempCMD = OPEN_PAGE ? WRITE : WRITE_P;	break;
		case WR256:		tempCMD = OPEN_PAGE ? WRITE : WRITE_P;	break;
		case MD_WR:		tempCMD = OPEN_PAGE ? WRITE : WRITE_P;	break;
		//Poseted Write
		case P_WR16:	tempCMD = OPEN_PAGE ? WRITE : WRITE_P;	tempPosted = true;	break;
		case P_WR32:	tempCMD = OPEN_PAGE ? WRITE : WRITE_P;	tempPosted = true;	break;
		case P_WR48:	tempCMD = OPEN_PAGE ? WRITE : WRITE_P;	tempPosted = true;	break;
		case P_WR64:	tempCMD = OPEN_PAGE ? WRITE : WRITE_P;	tempPosted = true;	break;
		case P_WR80:	tempCMD = OPEN_PAGE ? WRITE : WRITE_P;	tempPosted = true;	break;
		case P_WR96:	tempCMD = OPEN_PAGE ? WRITE : WRITE_P;	tempPosted = true;	break;
		case P_WR112:	tempCMD = OPEN_PAGE ? WRITE : WRITE_P;	tempPosted = true;	break;
		case P_WR128:	tempCMD = OPEN_PAGE ? WRITE : WRITE_P;	tempPosted = true;	break;
		case P_WR256:	tempCMD = OPEN_PAGE ? WRITE : WRITE_P;	tempPosted = true;	break;
		//Read
		case RD16:		tempCMD = OPEN_PAGE ? READ : READ_P;	pendingReadData.push_back(packet->TAG);	break;
		case RD32:		tempCMD = OPEN_PAGE ? READ : READ_P;	pendingReadData.push_back(packet->TAG);	break;
		case RD48:		tempCMD = OPEN_PAGE ? READ : READ_P;	pendingReadData.push_back(packet->TAG);	break;
		case RD64:		tempCMD = OPEN_PAGE ? READ : READ_P;	pendingReadData.push_back(packet->TAG);	break;
		case RD80:		tempCMD = OPEN_PAGE ? READ : READ_P;	pendingReadData.push_back(packet->TAG);	break;
		case RD96:		tempCMD = OPEN_PAGE ? READ : READ_P;	pendingReadData.push_back(packet->TAG);	break;
		case RD112:		tempCMD = OPEN_PAGE ? READ : READ_P;	pendingReadData.push_back(packet->TAG);	break;
		case RD128:		tempCMD = OPEN_PAGE ? READ : READ_P;	pendingReadData.push_back(packet->TAG);	break;
		case RD256:		tempCMD = OPEN_PAGE ? READ : READ_P;	pendingReadData.push_back(packet->TAG);	break;
		case MD_RD:		tempCMD = OPEN_PAGE ? READ : READ_P;	pendingReadData.push_back(packet->TAG);	break;
		//Arithmetic atomic
		case _2ADD8:	atomic = true;	tempCMD = READ;	pendingReadData.push_back(packet->TAG);	break;
		case ADD16:		atomic = true;	tempCMD = READ;	pendingReadData.push_back(packet->TAG);	break;
		case P_2ADD8:	atomic = true;	tempCMD = READ;	pendingReadData.push_back(packet->TAG);	tempPosted = true;	break;
		case P_ADD16:	atomic = true;	tempCMD = READ;	pendingReadData.push_back(packet->TAG);	tempPosted = true;	break;
		case _2ADDS8R:	atomic = true;	tempCMD = READ;	pendingReadData.push_back(packet->TAG);	break;
		case ADDS16R:	atomic = true;	tempCMD = READ; pendingReadData.push_back(packet->TAG);	break;
		case INC8:		atomic = true;	tempCMD = READ;	pendingReadData.push_back(packet->TAG);	break;
		case P_INC8:	atomic = true;	tempCMD = READ;	pendingReadData.push_back(packet->TAG);	tempPosted = true;	break;
		//Boolean atomic
		case XOR16:		atomic = true;	tempCMD = READ;	pendingReadData.push_back(packet->TAG);	break;
		case OR16:		atomic = true;	tempCMD = READ;	pendingReadData.push_back(packet->TAG);	break;
		case NOR16:		atomic = true;	tempCMD = READ;	pendingReadData.push_back(packet->TAG);	break;
		case AND16:		atomic = true;	tempCMD = READ;	pendingReadData.push_back(packet->TAG);	break;
		case NAND16:	atomic = true;	tempCMD = READ;	pendingReadData.push_back(packet->TAG);	break;
		//Comparison atomic
		case CASGT8:	atomic = true;	tempCMD = READ;	pendingReadData.push_back(packet->TAG);	break;
		case CASLT8:	atomic = true;	tempCMD = READ;	pendingReadData.push_back(packet->TAG);	break;
		case CASGT16:	atomic = true;	tempCMD = READ;	pendingReadData.push_back(packet->TAG);	break;
		case CASLT16:	atomic = true;	tempCMD = READ;	pendingReadData.push_back(packet->TAG);	break;
		case CASEQ8:	atomic = true;	tempCMD = READ;	pendingReadData.push_back(packet->TAG);	break;
		case CASZERO16:	atomic = true;	tempCMD = READ;	pendingReadData.push_back(packet->TAG);	break;
		case EQ16:		atomic = true;	tempCMD = READ;	pendingReadData.push_back(packet->TAG);	break;
		case EQ8:		atomic = true;	tempCMD = READ;	pendingReadData.push_back(packet->TAG);	break;
		//Bitwise atomic
		case BWR:		atomic = true;	tempCMD = READ;	pendingReadData.push_back(packet->TAG);	break;
		case P_BWR:		atomic = true;	tempCMD = READ;	pendingReadData.push_back(packet->TAG);	tempPosted = true;	break; 
		case BWR8R:		atomic = true;	tempCMD = READ;	pendingReadData.push_back(packet->TAG);	break; 
		case SWAP16:	atomic = true;	tempCMD = READ;	pendingReadData.push_back(packet->TAG);	break;	
#ifdef HMC_MAC
		case MX4B:		atomic = true;	tempCMD = READ;	macCNT = 8;	break;	//MAC command reads 32 bytes data from DRAM
		case MX8B:		atomic = true;	tempCMD = READ;	macCNT = 4;	break;	//4 times for 8 byte MAC command
		case MX16B:		atomic = true;	tempCMD = READ;	macCNT = 2;	break;	//2 times for 16 byte MAC command
		case ML4B:		atomic = true;	tempCMD = READ;	macCNT = 8;	break;
		case ML8B:		atomic = true;	tempCMD = READ;	macCNT = 4;	break;
		case ML16B:		atomic = true;	tempCMD = READ;	macCNT = 2;	break; 
#endif		
		
		default:
			ERROR(header<<"  == Error - WRONG packet command type  (CurrentClock : "<<currentClockCycle<<")");
			exit(0);
	}
	
#ifdef HMC_MAC
	if(packet->pktMAC) {
		while(1) {
			if(packet->macSize > 0 && commandQueue->AvailableSpace(bankAdd, 2)) {
				DEBUG(ALI(18)<<header<<ALI(15)<<*packet<<"Down) phyAdd : 0x"<<hex<<setw(9)<<setfill('0')<<packet->ADRS<<dec<<"  bankAdd : "<<bankAdd<<"  colAdd : "<<colAdd<<"  rowAdd : "<<rowAdd);
				DRAMCommand *actCMD = new DRAMCommand(ACTIVATE, packet->TAG, bankAdd, colAdd, rowAdd, 0, false, packet->trace, true, packet->CMD, atomic, packet->segment);
				commandQueue->Enqueue(bankAdd, actCMD);
				DRAMCommand *rwCMD = new DRAMCommand(tempCMD, packet->TAG, bankAdd, colAdd, rowAdd, reqSize, tempPosted, packet->trace, true, packet->CMD, atomic, packet->segment);
				commandQueue->Enqueue(bankAdd, rwCMD);
				pendingReadData.push_back(packet->TAG);
				
				packet->macSize = (packet->macSize > macCNT) ? packet->macSize - macCNT : 0;
				packet->ADRS += ADDRESS_MAPPING;
				AddressMapping(packet->ADRS, bankAdd, colAdd, rowAdd);
			}
			else if(packet->macSize <= 0){
				return true;
			}
			else {
				return false;
			}
		}
	}
	//Due to the internal 32-byte granularity of the DRAM data bus within each vault in the HMC (HMC spec v2.1 p.99)
	else if(commandQueue->AvailableSpace(bankAdd, ceil((double)reqSize/32)+1)) {
		DEBUG(ALI(18)<<header<<ALI(15)<<*packet<<"Down) phyAdd : 0x"<<hex<<setw(9)<<setfill('0')<<packet->ADRS<<dec<<"  bankAdd : "<<bankAdd<<"  colAdd : "<<colAdd<<"  rowAdd : "<<rowAdd);
		//cmdtype, tag, bnk, col, rw, *dt, dSize, pst, *lat
		DRAMCommand *actCMD = new DRAMCommand(ACTIVATE, packet->TAG, bankAdd, colAdd, rowAdd, 0, false, packet->trace, true, packet->CMD, atomic, packet->segment);
		commandQueue->Enqueue(bankAdd, actCMD);
		
		for(int i=0; i<ceil((double)reqSize/32); i++) {
			DRAMCommand *rwCMD;
			if(i < ceil((double)reqSize/32)-1) {
				if(tempCMD == WRITE_P) {
					rwCMD = new DRAMCommand(WRITE, packet->TAG, bankAdd, colAdd, rowAdd, reqSize, tempPosted, packet->trace, false, packet->CMD, atomic, packet->segment);
				}
				else if(tempCMD == READ_P) {
					rwCMD = new DRAMCommand(READ, packet->TAG, bankAdd, colAdd, rowAdd, reqSize, tempPosted, packet->trace, false, packet->CMD, atomic, packet->segment);
				}
				else {
					rwCMD = new DRAMCommand(tempCMD, packet->TAG, bankAdd, colAdd, rowAdd, reqSize, tempPosted, packet->trace, false, packet->CMD, atomic, packet->segment);
				}
			}
			else {
				rwCMD = new DRAMCommand(tempCMD, packet->TAG, bankAdd, colAdd, rowAdd, reqSize, tempPosted, packet->trace, true, packet->CMD, atomic, packet->segment);
			}
			commandQueue->Enqueue(bankAdd, rwCMD);
			if(tempCMD == READ || tempCMD == READ_P) {
				pendingReadData.push_back(packet->TAG);
			}
		}
		return true;
	}
#else
	//Due to the internal 32-byte granularity of the DRAM data bus within each vault in the HMC (HMC spec v2.1 p.99)
	if(commandQueue->AvailableSpace(bankAdd, ceil((double)packet->reqDataSize/32)+1)) {
		DEBUG(ALI(18)<<header<<ALI(15)<<*packet<<"Down) phyAdd : 0x"<<hex<<setw(9)<<setfill('0')<<packet->ADRS<<dec<<"  bankAdd : "<<bankAdd<<"  colAdd : "<<colAdd<<"  rowAdd : "<<rowAdd);
		//cmdtype, tag, bnk, col, rw, *dt, dSize, pst, *lat
		DRAMCommand *actCMD = new DRAMCommand(ACTIVATE, packet->TAG, bankAdd, colAdd, rowAdd, 0, false, packet->trace, true, packet->CMD, atomic, packet->segment);
		commandQueue->Enqueue(bankAdd, actCMD);
		
		for(int i=0; i<ceil((double)packet->reqDataSize/32); i++) {
			DRAMCommand *rwCMD;
			if(i < ceil((double)packet->reqDataSize/32)-1) {
				if(tempCMD == WRITE_P) {
					rwCMD = new DRAMCommand(WRITE, packet->TAG, bankAdd, colAdd, rowAdd, packet->reqDataSize, tempPosted, packet->trace, false, packet->CMD, atomic, packet->segment);
				}
				else if(tempCMD == READ_P) {
					rwCMD = new DRAMCommand(READ, packet->TAG, bankAdd, colAdd, rowAdd, packet->reqDataSize, tempPosted, packet->trace, false, packet->CMD, atomic, packet->segment);
				}
				else {
					rwCMD = new DRAMCommand(tempCMD, packet->TAG, bankAdd, colAdd, rowAdd, packet->reqDataSize, tempPosted, packet->trace, false, packet->CMD, atomic, packet->segment);
				}
			}
			else {
				rwCMD = new DRAMCommand(tempCMD, packet->TAG, bankAdd, colAdd, rowAdd, packet->reqDataSize, tempPosted, packet->trace, true, packet->CMD, atomic, packet->segment);
			}
			commandQueue->Enqueue(bankAdd, rwCMD);
			if(tempCMD == READ || tempCMD == READ_P) {
				pendingReadData.push_back(packet->TAG);
			}
		}
		return true;
	}
#endif
	else{
		return false;
	}
}

//
//Mapping physical address to bank, column, and row
//
void VaultController::AddressMapping(uint64_t physicalAddress, unsigned &bankAdd, unsigned &colAdd, unsigned &rowAdd)
{
	
#ifdef HMC_MAC
	//HMC_MAC [Row]/[Column]/[Vault]/[Bank]/[Max_block]
	unsigned maxBlockBit = _log2(ADDRESS_MAPPING);
	physicalAddress >>= maxBlockBit/*max block bits*/;
	
	//Extract bank address
	bankAdd = physicalAddress & (NUM_BANKS-1);
	physicalAddress >>= _log2(NUM_BANKS) + _log2(NUM_VAULTS)/*vault address bits*/;
#else
	//[Row]/[Column]/[Bank]/[Vault]/[Max_block]
	unsigned maxBlockBit = _log2(ADDRESS_MAPPING);
	physicalAddress >>= maxBlockBit/*max block bits*/ + _log2(NUM_VAULTS)/*vault address bits*/;

	//Extract bank address
	bankAdd = physicalAddress & (NUM_BANKS-1);
	physicalAddress >>= _log2(NUM_BANKS);
#endif
	
	//Extract column address
	colAdd = physicalAddress & ((NUM_COLS-1)>>maxBlockBit);
	colAdd <<= maxBlockBit;
	physicalAddress >>= (_log2(NUM_COLS)-maxBlockBit);
	
	//Extract row address
	rowAdd = physicalAddress & (NUM_ROWS-1);
}

//
//Power-down mode setting
//
void VaultController::EnablePowerdown()
{
	if(USE_LOW_POWER) {
		if(commandQueue->isEmpty() && !commandQueue->refreshWaiting) {
			if(dramP->powerDown()) {
				powerDown = true;
			}
		}
		else if(powerDown && dramP->bankStates[0]->nextPowerUp <= currentClockCycle) {
			dramP->powerUp();
			powerDown = false;
		}
	}
}

//
//Print current state in state log file
//
void VaultController::PrintState()
{
	int realInd = 0;
	if(downBuffers.size()>0) {
		STATEN(ALI(17)<<header);
		STATEN("Down ");
		for(int i=0; i<downBufferMax; i++) {
			if(i>0 && i%8==0) {
				STATEN(endl<<"                      ");
			}
			if(i < downBuffers.size()) {
				if(downBuffers[i] != NULL)	realInd = i;
				STATEN(*downBuffers[realInd]);
			}
			else if(i == downBufferMax-1) {
				STATEN("[ - ]");
			}
			else {
				STATEN("[ - ]...");
				break;
			}
		}
		STATEN(endl);
	}
	
	if(upBuffers.size()>0) {
		STATEN(ALI(17)<<header);
		STATEN(" Up  ");
		for(int i=0; i<upBufferMax; i++) {
			if(i>0 && i%8==0) {
				STATEN(endl<<"                      ");
			}
			if(i < upBuffers.size()) {
				if(upBuffers[i] != NULL)	realInd = i;
				STATEN(*upBuffers[realInd]);
			}
			else if(i == upBufferMax-1) {
				STATEN("[ - ]");
			}
			else {
				STATEN("[ - ]...");
				break;
			}
		}
		STATEN(endl);
	}
	
	commandQueue->PrintState();
}

} //namespace CasHMC