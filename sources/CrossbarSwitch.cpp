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

#include "CrossbarSwitch.h"

namespace CasHMC
{
	
CrossbarSwitch::CrossbarSwitch(ofstream &debugOut_, ofstream &stateOut_):
	DualVectorObject<Packet, Packet>(debugOut_, stateOut_, MAX_CROSS_BUF, MAX_CROSS_BUF)
{
	header = "        (CS)";
	
	inServiceLink = -1;

	downBufferDest = vector<DualVectorObject<Packet, Packet> *>(NUM_VAULTS, NULL);
	upBufferDest = vector<LinkMaster *>(NUM_LINKS, NULL);
}

CrossbarSwitch::~CrossbarSwitch()
{	
	downBufferDest.clear();
	upBufferDest.clear();
	pendingSegTag.clear(); 
	pendingSegPacket.clear(); 
}

//
//Callback Adding packet
//
void CrossbarSwitch::CallbackReceiveDown(Packet *downEle, bool chkReceive)
{
/*	if(chkReceive) {
		DEBUG(ALI(18)<<header<<ALI(15)<<*downEle<<"Down) RECEIVING packet");
	}
	else {
		DEBUG(ALI(18)<<header<<ALI(15)<<*downEle<<"Down) packet buffer FULL");
	}*/
}
void CrossbarSwitch::CallbackReceiveUp(Packet *upEle, bool chkReceive)
{
/*	if(chkReceive) {
		DEBUG(ALI(18)<<header<<ALI(15)<<*upEle<<"Up)   RECEIVING packet");
	}
	else {
		DEBUG(ALI(18)<<header<<ALI(15)<<*upEle<<"Up)   packet buffer FULL");
	}*/
}

//
//Update the state of crossbar switch
//
void CrossbarSwitch::Update()
{	
	//Downstream buffer state
	if(bufPopDelay == 0) {
		for(int i=0; i<downBuffers.size(); i++) {
			if(downBuffers[i] != NULL) {
				//Check request size and the maximum block size
#ifdef HMC_MAC
				if(downBuffers[i]->pktMAC) {
					unsigned macUnit;
					unsigned macMemSize;
					switch(downBuffers[i]->CMD) {
						case MX4B:	case ML4B:	macUnit = 4;	macMemSize = 4 * downBuffers[i]->macSize;	break;
						case MX8B:	case ML8B:	macUnit = 8;	macMemSize = 8 * downBuffers[i]->macSize;	break;
						case MX16B:	case ML16B:macUnit = 16;	macMemSize = 16 * downBuffers[i]->macSize;	break;
						default:
						ERROR(header<<"  == Error - WRONG packet command type  (CurrentClock : "<<currentClockCycle<<")");
						exit(0);
					}
					uint64_t startVault = downBuffers[i]->ADRS >> (_log2(ADDRESS_MAPPING) + _log2(NUM_BANKS));
					uint64_t macEndAdd = downBuffers[i]->ADRS + macMemSize - macUnit;
					uint64_t endVault = macEndAdd >> (_log2(ADDRESS_MAPPING) + _log2(NUM_BANKS));
				
					if(startVault < endVault) {
						downBuffers[i]->segment = true;
						int segPacket = endVault - startVault + 1;
					
						//the packet is divided into segment packets.
						Packet *tempPacket = downBuffers[i];
						downBuffers.erase(downBuffers.begin()+i, downBuffers.begin()+i+downBuffers[i]->LNG);
						for(int j=0; j<segPacket; j++) {
							Packet *vaultPacket = new Packet(*tempPacket);
							//Last segment packet
							if(j == segPacket - 1) {
								if(vaultPacket->macSize > (ADDRESS_MAPPING*NUM_BANKS)/macUnit) {
									ERROR(header<<"  == Error - WRONG divided segment packet "<<*tempPacket<<" (CurrentClock : "<<currentClockCycle<<")");
									exit(0);
								}
							}
							else {
								unsigned tempStartVault = tempPacket->ADRS >> (_log2(ADDRESS_MAPPING) + _log2(NUM_BANKS));
								unsigned tempEndAdd = (tempStartVault + 1) << (_log2(ADDRESS_MAPPING) + _log2(NUM_BANKS));
								unsigned currAdd = (vaultPacket->ADRS >> _log2(ADDRESS_MAPPING)) << _log2(ADDRESS_MAPPING);
								unsigned segMacSize = (tempEndAdd - currAdd)/macUnit;
								vaultPacket->macSize = segMacSize;
								//Update MAC size and address of the remaining packet
								tempPacket->macSize -= segMacSize;
								tempPacket->ADRS = tempEndAdd;
							}

							tempPacket->LNG = 1 + ceil(vaultPacket->macSize/4);	//one flit is 16 bytes
							if(j>0)	vaultPacket->trace = NULL;
							
							DEBUG(ALI(18)<<header<<ALI(15)<<*vaultPacket<<"Down) Packet is DIVIDED into ["<<j+1<<"/"<<segPacket<<"] segment packets by vault memory size");
							downBuffers.insert(downBuffers.begin()+i, vaultPacket);
							for(int k=1; k<vaultPacket->LNG; k++) {		//Virtual tail packet
								downBuffers.insert(downBuffers.begin()+i+1, NULL);
							}
							i += vaultPacket->LNG;
							pendingSegTag.push_back(vaultPacket->TAG);
						}
						delete tempPacket;
					}
					else {
						unsigned vaultMap = (downBuffers[i]->ADRS >> (_log2(ADDRESS_MAPPING) + _log2(NUM_BANKS))) & (NUM_VAULTS-1);
						//unsigned vaultMap = (downBuffers[i]->ADRS >> _log2(ADDRESS_MAPPING)) & (NUM_VAULTS-1);
						if(downBufferDest[vaultMap]->ReceiveDown(downBuffers[i])) {
							DEBUG(ALI(18)<<header<<ALI(15)<<*downBuffers[i]<<"Down) SENDING packet to vault controller "<<vaultMap<<" (VC_"<<vaultMap<<")");
							downBuffers.erase(downBuffers.begin()+i, downBuffers.begin()+i+downBuffers[i]->LNG);
							i--;
						}
						else {
							//DEBUG(ALI(18)<<header<<ALI(15)<<*downBuffers[i]<<"Down) Vault controller buffer FULL");	
						}
					}
				}
				else if(downBuffers[i]->reqDataSize > (ADDRESS_MAPPING*NUM_BANKS)) {	// depending on address mapping
					int segPacket = ceil((double)downBuffers[i]->reqDataSize/(ADDRESS_MAPPING*NUM_BANKS));
					downBuffers[i]->reqDataSize = ADDRESS_MAPPING*NUM_BANKS;
#else
				if(downBuffers[i]->reqDataSize > ADDRESS_MAPPING) {
					int segPacket = ceil((double)downBuffers[i]->reqDataSize/ADDRESS_MAPPING);
					downBuffers[i]->reqDataSize = ADDRESS_MAPPING;
#endif
					DEBUG(ALI(18)<<header<<ALI(15)<<*downBuffers[i]<<"Down) Packet is DIVIDED into "<<segPacket<<" segment packets by max block size");
					
					//the packet is divided into segment packets.
					Packet *tempPacket = downBuffers[i];
					downBuffers.erase(downBuffers.begin()+i, downBuffers.begin()+i+downBuffers[i]->LNG);
#ifdef HMC_MAC
					if(tempPacket->LNG > 1)	tempPacket->LNG = 1 + (ADDRESS_MAPPING*NUM_BANKS)/16;	//one flit is 16 bytes
#else
					if(tempPacket->LNG > 1)	tempPacket->LNG = 1 + ADDRESS_MAPPING/16;	//one flit is 16 bytes
#endif
					for(int j=0; j<segPacket; j++) {
						Packet *vaultPacket = new Packet(*tempPacket);
#ifdef HMC_MAC
						vaultPacket->ADRS += j*(ADDRESS_MAPPING*NUM_BANKS);
						if(j>0)	vaultPacket->trace = NULL;
#else
						vaultPacket->ADRS += j*ADDRESS_MAPPING;
						if(j>1)	vaultPacket->trace = NULL;
#endif
						downBuffers.insert(downBuffers.begin()+i, vaultPacket);
						for(int k=1; k<vaultPacket->LNG; k++) {		//Virtual tail packet
							downBuffers.insert(downBuffers.begin()+i+1, NULL);
						}
						i += vaultPacket->LNG;
						vaultPacket->segment = true;
						pendingSegTag.push_back(vaultPacket->TAG);
					}
					delete tempPacket;
				}
				else {
#ifdef HMC_MAC
					unsigned vaultMap = (downBuffers[i]->ADRS >> (_log2(ADDRESS_MAPPING) + _log2(NUM_BANKS))) & (NUM_VAULTS-1);
#else
					unsigned vaultMap = (downBuffers[i]->ADRS >> _log2(ADDRESS_MAPPING)) & (NUM_VAULTS-1);
#endif
					if(downBufferDest[vaultMap]->ReceiveDown(downBuffers[i])) {
						DEBUG(ALI(18)<<header<<ALI(15)<<*downBuffers[i]<<"Down) SENDING packet to vault controller "<<vaultMap<<" (VC_"<<vaultMap<<")");
						downBuffers.erase(downBuffers.begin()+i, downBuffers.begin()+i+downBuffers[i]->LNG);
						i--;
					}
					else {
						//DEBUG(ALI(18)<<header<<ALI(15)<<*downBuffers[i]<<"Down) Vault controller buffer FULL");	
					}
				}
			}
		}
	}
	
	//Upstream buffer state
	for(int i=0; i<upBuffers.size(); i++) {
		if(upBuffers[i] != NULL) {
			//If the segment packet is arrived
			if(upBuffers[i]->segment) {
				//Check a stored segment packet tag
				bool foundSeg = false;
				bool foundLastSeg = true;
				for(int j=0; j<pendingSegTag.size(); j++) {
					if(upBuffers[i]->TAG == pendingSegTag[j]) {
						pendingSegTag.erase(pendingSegTag.begin()+j);
						foundSeg = true;
						//Check whether upBuffers[i] packet is the last segment packet or not 
						for(int k=j; k<pendingSegTag.size(); k++) {
							if(upBuffers[i]->TAG == pendingSegTag[k]) {
								DEBUG(ALI(18)<<header<<ALI(15)<<*upBuffers[i]<<"Up)   Segment packet is WAITING for the others");
								foundLastSeg = false;
								break;
							}
						}
						if(foundLastSeg) {
							DEBUG(ALI(18)<<header<<ALI(15)<<*upBuffers[i]<<"Up)   The LAST segment packet is arrived");
						}
						break;
					}
				}
				if(!foundSeg) {
					ERROR(header<<"  == Error - pendingSegTag doesn't have segment packet tag ["<<*upBuffers[i]<<"]");
					exit(0);
				}
				
				//Segment packets are combined together
				foundSeg = false;
				for(int j=0; j<pendingSegPacket.size(); j++) {
					if(upBuffers[i]->TAG == pendingSegPacket[j]->TAG) {
#ifdef HMC_MAC
						if(!upBuffers[i]->pktMAC) {
							if(upBuffers[i]->LNG > 1)	pendingSegPacket[j]->LNG += (ADDRESS_MAPPING*NUM_BANKS)/16;
						}
#else
						if(upBuffers[i]->LNG > 1)	pendingSegPacket[j]->LNG += ADDRESS_MAPPING/16;
#endif
						if(upBuffers[i]->trace != NULL)	pendingSegPacket[j]->trace = upBuffers[i]->trace;
						//Delete a segment packet
						int packetLNG = upBuffers[i]->LNG;
						delete upBuffers[i];
						upBuffers.erase(upBuffers.begin()+i, upBuffers.begin()+i+packetLNG);
						foundSeg = true;
						//All segment packets are combined
						if(foundLastSeg) {
							Packet *combPacket = new Packet(*pendingSegPacket[j]);
							delete pendingSegPacket[j];
							pendingSegPacket.erase(pendingSegPacket.begin()+j);
							combPacket->segment = false;
							upBuffers.insert(upBuffers.begin(), combPacket);
							for(int k=1; k<combPacket->LNG; k++) {	//Virtual tail packet
								upBuffers.insert(upBuffers.begin()+1, NULL);
							}
						}
						else {
							i--;
						}
						break;
					}
				}
				//Thr first arrived segment packet
				if(!foundSeg) {
					pendingSegPacket.push_back(upBuffers[i]);
					upBuffers.erase(upBuffers.begin()+i, upBuffers.begin()+i+upBuffers[i]->LNG);
					i--;
				}
			}
			else {
				for(int l=0; l<NUM_LINKS; l++) {
					int link = FindAvailableLink(inServiceLink, upBufferDest);
					if(link == -1) {
						//DEBUG(ALI(18)<<header<<ALI(15)<<*upBuffers[i]<<"Up)   all packet buffer FULL");
					}
					else if(upBufferDest[link]->currentState != LINK_RETRY) {
						if(upBufferDest[link]->Receive(upBuffers[i])) {
							DEBUG(ALI(18)<<header<<ALI(15)<<*upBuffers[i]<<"Up)   SENDING packet to link master "<<link<<" (LM_U"<<link<<")");
							upBuffers.erase(upBuffers.begin()+i, upBuffers.begin()+i+upBuffers[i]->LNG);
							i--;
							break;
						}
						else {
							//DEBUG(ALI(18)<<header<<ALI(15)<<*upBuffers[i]<<"Up)   Link master buffer FULL");
						}
					}
				}
			}
		}
	}
	
	Step();
}

//
//Print current state in state log file
//
void CrossbarSwitch::PrintState()
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
		if(upBuffers.size() < upBufferMax) {
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
		}
		else {
			for(int i=0; i<upBuffers.size(); i++) {
				if(i>0 && i%8==0) {
					STATEN(endl<<"                      ");
				}
				if(upBuffers[i] != NULL)	realInd = i;
				STATEN(*upBuffers[realInd]);
			}
		}
		STATEN(endl);
	}
}

} //namespace CasHMC