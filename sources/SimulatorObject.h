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

#ifndef SIMULATOROBJ_H
#define SIMULATOROBJ_H

//SimulatorObject.h
//
//Header file for simulator object class
//

#include <stdint.h>		//uint64_t
#include <fstream>		//ofstream
#include <iomanip>		//setw()
#include <sstream>		//stringstream

using namespace std;

namespace CasHMC
{
	
class SimulatorObject
{
public:
	SimulatorObject(ofstream &debugOut_, ofstream &stateOut_):debugOut(debugOut_), stateOut(stateOut_) {
		currentClockCycle = 0;
	}
	virtual void Update()=0;
	void Step() {
		currentClockCycle++;
	}
	
	uint64_t currentClockCycle;

	//Output log files
	ofstream &debugOut;
	ofstream &stateOut;
	string header;
	stringstream classID;
	
};

}

#endif