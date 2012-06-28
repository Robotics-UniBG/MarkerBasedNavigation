/********************************************************************************
 *
 * MarkerSearcherCoordination
 *
 * Copyright (c) 2012
 * All rights reserved.
 *
 * Davide Brugali, Aldo Biziak, Luca Gherardi, Andrea Luzzana
 * University of Bergamo
 * Dept. of Information Technology and Mathematics
 *
 * -------------------------------------------------------------------------------
 *
 * File: MarkerSearcherCoordination.cpp
 * Created: June 13, 2012
 *
 * Author: <A HREF="mailto:luca.gherardi@unibg.it">Luca Gherardi</A>
 * Author: <A HREF="mailto:andrea.luzzana@unibg.it">Andrea Luzzana</A>
 * Author: <A HREF="mailto:aldo.biziak@unibg.it">Aldo Biziak</A>
 *
 * Supervised by: <A HREF="mailto:brugali@unibg.it">Davide Brugali</A>
 *
 * -------------------------------------------------------------------------------
 *
 * This sofware is published under a dual-license: GNU Lesser General Public
 * License LGPL 2.1 and BSD license. The dual-license implies that users of this
 * code may choose which terms they prefer.
 *
 * -------------------------------------------------------------------------------
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  - Neither the name of the University of Bergamo nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 2.1 of the
 * License, or (at your option) any later version or the BSD license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL and the BSD license for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL and BSD license along with this program.
 *
 *******************************************************************************/

#include "mbn_common/MarkerSearcherCoordination.hpp"

namespace mbn_common{


MarkerSearcherCoordination::MarkerSearcherCoordination(MarkerSearcherComputation* markerSearcherComputer){

	this->markerSearcherComputation = markerSearcherComputer;

	currentState = IDLE;

	outputEvent = "";

}

MarkerSearcherCoordination::~MarkerSearcherCoordination(void){
}

void MarkerSearcherCoordination::notifyTargetMarkerIDReceived(){

	if(currentState == IDLE){

		markerSearcherComputation->updateTargetMarkerID();
		markerFound = false;
		motionDone = false;
		outputEvent = "";
		currentState = TARGET_RECEIVED;
		cout << "Marker Searcher state changed: TARGET_RECEIVED" << endl;

	}
}

void MarkerSearcherCoordination::notifyVisibleMarkersIDsReceived(){


	if(currentState == TARGET_RECEIVED ||
			currentState == MOVING){

		markerFound = markerSearcherComputation->searchTargetIDInVisibleMarkersIDs();

		if(currentState == TARGET_RECEIVED && markerFound){

			outputEvent = TARGET_MARKER_FOUND_EVENT;
			currentState = IDLE;
			cout << "Marker Searcher state changed: IDLE" << endl;

		}

	}

}

void MarkerSearcherCoordination::notifyTimeElapsed(){

	if(currentState == TARGET_RECEIVED && (! markerFound)){

		markerSearcherComputation->computeNextSearchPose();
		motionDone = false;
		currentState = GOAL_GENERATED;
		cout << "Marker Searcher state changed: GOAL GENERATED" << endl;

	}else if(currentState == MOVING){

		if(markerFound){
			if(motionDone){
				outputEvent = TARGET_MARKER_FOUND_EVENT;
				currentState = IDLE;
				cout << "Marker Searcher state changed: IDLE" << endl;
			}
		}else{
			markerSearcherComputation->computeNextSearchPose();
			motionDone = false;
			currentState = GOAL_GENERATED;
			cout << "Marker Searcher state changed: GOAL GENERATED" << endl;
		}

	}
}

void MarkerSearcherCoordination::notifyMotionStarted(){

	if(currentState == GOAL_GENERATED){
		currentState = MOVING;
		cout << "Marker Searcher state changed: MOVING" << endl;
		//motionDone = false;
	}

}

void MarkerSearcherCoordination::notifySearchPoseReached(){

	if(currentState == MOVING){
		motionDone = true;
	}

}

string MarkerSearcherCoordination::getOutputEvent(){

	return outputEvent;
}

}


