/********************************************************************************
 *
 * MarkerPathIteratorCoordination
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
 * File: MarkerPathIteratorCoordination.cpp
 * Created: June 11, 2012
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


#include "mbn_common/MarkerPathIteratorCoordination.hpp"


namespace mbn_common{



MarkerPathIteratorCoordination::MarkerPathIteratorCoordination(MarkerPathIteratorComputation* markerPathIteratorComputer){

	this->markerPathIteratorComputation = markerPathIteratorComputer;

	currentState = IDLE;
	pathCompleted = false;

}

MarkerPathIteratorCoordination::~MarkerPathIteratorCoordination(void){
}

void MarkerPathIteratorCoordination::notifyMarkerPathReceived(){

	if(currentState == IDLE){
		markerPathIteratorComputation->updateMarkerPath();
		currentState = PATH_RECEIVED;
		cout << "Marker Path Iterator state changed: PATH RECEIVED" << endl;
	}

}

void MarkerPathIteratorCoordination::notifyDetectedMarkersReceived(){


	if(currentState == PATH_RECEIVED || currentState == MOVING){

		markerPathIteratorComputation->updateDetectedMarkers();
		// the method return true when the current or the next target has been
		// found
		bool targetMarkerFound = markerPathIteratorComputation->computeTargetMarkerPoseAndID();

		if((! targetMarkerFound) && ( currentState != MOVING)){
			currentState = SEARCH;
			outputEvent = createMarkerNotFoundEvent();
			cout << "Marker Path Iterator state changed: SEARCH (NO MARKERS)" << endl;
		}else if (targetMarkerFound){

			currentState = GOAL_GENERATED;
			cout << "Marker Path Iterator state changed: GOAL GENERATED" << endl;
		}

	}


}

void MarkerPathIteratorCoordination::notifyCurrentMarkerReached(){

	if(currentState == MOVING){

		if(markerPathIteratorComputation->isCurrentTargetTheLast()){
			pathCompleted = true;
			currentState = IDLE;
			outputEvent = MARKER_GOAL_REACHED_EVENT;
			cout << "Marker Path Iterator state changed: IDLE" << endl;
		}else{
			markerPathIteratorComputation->increaseCurrentMarkerIndex();
			currentState = SEARCH;
			outputEvent = createMarkerNotFoundEvent();
			cout << "Marker Path Iterator state changed: SEARCH (TARGET REACHED AND NO MARKERS)" << endl;

		}

	}

}

void MarkerPathIteratorCoordination::notifyMotionStarted(){

	if(currentState == GOAL_GENERATED){
		currentState = MOVING;
		cout << "Marker Path Iterator state changed: MOVING" << endl;
	}

}

void MarkerPathIteratorCoordination::notifyMarkerFound(){

	if(currentState == SEARCH){
		currentState = PATH_RECEIVED;
		cout << "Marker Path Iterator state changed: PATH_RECEIVED (MARKER SEARCH DONE)" << endl;
	}


}

string MarkerPathIteratorCoordination::getOutputEvent(){

	string returnEvent = outputEvent;
	outputEvent = "";
	if(returnEvent.compare("") != 0){
		return returnEvent;
	}else{
		return "";
	}

}

string MarkerPathIteratorCoordination::createMarkerNotFoundEvent(){
	stringstream event;
	event << TARGET_MARKER_NOT_FOUND_EVENT << markerPathIteratorComputation->getCurrentTargetMarkerID();
	return event.str();

}


}



