/********************************************************************************
 *
 * MarkerPathIteratorComputation
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
 * File: MarkerPathIteratorComputation.cpp
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


#include "mbn_common/MarkerPathIteratorComputation.hpp"


namespace mbn_common{



MarkerPathIteratorComputation::MarkerPathIteratorComputation(){

	currentMarkerPath.clear();
	currentMarkerTargetID = -1;
	nextMarkerTargetID = -1;

}

MarkerPathIteratorComputation::~MarkerPathIteratorComputation(void){
}

void MarkerPathIteratorComputation::setMarkerPath(vector<int> markerPath){

	this->tempMarkerPath = markerPath;


}

void MarkerPathIteratorComputation::updateMarkerPath(){

	currentMarkerPath = tempMarkerPath;
	currentMarkerIndex = 0;

	currentMarkerTargetID = currentMarkerPath.at(currentMarkerIndex);

	// check if a next marker goal exist and set its id
	if(currentMarkerIndex < (currentMarkerPath.size() - 1)){
		nextMarkerTargetID = currentMarkerPath.at(currentMarkerIndex + 1);
	}else{
		nextMarkerTargetID = currentMarkerPath.at(currentMarkerIndex);
	}

}

bool MarkerPathIteratorComputation::setDetectedMarkers(vector<tf::Pose> markersPoses, vector<int> markersIDs){

	if(markersPoses.size() != markersIDs.size()){
		return false;
	}

	this->tempDetectedMarkersPoses = markersPoses;
	this->tempDetectedMarkersIDs = markersIDs;

	return true;

}

void MarkerPathIteratorComputation::updateDetectedMarkers(){

	detectedMarkersIDs = tempDetectedMarkersIDs;
	detectedMarkersPoses = tempDetectedMarkersPoses;

	tempDetectedMarkersIDs.clear();
	tempDetectedMarkersPoses.clear();

}

void MarkerPathIteratorComputation::setOdometryPose(tf::Pose lastOdometryPose){

	this->lastOdometryPose = lastOdometryPose;

}

bool MarkerPathIteratorComputation::getGoalPose(tf::Pose& goalPose){
	if(goalID == -1){
		return false;
	}
	goalPose = this->goalPose;
	return true;
}

int MarkerPathIteratorComputation::getGoalID(){
	return goalID;
}

int MarkerPathIteratorComputation::getCurrentTargetMarkerID(){
	return currentMarkerTargetID;
}

int MarkerPathIteratorComputation::getNextTargetMarkerID(){
	return nextMarkerTargetID;
}

bool MarkerPathIteratorComputation::computeTargetMarkerPoseAndID(){

	tf::Pose detectedMarkerPose;
	int detectedMarkerID;

	bool goalFound = false;
	goalID = -1;

	if(currentMarkerPath.size() > 0){

		for(unsigned int i = 0; i < detectedMarkersPoses.size(); i++){

			detectedMarkerPose = detectedMarkersPoses.at(i);
			detectedMarkerID = detectedMarkersIDs.at(i);
			if(detectedMarkerID == currentMarkerTargetID || detectedMarkerID == nextMarkerTargetID){

				goalFound = true;
				goalID = detectedMarkerID;
				goalPose.mult(lastOdometryPose, detectedMarkerPose);

				// check if the next marker goal has been detected,
				// in that case increase current marker index
				if(detectedMarkerID == nextMarkerTargetID && currentMarkerTargetID != nextMarkerTargetID){
					increaseCurrentMarkerIndex();
				}

			}

		}

		if(goalFound){
			return true;
		}else{
			// goal not found
			return false;
		}

	}else{
		// The marker path is not yet set
		return false;
	}

}

void MarkerPathIteratorComputation::increaseCurrentMarkerIndex(){

	currentMarkerIndex ++;
	currentMarkerTargetID = currentMarkerPath.at(currentMarkerIndex);
	if(currentMarkerIndex < (currentMarkerPath.size() - 1)){
		nextMarkerTargetID = currentMarkerPath.at(currentMarkerIndex + 1);
	}else{
		nextMarkerTargetID = currentMarkerPath.at(currentMarkerIndex);
	}
	cout << "MARKER CHANGED" << endl;

}

bool MarkerPathIteratorComputation::isCurrentTargetTheLast(){

	return (currentMarkerTargetID == currentMarkerPath.at(currentMarkerPath.size() - 1));

}

}



