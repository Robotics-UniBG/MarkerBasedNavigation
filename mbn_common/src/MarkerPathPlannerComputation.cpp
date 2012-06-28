/********************************************************************************
 *
 * MarkerPathPlannerComputation
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
 * File: MarkerPathPlannerComputation.cpp
 * Created: June 4, 2012
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

#include "mbn_common/MarkerPathPlannerComputation.hpp"

namespace mbn_common{

MarkerPathPlannerComputation::MarkerPathPlannerComputation(){

}

MarkerPathPlannerComputation::MarkerPathPlannerComputation(vector<int> pathA1_A2, vector<int> pathA2_A1){

	setPathA1_A2(pathA1_A2);
	setPathA2_A1(pathA2_A1);

}

MarkerPathPlannerComputation::MarkerPathPlannerComputation(string pathA1_A2_filePath, string pathA2_A1_filePath){

	loadPathA1_A2(pathA1_A2_filePath);
	loadPathA2_A1(pathA2_A1_filePath);

}

MarkerPathPlannerComputation::~MarkerPathPlannerComputation(void){
}

void MarkerPathPlannerComputation::setPathA1_A2(vector<int> pathA1_A2){
	this->pathA1_A2 = pathA1_A2;
}

void MarkerPathPlannerComputation::setPathA2_A1(vector<int> pathA2_A1){
	this->pathA2_A1 = pathA2_A1;
}

vector<int> MarkerPathPlannerComputation::getPathA1_A2(){

	return pathA1_A2;

}

vector<int> MarkerPathPlannerComputation::getPathA2_A1(){

	return pathA2_A1;

}

void MarkerPathPlannerComputation::setVisibleMarkersIDs(vector<int> visibleMarkersIDs){

	this->visibleMarkers = visibleMarkersIDs;

}

bool MarkerPathPlannerComputation::computePath(int goalId, vector<int>& markerIdPath){

	markerIdPath.clear();

	// chek if one of the visible markers is the goal.
	// In this case we just have to return a path containing it
	for(vector<int>::iterator it = visibleMarkers.begin(); it != visibleMarkers.end(); it ++){
		if(*it == goalId){

			markerIdPath.push_back(goalId);
			return true;

		}
	}

	// check if the goal is in pathA1_A2
	vector<int>::iterator goalIterator = std::find(pathA1_A2.begin(), pathA1_A2.end(), goalId);
	bool goalInA1_A2 = goalIterator != pathA1_A2.end();
	if(! goalInA1_A2){
		// if not create the iterator to the goal from pathA2_A1
		goalIterator = std::find(pathA2_A1.begin(), pathA2_A1.end(), goalId);
		if(goalIterator == pathA2_A1.end()){
			// the goal is not in one of the two paths
			// we have to check if one of the visible markers is the goal
			return false;
		}
	}

	int bestId = -1;
	int bestCost = 1000000;

	for(vector<int>::iterator it = visibleMarkers.begin(); it != visibleMarkers.end(); it ++){

		// Analize the marker *it

		// Check if *it is in the path between Arenas 1 and 2
		vector<int>::iterator currentMarker = std::find(pathA1_A2.begin(), pathA1_A2.end(), *it);
		bool possibleStartInA1_A2 = currentMarker != pathA1_A2.end();
		if(! possibleStartInA1_A2){
			// if not Check if *it is in the path between Arenas 2 and 1
			currentMarker = std::find(pathA2_A1.begin(), pathA2_A1.end(), *it);
			if(currentMarker ==  pathA2_A1.end()){
				// *it is in not in one of the two paths
				continue;
			}
		}

		if(goalInA1_A2 == possibleStartInA1_A2){
			// Goal and *it are both ai pathA1_A2
			// compute its cost and store it if it is the best
			int cost = 0;
			while(currentMarker != goalIterator){
				currentMarker ++;
				cost ++;
			}

			if(cost<bestCost){
				bestCost = cost;
				bestId = *it;
				// *it is in the best up to now
			}

		}else{
			// *It is in a different path wrt goal
		}

	}

	if(bestId == -1){
		// no one of the visible marker is on the same path of the goal
		return false;
	}

	// now we know the best marker from which start

	vector<int>::iterator it;

	// retrieve an iterator to the best marker
	if(goalInA1_A2){
		// Best and Goal are both in pathA1_A2
		// We have to move towards the Arena 2

		it = std::find(pathA1_A2.begin(), pathA1_A2.end(), bestId);
		//vector<int>::iterator end = std::find(pathA1_A2.begin(), pathA1_A2.end(), goalId);
	}else{

		it = std::find(pathA2_A1.begin(), pathA2_A1.end(), bestId);


	}
	// the goal is not the end of one of the two paths, so we have to increase the iterator
	if(goalIterator != pathA1_A2.end() && goalIterator != pathA2_A1.end()){
		goalIterator ++;
	}

	// we push the path markers in the out parameter
	for( ; it != goalIterator; it ++){

		markerIdPath.push_back(*it);

	}

	return true;

}

bool MarkerPathPlannerComputation::loadPathA1_A2(string filePath){

	return loadMarkerPathFromFile(pathA1_A2, filePath);

}

bool MarkerPathPlannerComputation::loadPathA2_A1(string filePath){

	return loadMarkerPathFromFile(pathA2_A1, filePath);

}

bool MarkerPathPlannerComputation::loadMarkerPathFromFile(vector<int>& markerPath, string filePath){

	std::ifstream pathDescritptorFile(filePath.c_str());
	if (pathDescritptorFile.fail()) {
		cout << "ERROR: Marker Path Planner could not open " << filePath.c_str() << endl;
		return false;
	}
	YAML::Parser parser(pathDescritptorFile);
	YAML::Node doc;
	parser.GetNextDocument(doc);

	const YAML::Node& markers = doc["markers"];
	for(unsigned i=0;i<markers.size();i++) {
		int id;
		try {
			markers[i] >> id;
		} catch (YAML::InvalidScalar&) {
			cout << "ERROR: Something went wrong during the parsing of pathA1_A2" << endl;
			return false;
		}
		markerPath.push_back(id);

	}

	return true;

}

}


