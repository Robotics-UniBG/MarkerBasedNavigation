/********************************************************************************
 *
 * MarkerLocatorComputation
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
 * File: MarkerLocatorComputation.cpp
 * Created: Jan 20, 2012
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

#include "mbn_common/MarkerLocatorComputation.hpp"


using namespace std;

namespace mbn_common{


MarkerLocatorComputation::MarkerLocatorComputation(const string & name) {
	this->name = name;
	//default settings
	cameraDistorsionParamPath = "./camera/no_distortion.cal"; //by default component set itself for undirstosion lens
	cameraWidth = 640;
	cameraHeight = 480;
	minConfidence = 0.5;
	enableIDfilter = 0; //to find all markers
	enableDetectOnlyTheBest = 0;
	useBCH = 1;
	markersBaseWidth = 40.0;
	tf::Pose cameraPose;
	cameraPose.setOrigin(btVector3(0, 0, 0));
	cameraPose.setRotation(btQuaternion(0, 0, 0, 1));
//	tf::poseTFToMsg(tempPose, cameraPose);
}

MarkerLocatorComputation::~MarkerLocatorComputation(void){
}

string MarkerLocatorComputation::getCameraDistorsionParamPath(){
	return cameraDistorsionParamPath;
}

int MarkerLocatorComputation::getCameraHeight(){
	return cameraHeight;
}

int MarkerLocatorComputation::getCameraWidth(){
	return cameraWidth;
}

tf::Pose MarkerLocatorComputation::getCameraPose(){
	return cameraPose;
}

double MarkerLocatorComputation::getMarkersBaseWidth(){
	return markersBaseWidth;
}

double MarkerLocatorComputation::getMinConfidence(){
	return minConfidence;
}

bool MarkerLocatorComputation::isEnableDetectOnlyTheBest(){
	return enableDetectOnlyTheBest;
}

bool MarkerLocatorComputation::isEnableIDfilter(){
	return enableIDfilter;
}

bool MarkerLocatorComputation::isUseBCH(){
	return useBCH;
}

bool MarkerLocatorComputation::setCameraDistorsionParamPath(string cameraDistorsionParamPath){
	this->cameraDistorsionParamPath = cameraDistorsionParamPath;
	if (FILE * file = fopen(cameraDistorsionParamPath.c_str(), "r"))//check if file esist
	{
		fclose(file);
		this->cameraDistorsionParamPath = cameraDistorsionParamPath;
		return true;
	}
	return false;
}

bool MarkerLocatorComputation::setCameraHeight(int cameraHeight){
	if(cameraHeight > 1024)
		return false;

	this->cameraHeight = cameraHeight;
	return true;
}

bool MarkerLocatorComputation::setCameraWidth(int cameraWidth){
	if(cameraWidth > 1024)
		return false;

	this->cameraWidth = cameraWidth;
	return true;
}

void MarkerLocatorComputation::setEnableDetectOnlyTheBest(bool enableDetectOnlyTheBest){
	this->enableDetectOnlyTheBest = enableDetectOnlyTheBest;
}

void MarkerLocatorComputation::setEnableIDfilter(bool enableIDfilter){
	this->enableIDfilter = enableIDfilter;
}

void MarkerLocatorComputation::setCameraPose(tf::Pose cameraPose){
	this->cameraPose = cameraPose;
}

void MarkerLocatorComputation::setMarkersBaseWidth(double markersBaseWidth){
	this->markersBaseWidth = markersBaseWidth;
}

bool MarkerLocatorComputation::setMinConfidence(double minConfidence){
	if(minConfidence < 0.0 || minConfidence > 1.0)
		return false;

	this->minConfidence = minConfidence;
	return true;
}

void MarkerLocatorComputation::setUseBCH(bool useBch){
	useBCH = useBch;
}

void MarkerLocatorComputation::ComputeMarkerToCameraPose(Marker &markerToprocess, tf::Pose &camera2markerPose){
	float marker2CameraMatrix[3][4];
	float markerWidth2MarkerLocatorComputationEngineWidth = markerToprocess.getWidth() / markersBaseWidth;
	// get the classical AR pose matrix, no GL matrices used here
	tracker->getARMatrix(marker2CameraMatrix);
	//convert matrix distances from millimeters to meters
	marker2CameraMatrix[0][3] = marker2CameraMatrix[0][3] * MM_TO_METERS * markerWidth2MarkerLocatorComputationEngineWidth;
	marker2CameraMatrix[1][3] = marker2CameraMatrix[1][3] * MM_TO_METERS * markerWidth2MarkerLocatorComputationEngineWidth;
	marker2CameraMatrix[2][3] = marker2CameraMatrix[2][3] * MM_TO_METERS * markerWidth2MarkerLocatorComputationEngineWidth;
//	tf::Pose markerPose = markerToprocess->getPose();
	utility.getPoseFromARTKMatrix(marker2CameraMatrix, camera2markerPose);
//	markerToprocess->setPose(markerPose);
}

void MarkerLocatorComputation::addNewMarkerToFoundMarkersList(ARToolKitPlus::ARMarkerInfo *marker_found,
		vector<int> *detectedMarkersID,
		vector<tf::Pose> *detectedMarkersPoses){
	map<int,double>::iterator iter = mapID2Dimension.find(marker_found->id); //look for the marker in the lookup table
	double dimension = markersBaseWidth;
	if(iter != mapID2Dimension.end())
		dimension = iter->second; //value

	Marker newMarkFound = Marker(marker_found->id, dimension);
	tf::Pose camera2markerPose;
	ComputeMarkerToCameraPose(newMarkFound, camera2markerPose);

//	tf::Pose cameraPoseTf;
//	tf::poseMsgToTF(cameraPose,cameraPoseTf);
	camera2markerPose.mult(cameraPose, camera2markerPose);//the otput of this call overwrite camera2markerPose with the aspected final marker pose
	detectedMarkersID->push_back(marker_found->id);
	detectedMarkersPoses->push_back(camera2markerPose);
}

bool MarkerLocatorComputation::setBlackWhiteThreshold(int blackWhiteThreshold){
	if(blackWhiteThreshold < 0 || blackWhiteThreshold > 255)
		return false;

	if(blackWhiteThreshold > 0)
		thresholdToUse.push_back(blackWhiteThreshold);

	else{
		// give it up to 8 tries to set a good Threshold
		thresholdToUse.push_back(15);
		thresholdToUse.push_back(40);
		thresholdToUse.push_back(65);
		thresholdToUse.push_back(140);
		thresholdToUse.push_back(165);
		thresholdToUse.push_back(190);
		thresholdToUse.push_back(215);
		thresholdToUse.push_back(240);
	}
	return true;
}

/*
 * This function receives an image with BGR format (look at tracker->setPixelFormat call), and it looks for all markers inside it.
 * The vector vectorMarkersFound is filled with all markers informations (roto-translation camera2marker and its ID).
 */
void MarkerLocatorComputation::findMarkers(unsigned char *imageData, vector<int> *detectedMarkersID,vector<tf::Pose> *detectedMarkersPoses){
//void MarkerLocatorComputation::findMarkers(unsigned char *imageData, vector<Marker> *vectorMarkersFound){
	detectedMarkersID->clear();
	detectedMarkersPoses->clear();
	for(int i=0;i<(int)thresholdToUse.size();i++){//process same images with one or more thresholds to find markers
		tracker->setThreshold(thresholdToUse.at(i));
		ARToolKitPlus::ARMarkerInfo *marker_info = NULL;
		int numberOfMarkersDetected = 0;
		std::vector<int> markerId = tracker->calc(imageData, &marker_info, &numberOfMarkersDetected);
		if(numberOfMarkersDetected <= 0 || marker_info->id == -1)
			continue;

		if(!enableDetectOnlyTheBest)
			for(int markerFound = 0;markerFound < numberOfMarkersDetected;markerFound++){
				if(enableIDfilter){
					/*
					 * localization function rules:
					 * Can find more markers in a single image.
					 * It can find only one marker for each ID. Only the markers listed in markersVectorToFind are sent to the output port.
					 * This setting is useful to reduce false positive markers recognization.
					 */
					tracker->selectDetectedMarker(marker_info->id);
					for(vector<int>::iterator it = markersVectorIDtoFind.begin();it < markersVectorIDtoFind.end();it++){
						if(marker_info->id == *it && marker_info->cf >= minConfidence && marker_info->line[2][3] != 0.0 && marker_info->line[2][3] != -0.0){
							addNewMarkerToFoundMarkersList(marker_info, detectedMarkersID, detectedMarkersPoses);
						}
					}
				}
				else{
					/*
					 * localization function rules:
					 * Can find many markers in a single image. All markers detected are sent to the output port.
					 * This function is useful to recognize all markers without list them.
					 */
					if(marker_info->id >= 0){
						//for each good marker found
						tracker->selectDetectedMarker(marker_info->id);
						if(marker_info->cf >= minConfidence && marker_info->line[2][3] != 0.0 && marker_info->line[2][3] != -0.0){
							addNewMarkerToFoundMarkersList(marker_info, detectedMarkersID, detectedMarkersPoses);
						}
					}
				}
				marker_info++;
			}

		if(enableDetectOnlyTheBest){
			/*
			 * localization function rules:
			 * this setting is useful: the marker with the best confidence is choosen.
			 * This option is useful for high performances
			 */
			int id_best_markerDetected = tracker->selectBestMarkerByCf();
			if(id_best_markerDetected >= 0 && marker_info->cf >= minConfidence && marker_info->line[2][3] != 0.0 && marker_info->line[2][3] != -0.0){
				addNewMarkerToFoundMarkersList(marker_info, detectedMarkersID, detectedMarkersPoses);
			}
		}
		if(detectedMarkersID->size()!=0)//for multiple thresholds: don't set a new threshold if you've found any markers
			break;//go out for cycle
	}
	return;
}

bool MarkerLocatorComputation::configureMarkerLocatorComputation(){
	/*
	 *  create a tracker that:
	 *		- processes images of resolution cameraWidth x cameraHeight
	 *  	- supports 6x6 sized marker images (required for binary markers)
	 *  	- samples at a maximum of 18x18
	 *  	- can detect a maximum of 24 patterns in one image
	 */
	tracker = new TrackerSingleMarker(cameraWidth, cameraHeight, 24, 6, 6, 18, 0);
	tracker->setPixelFormat(ARToolKitPlus::PIXEL_FORMAT_BGR); //ARToolKitPlus::PIXEL_FORMAT_LUM for gay image
	//		tracker->setLoadUndistLUT(true);
	if(!tracker->init(cameraDistorsionParamPath.c_str(), 1.0f, 1000.0f)){
		cout << "ERROR: init() failed\n" << endl;
		delete tracker;
		return false;
	}
	// print the settings for us
	tracker->getCamera()->printSettings();
	// define size of the marker in OpenGL units
	tracker->setPatternWidth(markersBaseWidth);
	// the marker in the BCH test image has a thin border...
	tracker->setBorderWidth(useBCH ? 0.125f : 0.250f);
	// anlyze the full image
	tracker->setImageProcessingMode(ARToolKitPlus::IMAGE_FULL_RES);
	// use the RPP estimator as it has the lowest "jumping" rate
	tracker->setPoseEstimator(ARToolKitPlus::POSE_ESTIMATOR_ORIGINAL);
	// use the tool in tools/IdPatGen to generate markers
	tracker->setMarkerMode(useBCH ? ARToolKitPlus::MARKER_ID_BCH : ARToolKitPlus::MARKER_ID_SIMPLE);
	/*
	 * initialization of markers dimensions lookup table
	 */
	if(markersVectorIDtoFind.size() == markersDimensions.size()){
		for(int w = 0;w < (int)((markersVectorIDtoFind.size())); w++)
			mapID2Dimension.insert(pair<int,double>(markersVectorIDtoFind.at(w), markersDimensions.at(w)));
	}else{
		cout<<"The list of markers dimensions has a different size than markers ID"<<endl;
		return false;
	}
	return true;
}

}
