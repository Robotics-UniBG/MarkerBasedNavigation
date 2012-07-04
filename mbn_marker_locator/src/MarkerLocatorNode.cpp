/********************************************************************************
 *
 * MarkerLocatorNode
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
 * File: MarkerLocatorNode.cpp
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

#include "mbn_marker_locator/MarkerLocatorNode.hpp"
#include "mbn_msgs/MarkersPoses.h"
#include "mbn_msgs/MarkerPose.h"
#include "mbn_msgs/MarkersIDs.h"
#include <yaml-cpp/yaml.h>
#include <fstream>

using namespace cv_bridge;
using namespace sensor_msgs::image_encodings;
using namespace std;
using namespace mbn_common;

//MAIN
int main(int argc, char **argv) {

	//Setup ros node and topics
	ros::init(argc, argv, "rgb_marker_locator");
	ros::NodeHandle nodeHandle;
	image_transport::ImageTransport it(nodeHandle);
	image_transport::Subscriber imageSubscriber = it.subscribe("image_topic", 1, imageCallback);
	markersPosesPublisher = nodeHandle.advertise<mbn_msgs::MarkersPoses>("markers_poses_topic", 1000);
	markersIDsPublisher = nodeHandle.advertise<mbn_msgs::MarkersIDs>("markers_ids_topic", 1000);

	//Setup marker locator
	loadConstants();
	configureMarkerLocator();

	//Go spinning
	ros::spin();

	return 0;
}

void imageCallback(const sensor_msgs::ImageConstPtr& imageMsg) {
	MarkerLocatorStep(imageMsg);
}

bool configureMarkerLocator(){
	markerLocator=new MarkerLocatorComputation("MarkerLocator");
	if(!markerLocator->setCameraDistorsionParamPath(cameraDistorsionParamPath)){
		ROS_ERROR("Error setting camera distorsion parameters");
		return false;
	}
	if(!markerLocator->setCameraWidth(cameraWidth)){
		ROS_ERROR("Error setting camera width");
		return false;
	}
	if(!markerLocator->setCameraHeight(cameraHeight)){
		ROS_ERROR("Error setting camera height");
		return false;
	}
	if(!markerLocator->setMinConfidence(min_confidence)){//the minimal confidence used as edge to recognize the markers
		ROS_ERROR("Error setting minimum confidence");
		return false;
	}
	if(!markerLocator->setBlackWhiteThreshold(blackWhiteThreshold)){
		ROS_ERROR("Error setting black/white threshold");
		return false;
	}
	markerLocator->setEnableIDfilter(enableIDfilter);//if true, only the markers defined in markersVectorIDtoFind will be searched, you can enable this only if enableDetectOnlyTheBest is disabled
	markerLocator->setEnableDetectOnlyTheBest(enableDetectOnlyTheBest);//if true, marker locator will look for the marker with the best confidence in the image
	markerLocator->setUseBCH(useBCH);

	markerLocator->setMarkersBaseWidth(markersBaseWidth);//it's the default distance used for all markers

	markerLocator->markersVectorIDtoFind=markersVectorIDtoFind;//the list of BHC ID that we want to use
	markerLocator->markersDimensions=markersDimensions;//the list of markers dimensions
	markerLocator->setCameraPose(fixedCameratransform);

	return markerLocator->configureMarkerLocatorComputation();


}

bool loadConstants(){

	string pathFile = "configuration_marker_locator.yaml";
	std::ifstream pathDescritptorFile(pathFile.c_str());
	if (pathDescritptorFile.fail()) {
		ROS_ERROR("Configuration file cannot be opened %s", pathFile.c_str());
		return false;
	}
	YAML::Parser parser(pathDescritptorFile);
	YAML::Node doc;
	parser.GetNextDocument(doc);

	try {
		doc["cameraDistorsionParamPath"] >> cameraDistorsionParamPath;
		doc["cameraWidth"] >> cameraWidth;
		doc["cameraHeight"] >> cameraHeight;
		doc["blackWhiteThreshold"] >> blackWhiteThreshold;
		doc["min_confidence"] >> min_confidence;
		doc["enableIDfilter"] >> enableIDfilter;
		doc["enableDetectOnlyTheBest"] >> enableDetectOnlyTheBest;
		doc["useBCH"] >> useBCH;


		double x, y, z, qx, qy, qz, qw;

		//Load data
		doc["fixedCameratransform_orientation_x"] >> qx;
		doc["fixedCameratransform_orientation_y"] >> qy;
		doc["fixedCameratransform_orientation_z"] >> qz;
		doc["fixedCameratransform_orientation_w"] >> qw;
		doc["fixedCameratransform_position_x"] >> x;
		doc["fixedCameratransform_position_y"] >> y;
		doc["fixedCameratransform_position_z"] >> z;

		//Set data
		fixedCameratransform.getOrigin().setX(x);
		fixedCameratransform.getOrigin().setY(y);
		fixedCameratransform.getOrigin().setZ(z);
		fixedCameratransform.setRotation(btQuaternion(qx,qy,qz,qw));

		doc["markersBaseWidth"] >>  markersBaseWidth;

		const YAML::Node& markersIDs = doc["markersVectorIDtoFind"];
		for(unsigned i=0;i<markersIDs.size();i++) {
			int id;
			markersIDs[i] >> id;
			markersVectorIDtoFind.push_back(id);
		}

		const YAML::Node& markersSize = doc["markersDimensions"];
		for(unsigned i=0;i<markersSize.size();i++) {
			int size;
			markersSize[i] >> size;
			markersDimensions.push_back(size);
		}

	}catch (YAML::InvalidScalar&) {
		ROS_ERROR("Something went wrong during the parsing of configuration file");
		return false;
	}


	return true;
}

void MarkerLocatorStep(sensor_msgs::ImageConstPtr imageInput){

	cv_bridge::CvImageConstPtr openCvImage;
	try{
		// convert ROS image to openCv image,
		// sharing it when possible
		openCvImage = cv_bridge::toCvShare(imageInput,"bgr8");
	}catch (cv_bridge::Exception & e){
		ROS_ERROR("Could not convert from %s to 'bgr8'.", imageInput->encoding.c_str());
	}

	mbn_msgs::MarkersPoses detectedMarkersPoses;
	mbn_msgs::MarkersIDs detectedMarkersIDs;

	vector<int> vectorMarkersIDsFound;
	vector<tf::Pose> vectorMarkersPosesFound;
	markerLocator->findMarkers(openCvImage.get()->image.data,&vectorMarkersIDsFound,&vectorMarkersPosesFound);

	for(unsigned int x=0; x < vectorMarkersIDsFound.size(); x++){
		int currentMarkerID = vectorMarkersIDsFound.at(x);
		mbn_msgs::MarkerPose markerPose=mbn_msgs::MarkerPose();
		markerPose.marker_id=currentMarkerID;
		markerPose.header.stamp= imageInput->header.stamp;
		//log info useful for debugging
		ROS_INFO("Marker found ID: %i ", markerPose.marker_id);
		ROS_INFO("%s",utility.getPositionAndOrientation(vectorMarkersPosesFound.at(x)).c_str());

		tf::poseTFToMsg(vectorMarkersPosesFound.at(x), markerPose.poseWRTRobotFrame);

		detectedMarkersPoses.markersPoses.push_back(markerPose);
		detectedMarkersIDs.markersIDs.push_back(markerPose.marker_id);
	}
	markersPosesPublisher.publish(detectedMarkersPoses);
	markersIDsPublisher.publish(detectedMarkersIDs);

}


