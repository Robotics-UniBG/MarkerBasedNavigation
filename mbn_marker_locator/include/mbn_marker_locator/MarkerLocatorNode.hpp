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

#ifndef MARKER_LOCATOR_NODE_HPP
#define MARKER_LOCATOR_NODE_HPP

#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "image_transport/image_transport.h"
#include <opencv2/highgui/highgui.hpp>

#include "mbn_common/MarkerLocatorComputation.hpp"
#include "mbn_common/Marker.hpp"
#include "mbn_common/Utility.hpp"

using namespace mbn_common;
//PROPERTIES
// Camera setup and calibration
string cameraDistorsionParamPath;
int cameraWidth;
int cameraHeight;

// Marker detection
int	blackWhiteThreshold;
vector<int> thresholdToUse;
double min_confidence;
bool enableIDfilter;
bool enableDetectOnlyTheBest;
bool useBCH;
tf::Pose fixedCameratransform;
double markersBaseWidth; //it's the default distance used for all markers
vector<int> markersVectorIDtoFind;//the list of BHC ID that we want to use
vector<double> markersDimensions;//the list of markers dimensions
map<int, double> mapID2Dimension;

//Objects
/**
 * MarkerLocatorComputation instance
 */
mbn_common::MarkerLocatorComputation *markerLocator;

/**
 * Utility class instance
 */
Utility utility;

//Constants
/**
 * mm to meters conversion.
 */
static const double MM_TO_METERS = 0.001;

//Publishers
/**
 * Marker Pose publisher.
 */
ros::Publisher markersPosesPublisher;

/**
 * Marker IDs publisher.
 */
ros::Publisher markersIDsPublisher;

//Prototypes
/**
 * Configure the marker locator.
 */
bool configureMarkerLocator();

/**
 * Load data from file.
 */
bool loadConstants();

/**
 * Marker locator computationn step
 * \param[in] openCvImage image to be elaborated.
 */
void MarkerLocatorStep(sensor_msgs::ImageConstPtr openCvImage);

/**
 * Callback function
 * \param[in] imageMsg image to be elaborated.
 */
void imageCallback(const sensor_msgs::ImageConstPtr& imageMsg);
#endif
