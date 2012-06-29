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
 * File: MarkerLocatorComputation.hpp
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
#ifndef MARKER_LOCATOR_COMPUTATION_HPP
#define MARKER_LOCATOR_COMPUTATION_HPP

// ARToolKitPlus includes
#include <ar.h>
#include <ARToolKitPlus/TrackerSingleMarker.h>
#include <ARToolKitPlus/matrix.h>
#include <ARToolKitPlus/extra/rpp.h>

#include <map>
#include <vector>
#include <string>
#include <math.h>
#include <sys/time.h>
#include <iostream>
#include <sstream>

#include "Marker.hpp"
#include "mbn_common/Utility.hpp"

using namespace ARToolKitPlus;
using namespace std;
using namespace Utilities;

namespace mbn_common{


class MarkerLocatorComputation{

public:

	/**
	 * Class constructor.
	 * \param[in] name the name of the instance.
	 */
	MarkerLocatorComputation(const string & name);

	/**
	 * Class destroyer.
	 */
	~MarkerLocatorComputation(void);

	/**
	 * Configures the class.
	 * \return the status of the operation.
	 */
	bool configureMarkerLocatorComputation();

	/**
	 * \param[in] imageData reference to image to be analyzed.
	 * \param[out] list of all IDs of found markers
	 * \param[out] list of all poses of found markers
	 */
	void findMarkers(unsigned char *imageData, vector<int> *detectedMarkersID,vector<tf::Pose> *detectedMarkersPoses);

	/**
	 * Gets the camera distorsion file path.
	 * \return the status of the operation.
	 */
	string getCameraDistorsionParamPath();

	/**
	 * Get the camera height.
	 * \return the camera height.
	 */
	int getCameraHeight() ;

	/**
	 * Get the camera width.
	 * \return the camera width.
	 */
	int getCameraWidth() ;

	/**
	 * Enable detect only best marker mode status.
	 * \return true if active, false if not.
	 */
	bool isEnableDetectOnlyTheBest() ;

	/**
	 * Enable ID filter mode status.
	 * \return true if enabled, false if not.
	 */
	bool isEnableIDfilter() ;

	/**
	 * Get camera position and orientation.
	 * \return camera position and orientation in a Pose.
	 */
	tf::Pose getCameraPose() ;

	/**
	 * Get marker base width.
	 * \return marker base width in mm.
	 */
	double getMarkersBaseWidth() ;

	/**
	 * Get minimum confidence.
	 * \return minimum confidence.
	 */
	double getMinConfidence() ;

	/**
	 * Use BCH status.
	 * \return true if active, false if not.
	 */
	bool isUseBCH() ;

	/**
	 * Set the camera distortion file path.
	 * \param[in] cameraDistorsionParamPath string containing the file path.
	 */
	bool setCameraDistorsionParamPath(string cameraDistorsionParamPath);

	/**
	 * Set camera height.
	 * \param[in] cameraHeight the camera vertical size.
	 */
	bool setCameraHeight(int cameraHeight);

	/**
	 * Set camera width.
	 * \param[in] cameraWidth the camera horizontal size.
	 */
	bool setCameraWidth(int cameraWidth);

	/**
	 * Set enable detect only the best.
	 * \param[in] setEnableDetectOnlyTheBest flag, true if active.
	 */
	void setEnableDetectOnlyTheBest(bool enableDetectOnlyTheBest);

	/**
	 * Enable ID filter.
	 * \param[in] enableIDfilter enables the ID filtering if true.
	 */
	void setEnableIDfilter(bool enableIDfilter);

	/**
	 * set camera Pose
	 * \param[in] cameraPose the camera rototraslation wrt the center of the robot or else.
	 */
	void setCameraPose(tf::Pose cameraPose);

	/**
	 * Set markers base width.
	 * \param[in] markerBaseWidth the measure of the markers in mm.
	 */
	void setMarkersBaseWidth(double markersBaseWidth);

	/**
	 * Set minimum confidence level.
	 * \param[in] minConfidence min. confidence value [0...1]
	 */
	bool setMinConfidence(double minConfidence);

	/**
	 * Set the use of BCH coding for markers.
	 * \param[in] useBCH true if active.
	 */
	void setUseBCH(bool useBCH);

	/**
	 * Set threshold.
	 * \param[in] blackWhiteThreshold threshold to be tuned according to environment lighting.
	 */
	bool setBlackWhiteThreshold(int blackWhiteThreshold);

	/**
	 * The list of markers dimensions
	 */
	vector<double> markersDimensions;
	/**
	 * The list of markers ID that we want to use
	 */
	vector<int> markersVectorIDtoFind;

private:
	/**
	 * millimeters to meters conversion
	 */
	static const double MM_TO_METERS = 0.001;

	//PROPERTIES
	/**
	 *   the name of the file that contains camera distorsion parameters
	 */
	string cameraDistorsionParamPath;

	/**
	 *  camera resolution
	 */
	int cameraWidth;

	/**
	 *  camera resolution
	 */
	int cameraHeight;
	/**
	 * Black-White threshold,if value is <=0, a set of thresholds will be set automatically
	 */
	int	blackWhiteThreshold;

	/**
	 *  The minimal requested confidence necessary to recognize the markers
	 */
	double minConfidence;


	/**
	 * if true, only the markers defined in markersVectorIDtoFind will be searched, you should enable this only if enableDetectOnlyTheBest is disabled
	 */
	bool enableIDfilter;

	/**
	 *  if true, marker locator will look for the marker with the best confidence in the image
	 */
	bool enableDetectOnlyTheBest;

	/**
	 * if true it will use markers with BCH encoding, otherwise it will use 9bit encoding
	 */
	bool useBCH;

	/**
	 * The default distance used for all markers
	 */
	double markersBaseWidth;

	/**
	 * camera position
	 */
	tf::Pose cameraPose;

	/**
	 * all black/white thresholds to be used
	 */
	vector<int> thresholdToUse;

	/**
	 * name of the instance
	 */
	string name;

	/**
	 * map of association marker ID <-> marker dimension
	 */
	map<int, double> mapID2Dimension;

	/**
	 * marker search engine
	 */
	ARToolKitPlus::TrackerSingleMarker *tracker;


	/**
	 * \param[in] markers found to add
	 * \param[out] list of all IDs of found markers
	 * \param[out] list of all poses of found markers
	 *
	 * Adds a found marker to the list of all found markers computing and setting camera to marker transform
	 */
	void addNewMarkerToFoundMarkersList(ARToolKitPlus::ARMarkerInfo *marker_found, vector<int> *detectedMarkersID, vector<tf::Pose> *detectedMarkersPoses);

	/**
	 * \param[in] a marker that need roto-traslation initialization
	 * \param[in] roto-traslation found by ARTK+
	 *
	 * Fills the roto-translation field of the found marker
	 */
	void ComputeMarkerToCameraPose(Marker &markerToprocess, tf::Pose &camera2markerPose);

	/**
	 * inner utility class
	 */
	Utility utility;

};

}
#endif /* COMPUTATION_HPP_ */
