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
 * File: MarkerPathIteratorComputation.hpp
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

#ifndef MARKER_PATH_ITERATOR_COMPUTATION_HPP
#define MARKER_PATH_ITERATOR_COMPUTATION_HPP

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <algorithm>
#include <vector>

#include <tf/tf.h>

using namespace std;

namespace mbn_common{

class MarkerPathIteratorComputation{

public:

	/**
	 * Create a new MarkerPathIteratorComputation
	 */
	MarkerPathIteratorComputation();

	/**
	 * Class destroyer.
	 */
	~MarkerPathIteratorComputation(void);

	/**
	 * \param markerPath the vector of the markers IDs that have to be reached
	 *
	 * Store a temporarily marker path, which can be committed later by calling
	 * the method updateMarkerPath
	 */
	void setMarkerPath(vector<int> markerPath);

	/**
	 * Commit the last marker path stored with the method setMarkerPath. Only
	 * when this method has been called the marker path can be iterated.
	 */
	void updateMarkerPath();

	/**
	 * \param markersPoses the vector of the relative position of the detected
	 * markers
	 * \param markersIDs the vector of the identifiers of the detected
	 * markers
	 * \return true if the vectors have the same size, false otherwise (it means
	 * that the operation went wrong)
	 *
	 * Store in a temporarily vector a set of detected markers. This vector can be
	 * committed later by calling the method updateDetectedMarkers
	 */
	bool setDetectedMarkers(vector<tf::Pose> markersPoses, vector<int> markersIDs);

	/**
	 * Commit the detected markers stored with the method setDetectedMarkers. Only
	 * when this method has been called the detected markers can be processed.
	 */
	void updateDetectedMarkers();

	/**
	 * \param lastOdometryPose the last (TF) pose of the robot
	 *
	 * Stores the robot odometry
	 */
	void setOdometryPose(tf::Pose lastOdometryPose);

	/**
	 * \param goalPose the (TF) pose in which the current marker target pose will be
	 * returned
	 * \return true if the current target marker has been detected,
	 * false otherwise
	 *
	 * Return the position of the last detected target marker on the path
	 */
	bool getGoalPose(tf::Pose& goalPose);

	/**
	 * Return the ID of the last detected marker on the path.
	 * Is is -1 if no one of the visible markers are on the path.
	 */
	int getGoalID();

	/**
	 * Return the id of the current marker the robot has to reach on the path
	 */
	int getCurrentTargetMarkerID();

	/**
	 * Return the id of the next marker the robot has to reach on the path
	 */
	int getNextTargetMarkerID();

	/**
	 * \return true if the target marker has been found in the latest vector
	 * of detected markers
	 *
	 * Compute the absolute position of the current marker goal or next goal,
	 * which can be retrieved by calling the method getGoalPose.
	 */
	bool computeTargetMarkerPoseAndID();

	/**
	 * \return true if the current target marker is the last marker of the path
	 */
	bool isCurrentTargetTheLast();

	/**
	 * Increase the index that defines the position of the current marker target
	 * in the marker path and updates the current and next marker targets.
	 */
	void increaseCurrentMarkerIndex();


private:

	/**
	 * Store the last received robot odometry
	 */
	tf::Pose lastOdometryPose;

	/**
	 * Store the identifiers of the last detected markers.
	 * This is the vector processed by the method computeNextMarkerPoseAndID.
	 */
	vector<int> detectedMarkersIDs;

	/**
	 * Store the (TF) relative pose (wrt the robot reference frame)
	 * of the last detected markers.
	 * This is the vector processed by the method computeNextMarkerPoseAndID.
	 */
	vector<tf::Pose> detectedMarkersPoses;

	/**
	 * Store in a temporarily vector the identifiers of the last detected
	 * markers
	 */
	vector<int> tempDetectedMarkersIDs;

	/**
	 * Store in a temporarily vector the (TF) relative pose (wrt the robot reference frame)
	 * of the last detected markers
	 */
	vector<tf::Pose> tempDetectedMarkersPoses;

	/**
	 * The (TF) relative pose (wrt the robot reference frame)
	 * of the detected target marker
	 */
	tf::Pose goalPose;

	/**
	 * The ID of the detected target Marker
	 */
	int goalID;

	/**
	 * It stores the current marker path.
	 * This is the path processed by the method computeNextMarkerPoseAndID.
	 */
	vector<int> currentMarkerPath;

	/**
	 * Store in a temporarily vector a marker path
	 */
	vector<int> tempMarkerPath;

	/**
	 *T he position on the current marker target in the currentMarkerPath
	 */
	unsigned int currentMarkerIndex;

	/**
	 * The id of the current marker the robot has to reach on the path
	 */
	int currentMarkerTargetID;

	/**
	 * The id of the next marker the robot has to reach on the path
	 */
	int nextMarkerTargetID;

};

}

#endif
