/********************************************************************************
 *
 * MarkerSearcherComputation
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
 * File: MarkerSearcherComputation.hpp
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

#ifndef MARKER_SEARCHER_COMPUTATION_HPP
#define MARKER_SEARCHER_COMPUTATION_HPP

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <algorithm>
#include <vector>

#include <tf/tf.h>
#include <angles/angles.h>

using namespace std;

namespace mbn_common{

class MarkerSearcherComputation{

public:

	/**
	 * Creates a new MarkerSearcherComputation
	 */
	MarkerSearcherComputation();

	/**
	 * Class destroyer.
	 */
	~MarkerSearcherComputation(void);


	/**
	 * \param targetMarkerId the new target marker ID
	 *
	 * Store a temporarily target marker ID, which can be committed later by calling
	 * the method updateTargetMarkerID
	 */
	void setTargetMarkerID(int targetMarkerID);

	/**
	 * Commit the target marker ID stored with the method setTargetMarkerID.
	 * Only when this method has been called the marker search can be executed.
	 */
	void updateTargetMarkerID();

	/**
	 * \param lastOdometryPose the new value of the robot odometry (TF) pose
	 *
	 * Store the robot odometry (TF) pose
	 */
	void setOdometryPose(tf::Pose lastOdometryPose);

	/**
	 * \param visibleMarkers the new set of visible markers
	 *
	 * Store a vector of visible markers, which can be processed later by calling
	 * the method searchTargetIDInVisibleMarkersIDs
	 */
	void setVisibleMarkersIDs(vector<int> visibleMarkersIDs);

	/**
	 * \return true if the target marker is in the set of visible markers,
	 * false otherwise
	 *
	 * Search the target marker in the set of visible markers IDs
	 * stored with the method setVisibleMarkersID.
	 */
	bool searchTargetIDInVisibleMarkersIDs();

	/**
	 * \param returnPose the (TF) pose in which will be saved the next research
	 * pose
	 * \return true if the (TF) pose has not yet been returned, false otherwise.
	 *
	 * Return the last computed research pose as output parameter and specify
	 * if it has been already returned or not.
	 **/
	bool getNextSearchPose(tf::Pose& returnPose);

	/**
	 * Compute a new (TF) pose for searching the target marker, which can be retrieved
	 * by calling the method getNextSearchPose
	 */
	void computeNextSearchPose();

	/**
	 * \return the angle increment
	 *
	 * Return the value of the variable angleIncrement
	 */
	double getAngleIncrement();

	/**
	 * \param angleIncrement the new value of the angle increment
	 *
	 * Set the value of the variable angleIncrement
	 */
	void setAngleIncrement(double angleIncrement);

private:

	/**
	 * Store the Identifier of the target marker.
	 * This is the vector processed by the method computeNextMarkerPoseAndID.
	 */
	int targetMarkerID;

	/**
	 * Store in a temporarily integer the identifier of the target marker.
	 */
	int tempTargetMarkerID;

	/**
	 * lastOdometryPose the last (TF) pose of the robot
	 * \return the (TF) pose in which the goal absolute pose
	 * is saved and returned.
	 */
	tf::Pose lastOdometryPose;

	/**
	 * The next research pose that will be returned by the method getNextSearchPose.
	 * It is computed according to the inputs set with the "set methods".
	 */
	tf::Pose nextSearchPose;

	/**
	 * Store the set of visible markers.
	 * This is the vector processed by the method searchTargetIDInVisibleMarkersIDs.
	 */
	vector<int> visibleMarkersIDs;

	/**
	 * The orientation increment that has to be sent to the robot when the method
	 * computeNextSearchPose is called
	 */
	double angleIncrement;

	/**
	 * This variable is true when the last computed pose has been returned
	 */
	bool poseReturned;

	/**
	 * True if the target marker has been found
	 */
	bool markerFound;

};

}

#endif
