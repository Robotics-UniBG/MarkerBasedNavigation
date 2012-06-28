/********************************************************************************
 *
 * MarkerSearcherCoordinationio
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
 * File: MarkerSearcherCoordination.hpp
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

#ifndef MARKER_SEARCHER_COORDINATION_HPP
#define MARKER_SEARCHER_COORDINATION_HPP

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <algorithm>
#include <vector>

#include <tf/tf.h>
#include <angles/angles.h>

#include "mbn_common/MarkerSearcherComputation.hpp"
#include "mbn_common/Events.hpp"

using namespace std;

namespace mbn_common{

class MarkerSearcherCoordination{

private:

	/**
	 * The possible states of the state machine
	 */
	typedef enum {
		IDLE = 0, 				/** The marker searcher is idle */
		TARGET_RECEIVED = 1,	/** The marker searcher has received the target marker and is waiting for new inputs */
		GOAL_GENERATED = 2,     /** The marker searcher has produced a search pose and is waiting for the movement start */
		MOVING = 3				/** The marker searcher has produced a search pose and the movement has started */
	} state;

public:

	/**
	 * \param markerSearcherComputer a pointer to the computation instance
	 *
	 * Creates a new MarkerSearcherCoordinator.
	 */
	MarkerSearcherCoordination(MarkerSearcherComputation* markerSearcherComputer);

	/**
	 * Class destroyer.
	 */
	~MarkerSearcherCoordination(void);

	/**
	 * This method trigger the state machine and has to be called
	 * when a new target marker is provided
	 */
	void notifyTargetMarkerIDReceived();

	/**
	 * This method trigger the state machine and has to be called
	 * when new visible markers IDs are provided
	 */
	void notifyVisibleMarkersIDsReceived();

	/**
	 * This method trigger the state machine and has to be called
	 * periodically. It looks whether the target marker has been found
	 * or not and if needed (marker not found) ask to markerSearcherComputation
	 * to generate a new search pose.
	 */
	void notifyTimeElapsed();

	/**
	 * This method trigger the state machine and has to be called
	 * when the robot starts to move towards the last computed search
	 * pose
	 */
	void notifyMotionStarted();

	/**
	 * This method trigger the state machine and has to be called
	 * when the robot has reached the last computed search pose
	 */
	void notifySearchPoseReached();

	/**
	 * This method return an output event which is equal to
	 * "TARGET_MARKER_FOUND" when the target marker has been found.
	 */
	string getOutputEvent();

private:

	/**
	 * True if the target marker has been found
	 */
	bool markerFound;
	/**
	 * True if the last generated search pose has been reached
	 */
	bool motionDone;

	/**
	 * The current state of the state machine
	 */
	state currentState;

	/**
	 * The output event that will be returned by the method getOutputEvent.
	 */
	string outputEvent;

	/**
	 * The Marker searcher computer, which is in charge of computing the next search
	 * pose based on the inputs
	 */
	MarkerSearcherComputation* markerSearcherComputation;

};

}

#endif
