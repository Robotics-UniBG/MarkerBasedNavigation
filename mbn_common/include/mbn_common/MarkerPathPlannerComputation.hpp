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
 * File: MarkerPathPlannerComputation.hpp
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

#ifndef MARKER_PATH_PLANNER_COMPUTATION_HPP
#define MARKER_PATH_PLANNER_COMPUTATION_HPP

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <vector>

#include <yaml-cpp/yaml.h>

using namespace std;

namespace mbn_common{

class MarkerPathPlannerComputation{

public:

	/**
	 * Creates a new marker path planner with empty paths.
	 */
	MarkerPathPlannerComputation();

	/**
	 * Class destroyer.
	 */
	~MarkerPathPlannerComputation(void);

	/**
	 * \param pathA1_A2 the marker path (IDs) between the position A1 and the position A2
	 * \param pathA2_A1 the marker path (IDs) between the position A2 and the position A1
	 *
	 * Create a new marker path planner with the given markers paths
	 */
	MarkerPathPlannerComputation(vector<int> pathA1_A2, vector<int> pathA2_A1);

	/**
	 * \param pathA1_A2_filePath the path of the file containing the marker
	 * IDs path between the position A1 and the position A2
	 * \param pathA2_A1_filePath the path of the file containing the marker
	 * IDs path between the position A2 and the position A1
	 *
	 * Create a new marker path planner and load the markers paths by using
	 * the given files paths
	 */
	MarkerPathPlannerComputation(string pathA1_A2_filePath, string pathA2_A1_filePath);

	/**
	 * \param visibleMarkers the vector of the visible markers.
	 *
	 * Store the visible markers which will be processed by the method
	 * computePath
	 */
	void setVisibleMarkersIDs(vector<int> visibleMarkersIDs);

	/**
	 * \param goalId the last marker of the computed path.
	 * \param markerIdPath the vector in which the computed path will be saved.
	 * \return true if one of the visible markers allows to reach the goal marker,
	 * false otherwise.
	 *
	 * Compute the shortest marker Path between one of the visible markers and goalId.
	 */
	bool computePath(int goalId, vector<int>& markerIdPath);

	/**
	 * \param pathA1_A2 the marker path (IDs) between the position A1 and the position A2
	 *
	 * Set the path between the position A1 and the position A2
	 */
	void setPathA1_A2(vector<int> pathA1_A2);

	/**
	 * \param pathA2_A1 the marker path (IDs) between position A2 and the position A1
	 *
	 * Load the path between the first and the position A2 and the position A1
	 */
	void setPathA2_A1(vector<int> pathA2_A1);

	/**
	 * \param filePath is the YAML file containing the path
	 *
	 * Load the marker path (IDs) between the position A1 and the position A2
	 */
	bool loadPathA1_A2(string filePath);

	/**
	 * \param filePath is the YAML file containing the path
	 *
	 * Load the marker path (IDs) between the position A2 and the position A1
	 */
	bool loadPathA2_A1(string filePath);

	/**
	 * Return the marker path (IDs) between the position A1 and the position A2
	 */
	vector<int> getPathA1_A2();

	/**
	 * Return the marker path (IDs) between the position A2 and the position A1
	 */
	vector<int> getPathA2_A1();

private:

	/**
	 * Store the identifiers of the makers placed between the position A1 and the position A2.
	 * The element at index 0 is the first.
	 */
	vector<int> pathA1_A2;

	/**
	 * Store the identifiers of the makers placed between the position A2 and the position A1.
	 * The element at index 0 is the first.
	 */
	vector<int> pathA2_A1;

	/**
	 * Store the set of visibile markers. The marker path planner try to compute the shortest
	 * path between one of these and the goal marker.
	 */
	vector<int> visibleMarkers;

	bool loadMarkerPathFromFile(vector<int>& markerPath, string filePath);
};

}

#endif
