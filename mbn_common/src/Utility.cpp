/********************************************************************************
 *
 * Utility
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
 * File: MarkerLocator.cpp
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

#include "mbn_common/Utility.hpp"
#include <math.h>



namespace Utilities{

string Utility::getPositionAndOrientation(tf::Pose pose){
	stringstream result;
	btScalar roll, pitch, yaw;
	pose.getBasis().getRPY(roll, pitch, yaw,1);

	result  << "X:" << pose.getOrigin().getX()
			<<" Y:" << pose.getOrigin().getY()
			<<" Z:" << pose.getOrigin().getZ() << "\n"
			<<" Roll: "<<roll*180.0/M_PI
			<<" Pitch: "<<pitch*180.0/M_PI
			<<" Yaw: "<<yaw*180/M_PI<<"\n";

	return result.str();
}

void Utility::getPoseFromARTKMatrix(float (&matrix)[3][4], tf::Pose &markerTfPose){
	btMatrix3x3 basis=btMatrix3x3(matrix[0][0],matrix[0][1],matrix[0][2],
			matrix[1][0],matrix[1][1],matrix[1][2],
			matrix[2][0],matrix[2][1],matrix[2][2]);
	markerTfPose.setBasis(basis);

	double x = markerTfPose.getRotation().getX();
	double y = markerTfPose.getRotation().getY();
	double z = markerTfPose.getRotation().getZ();
	double w = markerTfPose.getRotation().getW();

	double norm = sqrt( x*x + y*y + z*z + w*w);

	markerTfPose.getRotation().setX(x/norm);
	markerTfPose.getRotation().setY(y/norm);
	markerTfPose.getRotation().setZ(z/norm);
	markerTfPose.getRotation().setW(w/norm);

	tf::Vector3 markerTranslation;
	markerTranslation.setX(matrix[0][3]);
	markerTranslation.setY(matrix[1][3]);
	markerTranslation.setZ(matrix[2][3]);
	markerTfPose.setOrigin(markerTranslation);
}
}
