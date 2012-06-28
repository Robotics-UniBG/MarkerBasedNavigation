/********************************************************************************
 *
 * RGBCameraDriverNode
 *
 * Copyright (c) 2012
 * All rights reserved.
 *
 * Luca Gherardi and Andrea Luzzana
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

#include "mbn_rgb_camera_driver/RGBCameraDriverNode.hpp"



using namespace mbn_common;
using namespace cv_bridge;
using namespace cv;
using namespace sensor_msgs::image_encodings;



int main(int argc, char **argv) {

	//setup ros node and topics
	ros::init(argc, argv, "mbn_rgb_camera_driver");
	ros::NodeHandle nodeHandle;
	image_transport::ImageTransport it(nodeHandle);
	image_transport::Publisher pub = it.advertise("image_topic", 1);
	ros::Rate loop_rate(30);

	// Camera Object
	camera = new RGBCameraDriver();

	//Setup configuration and set data.
	loadConstants();
	camera->setAlwaysCopy(alwaysCopyFlag);
	camera->setCameraDevice(cameraDevice);
	camera->setHeight(height);
	camera->setWidth(width);
	camera->configureRGBCameraOpenCv();

	//Cycle
	while(ros::ok()){
		//Image
		cv_bridge::CvImage image;
		//Set encoding and timestamp
		image.encoding=sensor_msgs::image_encodings::BGR8;
		camera->getImage(image.image);
		image.header.stamp= ros::Time::now();

		//Convert image
		sensor_msgs::ImageConstPtr imagePtr = image.toImageMsg();
		//Publish image
		pub.publish(imagePtr);

		//Wait for next period
		ros::spinOnce();
		loop_rate.sleep();
	}

	delete camera;
	return 0;
}

bool loadConstants(){

	string pathFile = "configuration_rgb_camera_driver.yaml";
	std::ifstream pathDescritptorFile(pathFile.c_str());
	if (pathDescritptorFile.fail()) {
		ROS_ERROR("Configuration file cannot be opened %s", pathFile.c_str());
		return false;
	}
	YAML::Parser parser(pathDescritptorFile);
	YAML::Node doc;
	parser.GetNextDocument(doc);

	try {
		doc["alwaysCopyFlag"] >> alwaysCopyFlag;
		doc["cameraDevice"] >> cameraDevice;
		doc["height"] >> height;
		doc["width"] >> width;

	}catch (YAML::InvalidScalar&) {
		ROS_ERROR("Something went wrong during the parsing of configuration file");
		return false;
	}

	return true;

}
