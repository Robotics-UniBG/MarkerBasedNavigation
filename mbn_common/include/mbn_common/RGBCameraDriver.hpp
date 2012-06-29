/********************************************************************************
 *
 * RGBCameraDriver
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
 * File: RGBCameraDriver.hpp
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

#ifndef RGBCAMERADRIVER_HPP_
#define RGBCAMERADRIVER_HPP_


#include <opencv2/opencv.hpp>
#include <map>
#include <vector>

using namespace cv;

namespace mbn_common{

class RGBCameraDriver {

public:
	/**
	 * Class constructor.
	 */
	RGBCameraDriver();

	/**
	 * Class destroyer.
	 */
	~RGBCameraDriver();

	/**
	 * Configures the camera driver according to the parameters previously set.
	 * If no parameters has been set, the driver will be configured with the default parameters:
	 * Camera Device = 0, Always Copy = false, Height = 480, Width = 640.
	 */
	bool configureRGBCameraOpenCv();

	/**
	 * Saves the grabbed image into the cv::Mat structure passed as parameter.
	 * If alwaysCopy flag is true the image data are copied, if the flag is false only the reference is set
	 *
	 * \param[out] image the image.
	 */
	void getImage(cv::Mat &image);

	/**
	 * Provides the currently used camera device
	 */
	int getCameraDevice();

	/**
	 * Provides the current vetical resolution of the camera
	 */
	int getHeight();

	/**
	 *
	 */
	int getWidth();

	/**
	 * Provides the current horizontal resolution of the camera
	 */
	bool isAlwaysCopy();

	/**
	 * Provides the current status of the alwaysCopy flag
	 */
	void setAlwaysCopy(bool alwaysCopy);

	/**
	 * Sets the camera device number. E.g. set 0 for /video0
	 * \param[in] cameraDevice the camera device.
	 */
	void setCameraDevice(int cameraDevice);

	/**
	 * Sets the vertical resolution of the camera.
	 * \param[in] height the vertical resolution.
	 */
	void setHeight(int height);

	/**
	 * Sets the horizontal resolution of the camera.
	 * \param[in] width the horizontal resolution of the camera.
	 */
	void setWidth(int width);

private:
	/**
	 * if true, a copy of image data will be performed for each new grabbed image
	 */
	bool alwaysCopy;
	/**
	 * kind of image encoding
	 */
	string encoding;
	/**
	 * width camera resolution
	 */
	int width;
	/**
	 * height camera resolution
	 */
	int height;
	/**
	 * camera device number id (i.e. if you want use /dev/video1 you have to set it to 1)
	 */
	int cameraDevice;
	/**
	 * this object represent camera and it useful to handle it
	 */
	CvCapture* capture;
};

}



#endif /* RGBCAMERADRIVER_HPP_ */
