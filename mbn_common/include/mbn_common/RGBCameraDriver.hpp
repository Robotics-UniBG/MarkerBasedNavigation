/*
 * RGBCameraDriver.hpp
 *
 *  Created on: Jun 19, 2012
 *      Author: aldo
 */

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
