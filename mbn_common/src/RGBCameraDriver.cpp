#include "mbn_common/RGBCameraDriver.hpp"

#include <math.h>

using namespace std;

namespace mbn_common{

RGBCameraDriver::RGBCameraDriver(){
	this->alwaysCopy = false;
	this->cameraDevice = 0;
	this->height = 480;
	this->width = 640;
}

RGBCameraDriver::~RGBCameraDriver(){
	cvReleaseCapture( &this->capture );
}

void RGBCameraDriver::getImage(cv::Mat &image){

	IplImage *frame = cvQueryFrame( this->capture );

	if( !frame ){
		cout<<"RGBCameraOpenCv - frame is null..."<<endl;
		return;
	}

	image=Mat(frame,this->alwaysCopy);
}

bool RGBCameraDriver::configureRGBCameraOpenCv(){

	if(this->cameraDevice<0){
		this->cameraDevice = CV_CAP_ANY;
	}

	this->capture = cvCaptureFromCAM(this->cameraDevice);

	if( !this->capture ){
		cout<<"RGBCameraOpenCv - capture is NULL"<<endl;
		return false;
	}

	cvSetCaptureProperty(this->capture, CV_CAP_PROP_FRAME_WIDTH, this->width);
	cvSetCaptureProperty(this->capture, CV_CAP_PROP_FRAME_HEIGHT, this->height);

	IplImage *frame = cvQueryFrame( this->capture );

	if( !frame ){
		cout<<"RGBCameraOpenCv - frame is null..."<<endl;
		return false;
	}
	return true;
}

int RGBCameraDriver::getCameraDevice(){
	return this->cameraDevice;
}

int RGBCameraDriver::getHeight(){
	return this->height;
}

int RGBCameraDriver::getWidth(){
	return this->width;
}

bool RGBCameraDriver::isAlwaysCopy(){
	return this->alwaysCopy;
}

void RGBCameraDriver::setAlwaysCopy(bool alwaysCopy){
	this->alwaysCopy = alwaysCopy;
}

void RGBCameraDriver::setCameraDevice(int cameraDevice){
	this->cameraDevice = cameraDevice;
}

void RGBCameraDriver::setHeight(int height){
	this->height = height;
}

void RGBCameraDriver::setWidth(int width){
	this->width = width;
}


}
