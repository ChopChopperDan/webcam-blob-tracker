#ifdef SendMessage
#undef SendMessage
#endif

#include <RobotRaconteur.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>

#include "edu__rpi__cats__sensors__webcam.h"
#include "edu__rpi__cats__sensors__webcam_stubskel.h"

#include <Windows.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <cmath>

#include <boost/enable_shared_from_this.hpp>
#include <map>

#pragma once

class webcam_impl : public edu::rpi::cats::sensors::webcam::Webcam, public boost::enable_shared_from_this < webcam_impl >
{
public:
	webcam_impl();
	~webcam_impl();

	virtual RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::Image > getCurrentImage();
	virtual RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::ImageHeader > getImageHeader();

	virtual void StartStreaming();
	virtual void StopStreaming();

	virtual RR_SHARED_PTR<RobotRaconteur::Pipe<RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::Image > > > get_ImageStream();
	virtual void set_ImageStream(RR_SHARED_PTR<RobotRaconteur::Pipe<RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::Image > > > value);
private:
	cv::VideoCapture cam;
	bool _streaming, _converted;
	int image_width, image_height, image_channels;
	cv::Mat image_frame;

	RR_SHARED_PTR<RobotRaconteur::Pipe<RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::Image > > > image_pipe;

	void background_worker();

	boost::mutex mtx_;
	boost::thread t1;
};