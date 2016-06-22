//The MIT License(MIT)
//
//Copyright(c) 2016 Manu Lange
//
//Permission is hereby granted, free of charge, to any person obtaining a copy
//of this software and associated documentation files(the "Software"), to deal
//in the Software without restriction, including without limitation the rights
//to use, copy, modify, merge, publish, distribute, sublicense, and / or sell
//copies of the Software, and to permit persons to whom the Software is
//furnished to do so, subject to the following conditions :
//
//The above copyright notice and this permission notice shall be included in all
//copies or substantial portions of the Software.
//
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//SOFTWARE.

#pragma once
//System includes
#include <iostream>
#include <ostream>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <exception>
#include <signal.h>
#include <condition_variable>

//Duo Includes
#include <DUOInterface.h>

//ROS Includes
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/fill_image.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>

//Local Includes

//Define to control ROS Console output
#define ROSCONOUT

//Function definitions
void	yaml2ros(std::shared_ptr<duo::openCVYaml> input, sensor_msgs::CameraInfo	&output, bool useRight = false);
void	runImagePub(const image_transport::CameraPublisher &camPub, 
	std::shared_ptr<sensor_msgs::CameraInfo> camInfo,
	std::shared_ptr<sensor_msgs::Image> camImg);
void duoCallBack(const PDUOFrame pFrameData, void *pUserData);	//Callback must return ASAP

//Boost to STD smart pointers
template<class T>
	boost::shared_ptr<T> make_shared_ptr(const std::shared_ptr<T>& ptr)
	{
		return boost::shared_ptr<T>(ptr.get(), [ptr](T*) {});
	}

template<class T>
	std::shared_ptr<T> make_shared_ptr(const boost::shared_ptr<T>& ptr)
	{
		return std::shared_ptr<T>(ptr.get(), [ptr](T*) {});
	}