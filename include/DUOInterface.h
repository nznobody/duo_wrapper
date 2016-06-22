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
// Standard Includes
#include <string>
#include <stddef.h>
#include <memory>		//for std::shared_ptr
#include <functional>	//for std::function
#include <thread>		//Locking
#include <mutex>		//Locking
#include <ctime>		//Measuring execution time
//Include Duo Headers
#include "../include/duosdk/DUOLib.h"
#ifdef DUOMT	//Use DUO multithreading os singlethreading
#include "../include/duosdk/Dense3DMT.h"
#else
#include "../include/duosdk/Dense3D.h"
#endif // DUOMT

//Include openCV if found / chosen
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#ifdef WITH_GPU
#include <opencv2/gpu/gpu.hpp>
#endif // WITH_GPU
#include <opencv2/calib3d/calib3d.hpp>

//Include visionworks if found / chosen
#ifdef WITH_NVX
#include <NVX/nvx.h>
#include <NVX/nvx_opencv_interop.hpp>
#endif // WITH_NVX



//Hardcoded defaults
#define GAIN		0.0f
#define EXPOSURE	100.0f
#define LEDS		50.0f
#define FPS			30.0f
#define WIDTH		752
#define HEIGHT		480
#define	BINNING		DUO_BIN_NONE


namespace duo
{
	class openCVYaml {
	public:
		openCVYaml(void) {
			distortion_model = "plumb_bob";
			camera_matrix[0] = cv::Mat::zeros(3, 3, CV_64FC1);
			camera_matrix[1] = cv::Mat::zeros(3, 3, CV_64FC1);
			distortion_coefficients[0] = cv::Mat::zeros(1, 5, CV_64FC1);
			distortion_coefficients[1] = cv::Mat::zeros(1, 5, CV_64FC1);
			rectification_matrix[0] = cv::Mat::zeros(3, 3, CV_64FC1);
			rectification_matrix[1] = cv::Mat::zeros(3, 3, CV_64FC1);
			projection_matrix[0] = cv::Mat::zeros(3, 4, CV_64FC1);
			projection_matrix[1] = cv::Mat::zeros(3, 4, CV_64FC1);
		}
		~openCVYaml(void) {}
		std::string	camera_name[2];
		cv::Size resolution;
		std::string distortion_model;
		cv::Mat camera_matrix[2];			//K 3x3
		cv::Mat distortion_coefficients[2];	//D 1x5
		cv::Mat rectification_matrix[2];	//R 3x3
		cv::Mat projection_matrix[2];		//P 3x4
	};
	
	class DUOInterface
	{
	public:
		DUOInterface(void);
		~DUOInterface(void);
		DUOInterface(const DUOInterface&) = delete;
		DUOInterface& operator=(const DUOInterface&) = delete;
		DUOInterface(DUOInterface&&) = delete;
		DUOInterface& operator=(DUOInterface&&) = delete;
		
		//Static defines
		static const int TWO_CAMERAS	= 2;
		static const int LEFT_CAM 		= 0;
		static const int RIGHT_CAM		= 1;
		static const std::string CAM_NAME;
		
		friend void CALLBACK DUOCallback(const PDUOFrame pFrameData, void *pUserData);
		
		//Impliments singleton instance
		static std::shared_ptr<DUOInterface>	GetInstance(void){
			static  std::shared_ptr<DUOInterface> pSingleton = std::make_shared<DUOInterface>();
			return pSingleton;
		}

		bool initializeDUO(void);
		void startDUO(void);
		void shutdownDUO(void);
		
		//Duo parameter functions
		void SetExposure(double val){_exposure = val; SetDUOExposure(_duoInstance, val); }
		void SetGain(double val){_gain = val; SetDUOGain(_duoInstance, val); }
		void SetHFlip(bool val){_flipH = val; SetDUOHFlip(_duoInstance, val); }
		void SetVFlip(bool val){_flipV = val; SetDUOVFlip(_duoInstance, val); }
		void SetCameraSwap(bool val){_swap = val; SetDUOCameraSwap(_duoInstance, val); }
		void SetLedPWM(double val){_leds = val; SetDUOLedPWM(_duoInstance, val); }
		bool ReadYAML(std::string left, std::string right);
		bool ReadYAMLFromDuo();
		void WriteCALIB(const openCVYaml& input, std::string prefix);
		bool ReadINI(std::string settings);
		bool WriteINI(std::string settings);
		void EnableCVSettings();
		
		//This is a callback function that will get passed the PDUOFrame when it is ready
		static std::function<void(const PDUOFrame pFrameData, void *pUserData)>	_extcallback;
		
		//Encapsulations
		bool GetOpencvCalib() const { return _opencvCalib; }
		bool GetRectifyOpencv() const { return _rectifyOpencv; }
		void SetRectifyOpencv(bool val) { _rectifyOpencv = val; }
		void SetUseDuoCalib(bool val) { _useDuoCalib = val; }	//Note this has interesting ouput at the moment
		bool GetUseCUDA() const { return _useCUDA; }
		void SetUseCUDA(bool val) { _useCUDA = val; }
		std::shared_ptr<openCVYaml> GetCurrentCalib();	//Safe interface to get the calib settings being used. mainly to populate ROS CamInfo MSG

		//Public variables
		//Camera characteristics storage (modeled off http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html)
		
	protected:

		DUOInstance 		_duoInstance = NULL;	//Fixed an issue with heap vs stack allocaation
		DUOResolutionInfo 	_duoResolutionInfo;
		//Duo parameters
		double	_gain		= GAIN;
		double	_exposure	= EXPOSURE;
		double	_leds		= LEDS;
		bool	_flipH		= false;
		bool	_flipV		= false;
		bool	_swap		= false;
		size_t	_width		= WIDTH;
		size_t	_height		= HEIGHT;
		size_t	_fps		= FPS;
		int		_binning	= BINNING;
		std::string	_duoDeviceName;
		std::string	_duoDeviceSerialNumber;
		std::string	_duoDeviceFirmwareVersion;
		std::string	_duoDeviceFirmwareBuild;
		//const DUOLEDSeq _ledSequence[2]; 
		
		//Library settings
		bool	_opencvCalib	= false;	//This indicates if openCV Calib files were found and successfully loaded
		bool	_duoCalib		= false;	//This indicates if Duo Calib was extracted from the camera
		bool	_opencvSettings	= false;	//This indicated that settings were loaded from disk for the class (gain, exp, leds...)
		bool	_rectifyOpencv	= false;	//This indicates if we should use openCV to rectify images, or inbuilt rectification.
		bool	_useDuoCalib	= false;	//Must be set before startDuo(), decides which calib setting to use for rectification.
		bool	_useCUDA		= false;	//This indicates if we should use gpu (cuda) to rectify the images
		bool	_calcDense3D	= false;	//Not implemented yet
		
		//General variables
		std::mutex		_mutex;			//Threading lock used within this class
		bool			_duoInitialised = false;	//Can be checked to see if init() has been called
		bool			_rectInitialised = false;	//Can be checked to see if rectification maps have been created.
		static const	std::string CameraNames[TWO_CAMERAS]; // = {"left","right"};
		
		//OpenCV specifics
		void initRect(const std::shared_ptr<openCVYaml> input);
		bool rectifyCV(const PDUOFrame pFrameData, void *pUserData, cv::Mat &leftR, cv::Mat &rightR);
		static void on_trackbar(int, void*);
		void		calib_cv2duo(const std::shared_ptr<openCVYaml> input, DUO_STEREO& output);
		void		calib_duo2cv(const DUO_STEREO& input, std::shared_ptr<openCVYaml> output);
		std::shared_ptr<openCVYaml>	_cameraCalibCV;
		std::shared_ptr<openCVYaml>	_cameraCalibDuo;
		cv::Mat _mapL[2], _mapR[2];	//stores the rectification maps
#ifdef WITH_GPU
		cv::gpu::GpuMat _g_mapL[2], _g_mapR[2];	//stores the rectification maps
#endif // WITH_GPU
		
		//Trackbar variables
		//Trackbars for Dense3D Settings
		int _t_gain = GAIN;
		int _t_exposure = EXPOSURE;
		int _t_leds = LEDS;
		int _t_scale = 0;
		int _t_mode = 0;
		int _t_pre_filter = 1;
		int _t_num_disps = 2;
		int _t_SAD_window = 2;
		int _t_unique_r = 1;
		int _t_speckle_w = 0;
		int _t_speckle_r = 0;

		const int _t_gain_max = 100;
		const int _t_exposure_max = 100;
		const int _t_leds_max = 100;
		const int _t_scale_max = 3;
		const int _t_mode_max = 3;
		const int _t_pre_filter_max = 63;
		const int _t_num_disps_max = 16;
		const int _t_SAD_window_max = 10;
		const int _t_unique_r_max = 100;
		const int _t_speckle_w_max = 256;
		const int _t_speckle_r_max = 32;
	};
}



