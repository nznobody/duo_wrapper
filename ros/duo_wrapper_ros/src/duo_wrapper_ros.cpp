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

#include "../include/duo_wrapper_ros.h"

bool	g_quit = false;	//Every thread checks this for closing time
std::condition_variable g_cv;

int main(int argc, char *argv[]) {
	// Setup Section ============================================================
	//ROS Naming Strings
	const std::string node_namespace = "duo";
	const std::string frame_id = "duo_frame";
	std::string left_topic = node_namespace + "/left";
	std::string right_topic = node_namespace + "/right";
	std::string depth_topic = node_namespace + "/depth";
	//ROS Setting Varibales - Duo Camera Settings
	double	_gain = 0.0f;
	double	_exposure = 80.0f;
	double	_leds = 20.0f;
	//ROS Setting Varibales - DuoInterface Library Settings
	bool	_rectWithOpencv = true;
	bool	_useDuoCalib = true;	//This should default to true, testing atm
	bool	_useCuda = true;
	
	// Init Section =============================================================
	try
	{
		//Allocate Duo Singleton and initialise
		std::shared_ptr<duo::DUOInterface>	_duo = duo::DUOInterface::GetInstance();
		if (_duo->initializeDUO())
		{
			//duo::DUOInterface::_extcallback = duoCallBack;
			_duo->SetGain(_gain);
			_duo->SetExposure(_exposure);
			_duo->SetLedPWM(_leds);
			_duo->SetRectifyOpencv(_rectWithOpencv);
			_duo->SetUseDuoCalib(_useDuoCalib);
			_duo->SetUseCUDA(_useCuda);
		}
		else
			throw std::runtime_error("Couldn't initialise Duo Camera\n");
		
		ros::init(argc, argv, "duo_wrapper_ros");	//Init before first NodeHandle creation
	
		ros::NodeHandle nh;	//Base NodeHandle
		ros::NodeHandle nh_ns("~");	//Local Namespace NodeHandle
		// Get parameters from launch file
		nh_ns.getParam("gain", _gain);
		nh_ns.getParam("exposure", _exposure);
		nh_ns.getParam("leds", _leds);
		nh_ns.getParam("rectWithOpencv", _rectWithOpencv);
		nh_ns.getParam("useDuoCalib", _useDuoCalib);
		nh_ns.getParam("useCuda", _useCuda);
		
		//Setup CamInfos + Publishers
		image_transport::ImageTransport it(nh);
		image_transport::CameraPublisher camPubL = it.advertiseCamera(left_topic, 1);
		image_transport::CameraPublisher camPubR = it.advertiseCamera(right_topic, 1);
		image_transport::CameraPublisher camPubD = it.advertiseCamera(depth_topic, 1);
		std::shared_ptr<camera_info_manager::CameraInfoManager> camInfoManL = std::make_shared<camera_info_manager::CameraInfoManager>(nh);
		std::shared_ptr<camera_info_manager::CameraInfoManager> camInfoManR = std::make_shared<camera_info_manager::CameraInfoManager>(nh);
		std::shared_ptr<camera_info_manager::CameraInfoManager> camInfoManD = std::make_shared<camera_info_manager::CameraInfoManager>(nh);
		std::shared_ptr<sensor_msgs::CameraInfo> camInfoL = std::make_shared<sensor_msgs::CameraInfo>();
		std::shared_ptr<sensor_msgs::CameraInfo> camInfoR = std::make_shared<sensor_msgs::CameraInfo>();
		std::shared_ptr<sensor_msgs::CameraInfo> camInfoD = std::make_shared<sensor_msgs::CameraInfo>();
		std::shared_ptr<sensor_msgs::Image>	camImgL = std::make_shared<sensor_msgs::Image>();
		std::shared_ptr<sensor_msgs::Image>	camImgR = std::make_shared<sensor_msgs::Image>();
		std::shared_ptr<sensor_msgs::Image>	camImgD = std::make_shared<sensor_msgs::Image>();
		//Get CamInfos from DuoInterface. For now Depth info is Left info
		yaml2ros(_duo->GetCurrentCalib(), camInfoL, false);
		yaml2ros(_duo->GetCurrentCalib(), camInfoR, true);
		yaml2ros(_duo->GetCurrentCalib(), camInfoD, false);	//Depth uses left camera
		//Set Frame IDs for camInfo
		camInfoL->header.frame_id = frame_id;
		camInfoR->header.frame_id = frame_id;
		camInfoD->header.frame_id = frame_id;
		//Set camera names
		camInfoManL->setCameraInfo(*camInfoL.get());	//This is a potential bug. Have to pass underlying pointer. Won't increment counter of ptr
		camInfoManR->setCameraInfo(*camInfoR.get());	//This is a potential bug. Have to pass underlying pointer. Won't increment counter of ptr
		camInfoManD->setCameraInfo(*camInfoD.get());	//This is a potential bug. Have to pass underlying pointer. Won't increment counter of ptr
		camInfoManL->setCameraName("duo_left");	//Hardcoded for now. Will have to get this from calib files. But DuoInterface hard codes this for duo calib anyway
		camInfoManR->setCameraName("duo_right");
		camInfoManD->setCameraName("duo_depth");
		//Setup the image details
		camImgL->header.frame_id = frame_id;	//Potentially move up to join others
		camImgR->header.frame_id = frame_id;
		camImgD->header.frame_id = frame_id;
		
		
#ifdef ROSCONOUT
		ROS_INFO_STREAM("Advertized on topic " << left_topic);
		ROS_INFO_STREAM("Advertized on topic " << right_topic);
		ROS_INFO_STREAM("Advertized on topic " << depth_topic);
#endif // ROSCONOUT

#ifdef ROSCONOUT
		ROS_INFO("duo_wrapper_ros Node initialized");
#endif // ROSCONOUT
	}
	catch (const std::exception& ex)
	{
#ifdef ROSCONOUT
		ROS_ERROR("duo_wrapper_ros initialisation failed, quitting");
#endif // ROSCONOUT
		std::cerr << ex.what() << "\nFatal error" << std::endl;
		return -1;
	}

	
	return 0;
}

//This helper function converts a calibration received from the DuoInterface to ros sensor_msgs::CameraInfo. 
//The bool defines if it should use input's right info (defaults false, use left)
void	yaml2ros(const std::shared_ptr<duo::openCVYaml>	input, std::shared_ptr<sensor_msgs::CameraInfo>	output, bool useRight)
{
	output->height = input->resolution.height;
	output->width = input->resolution.width;
	output->distortion_model = input->distortion_model;
	//Use std::copy, be carefull with memory space! Should bound check! Also using a bool as array access is bad!
	std::copy(input->camera_matrix[useRight].datastart, input->camera_matrix[useRight].dataend, output->K.begin());
	std::copy(input->distortion_coefficients[useRight].datastart, input->distortion_coefficients[useRight].dataend, output->D.begin());
	std::copy(input->rectification_matrix[useRight].datastart, input->rectification_matrix[useRight].dataend, output->R.begin());
	std::copy(input->projection_matrix[useRight].datastart, input->projection_matrix[useRight].dataend, output->P.begin());
	return;
}

void	runImagePub(const image_transport::CameraPublisher &camPub, 
	std::shared_ptr<camera_info_manager::CameraInfoManager> camInfo,
	std::shared_ptr<sensor_msgs::Image> camImg,
	std::mutex &m,
	std::condition_variable &cVar,
	std::atomic_bool &newFrame)
{
	while (!g_quit)
	{
		//Implements thread synchronisation via condition variables and notifies. lock only received after wait
		std::unique_lock<std::mutex> lk(m);
		cVar.wait(lk, [&newFrame]{return newFrame.load(std::memory_order_relaxed);}); 
		//Check for subscribers...
		if (camPub.getNumSubscribers() == 0)
		{
			newFrame.store(false, std::memory_order_relaxed);
			continue;
		}
		sensor_msgs::CameraInfoPtr	ci(new sensor_msgs::CameraInfo(camInfo->getCameraInfo()));
		//Update ImgHeader timestamp, copy to camInfo
		camImg->header.stamp = ci->header.stamp = ros::Time::now();
		//Publish	
		camPub.publish(make_shared_ptr(camImg), ci);
	}
}