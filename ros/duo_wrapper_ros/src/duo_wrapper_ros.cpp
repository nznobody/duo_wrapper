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
std::mutex				g_mutex;
std::atomic_bool		g_newFrame;
std::shared_ptr<sensor_msgs::Image>	g_camImgL, g_camImgR, g_camImgD;
//Moving pubs to global. Eventually eveything should be encapsulated in a class
std::shared_ptr<ros::Publisher> g_imuPub;

//ROS Quit handler
void sigIntHandler(int sig) {
	ROS_DEBUG("--> SIGINT Handler called <--");
	ros::shutdown();
	g_quit = true;
}

int main(int argc, char *argv[]) {
	// Setup Section ============================================================
	//ROS Naming Strings
	const std::string node_namespace = "";	//empty due to using namespace in launch file
	const std::string frame_id = "duo_frame";
	std::string left_topic = node_namespace + "/left/image_raw";
	std::string right_topic = node_namespace + "/right/image_raw";
	std::string depth_topic = node_namespace + "/depth/image_raw";
	std::string left_info_topic = node_namespace + "/left/camera_info";
	std::string right_info_topic = node_namespace + "/right/camera_info";
	std::string depth_info_topic = node_namespace + "/depth/camera_info";
	//ROS Setting Varibales - Duo Camera Settings
	double	_gain = 0.0f;
	double	_exposure = 80.0f;
	double	_leds = 20.0f;
	//ROS Setting Varibales - DuoInterface Library Settings
	bool	_rectWithOpencv = true;
	bool	_useDuoCalib = true;	//This should default to true, testing atm
	bool	_useCuda = true;
	//Thread pool
	std::vector<std::thread>	tPool;
	
	// Init Section =============================================================
	try
	{
		//Register SIGNT Handler
		signal(SIGINT, sigIntHandler);
		
		//Allocate Duo Singleton and initialise
		std::shared_ptr<duo::DUOInterface>	_duo = duo::DUOInterface::GetInstance();
		if (_duo->initializeDUO())
		{
			duo::DUOInterface::_extcallback = duoCallBack;
			_duo->SetGain(_gain);
			_duo->SetExposure(_exposure);
			_duo->SetLedPWM(_leds);
			_duo->SetRectifyOpencv(_rectWithOpencv);
			_duo->SetUseDuoCalib(_useDuoCalib);
			_duo->SetUseCUDA(_useCuda);
		}
		else
			throw std::runtime_error("Couldn't initialise Duo Camera\n");
		
		ros::init(argc, argv, "duo_wrapper_ros", ros::init_options::NoSigintHandler);	//Init before first NodeHandle creation
	
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
		g_imuPub = std::make_shared<ros::Publisher>(nh.advertise<sensor_msgs::Imu>("imu", 10));	//Setting up basic IMU and TEMP Pubs
		//std::shared_ptr<ros::Publisher> tempPub = std::make_shared<ros::Publisher>(nh.advertise<sensor_msgs::Imu>("imu", 10));
		g_camImgL = std::make_shared<sensor_msgs::Image>();	//these have to be global so the callback can access them
		g_camImgR = std::make_shared<sensor_msgs::Image>();
		g_camImgD = std::make_shared<sensor_msgs::Image>();
		std::shared_ptr<sensor_msgs::CameraInfo> camInfoL = std::make_shared<sensor_msgs::CameraInfo>();
		std::shared_ptr<sensor_msgs::CameraInfo> camInfoR = std::make_shared<sensor_msgs::CameraInfo>();
		std::shared_ptr<sensor_msgs::CameraInfo> camInfoD = std::make_shared<sensor_msgs::CameraInfo>();
		yaml2ros(_duo->GetCurrentCalib(), *camInfoL, false);
		yaml2ros(_duo->GetCurrentCalib(), *camInfoR, true);
		yaml2ros(_duo->GetCurrentCalib(), *camInfoD, false);	//Depth uses left camera
		//Set Frame IDs for camInfo
		camInfoL->header.frame_id = frame_id;
		camInfoR->header.frame_id = frame_id;
		camInfoD->header.frame_id = frame_id;
		//Setup the image details
		g_camImgL->header.frame_id = frame_id;	//Potentially move up to join others
		g_camImgR->header.frame_id = frame_id;
		g_camImgD->header.frame_id = frame_id;
#ifdef ROSCONOUT
		ROS_INFO_STREAM("Advertized on topic " << left_topic);
		ROS_INFO_STREAM("Advertized on topic " << right_topic);
		ROS_INFO_STREAM("Advertized on topic " << depth_topic);
#endif // ROSCONOUT
		
		//Initialise threads
		tPool.push_back(std::thread(runImagePub, camPubL, camInfoL, g_camImgL));
		tPool.push_back(std::thread(runImagePub, camPubR, camInfoR, g_camImgR));
		tPool.push_back(std::thread(runImagePub, camPubD, camInfoD, g_camImgD));
		
		//Start the Duo
		if(!_duo->startDUO())
			throw std::runtime_error("Couldn't start Duo Camera\n");

#ifdef ROSCONOUT
		ROS_INFO("duo_wrapper_ros Node initialized");
#endif // ROSCONOUT

		while (!g_quit && ros::ok())
		{
			ros::spin();
		}
	}
	catch (...)
	{
		
	}
	//Join threads
	g_quit = true;
//	for (auto &var : tPool)
//	{
//		var.join();
//	}
#ifdef ROSCONOUT
	ROS_INFO("duo_wrapper_ros gracefully closed");
#endif // ROSCONOUT
	
	return 0;
}

//This helper function converts a calibration received from the DuoInterface to ros sensor_msgs::CameraInfo. 
//The bool defines if it should use input's right info (defaults false, use left)
void	yaml2ros(std::shared_ptr<duo::openCVYaml> input, sensor_msgs::CameraInfo	&output, bool useRight)
{
	output.height = input->resolution.height;
	output.width = input->resolution.width;
	output.distortion_model = input->distortion_model;
	output.D.resize(5); //Allocate sizes
	for (size_t i = 0; i < 12; i++)	//Manually copy, std::copy errors
	{
		if (i < 5)
			output.D[i] = input->distortion_coefficients[useRight].at<double>(i);
		if (i < 9)
			output.K[i] = input->camera_matrix[useRight].at<double>(i);
		if (i < 9)
			output.R[i] = input->rectification_matrix[useRight].at<double>(i);
		if (i < 12)
			output.P[i] = input->projection_matrix[useRight].at<double>(i);
	}
	//Use std::copy, be carefull with memory space! Should bound check! Also using a bool as array access is bad!
	//std::copy(input.camera_matrix[useRight].datastart, input.camera_matrix[useRight].dataend, output.K.begin());
	//std::copy(input.distortion_coefficients[useRight].datastart, input.distortion_coefficients[useRight].dataend, output.D.begin());
	//std::copy(input.rectification_matrix[useRight].datastart, input.rectification_matrix[useRight].dataend, output.R.begin());
	//std::copy(input.projection_matrix[useRight].datastart, input.projection_matrix[useRight].dataend, output.P.begin());
	return;
}

void	runImagePub(const image_transport::CameraPublisher &camPub, 
	std::shared_ptr<sensor_msgs::CameraInfo> camInfo,
	std::shared_ptr<sensor_msgs::Image> camImg)
{
	while (!g_quit)
	{
		//Implements thread synchronisation via condition variables and notifies. lock only received after wait
		//std::unique_lock<std::mutex> lk(g_mutex);
		//g_cv.wait(lk, []{return g_newFrame.load(std::memory_order_relaxed);}); 
		//Check for subscribers...
		std::this_thread::sleep_for(std::chrono::milliseconds(1)); //Sleep so that cpu is saved
		if (!g_newFrame.load(std::memory_order_relaxed))
			continue;
		if (camPub.getNumSubscribers() == 0)
		{
			g_newFrame.store(false, std::memory_order_relaxed);
			continue;
		}
		std::lock_guard<std::mutex> lock(g_mutex);
		//Update ImgHeader timestamp, copy to camInfo
		camImg->header.stamp = camInfo->header.stamp = ros::Time::now();
		//Publish	
		camPub.publish(make_shared_ptr(camImg), make_shared_ptr(camInfo));
	}
}

void duoCallBack(const PDUOFrame pFrameData, void *pUserData)
{
	std::lock_guard<std::mutex> lock(g_mutex);
	//Copy images to local buffer and return
	sensor_msgs::fillImage(	*g_camImgL.get(), 								// image reference
		sensor_msgs::image_encodings::MONO8, 	// type of encoding
		pFrameData->height, 					// columns in pixels 
		pFrameData->width,						// rows in pixels
		pFrameData->width,						// step size 
		pFrameData->leftData);					// left camera data pointer
	sensor_msgs::fillImage(	*g_camImgR.get(), 								// image reference
		sensor_msgs::image_encodings::MONO8, 	// type of encoding
		pFrameData->height, 					// columns in pixels 
		pFrameData->width,						// rows in pixels
		pFrameData->width,						// step size 
		pFrameData->rightData);					// left camera data pointer
	//TODO: Depth image
	//TODO: IMU Data
	if (pFrameData->IMUPresent)
	{
		ros::Time lastFrame = ros::Time::now();
		float x, y, z, lx, ly, lz;
		//Calc time constants needed
		//uint32_t maxTimeDiff = pFrameData->IMUData[pFrameData->IMUSamples].timeStamp - pFrameData->IMUData[0].timeStamp;	//This is time between first and last frame in 100usecs
		for (size_t imuCounter = 0; imuCounter < pFrameData->IMUSamples; imuCounter++)
		{
			sensor_msgs::ImuPtr imuData(new sensor_msgs::Imu);
			//The time calculation is tricky. We assume that we can basically count backwards from now()
			ros::Duration step((1.e-4)*(pFrameData->IMUSamples - (imuCounter+1)));
			imuData->header.stamp = lastFrame - step;
			imuData->header.frame_id = "duo_imu_frame";

					//imuData->orientation.x;	//TODO: Deal with this not being present...
			//Note that the duo uses units g's for acceleration and degs/sec (TBC) for rotation
			//https://duo3d.com/docs/articles/#DUOAccelRangeLink
			
			imuData->angular_velocity.x = x = pFrameData->IMUData[imuCounter].gyroData[2];
			imuData->angular_velocity.y = y = -pFrameData->IMUData[imuCounter].gyroData[0];
			imuData->angular_velocity.z = z = pFrameData->IMUData[imuCounter].gyroData[1];

			imuData->linear_acceleration.x = lx = -pFrameData->IMUData[imuCounter].accelData[2] * GRAVITY * DUO_MAX_G;
			imuData->linear_acceleration.y = ly = -pFrameData->IMUData[imuCounter].accelData[0] * GRAVITY * DUO_MAX_G;
			imuData->linear_acceleration.z = lz = pFrameData->IMUData[imuCounter].accelData[1] * GRAVITY * DUO_MAX_G;	//Fixing for Duo's frame orientation and scale
		
			g_imuPub->publish(imuData);
		}
		//std::cout.width(10);
		std::cout << std::setprecision(4) << std::setw(10) << x << " " << std::setw(10) << y << " " << std::setw(10) << z << " || " << std::setw(10) << lx << " " << std::setw(10) << ly << " " << std::setw(10) << lz << std::endl;
	}
	//Set and notify threads
	g_newFrame.store(true, std::memory_order_relaxed);
	//g_cv.notify_one();
	return;
}