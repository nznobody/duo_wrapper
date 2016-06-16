#include "../include/DUOInterface.h"
#include <opencv2/opencv.hpp>
#include <thread>
#include <mutex>
#include <ctime>
#include <condition_variable>

#include <iostream>	//std::cout

std::mutex	_imLock;
std::condition_variable _newImg;
cv::Mat left, right;	//For thread Access
cv::Mat leftBG, rightBG;
cv::Mat dispN;
bool ready = false;
bool processed = false;

void t_imshow()
{
	cv::namedWindow("Duo Left");
	//cv::namedWindow("Duo Disp");
	//cv::namedWindow("Duo LeftBG");
	std::clock_t begin	= std::clock();
	std::clock_t end	= std::clock();
			//double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
			//std::cout << elapsed_secs << std::endl;
	while (true)
	{
	    // Wait until for new frame
		end	= std::clock();
		double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
		double fps = 0;
		if (elapsed_secs)
			fps = 1.0 / elapsed_secs;
		begin	= std::clock();
		std::unique_lock<std::mutex> lk(_imLock);
		_newImg.wait(lk, []{return ready;});
	
		 //after the wait, we own the lock.
		if (!left.empty())
		{
			cv::putText(left, cv::format("Average FPS=%d", (int )fps), cv::Point(30, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255));
			imshow("Duo Left", left);
		}
		if (!dispN.empty())
		{
			imshow("Duo Disp", dispN);
		}
		int key = cv::waitKey(30);
		if (key == 27)
			return;
		ready = false;
		// Manual unlocking is done before notifying, to avoid waking up
		// the waiting thread only to block again (see notify_one for details)
		lk.unlock();   
	}
}

class CallBacker
{
public:
	CallBacker() {}
	~CallBacker() {}
	void TestBack(const PDUOFrame pFrameData, void *pUserData)
	{
		duo::DUOInterface& _duo = duo::DUOInterface::GetInstance();
		std::lock_guard<std::mutex> lk(_imLock);
		left = cv::Mat(cv::Size(WIDTH, HEIGHT), CV_8UC1, pFrameData->leftData);
		right = cv::Mat(cv::Size(WIDTH, HEIGHT), CV_8UC1, pFrameData->rightData);
		
		ready = true;
		
		_newImg.notify_all();
	}
private:
	
};

int main()
{
	duo::DUOInterface& _duo = duo::DUOInterface::GetInstance();
	{ //Test rectification
		//_duo.SetRectifyOpencv(true);
		//_duo.SetOpencvCalib(true);
		
		//Test CUDA
		_duo.SetUseCUDA(true);
		
		//TestHWBackgroundSubtraction
		//_duo.SetHWBackground(true);
		
		//Test openCV Stereo
		//_duo.SetOpencvStereo(true);
	}
	
	if (_duo.initializeDUO())
	{
		{ //Test member callback
			CallBacker test;
			auto callbackFram = std::bind(&CallBacker::TestBack, &test, std::placeholders::_1, std::placeholders::_2);
			duo::DUOInterface::_extcallback = callbackFram;
		}
		_duo.SetUseDuoCalib(true);
		_duo.SetLedPWM(85);
		_duo.EnableCVSettings();
		_duo.startDUO();
	}
	
	//Start thread that shows image
	std::thread worker(t_imshow);
	worker.join();
	
	_duo.shutdownDUO();
}