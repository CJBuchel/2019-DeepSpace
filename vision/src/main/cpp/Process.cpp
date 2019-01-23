#include "Process.h"
#include "Capture.h"
#include <opencv2/opencv.hpp>
#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include <stdio.h>
#include <iostream>
#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <cameraserver/CameraServer.h>
#include <networktables/NetworkTableInstance.h>
#include <cscore.h>

#include "devices/kinect.h"

using namespace cv;
using namespace std;
//Set _capture as a Capture object reference
Process::Process(Capture &capture) : _capture(capture) {}

Capture &Process::GetCapture() {
    return _capture;
}

void Process::CopyImgTrack(cv::Mat &imgTrack) {
  std::lock_guard<std::mutex> lock(_classMutex);
  _imgTrack.copyTo(imgTrack);
}

void Process::CopyImgOriginal(cv::Mat &imgOriginal) {
  std::lock_guard<std::mutex> lock(_classMutex);
  _imgOriginal.copyTo(imgOriginal);
}



void Process::Init() {
  
  std::cout << "Process Init Started" << std::endl;
  // Wait until Periodic() sends data
  std::unique_lock<std::mutex> lk(_classMutex);
  _conVar.wait(lk);
 
  // after the wait, we own the lock.
  std::cout << "Worker thread is processing data\n";
  
  _videoMode = _capture.GetVideoMode();
  _imgTrack = cv::Mat{_videoMode.height, _videoMode.width, CV_8UC3};
  _imgOriginal = cv::Mat{_videoMode.height, _videoMode.width, CV_8UC3};
 
  // Send data back to Init()
  _processed = true;
  std::cout << "Worker thread signals data processing completed\n";
 
  // Manual unlocking is done before notifying, to avoid waking up
  // the waiting thread only to block again (see notify_one for details)
  lk.unlock();
  _conVar.notify_all();
  

  std::cout << "Process Init Ended" << std::endl;
}

void Process::Periodic() {}