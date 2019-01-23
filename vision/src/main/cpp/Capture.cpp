#include "Capture.h"

#include <opencv2/opencv.hpp>
#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <stdio.h>
#include <iostream>

#include <cameraserver/CameraServer.h>
#include <networktables/NetworkTableInstance.h>
#include <cscore.h>


#include "devices/kinect.h"

using namespace cv;
using namespace std;


Capture::Capture(int port) : _cam("USBCam", port) {}

// Getters
cs::VideoMode Capture::GetVideoMode() {
  return _videoMode;
}

// Copiers
void Capture::CopyCaptureMat(cv::Mat &captureMat) {
  std::lock_guard<std::mutex> lock(_classMutex);
  _captureMat.copyTo(captureMat);
}

void Capture::CopyImgTrack(cv::Mat &imgTrack) {
  std::lock_guard<std::mutex> lock(_classMutex);
  _imgTrack.copyTo(imgTrack);
}

void Capture::CopyImgOriginal(cv::Mat &imgOriginal) {
  std::lock_guard<std::mutex> lock(_classMutex);
  _imgOriginal.copyTo(imgOriginal);
}

void Capture::Init() {

  _sink.SetSource(_cam);
  _cam.SetExposureManual(-100);

  // The camera defaults to a lower resolution, but you can choose any compatible resolution here.
  _cam.SetResolution(640, 480);

  //auto _videoMode = _cam.GetVideoMode();
  _videoMode = _cam.GetVideoMode();
  std::cout << "Width: " << _videoMode.width << " Height: " << _videoMode.height << std::endl;

  _captureMat = cv::Mat::zeros(_videoMode.height, _videoMode.width, CV_8UC3);
  _imgTrack = cv::Mat{_videoMode.height, _videoMode.width, CV_8UC3};
	_imgOriginal = cv::Mat{_videoMode.height, _videoMode.width, CV_8UC3};
}

void Capture::Periodic() {
  code = _sink.GrabFrame(_captureMat);
  _isValid = code == 0;
}

bool Capture::IsValidFrame() {
  return _isValid;
}

int Capture::GetPort() {
  return camPort;
}

int Capture::GetCode() {
  return code;
}
