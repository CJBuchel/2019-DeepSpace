#include "capture.h"

#include <opencv2/opencv.hpp>
#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"

#include <stdio.h>
#include <iostream>
#include <cameraserver/CameraServer.h>
#include <cscore.h>
#include <thread>
#include <mutex>

cs::UsbCamera camTapeAlign{"USBCam", 0};
cs::UsbCamera camBallAlign{"USBCam1", 1};
cs::CvSink sink{"USB"};
auto videoModeTape = camTapeAlign.GetVideoMode();
auto videoModeBall = camBallAlign.GetVideoMode();
std::mutex classMutex; // is a class-level mutex ok, or do I need one for each

// The capMat is what comes from the camera, and the outMat is what goes to the dashboard. Note: 
// the height - width order is reversed here (height first, width second), unlike other parts.
cv::Mat imgOriginalTape{videoModeTape.height, videoModeTape.width, CV_8UC3};
cv::Mat imgOriginalBall{videoModeBall.height, videoModeBall.width, CV_8UC3};
cv::Mat imgHSVTape{videoModeTape.height, videoModeTape.width, CV_8UC3};
cv::Mat imgHSVBall{videoModeBall.height, videoModeBall.width, CV_8UC3};

static Capture *capture;

static Capture *GetInstance() { // will this work ? are there multiple instances ?
  if (capture == NULL) {
    capture = new Capture();
    capture->Init();
  }
  return capture;
}

// Getters (might still need to mutex/atomic these to prevent writing while reading (essentially vsync)) *

auto GetVideoModeTape() { return videoModeTape; }
auto GetVideoModeBall() { return videoModeBall; }
cv::Mat GetImgOriginalTape() { return imgOriginalTape; }
cv::Mat GetImgOriginalBall() { return imgOriginalBall; }
cv::Mat GetImgHSVTape() {
  std::lock_guard<std::mutex> lock(classMutex);
  return imgHSVTape;
}
cv::Mat GetImgHSVBall() {
  std::lock_guard<std::mutex> lock(classMutex);
  return imgHSVTape;
}

// Main methods

void Init() {
	// This creates a webcam on USB, and dumps it into a sink. The sink allows us to access the image with sink.GrabFrame
  sink.SetSource(camTapeAlign);
	sink.SetSource(camBallAlign);
  camTapeAlign.SetExposureManual(-100);
  camBallAlign.SetExposureManual(100);

  // The camera defaults to a lower resolution, but you can choose any compatible resolution here.
  camTapeAlign.SetResolution(640, 480);
  camBallAlign.SetResolution(640, 480);

  std::cout << "Width: " << videoModeTape.width << " Height: " << videoModeTape.height << std::endl;
	std::cout << "Width: " << videoModeBall.width << " Height: " << videoModeBall.height << std::endl;
}

void Run() {
	// Grab a frame when possible, then convert to grayscale and send to the dashboard.
	if (sink.GrabFrame(imgOriginalTape) != 0) {
		cv::cvtColor(imgOriginalTape, Capture::GetInstance()->GetImgHSVTape(), cv::COLOR_RGB2HSV); // Use mutex getter
  }

	if (sink.GrabFrame(imgOriginalBall) != 0) {
		cv::cvtColor(imgOriginalBall, Capture::GetInstance()->GetImgHSVBall(), cv::COLOR_RGB2HSV);
  }
}

void Start() {
  std::thread captureThread(&Run);
  captureThread.detach();
}