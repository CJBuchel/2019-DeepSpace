#include "Display.h"

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

#include "devices/kinect.h"

Display::Display(Process &process) : _process(process) {}

void Display::Init() {
  std::cout << "Display Init Started" << std::endl;
  Capture &capture = _process.GetCapture();
  _videoMode = capture.GetVideoMode();

  // Set up output
  _output = frc::CameraServer::GetInstance()->PutVideo("USB Camera", _videoMode.width, _videoMode.height);
   std::cout << "Display Init Ended" << std::endl;
}

void Display::Periodic() {
  std::cout << "Display Periodic Started" << std::endl;
  Capture &capture = _process.GetCapture();
  _process.CopyImgTrack(_imgTrack);
  if(capture.IsValidFrame()) {
    // Grab a frame. If it's not an error (!= 0), convert it to grayscale and send it to the dashboard.
  #ifdef __DESKTOP__
    imshow( "Output",_imgTrack);
  #endif
    _output.PutFrame(_imgTrack);
		std::cout << "Origin Image Processed" << std::endl;
    // other output if needed
  }
  else {
    std::cout << "Origin Image Not Available" << std::endl;
  }
  std::cout << "Display Periodic Ended" << std::endl;
}

