#include "capture.h"
#include "process.h"
#include "display.h"

#include <opencv2/opencv.hpp>
#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"

#include <cameraserver/CameraServer.h>
#include <cscore.h>
#include <thread>

// need to get videoModeTape and ball, and imgOriginalTape and ball

cs::CvSource outputTape = frc::CameraServer::GetInstance()->PutVideo("USB Camera", Capture::GetInstance()->GetVideoModeTape().width, Capture::GetInstance()->GetVideoModeTape().height);
cs::CvSource outputBall = frc::CameraServer::GetInstance()->PutVideo("USB Camera", Capture::GetInstance()->GetVideoModeBall().width, Capture::GetInstance()->GetVideoModeBall().height);

void Run() {
  //imshow("HSV Image", imgHSV);
	//imshow("center Calc", drawingcenter);
	//imshow("Contours", drawingBox);
	cv::imshow("OriginalTape", Capture::GetInstance()->GetImgOriginalTape()); // Shows the original image
	cv::imshow("OriginalBall", Capture::GetInstance()->GetImgOriginalBall());
	//imshow("Track Output", greenHueImage); // Shows the Threhold Image
	cv::imshow("Threshold Image", Process::GetInstance()->GetGreenHueImage());
	
	cv::Mat drawing = Process::GetInstance()->GetDrawing(); // Error workaround
	outputTape.PutFrame(drawing);
	outputBall.PutFrame(drawing);
}

void Start() {
	std::thread displayThread(&Run);
  displayThread.detach();
}