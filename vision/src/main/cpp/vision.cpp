//To build Program use .\gradlew vision:build
//To Run Program use .\gradlew vision:runvision
//To deploy to a tinkerboard or pi us .\gradlew vision:deploy  Note that when using tinkerboard deploy might not work to OS, you might need to use Raspbian ISO properly installed to the sd
#include "Vision.h"
#include <opencv2/opencv.hpp>
#include <cameraserver/CameraServer.h>
#include <cscore.h>

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

#include <cameraserver/CameraServer.h>
#include <cscore.h>
#include <thread>

using namespace cv;
using namespace std;

RNG rng(12345);
Rect bounding_rect;
int thresh = 100;
float height_offset;
float width_offset;
float width_goal = 320;
float height_goal = 240;

// Target vectors
vector<cv::Point2f> centres;
vector<bool> lefts;
vector<bool> rights;
vector<cv::Point2f> targets;
vector<float> angles;
vector<float> heights;
vector<float> distances;

//void curtin_frc_vision::run() {
	//frc::CameraServer::GetInstance()->StartAutomaticCapture(0);
//}

void curtin_frc_vision::run() {
	cs::CvSource outputBall;

	cs::UsbCamera cam{"USBCam", 0};
	cs::CvSink sink{"USB"};
	sink.SetSource(cam);

	// The camera defaults to a lower resolution, but you can choose any compatible resolution here.
	cam.SetResolution(640, 480);
	cam.SetExposureManual(-100);

	auto video_mode = cam.GetVideoMode();
	std::cout << "Width: " << video_mode.width << " Height: " << video_mode.height << std::endl;
	// This lets us see the camera output on the robot dashboard. We give it a name, and a width and height.
	cs::CvSource output = frc::CameraServer::GetInstance()->PutVideo("USB Camera", video_mode.width, video_mode.height);

	// The capMat is what comes from the camera, and the outMat is what goes to the dashboard. Note: 
	// the height - width order is reversed here (height first, width second), unlike other parts.
	cv::Mat imgTrack{video_mode.height, video_mode.width, CV_8UC3};
	cv::Mat imgOriginal{video_mode.height, video_mode.width, CV_8UC3};
	//cv::Mat imgHSVBall{video_mode.height, video_mode.width, CV_8UC3};
	//cv::Mat imgTrack{video_mode.height, video_mode.width, CV_8UC3};
	//cv::Mat greenHueImage{video_mode.height, video_mode.width, CV_8UC3};
	//cv::Mat imgTrack{video_mode.height, video_mode.width, CV_8UC3};
	//cv::Mat imgTrack{video_mode.height, video_mode.width, CV_8UC3}; 

  while(true) {
	  	if(sink.GrabFrame(imgOriginal) != 0){
			std::cout << "Origin Image Found" << endl;
			//Green Hue Processing Block
			//========================================================================================================
			//--------------------------------------------------------------------------------------------------------
			//========================================================================================================

			// Threshold the HSV image, keep only the green pixels (RetroBall)
			cv::cvtColor(imgOriginal, imgTrack, cv::COLOR_RGB2HSV);
			//========================================================================================================
			//--------------------------------------------------------------------------------------------------------
			//========================================================================================================




			// Contours Blocks (Draws a convex shell over the thresholded image.)
			//________________________________________________________________________________________________________
			//________________________________________________________________________________________________________
				
			//kinectCondition t_condition;
			vector<vector<Point> > contours;
			vector<vector<Point> > filteredContoursBall;
			vector<vector<Point> > filteredHullsBall;
			vector<Rect> ir_rects;
			int active_contour;
			Scalar hsl_low, hsl_high;
			bool show_window;

			//cv::Mat tmp(image);

			// Usually this is used to convert it to HLS, but since it's an IR (monochrome) image,
			// there's no need to waste the CPU Time doing this 
			// cvtColor(tmp, tmp, CV_RGB2HLS);

			double largestArea = 0.0;
			active_contour = -1;
			// Filters size for Reflective Ball
			cv::inRange(imgTrack, cv::Scalar(7, 100, 100), cv::Scalar(20, 255, 255), imgTrack); // get img via getter
			findContours(imgTrack, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_TC89_KCOS);

			for (int i = 0; i < contours.size(); i++) {
				vector<Point> contour = contours[i];
				Rect r = boundingRect(contour);
				
				double area = contourArea(contour);
				if (area > 300.0) {
					vector<Point> hull;
					convexHull(contour, hull);
					double solidity = 100 * area / contourArea(hull);

					if (solidity < 60.0) {
					if (area > largestArea) {
						largestArea = area;
						active_contour = filteredContoursBall.size();
					}
					filteredContoursBall.push_back(contour);
					filteredHullsBall.push_back(hull);
					ir_rects.push_back(r);
					}
				}
			}


			/// Detect edges using Canny
			Canny(imgTrack, imgTrack, thresh, thresh * 2);

			/// Find contours
			//vector<vector<Point> > contours;
			vector<Vec4i> hierarchy;
			threshold( imgTrack, imgTrack, thresh, 255, THRESH_BINARY );
			//findContours( threshold_output, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );
			//findContours(greenHueImage, contours, hierarchy, RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

			// ----Ball
			/// Find the convex hull object for each contour
			vector<vector<Point> >hullBall(filteredContoursBall.size());
			for (size_t i = 0; i < filteredContoursBall.size(); i++) {
				convexHull(filteredContoursBall[i], hullBall[i]);
			}

			/// Draw filteredContours + hull results
			imgTrack = Mat::zeros(imgTrack.size(), CV_8UC3);
			vector<Rect> boundRectBall( filteredContoursBall.size() );


			for (size_t i = 0; i < filteredContoursBall.size(); i++) {
				Scalar color = Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
				drawContours(imgTrack, filteredContoursBall, (int)i, color);
				drawContours(imgTrack, hullBall, (int)i, color);
			}


			for (size_t i = 0; i < filteredContoursBall.size(); i++) {
				Scalar color = Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
				drawContours(imgTrack, filteredContoursBall, (int)i, color);
				drawContours(imgTrack, hullBall, (int)i, color);
			}

			//________________________________________________________________________________________________________
			//________________________________________________________________________________________________________

			/*
			//Get RotatedRectangles 
			centres.clear(); //clear the vectors
			heights.clear();
			lefts.clear();
			rights.clear();

			for (int i=0; i<filteredContoursBall.size(); i++) {
		
				cv::RotatedRect rotatedRect = cv::minAreaRect(contours[i]);

				float angle = rotatedRect.angle;

				cv::Point2f centre = rotatedRect.center;

				cv::Point2f rectPoints[4];
				rotatedRect.points(rectPoints);

				float min = rectPoints[0].y;
				float max = rectPoints[0].y;
			
				for (int j=1; j<4; j++) { //find the minimum and maximum y-values of each rectangle
					if (rectPoints[j].y > max) {
						max = rectPoints[j].y;
					}
					if (rectPoints[j].y < min) {
						min = rectPoints[j].y;
					}
				}

				float height = max - min; //get the height of each rectangle

				centres.push_back(centre);
				heights.push_back(height);

				if (angle > 10 && angle < 19) {
					rights.push_back(true);
					lefts.push_back(false);
				}
				else if (angle < -10 && angle > -19) {
					rights.push_back(false);
					lefts.push_back(true);
				}
				else {
					rights.push_back(false);
					lefts.push_back(false);
				}

				std::stringstream ss;	ss<<angle; //magic shit, idk
				std::stringstream hei;	hei<<height;	
				cv::putText(imgTrack, ss.str() + " height:" + hei.str(), centre + cv::Point2f(-25,25), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(255,0,255)); //label the angle on each rectangle
			}

			int leftmost = -1;
			float leftPos = 1280;
			targets.clear();
			angles.clear();
			distances.clear();

			for (int i=0; i<filteredContoursBall.size(); i++) {
				if (lefts[i]) { //checks if current iteration is a left
					for (int j=0; j<filteredContoursBall.size(); j++) {
						if (rights[j] && centres[j].x < leftPos && centres[j].x > centres[i].x) { //checks if nested iteration is a right and left of the last checked one
							leftmost = j;
						}
					}

					if (leftmost > -1) {
						targets.push_back((centres[i]+centres[leftmost])/2); //adds the Points2f position of each target to a vector
						distances.push_back(200/(heights[i]+heights[leftmost])); //adds the estimated distance to each target. Calibrate by changing the number.
					}
				}
			}

			Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );

			for (int i=0; i<targets.size(); i++) {
				std::stringstream dis;	dis<<distances[i];
				cv::rectangle(imgTrack, targets[i] + Point2f(-3,-3), targets[i] + Point2f(3,3), color, 2); //draw small rectangle on target locations
				cv::putText(imgTrack, dis.str(), targets[i] + cv::Point2f(-25,25), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(255,0,255));
			}

			*/
			// Bounding Box Block, (Draws a border around the processed image)
			//,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,

			//vector<vector<Point> > contoursBox;

			/// Detect edges using Threshold
			threshold( imgTrack, imgTrack, thresh, 255, THRESH_BINARY );
			/// Find contoursBox



			/// Approximate contoursBox to polygons + get bounding rects and circles -------Ball
			vector<vector<Point> > hullBall_poly( hullBall.size() );
			vector<Point2f>centerBall( hullBall.size() );
			vector<float>radiusBall( hullBall.size() );

			for( int i = 0; i < hullBall.size(); i++ ) { approxPolyDP( Mat(hullBall[i]), hullBall_poly[i], 3, true );
				boundRectBall[i] = boundingRect( Mat(hullBall_poly[i]) );
				minEnclosingCircle( (Mat)hullBall_poly[i], centerBall[i], radiusBall[i] );
			}


			/// Draw polygonal contour + bonding rects + circles
			for( int i = 0; i< hullBall.size(); i++ ) {
				Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
				drawContours( imgTrack, hullBall_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
				bounding_rect=boundingRect(filteredContoursBall[i]); // Find the bounding rectangle for biggest contour
				rectangle( imgTrack, boundRectBall[i].tl(), boundRectBall[i].br(), color, 2, 8, 0 );
				circle( imgTrack, centerBall[i], (int)radiusBall[i], color, 2, 8, 0 );
			}


			//_____________________Center Calcs______(Calculates the center from Border Box, And calculates X,Y Offset)_______ Ok.. it's suppose to calculate from borderbox, but not yet. using hull instead
			// get the moments 
			// -------Ball

			vector<Moments> muBall(hullBall_poly.size());
			for( int i = 0; i<hullBall_poly.size(); i++ ) {
				muBall[i] = moments( hullBall_poly[i], false );
			}

			// get the centroid of figures.
			vector<Point2f> mcBall(hullBall_poly.size());
			for( int i = 0; i<hullBall_poly.size(); i++) {
				mcBall[i] = Point2f( muBall[i].m10/muBall[i].m00 , muBall[i].m01/muBall[i].m00 );
			}

			// draw filteredContours
			//Mat imgTrackcenter(canny_output.size(), CV_8UC3, Scalar(255,255,255));


			for( int i = 0; i<hullBall_poly.size(); i++ ) {
				Scalar color = Scalar(167,151,0); // B G R values
				//drawContours(imgTrack, hull_poly, i, color, 2, 8, hierarchy, 0, Point());
				circle( imgTrack, mcBall[i], 4, color, -1, 8, 0 );

				// offsets from centerBall
				Point centerBall = Point((mcBall[i].x), (mcBall[i].y));
				width_offset = width_goal - centerBall.x;
				height_offset = height_goal - centerBall.y;
				cout << "Offset From CenterBall x,y =" << height_offset << "," << width_offset << endl; //The output values... are a bit strange, need to look into that
			}


			// Grab a frame. If it's not an error (!= 0), convert it to grayscale and send it to the dashboard.
			//output.PutFrame(imgOriginal);
			output.PutFrame(imgTrack);
			cout << "Origin Image Processed" << endl;
  		}

		else{
			std::cout << "Origin Image is Not Available" << endl;
		}

	}


}