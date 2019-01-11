//To build Program use .\gradlew vision:build
//To Run Program use .\gradlew vision:runvision
//To deploy to a tinkerboard or pi us .\gradlew vision:deploy  Note that when using tinkerboard deploy might not work to OS, you might need to use Raspbian ISO properly installed to the sd
#include "vision.h"
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

using namespace cv;
using namespace std;


RNG rng(12345);
Rect bounding_rect;
int thresh = 100;
float height_offset;
float width_offset;
float width_goal = 320;
float height_goal = 240;

cs::UsbCamera camTapeAlign{"USBCam", 0};
cs::UsbCamera camBallAlign{"USBCam1", 1};
cs::CvSink sink{"USB"};
auto video_modeTape = camTapeAlign.GetVideoMode();
auto video_modeBall = camBallAlign.GetVideoMode();
cv::Mat drawing;
cv::Mat green_hue_image;
cv::Mat orange_hue_image;

// Target vectors
vector<cv::Point2f> centres;
vector<cv::Point2f> targets;
vector<bool> lefts;
vector<bool> rights;
vector<float> angles;
vector<float> heights;
vector<float> distances;

// This lets us see the camera output on the robot dashboard. We give it a name, and a width and height.
cs::CvSource outputTape = frc::CameraServer::GetInstance()->PutVideo("USB Camera", video_modeTape.width, video_modeTape.height);
cs::CvSource outputBall = frc::CameraServer::GetInstance()->PutVideo("USB Camera", video_modeBall.width, video_modeBall.height);


// The capMat is what comes from the camera, and the outMat is what goes to the dashboard. Note: 
// the height - width order is reversed here (height first, width second), unlike other parts.
cv::Mat imgOriginalTape{video_modeTape.height, video_modeTape.width, CV_8UC3};
cv::Mat img_HSVTape{video_modeTape.height, video_modeTape.width, CV_8UC3};

cv::Mat imgOriginalBall{video_modeBall.height, video_modeBall.width, CV_8UC3};
cv::Mat img_HSVBall{video_modeBall.height, video_modeBall.width, CV_8UC3};

void curtin_frc_vision::init() {
	// This creates a webcam on USB, and dumps it into a sink. The sink allows us to access the image with sink.GrabFrame
  sink.SetSource(camTapeAlign);
	sink.SetSource(camBallAlign);
  camTapeAlign.SetExposureManual(-100);
  camBallAlign.SetExposureManual(100);

  // The camera defaults to a lower resolution, but you can choose any compatible resolution here.
  camTapeAlign.SetResolution(640, 480);
  camBallAlign.SetResolution(640, 480);

  std::cout << "Width: " << video_modeTape.width << " Height: " << video_modeTape.height << std::endl;
	std::cout << "Width: " << video_modeBall.width << " Height: " << video_modeBall.height << std::endl;
};

void curtin_frc_vision::capture() {
	// Grab a frame when possible, then convert to grayscale and send to the dashboard.
	if (sink.GrabFrame(imgOriginalTape) != 0) {
		cv::cvtColor(imgOriginalTape, img_HSVTape, cv::COLOR_RGB2HSV);
  }

	// Grab a frame when possible, then convert to grayscale and send to the dashboard.
	if (sink.GrabFrame(imgOriginalBall) != 0) {
		cv::cvtColor(imgOriginalBall, img_HSVBall, cv::COLOR_RGB2HSV);
  }
};

void curtin_frc_vision::process() {
	//Green Hue Processing Block
		//========================================================================================================
		//--------------------------------------------------------------------------------------------------------
		//========================================================================================================
	



		
		// Threshold the HSV image, keep only the green pixels (RetroTape)
		cv::inRange(img_HSVTape, cv::Scalar(35, 100, 30), cv::Scalar(78, 255, 255), green_hue_image);
		cv::inRange(img_HSVBall, cv::Scalar(10, 100, 100), cv::Scalar(20, 255, 255), orange_hue_image);


		//========================================================================================================
		//--------------------------------------------------------------------------------------------------------
		//========================================================================================================




		// Contours Blocks (Draws a convex shell over the thresholded image.)
		//________________________________________________________________________________________________________
		//________________________________________________________________________________________________________

    
    //KinectProcessor();
    //void processRGB(Mat mat);
    //void processIR(Mat mat);
        
    //kinectCondition t_condition;
    vector<vector<Point> > contours;
    vector<vector<Point> > filteredContoursTape;
		vector<vector<Point> > filteredContoursBall;
		vector<vector<Point> > filteredHullsTape;
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
    // Filters size for Reflective Tape
    findContours(green_hue_image, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_TC89_KCOS);
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
                    active_contour = filteredContoursTape.size();
                }
                filteredContoursTape.push_back(contour);
                filteredHullsTape.push_back(hull);
                ir_rects.push_back(r);
            }
        }
    }
	// Filters size for Ball
	findContours(orange_hue_image, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_TC89_KCOS);
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
		cv::Mat cannyTape_output;
		cv::Mat cannyBall_output;
		Canny(green_hue_image, cannyTape_output, thresh, thresh * 2);
		Canny(orange_hue_image, cannyBall_output, thresh, thresh * 2);

		/// Find contours
		//vector<vector<Point> > contours;
		cv::Mat thresholdTape_output;
		cv::Mat thresholdBall_output;
		vector<Vec4i> hierarchy;
		threshold( green_hue_image, thresholdTape_output, thresh, 255, THRESH_BINARY );
		threshold( orange_hue_image, thresholdBall_output, thresh, 255, THRESH_BINARY );
		//findContours( threshold_output, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );
		//findContours(green_hue_image, contours, hierarchy, RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

    // ----Tape
		/// Find the convex hull object for each contour
		vector<vector<Point> >hullTape(filteredContoursTape.size());
		for (size_t i = 0; i < filteredContoursTape.size(); i++)
		{
			convexHull(filteredContoursTape[i], hullTape[i]);
		}

		/// Draw filteredContours + hull results
		drawing = Mat::zeros(cannyTape_output.size(), CV_8UC3);
		vector<Rect> boundRectTape( filteredContoursTape.size() );


		for (size_t i = 0; i < filteredContoursTape.size(); i++)
		{
			Scalar color = Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
			drawContours(drawing, filteredContoursTape, (int)i, color);
			drawContours(drawing, hullTape, (int)i, color);
		}

	// -------Ball

		/// Find the convex hull object for each contour
		vector<vector<Point> >hullBall(filteredContoursBall.size());
		for (size_t i = 0; i < filteredContoursBall.size(); i++)
		{
			convexHull(filteredContoursBall[i], hullBall[i]);
		}

		/// Draw filteredContours + hull results
		drawing = Mat::zeros(cannyBall_output.size(), CV_8UC3);
		vector<Rect> boundRectBall( filteredContoursBall.size() );






		for (size_t i = 0; i < filteredContoursTape.size(); i++)
		{
			Scalar color = Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
			drawContours(drawing, filteredContoursTape, (int)i, color);
			drawContours(drawing, hullTape, (int)i, color);
		}
		
		for (size_t i = 0; i < filteredContoursBall.size(); i++)
		{
			Scalar color = Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
			drawContours(drawing, filteredContoursBall, (int)i, color);
			drawContours(drawing, hullBall, (int)i, color);
		}
    
		//________________________________________________________________________________________________________
		//________________________________________________________________________________________________________

		
		//Get RotatedRectangles 
		centres.clear(); //clear the vectors
		heights.clear();
		lefts.clear();
		rights.clear();

		for (int i=0; i<filteredContoursTape.size(); i++) {
			
			cv::RotatedRect rotatedRect = cv::minAreaRect(contours[i]);

			cv::Point2f centre = rotatedRect.center;

			cv::Point2f rectPoints[4];
			rotatedRect.points(rectPoints);

			float angle;

			//I got this code from StackOverFlow, so it's not my fault if it breaks
			cv::Point2f edge1 = cv::Vec2f(rectPoints[1].x, rectPoints[1].y) - cv::Vec2f(rectPoints[0].x, rectPoints[0].y);
      cv::Point2f edge2 = cv::Vec2f(rectPoints[2].x, rectPoints[2].y) - cv::Vec2f(rectPoints[1].x, rectPoints[1].y);

      cv::Point2f usedEdge = edge1;
      if(cv::norm(edge2) > cv::norm(edge1))
      	usedEdge = edge2;

      cv::Point2f reference = cv::Vec2f(1,0); // horizontal edge

      angle = 180.0f/CV_PI * acos((reference.x*usedEdge.x + reference.y*usedEdge.y) / (cv::norm(reference) *cv::norm(usedEdge)));
			//end StackOverFlow code

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

			if (angle > 110 && angle < 119) { //angle range for right classification
				rights.push_back(true);
				lefts.push_back(false);
			} else if (angle < 80 && angle > 71) { //angle range for left classification
				rights.push_back(false);
				lefts.push_back(true);
			} else {
				rights.push_back(false);
				lefts.push_back(false);
			}

			std::stringstream ss;	ss<<angle; //magic shit, idk
			std::stringstream hei;	hei<<height;	
			cv::putText(drawing, ss.str() + " height:" + hei.str(), centre + cv::Point2f(-25,25), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(255,0,255)); //label the angle on each rectangle
		}

		int leftmost = -1;
		float leftPos = 640;
		targets.clear();
		angles.clear();
		distances.clear();

		for (int i=0; i<filteredContoursTape.size(); i++) {
			if (lefts[i]) { //checks if current iteration is a left
				for (int j=0; j<filteredContoursTape.size(); j++) {
					if (rights[j] && centres[j].x < leftPos && centres[j].x > centres[i].x) { //checks if nested iteration is a right and left of the last checked one
						leftmost = j;
					}
				}

				if (leftmost > -1) {
					targets.push_back((centres[i]+centres[leftmost])/2); //adds the Points2f position of each target to a vector
					distances.push_back(200/(heights[i]+heights[leftmost])); //adds the estimated distance to each target. Calibrate by changing the number.
					float widthAdjust = 0.01 * distances[distances.size() - 1] * abs(centres[i].x - centres[leftmost].x); //Calibrate distance, then adjust the first number until robot facing target gives 0 degrees.
					if (widthAdjust > 1.0) {
						widthAdjust = 1.0;
					}
					try {
						angles.push_back(heights[leftmost] > heights[i] ? acos(widthAdjust) : -acos(widthAdjust));
					} catch (...) {
						angles.push_back(0);
					}
						
				}
			}
		}

		Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );

		for (int i=0; i<targets.size(); i++) {
			std::stringstream dis;	dis<<distances[i];
			cv::rectangle(drawing, targets[i] + Point2f(-3,-3), targets[i] + Point2f(3,3), color, 2); //draw small rectangle on target locations
			cv::putText(drawing, dis.str(), targets[i] + cv::Point2f(-25,25), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(255,0,255));
		}



		

		// Bounding Box Block, (Draws a border around the processed image)
		//,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,
		
		//vector<vector<Point> > contoursBox;

		/// Detect edges using Threshold
		threshold( green_hue_image, thresholdTape_output, thresh, 255, THRESH_BINARY );
		threshold( green_hue_image, thresholdBall_output, thresh, 255, THRESH_BINARY );
		/// Find contoursBox


		
		/// Approximate contoursBox to polygons + get bounding rects and circles -------tape
		vector<vector<Point> > hullTape_poly( hullTape.size() );
		vector<Point2f>centerTape( hullTape.size() );
		vector<float>radiusTape( hullTape.size() );

		for( int i = 0; i < hullTape.size(); i++ )
			{ approxPolyDP( Mat(hullTape[i]), hullTape_poly[i], 3, true );
			boundRectTape[i] = boundingRect( Mat(hullTape_poly[i]) );
			minEnclosingCircle( (Mat)hullTape_poly[i], centerTape[i], radiusTape[i] );
			}


		/// Draw polygonal contour + bonding rects + circles
		for( int i = 0; i< hullTape.size(); i++ )
			{
			Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
			drawContours( drawing, hullTape_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
			bounding_rect=boundingRect(filteredContoursTape[i]); // Find the bounding rectangle for biggest contour
			rectangle( drawing, boundRectTape[i].tl(), boundRectTape[i].br(), color, 2, 8, 0 );
			circle( drawing, centerTape[i], (int)radiusTape[i], color, 2, 8, 0 );
			}
		
		/// Approximate contoursBox to polygons + get bounding rects and circles -------ball
		vector<vector<Point> > hullBall_poly( hullBall.size() );
		vector<Point2f>centerBall( hullBall.size() );
		vector<float>radiusBall( hullBall.size() );

		for( int i = 0; i < hullBall.size(); i++ ) {
			approxPolyDP( Mat(hullBall[i]), hullBall_poly[i], 3, true );
			boundRectBall[i] = boundingRect( Mat(hullBall_poly[i]) );
			minEnclosingCircle( (Mat)hullBall_poly[i], centerBall[i], radiusBall[i] );
		}


		/// Draw polygonal contour + bonding rects + circles
		for( int i = 0; i< hullBall.size(); i++ )
			{
			Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
			drawContours( drawing, hullBall_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
			bounding_rect=boundingRect(filteredContoursTape[i]); // Find the bounding rectangle for biggest contour
			rectangle( drawing, boundRectBall[i].tl(), boundRectBall[i].br(), color, 2, 8, 0 );
			circle( drawing, centerBall[i], (int)radiusBall[i], color, 2, 8, 0 );
			}

		
			

    //_____________________Center Calcs______(Calculates the center from Border Box, And calculates X,Y Offset)_______ Ok.. it's suppose to calculate from borderbox, but not yet. using hull instead
      // get the moments 

	  // -------Ball
	 vector<Moments> muBall(hullBall_poly.size());
    for( int i = 0; i<hullBall_poly.size(); i++ )
      { muBall[i] = moments( hullBall_poly[i], false ); }
 
    // get the centroid of figures.
    vector<Point2f> mcBall(hullBall_poly.size());
    for( int i = 0; i<hullBall_poly.size(); i++)
      { mcBall[i] = Point2f( muBall[i].m10/muBall[i].m00 , muBall[i].m01/muBall[i].m00 ); }

    // draw filteredContours
    //Mat drawingcenter(canny_output.size(), CV_8UC3, Scalar(255,255,255));
  

    for( int i = 0; i<hullBall_poly.size(); i++ )
      {
      Scalar color = Scalar(167,151,0); // B G R values
      //drawContours(drawing, hull_poly, i, color, 2, 8, hierarchy, 0, Point());
	  circle( drawing, mcBall[i], 4, color, -1, 8, 0 );

			// offsets from centerBall
			Point centerBall = Point((mcBall[i].x), (mcBall[i].y));
			width_offset = width_goal - centerBall.x;
			height_offset = height_goal - centerBall.y;
			cout << "Offset From CenterBall x,y =" << height_offset << "," << width_offset << endl; //The output values... are a bit strange, need to look into that
			
    }

	// -------Tape

	vector<Moments> muTape(hullTape_poly.size());
    for( int i = 0; i<hullTape_poly.size(); i++ )
      { muTape[i] = moments( hullTape_poly[i], false ); }
 
    // get the centroid of figures.
    vector<Point2f> mcTape(hullTape_poly.size());
    for( int i = 0; i<hullTape_poly.size(); i++)
      { mcTape[i] = Point2f( muTape[i].m10/muTape[i].m00 , muTape[i].m01/muTape[i].m00 ); }

    // draw filteredContours
    //Mat drawingcenter(canny_output.size(), CV_8UC3, Scalar(255,255,255));
  

    for( int i = 0; i<hullTape_poly.size(); i++ )
      {
      Scalar color = Scalar(167,151,0); // B G R values
      //drawContours(drawing, hull_poly, i, color, 2, 8, hierarchy, 0, Point());
	  circle( drawing, mcTape[i], 4, color, -1, 8, 0 );

			// offsets from centerTape
			Point centerTape = Point((mcTape[i].x), (mcTape[i].y));
			width_offset = width_goal - centerTape.x;
			height_offset = height_goal - centerTape.y;
			cout << "Offset From CenterTape x,y =" << height_offset << "," << width_offset << endl; //The output values... are a bit strange, need to look into that
			
    }
		//,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,
};

void curtin_frc_vision::display() {
	// Show in a window
	
	imshow("Shell & Bounding", drawing);
	//imshow("HSV Image", img_HSV);
	//imshow("center Calc", drawingcenter);
	//imshow("Contours", drawingBox);
	imshow("OriginalTape", imgOriginalTape); // Shows the original image
	imshow("OriginalBall", imgOriginalBall);
	//imshow("Track Output", green_hue_image); // Shows the Threhold Image
	imshow("Threshold Image", green_hue_image);
	
	outputTape.PutFrame(drawing);
	outputBall.PutFrame(drawing);
};

void curtin_frc_vision::run() {
  // This is just a demonstration so you can see how this kind of code works. You'll be replacing this
  // with our actual vision tracking software!
  // You can view the vision output with Shuffleboard. Launch with `./gradlew :vision:ShuffleBoard`

	/* thread captureThread(&curtin_frc_vision::capture, this);
	thread processThread(&curtin_frc_vision::process, this);
	thread displayThread(&curtin_frc_vision::display, this);
	captureThread.detach();
	processThread.detach();
	display.detach(); */

  while (true) {
		curtin_frc_vision::capture();
		curtin_frc_vision::process();
		curtin_frc_vision::display();
  }
}