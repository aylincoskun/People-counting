/*
 * node.h
 *
 *  Created on: Nov 22, 2014
 *      Author: anil
 */

#ifndef SRC_NODE_H_
#define SRC_NODE_H_

#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <pcl_ros/point_cloud.h>

#include "cv.h"
#include "highgui.h"
#include "opencv2/imgproc/imgproc.hpp"

//for publishing and subscribing to images
//in ROS allows you to subscribe to compressed image streams
#include <image_transport/image_transport.h>

//includes the header for CvBridge as well as some useful constants and functions related to image encodings
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

//Includes the headers for OpenCV's image processing and GUI modules
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <opencv2/video/background_segm.hpp>

//Timer
#include <cstdio>
#include <ctime>

using namespace ros;
using namespace sensor_msgs;
using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;
#include"waterfilling.h"

class Subscribe_Depth {
	cv_bridge::CvImagePtr cv_ptr;
	NodeHandle nh;
	Subscriber sub;
	Mat a, b;
	double min, max;
	Ptr<BackgroundSubtractor> pMOG1;
	Mat fgMaskMOG1;
	int x =0 , y=0  ;

	//default capture width and height
	const int FRAME_WIDTH = 320;
	const int FRAME_HEIGHT = 240;
	//max number of objects to be detected in frame
	const int MAX_NUM_OBJECTS=50;
	//minimum and maximum object area
	const int MIN_OBJECT_AREA = 5*5;
	const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1.5;
	//names that will appear at the top of each window
	const string windowName = "Original Image";
	const string windowName1 = "HSV Image";
	const string windowName2 = "Thresholded Image";
	const string windowName3 = "After Morphological Operations";
	const string trackbarWindowName = "Trackbars";

public:
	Subscribe_Depth() {

		sub = nh.subscribe("/camera/depth/image", 1, &Subscribe_Depth::callback,
				this);
		pMOG1 = new BackgroundSubtractorMOG(); //MOG2 approach

	}
	Mat depthSubtrac(Mat depth_map, Mat back) {
		typedef uchar imgType;
		imgType* m_a = NULL;
		imgType* m_b = NULL;
		imgType* m_c = NULL;

		int picWidth = depth_map.cols; //picture width
		int picHeight = depth_map.rows; //picture height

		Mat outputImg(picHeight, picWidth, depth_map.type()); //initialize the output image

		m_a = (imgType*) back.data;
		m_b = (imgType*) depth_map.data;
		m_c = (imgType*) outputImg.data;

		for (int i = 0; i < picHeight * picWidth; i++, m_a++, m_b++, m_c++) {
			if (*m_a < 2) {
				*m_a = 0;
				*m_c = (*m_a) * (*m_b);

			} else {
				*m_a = 1;
				*m_c = (*m_a) * (*m_b);

			}

		}

		return outputImg;
	}

	~Subscribe_Depth() {
	}

	void trackFilteredObject(int &x, int &y, Mat threshold, Mat &cameraFeed){

		Mat temp;
		threshold.copyTo(temp);
		//these two vectors needed for output of findContours
		vector< vector<Point> > contours;
		vector<Vec4i> hierarchy;
		//find contours of filtered image using openCV findContours function
		findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );
		//use moments method to find our filtered object
		double refArea = 0;
		bool objectFound = false;
		if (hierarchy.size() > 0) {
			int numObjects = hierarchy.size();
	        //if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
	        if(numObjects<MAX_NUM_OBJECTS){
				for (int index = 0; index >= 0; index = hierarchy[index][0]) {

					Moments moment = moments((cv::Mat)contours[index]);
					double area = moment.m00;

					//if the area is less than 20 px by 20px then it is probably just noise
					//if the area is the same as the 3/2 of the image size, probably just a bad filter
					//we only want the object with the largest area so we safe a reference area each
					//iteration and compare it to the area in the next iteration.
	                if(area>MIN_OBJECT_AREA && area<MAX_OBJECT_AREA && area>refArea){
						x = moment.m10/area;
						y = moment.m01/area;
						objectFound = true;
						refArea = area;
					}else objectFound = false;


				}
				//let user know you found an object
				/*
				 *
				 *   rgb ye çevirme rankli
				 *
				 */
				cvtColor(cameraFeed, cameraFeed, CV_GRAY2RGB);

				if(objectFound ==true){
					putText(cameraFeed,"Tracking Object",Point(0,50),2,1,Scalar(0,255,0),2);
					//draw object location on screen
					drawObject(x,y,cameraFeed);}

			}else putText(cameraFeed,"TOO MUCH NOISE! ADJUST FILTER",Point(0,50),1,2,Scalar(0,0,255),2);
		}
	}

	void drawObject(int x, int y,Mat &frame){

		//use some of the openCV drawing functions to draw crosshairs
		//on your tracked image!

	    //UPDATE:JUNE 18TH, 2013
	    //added 'if' and 'else' statements to prevent
	    //memory errors from writing off the screen (ie. (-25,-25) is not within the window!)


		circle(frame,Point(x,y),20,Scalar(0,200,0),2);
	    if(y-25>0)
	    line(frame,Point(x,y),Point(x,y-25),Scalar(0,200,0),2);
	    else line(frame,Point(x,y),Point(x,0),Scalar(0,200,0),2);
	    if(y+25<FRAME_HEIGHT)
	    line(frame,Point(x,y),Point(x,y+25),Scalar(0,200,0),2);
	    else line(frame,Point(x,y),Point(x,FRAME_HEIGHT),Scalar(0,200,0),2);
	    if(x-25>0)
	    line(frame,Point(x,y),Point(x-25,y),Scalar(0,200,0),2);
	    else line(frame,Point(x,y),Point(0,y),Scalar(0,200,0),2);
	    if(x+25<FRAME_WIDTH)
	    line(frame,Point(x,y),Point(x+25,y),Scalar(0,200,0),2);
	    else line(frame,Point(x,y),Point(FRAME_WIDTH,y),Scalar(0,200,0),2);

		putText(frame,intToString(x)+","+intToString(y),Point(x,y+30),1,1,Scalar(0,200,0),2);

	}

	void morphOps(Mat &thresh){

		//create structuring element that will be used to "dilate" and "erode" image.
		//the element chosen here is a 3px by 3px rectangle

		Mat erodeElement = getStructuringElement( MORPH_RECT,Size(3,3));
	    //dilate with larger element so make sure object is nicely visible
		Mat dilateElement = getStructuringElement( MORPH_RECT,Size(8,8));

		erode(thresh,thresh,erodeElement);
		erode(thresh,thresh,erodeElement);


		dilate(thresh,thresh,dilateElement);
		dilate(thresh,thresh,dilateElement);



	}
	string intToString(int number){


		std::stringstream ss;
		ss << number;
		return ss.str();
	}

	void callback(const sensor_msgs::ImageConstPtr& im) {

		clock_t start1, end1, start2,end2,start3,end3,start4,end4;
		double duration1,duration2,duration3,duration4;



		Mat depth_map, sub_map, filter_map, new_im, temp;

		//convert ros to opencv (32 bits float)
		cv_ptr = cv_bridge::toCvCopy(im,sensor_msgs::image_encodings::TYPE_32FC1);

		//min max range
		minMaxIdx(cv_ptr->image, &min, &max);

		// expand your range to 0..255. Similar to histEq();
		/*
		 * CV_8U is unsigned 8bit/pixel - ie a pixel can have values 0-255,
		 * this is the normal range for most image and video formats.
		 *
		 */
		cv_ptr->image.convertTo(depth_map, CV_8U, 255 / (max - min), -min);

		Mat median_depth;

		medianBlur(depth_map,median_depth , 19);

		pMOG1->operator()(median_depth, fgMaskMOG1);

		Mat outputImg = depthSubtrac(median_depth, fgMaskMOG1);


		WaterFilling water_obje;

		Mat result = water_obje.WaterDrop(outputImg, 1000);



		Mat thresh_im,median_depth2;

		threshold(result,thresh_im,120,255,THRESH_BINARY);

		medianBlur(thresh_im,median_depth2 , 19);

		//morphOps(thresh_im);

		trackFilteredObject(x,y,median_depth2, median_depth2);


		imshow("depth",depth_map);
		imshow("median depth", median_depth);
		imshow("thresh", thresh_im);
		imshow("median_depth2", median_depth2);
		imshow("out",outputImg);
		imshow("waterfilling", result);
		imwrite("/home/anil/Desktop/depth.png", depth_map);
		imwrite("/home/anil/Desktop/out.png", outputImg);
		imwrite("/home/anil/Desktop/water.png", result);
		imwrite("/home/anil/Desktop/thresh.png", thresh_im);
		imwrite("/home/anil/Desktop/median_depth2.png", median_depth2);
		waitKey(3);



	}

};

#endif /* SRC_NODE_H_ */
