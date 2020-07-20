#include "nav_header.h"
#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/imgproc/imgproc.hpp"
	
using namespace cv;

cv::Mat img;

void imageCallback(const sensor_msgs::ImageConstPtr& msg){
	try{
		img = cv_bridge::toCvShare(msg, "bgr8")->image;
	}
	catch (cv_bridge::Exception& e){
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
		img.release();
	}
}

int findPic(ros::NodeHandle nh, vector<cv::Mat> imgs_track){

	cv::namedWindow("view");
	cv::startWindowThread();
	image_transport::ImageTransport it(nh);
	//image_transport::Subscriber sub = it.subscribe("camera/image/", 1, imageCallback); //--for the webcam
	image_transport::Subscriber sub = it.subscribe("camera/rgb/image_raw", 1, imageCallback); //--for the kinect
	int foundPic;
  
	cv::Mat video, img_roi;
  	ROS_INFO("Before while loop");
	for(int i = 0; i <= 5; i++){
		ros::spinOnce();
		
	
		//ROS_INFO("Inside while loop");
		sleep(1); //note anything less than 1s seems to result in a blank camera screen (likely camera too slow...)
				  //Alternatively use the ROS_INFO line above...
		if(!img.empty()){
			img.copyTo(video);
			img.release();

			Rect roi(50, 100, 590, 375); //only focus on the middle strip of the image
			img_roi = video(roi);
			//*****************************************
			//Testing image preprocessing
			//cvtColor(img_roi,img_roi,CV_BGR2GRAY);
			//equalizeHist(img_roi,img_roi);
			//*****************************************
			Rect WhereRec(0,0, img_roi.cols,img_roi.rows);
			img_roi.copyTo(video(WhereRec));
			cv::imshow("view", video);
			//video.release();
		}

		if (i==5){
		Mat img_object = img_roi;
		Mat img_RB = imgs_track[0];
		Mat img_RK = imgs_track[1];
		Mat img_CTC = imgs_track[2];

		if(!img_object.data)
			ROS_INFO("Object img problem");
		if(!img_RB.data)
			ROS_INFO("RB img problem");
		if(!img_RK.data)
			ROS_INFO("RK img problem");
		if(!img_CTC.data)
			ROS_INFO("CTC img problem");

		//-- Step 1: Detect the keypoints using SURF Detector
		int minHessian = 600;

		SurfFeatureDetector detector( minHessian );

		std::vector<KeyPoint> keypoints_object, keypoints_scene;
		std::vector<KeyPoint> keypoints_RK, keypoints_RB, keypoints_CTC;

		detector.detect( img_object, keypoints_object );
		detector.detect(img_RB, keypoints_RB);
		detector.detect(img_RK, keypoints_RK);
		detector.detect(img_CTC, keypoints_CTC);
	  
		//-- Step 2: Calculate descriptors (feature vectors)
		SurfDescriptorExtractor extractor;

		Mat descriptors_object, descriptors_scene;
		Mat descriptors_RB, descriptors_RK, descriptors_CTC;

		extractor.compute( img_object, keypoints_object, descriptors_object );
		//extractor.compute( img_scene, keypoints_scene, descriptors_scene );
		extractor.compute(img_RB, keypoints_RB, descriptors_RB);
		extractor.compute(img_RK, keypoints_RK, descriptors_RK);
		extractor.compute(img_CTC, keypoints_CTC, descriptors_CTC);

		//-- Step 3: Matching descriptor vectors using FLANN matcher
		//FlannBasedMatcher matcher;
		BFMatcher matcher(NORM_L2);
		//std::vector< DMatch > matches;
		//matcher.match( descriptors_object, descriptors_scene, matches );

		std::vector< DMatch > matches_RB, matches_RK, matches_CTC;	
		matcher.match(descriptors_object, descriptors_RB, matches_RB);
		matcher.match(descriptors_object, descriptors_RK, matches_RK);
		matcher.match(descriptors_object, descriptors_CTC, matches_CTC);

		double min_dist = 100;
		
		//-- Quick calculation of min distance between keypoints
		for(int i = 0; i < descriptors_object.rows; i++){
			if(matches_RB[i].distance < min_dist)
				min_dist = matches_RB[i].distance;
			if(matches_RK[i].distance < min_dist)
				min_dist = matches_RK[i].distance;
			if(matches_CTC[i].distance < min_dist)
				min_dist = matches_CTC[i].distance;
		}
		int nMatches_RB = 0;
		int nMatches_RK = 0;
        int nMatches_CTC = 0;

		printf("minmin = %f\n", min_dist);
		
		//Count number of keypoint matches for object against each reference image
		for( int i = 0; i < descriptors_object.rows; i++ ) { 
			if( matches_RB[i].distance < 3*min_dist )
	     		nMatches_RB++;
			if( matches_RK[i].distance < 3*min_dist )
	     		nMatches_RK++;
			if( matches_CTC[i].distance < 3*min_dist )
	     		nMatches_CTC++;
		}

		printf("RB = %i matches \n", nMatches_RB);
		printf("RK = %i matches \n", nMatches_RK);
		printf("CTC = %i matches \n", nMatches_CTC);
		
		if((nMatches_RK <= 20) && (nMatches_RK <= 20) && (nMatches_RK <= 20)){
				ROS_INFO("BLANK!");
				foundPic = 0;
			}

		else if((nMatches_RB > nMatches_RK + 5) && (nMatches_RB > nMatches_CTC + 5)){
			ROS_INFO("Found Raisin Bran!");
			foundPic = 1;
		}
		else if((nMatches_RK > nMatches_RB + 5) && (nMatches_RK > nMatches_CTC+5)){
			ROS_INFO("Found Rice Krispies!");
			foundPic = 2;
		}
		else if((nMatches_CTC > nMatches_RK+5) && (nMatches_CTC > nMatches_RB+5)){
			ROS_INFO("Found Cinnamon Toast Crunch!");
			foundPic = 3;
		}
		/*else if((nMatches_RK <= 20) && (nMatches_RK <= 20) && (nMatches_RK <= 20)){
				ROS_INFO("BLANK!!");
				foundPic = 0;
			}*/
		else{
			ROS_INFO("BLANK!!");
			foundPic = 0;
		}

		ROS_INFO("done");
		sleep(1);
		}
	  
	}
video.release();
return foundPic;
}
