#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <kobuki_msgs/BumperEvent.h>
#include <kobuki_msgs/CliffEvent.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/image_encodings.h>
#include <stdio.h>
#include <cmath>
#include <eStop.h>

#include <cv.h>
#include <opencv2/highgui/highgui.hpp>
#include "highgui.h"
#include <cv_bridge/cv_bridge.h>

#include <sound_play/sound_play.h>

//Timer
#include <time.h>

//ImgProc
#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/imgproc/imgproc.hpp"

//Add Odometry, Angle Conversions, LaserScan, and Time
//#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/LaserScan.h>
#include <time.h>

//laser variables
double laserRange = 10;
int laserSize = 0, laserOffset = 0, desiredAngle = 5;

using namespace std;

geometry_msgs::Twist follow_cmd;
int world_state;
double velX, angZ;
bool bumperLeft = 0, bumperCentre = 0, bumperRight = 0;
bool cliffOne = 0, cliffTwo = 0, cliffThree = 0;

//for ImgProc
using namespace cv;
Mat img0, img2, img3, img4, img4_1, img5;
int minHessian = 600;

double pi = 3.1416;

void followerCB(const geometry_msgs::Twist msg){
    follow_cmd = msg;
    velX = msg.linear.x;
    angZ = msg.angular.z;
}

void bumperCB(const kobuki_msgs::BumperEvent msg){
    if(msg.bumper == 0)
		bumperLeft = !bumperLeft;
	else if(msg.bumper == 1)
		bumperCentre = !bumperCentre;
	else if(msg.bumper == 2)
		bumperRight = !bumperRight;	
}

void cliffCB(const kobuki_msgs::CliffEvent msg){
    if(msg.sensor == 0)
		cliffOne = !cliffOne;
	else if(msg.sensor == 1)
		cliffTwo = !cliffTwo;
	else if(msg.sensor == 2)
		cliffThree = !cliffThree;	
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
	laserSize = (msg->angle_max - msg->angle_min)/msg->angle_increment;
	laserOffset = desiredAngle*pi/(180*msg->angle_increment);
	laserRange = 11;

	if(desiredAngle*pi/180 < msg->angle_max && -desiredAngle*pi/180 > msg->angle_min){
		for(int i = laserSize/2 - laserOffset; i < laserSize/2 + laserOffset; i++){
			if(laserRange>msg->ranges[i])
				laserRange = msg->ranges[i];
		}
	}
	else{
		for(int i = 0; i < laserSize; i++){
			if(laserRange>msg->ranges[i])
				laserRange = msg->ranges[i];
		}
	}

	if(laserRange == 11){
		laserRange = 0;
	}
}

void rgbCallback(const sensor_msgs::ImageConstPtr& msg){
}

void depthCallback(const sensor_msgs::ImageConstPtr& dmsg){
}

//-------------------------------------------------------------

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;
    sound_play::SoundClient sc;
    string path_to_sounds = "/home/turtlebot/catkin_ws/Sounds/";
    teleController eStop;

    image_transport::ImageTransport it(nh);
	namedWindow("view");
	img0 = imread("/home/turtlebot/catkin_ws/src/mie443_contest3/pic/FollowingFace.jpg",CV_LOAD_IMAGE_COLOR);
	img2 = imread("/home/turtlebot/catkin_ws/src/mie443_contest3/pic/HappyFace.jpg",CV_LOAD_IMAGE_COLOR); 
	img3 = imread("/home/turtlebot/catkin_ws/src/mie443_contest3/pic/DisgustFace.jpg",CV_LOAD_IMAGE_COLOR);
	img4 = imread("/home/turtlebot/catkin_ws/src/mie443_contest3/pic/CashMeOutside.jpg",CV_LOAD_IMAGE_COLOR);
	img4_1 = imread("/home/turtlebot/catkin_ws/src/mie443_contest3/pic/RageFace.jpg",CV_LOAD_IMAGE_COLOR);
	img5 = imread("/home/turtlebot/catkin_ws/src/mie443_contest3/pic/HeartFace.jpg",CV_LOAD_IMAGE_COLOR);

	//subscribers
    ros::Subscriber follower = nh.subscribe("follower_velocity_smoother/smooth_cmd_vel", 10, &followerCB);
	ros::Subscriber bumper = nh.subscribe("mobile_base/events/bumper", 10, &bumperCB);
    ros::Subscriber cliff = nh.subscribe("mobile_base/events/cliff", 10, &cliffCB);
    image_transport::Subscriber rgb_sub = it.subscribe("camera/rgb/image_raw", 1, rgbCallback);
	ros::Subscriber laser = nh.subscribe("scan", 1, laserCallback);
	
	//publishers
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop",1);

    double angular = 0.0;
    double linear = 0.0;

    geometry_msgs::Twist vel;
    vel.angular.z = angular;
    vel.linear.x = linear;

    //Set up timer
    time_t timer; //set up timer
    struct tm y2k = {0};
    double startTime, seconds, startTime1, startTime2, startTime3;
    int init0 = 0, init1 = 0, init2 = 0;
	//cout << "here" << endl;  
	//cout << "now here" << endl;
	imshow("view", img0);
	waitKey(50);
	while(ros::ok()){
	cout << "start loop" << endl;
        ros::spinOnce();
		waitKey(50);
        //.....**E-STOP DO NOT TOUCH**.......
        eStop.block();
        //...................................


	// **************************************************************
	// * States						                           		*
	// * 0 = following					                       		*
	// * 2 = loses sight of person -> positively excited	   		*
	// * 3 = hit any bumper only once (within 5s) -> disgusted 		*
	// * 4 = hit any bumper 2 times (within 5s) -> angry, third		*
	// *  	 time -> rage		                                    *
	// * 5 = pick up turtlebot -> infatuated						*
	// **************************************************************

		if(world_state == 0){
			//fill with your code
			ros::spinOnce();
			vel_pub.publish(follow_cmd);
			sleep(0.5);

			//sc.playWave(path_to_sounds + "sound.wav");
	    
			//printf("linear x: %f\n", velX);
			//printf("angular z: %f\n", angZ);
			printf("laserRange = %f\n", laserRange);
			printf("world state = %i\n", world_state);	
			
			//If the velocity (linear & angular) drops below 0-threshold (doesn't hit exactly 0), then save initial time

			if((laserRange < 1) && (abs(velX) < 0.01) && (abs(angZ) < 0.02) && (init0 == 0)){
				startTime = time(&timer);
				init0 = 1;		
			}
			//If the TurtleBot loses sight of person for more than 5s, it will enter "lost mode"
			if(((laserRange > 1) || (laserRange < 0.1)) && (init1 == 0)){
				startTime1 = time(&timer);
				init1 = 1;
			}
			if(((laserRange > 1) || (laserRange < 0.1)) && (init1 == 1)){
				if(difftime(time(&timer),startTime1) > 5){
					world_state = 2;
					init1 = 0;
					init0 = 0; //reset other init's to mitigate queuing of emotions
					init2 = 0;
				}	
			}
			else
				init1 = 0;
			
			if((bumperCentre || bumperLeft || bumperRight) && (init2 == 0) && (world_state == 0)){
				startTime2 = time(&timer);
				world_state = 3;
				init2 = 1;
				init1 = 0; //reset other init's to mitigate queuing of emotions
				init0 = 0;
			}
			
			if(cliffOne || cliffTwo || cliffThree){ // if robot is picked up
			    world_state = 5;
			    init2 = 0;
				init1 = 0; //reset other init's to mitigate queuing of emotions
				init0 = 0;
			}
			
        } 
		else if(world_state == 2){
			linear = 0.0; //ensure initially is stopped completely
			angular = 0.0;
			vel.linear.x = linear;
			vel.angular.z = angular;
			vel_pub.publish(vel);
			//turn ~45deg both ways twice
			angular = pi/2;
			seconds = time(&timer);
			while(difftime(time(&timer),seconds) < 1 && ros::ok()){
				vel.angular.z = angular;
				vel_pub.publish(vel);	
				waitKey(50);
			}
			angular = 0.0;
			vel.angular.z = angular;
			vel_pub.publish(vel);
			
			sleep(0.5); //stop and pause for a second
			for(int i = 0; i < 2; i++){
				waitKey(50);
				angular = -pi/2;
				seconds = time(&timer);
				while(difftime(time(&timer),seconds) < 2 && ros::ok()){
					vel.angular.z = angular;
					vel_pub.publish(vel);			
				}
				angular = 0.0;
				vel.angular.z = angular;
				vel_pub.publish(vel);
				sleep(1); //stop and pause for a second
				angular = pi/2;
				seconds = time(&timer);
				while(difftime(time(&timer),seconds) < 2 && ros::ok()){
					vel.angular.z = angular;
					vel_pub.publish(vel);			
				}
				angular = 0.0;
				vel.angular.z = angular;
				vel_pub.publish(vel);
				sleep(0.5); //stop and pause for a second
			}
			sc.playWave(path_to_sounds + "Dobby.wav"); //Play Dobby sound
			sleep(3);
			waitKey(50);
			sleep(3);
			//start turning and run away
			imshow("view", img2); 
			waitKey(50);
			sc.playWave(path_to_sounds + "Celebrate.wav"); //Play Celebration Song
			angular = pi/3;
			linear = 0.25;
			seconds = time(&timer);
			while(difftime(time(&timer),seconds) < 30 && ros::ok()){
				vel.linear.x = linear;
				vel.angular.z = angular;
				vel_pub.publish(vel);	
				waitKey(50);				
			}
			angular = 0.0;
			linear = 0.0;
			vel.linear.x = linear;
			vel.angular.z = angular;
			vel_pub.publish(vel);	
			ros::spinOnce();
			while(laserRange > 1 && ros::ok()){
				ros::spinOnce();
				printf("waiting for human...\n");
				sleep(0.5);
				waitKey(50);
			}
			world_state = 0;
			imshow("view", img0);
			waitKey(50);
		}
		else if(world_state == 3){
			imshow("view", img3); 
			waitKey(50);
			sc.playWave(path_to_sounds + "Dont_touch.wav"); //Play disgusted sound
			int sleepTime = 1.5; //adjust duration as necessary to account for above sound
			sleep(sleepTime);
			init2 = 0;
			while(difftime(time(&timer),startTime2) < (5+sleepTime) && ros::ok()){ 
				//if the bumpers are hit again, enter rage state
				waitKey(50);
				ros::spinOnce();
				if(bumperCentre || bumperLeft || bumperRight){
					world_state = 4;
					sc.playWave(path_to_sounds + "hey.wav"); // Yell Hey!!!
			        sleep(1);
					break; //immediately exit the loop
				}
				else
					world_state = 0;	
			}
			imshow("view", img0);
			waitKey(50);
		}
		else if(world_state == 4){
			//note person must stand still in front of TurtleBot. No additional checks to follow person.
			while(difftime(time(&timer),startTime2) < 7 && ros::ok()){ //add 2 seconds (5+2) to account for state 3
				if(bumperCentre || bumperLeft || bumperRight){
					imshow("view", img4);
					waitKey(50);
					sc.playWave(path_to_sounds + "cash_me_outside.wav"); //Play cash me outside
					sleep(4);
					waitKey(50);
					sleep(4);
					imshow("view", img4_1); 
					waitKey(50);
					sc.playWave(path_to_sounds + "Move_ludacris.wav"); //move out the way by Ludacris
					startTime3 = time(&timer);
					while(difftime(time(&timer), startTime3) < 20 &&ros::ok()){ //adjust for sound
						linear = 0.4;
						ros::spinOnce();
						while(!bumperCentre &&ros::ok()){
							ros::spinOnce();
							vel.linear.x = linear;
							vel_pub.publish(vel);			
						}
						seconds = time(&timer);
						linear = -0.4;
						while(difftime(time(&timer),seconds) < 1 && ros::ok()){
							vel.linear.x = linear;
							vel_pub.publish(vel);
						}
						linear = 0.0;
						vel.linear.x = linear;
						vel_pub.publish(vel);
						sleep(0.5);//short pause to stop
						waitKey(50);
					}
				}
				else
					break;
				world_state = 0;
				imshow("view", img0);
				waitKey(50);
			}    
		}
    	else if(world_state == 5){
			//cout << "here" << endl;
			imshow("view", img5);  
			waitKey(50);
			sc.playWave(path_to_sounds + "ooh_lala.wav"); //Play sounds
			sleep(2);
			sc.playWave(path_to_sounds + "lets_get_it_on.wav");
			waitKey(50);
			for(int i = 0; i < 6 && ros::ok ; i++){ //prevent image from graying out
				sleep(5);
				waitKey(50);
			}
			world_state = 0;
			imshow("view", img0);
			waitKey(50);
			//cout << "now here" << endl;
        }
	}
	return 0;

}
		    
