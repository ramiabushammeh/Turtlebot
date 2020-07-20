#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <eStop.h>

#include <cv.h>
#include <opencv2/highgui/highgui.hpp>
#include "highgui.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <stdio.h>
#include <cmath>

//Add Odometry, Angle Conversions, LaserScan, and Time
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/LaserScan.h>
#include <time.h>
#include <nav_msgs/OccupancyGrid.h>


using namespace std;
ros::Publisher vel_pub;

//linear (m/s) and angular velocity (rad/s)
double linear = 0.0;
double angular = 0.0;

//Odom variables
double posX, posY, yaw;
double pi = 3.1416;

//bumper variables
bool bumperLeft = 0, bumperCentre = 0, bumperRight = 0;

//laser variables
double laserRange = 10;
int laserSize = 0, laserOffset = 0, desiredAngle = 5;

//Random Roving Algorithm
int counter = 0; //to count the original direction
		 // 0=left 30, 1=left 30, 2=left 30, 3=right 120, 4=right 30, 5=right 30, 6=right 30, 7=right 60 (U-turn overall)
int sleeptime = 1;

void bumperCallback(const kobuki_msgs::BumperEvent msg){
	if(msg.bumper == 0)
		bumperLeft = !bumperLeft;
	else if(msg.bumper == 1)
		bumperCentre = !bumperCentre;
	else if(msg.bumper == 2)
		bumperRight = !bumperRight;	
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
	posX = msg->pose.pose.position.x;
	posY = msg->pose.pose.position.y;
	yaw = tf::getYaw(msg->pose.pose.orientation);
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

void occupancyCallback(const nav_msgs::OccupancyGrid& msg){
	//ROS_INFO("Width: %i, Height: %i, Resolution: %f, Origin: (%f, %f), Random Map: %d", msg.info.width, msg.info.height, msg.info.resolution, msg.info.origin.position.x, msg.info.origin.position.y, msg.data[msg.info.width*msg.info.height-1]);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    teleController eStop;

	ros::Subscriber bumper = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
    image_transport::Subscriber depth_sub = it.subscribe("camera/depth_registered/image_raw", 1, depthCallback);

	//subscribe to odometry and laserScan
	ros::Subscriber odom = nh.subscribe("odom", 1, odomCallback); 
	ros::Subscriber laser = nh.subscribe("scan", 1, laserCallback);
	ros::Subscriber map = nh.subscribe("map", 10, &occupancyCallback);

	vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop",1);

	geometry_msgs::Twist vel;

	time_t timer; //set up timer
	struct tm y2k = {0};
	double seconds;
	double lastSpun;
	double runTime = 0;

	int leftright; //decision maker
	srand (time(NULL)); //initialize random seed

	seconds = time(&timer); // spin once at start of contest
	while(difftime(time(&timer),seconds) < 8){
		//linear = 0.0;
		angular = -pi/3;
		//vel.linear.x = linear;
		vel.angular.z = angular;
		vel_pub.publish(vel);
	}
	angular = 0.0;
	vel.angular.z = angular;
	vel_pub.publish(vel);
	lastSpun = time(&timer);

	while(ros::ok()){
		ros::spinOnce();
        //.....**E-STOP DO NOT TOUCH**.......
        eStop.block();
        //...................................

    if(difftime(time(&timer),runTime) < 300){
      
    	//(1) Random Rovering Algorithm
    	//Random Roving Algorithm
    	if(laserRange > 0.8 && !bumperRight && !bumperCentre && !bumperLeft && (time(&timer))%30 > 1){
    		linear = 0.25; //max speed when far from walls
    	}
    	else if(laserRange <= 0.8 && laserRange > 0.6 && !bumperRight && !bumperCentre && !bumperLeft  && (time(&timer))%30 > 1){
    		linear = 0.1; //slow down upon approaching a wall
    	}
    	else if(laserRange <= 0.6 && !bumperRight && !bumperCentre && !bumperLeft  && (time(&timer))%30 > 1){
    		linear = 0.0;
    		vel.linear.x = linear;
    		vel_pub.publish(vel); //stop immediately
    		leftright = rand() % 2 + 1; //random number between 1 (left), and 2 (right)
    		if(leftright == 1){
    			while(laserRange <= 0.8){
    				if(counter == 0 || counter == 1 || counter == 2){
    					angular = pi/6;
					ROS_INFO("%d", counter);
    				}
    				else if(counter == 3 || counter == 4 || counter == 5 || counter == 6 || counter == 7){
    					angular = -pi/6;
    				}
    				if(counter == 0 || counter == 1 || counter == 2 || counter == 4 || counter == 5 || counter == 6){
    					sleeptime = 1; //30deg turns
    				}
    				else if(counter == 3){
    					sleeptime = 4; //120deg turns
    				}
    				else if(counter == 7){
    					sleeptime = 3; //90deg turns
    				}
    				ROS_INFO("1st while CCW");
    				seconds = time(&timer);
    				while(difftime(time(&timer),seconds) < sleeptime){
    					vel.angular.z = angular;
    					vel_pub.publish(vel); //rotate CCW 30deg by setting angular vel. then sleep 1s
    					ROS_INFO("2nd while CCW");
    				}			
    				counter = counter + 1;
    				ros::spinOnce(); //update Callbacks
    			}
    		}
    		else {
    			while(laserRange <= 0.8){
    				if(counter == 0 || counter == 1 || counter == 2){
    					angular = -pi/6;
					ROS_INFO("%d", counter);
    				}
    				else if(counter == 3 || counter == 4 || counter == 5 || counter == 6 || counter == 7){
    					angular = pi/6;
    				}
    				if(counter == 0 || counter == 1 || counter == 2 || counter == 4 || counter == 5 || counter == 6){
    					sleeptime = 1; //30deg turns
    				}
    				else if(counter == 3){
    					sleeptime = 4; //120deg turns
    				}
    				else if(counter == 7){
    					sleeptime = 3; //90deg turns
    				}
    				ROS_INFO("1st while CW");
    				seconds = time(&timer);
    				while(difftime(time(&timer),seconds) < sleeptime){
    					vel.angular.z = angular;
    					vel_pub.publish(vel); //rotate CW 30deg by setting angular vel. then sleep 1s
    					ROS_INFO("2nd while CW");
    				}			
    				counter = counter + 1;
    				ros::spinOnce(); //update Callbacks
    			}
    		}
    		counter = 0;
    		angular = 0.0; //once while loop exits, set angular to 0 to prevent further rotating
    		vel.angular.z = angular;
    		vel_pub.publish(vel); 
    	}

    	//bumper conditions
    	if(!bumperCentre && !bumperLeft && !bumperRight){ //for no bumper hits
    	}	
    	else if(bumperCentre && !bumperLeft && !bumperRight){ //front bumper only
    		linear = 0.0; //stop immediately
    		angular = 0.0; 
    		vel.linear.x = linear;
    		vel.angular.z = angular;
    		vel_pub.publish(vel);

    		linear = -0.1; //move back 0.6m (to start turning algorithm)
    		seconds = time(&timer);
    		while(difftime(time(&timer),seconds) < 6){
    			vel.linear.x = linear;
    			vel_pub.publish(vel);
    			//ROS_INFO("front only");
    		}
    		linear = 0.0; //stop movement
    		vel.linear.x = linear;
    		vel_pub.publish(vel);
    	}
    	else if(!bumperCentre && bumperLeft && !bumperRight){ //left bumper only
    		linear = 0.0; //stop immediately
    		angular = 0.0; 
    		vel.linear.x = linear;
    		vel.angular.z = angular;
    		vel_pub.publish(vel);
    		linear = -0.1; //move back 0.2m, turn CW 30deg
    		seconds = time(&timer);
    		while(difftime(time(&timer),seconds) < 2){
    			vel.linear.x = linear;
    			vel_pub.publish(vel);
    			//ROS_INFO("left only back");
    		}
    		linear = 0.0;
    		angular = -pi/6;
    		seconds = time(&timer);
    		while(difftime(time(&timer),seconds) < 1){
    			vel.angular.z = angular;
    			vel_pub.publish(vel);
    			//ROS_INFO("left-turn only"); 
    		}
    		angular = 0.0; //stop rotation
    		vel.angular.z = angular;
    		vel_pub.publish(vel);
    	}
    	else if(!bumperCentre && !bumperLeft && bumperRight){ //right bumper only
    		linear = 0.0; //stop immediately
    		angular = 0.0; 
    		vel.linear.x = linear;
    		vel.angular.z = angular;
    		vel_pub.publish(vel);
    		linear = -0.1; //move back 0.2m, turn CCW 30deg
    		seconds = time(&timer);
    		while(difftime(time(&timer),seconds) < 2){
    			vel.linear.x = linear;
    			vel_pub.publish(vel);
    			//ROS_INFO("right only");
    		}
    		linear = 0.0;
    		angular = pi/6;
    		seconds = time(&timer);
    		while(difftime(time(&timer),seconds) < 1){
    			vel.angular.z = angular;
    			vel_pub.publish(vel);
    			//ROS_INFO("right-turn only");
    		}
    		angular = 0.0; //stop rotation
    		vel.angular.z = angular;
    		vel_pub.publish(vel);
    	}	
    	else{ //for any other potential scenario --> keep rotating and look for a way out
    		angular = pi/6;		
    		while(laserRange < 0.8){
    			ros::spinOnce();
    			vel.angular.z = angular;
    			vel_pub.publish(vel);
    			//ROS_INFO("last bumper only");
    		}
    		angular = 0.0; //stop rotating once a way out is found
    		vel.angular.z = angular;
    		vel_pub.publish(vel);
    	}
    }
    
    //end of (1) Random Roving Algorithm

	
	//(2) Start of the Non-stop Roving Algorithm
	
  else{
    
  	if(laserRange > 0.9 && !bumperRight && !bumperCentre && !bumperLeft){
  		linear = 0.25; //max speed when far from walls
  	}
  	else if(laserRange <= 0.9 && !bumperRight && !bumperCentre && !bumperLeft){
  		seconds = time(&timer);
  		linear = 0.075; //slow down upon approaching a wall
  		vel.linear.x = linear;
  		leftright = rand() % 2 + 1; //random number between 1 (left), and 2 (right)
  		if(leftright == 1){
  			while((difftime(time(&timer),seconds) < 1.5) && laserRange <= 0.9 && laserRange >= 0.6 && !bumperRight && !bumperCentre && !bumperLeft){ //turn right up to 45deg 
  				ros::spinOnce();
  				angular = -pi/4;
  				vel.angular.z = angular;
  				vel_pub.publish(vel);
  			}
  			angular = 0.0; //stop spinning momentarily
  			vel.angular.z = angular;
  			vel_pub.publish(vel);
  		
  			while(laserRange < 2 && !bumperRight && !bumperCentre && !bumperLeft){ //keep turning left if first 45deg CW doesn't have any space
  				ros::spinOnce();
  				angular = pi/4;
  				vel.angular.z = angular;
  				vel_pub.publish(vel);
  			}
  			angular = 0.0; //stop spinning once it's free!!
  			vel.angular.z = angular;
  			vel_pub.publish(vel);
  		}
  		else{
  			while((difftime(time(&timer),seconds) < 1.5) && laserRange <= 0.9 && laserRange >= 0.6 && !bumperRight && !bumperCentre && !bumperLeft){ //turn right up to 45deg 
  				ros::spinOnce();
  				angular = pi/4;
  				vel.angular.z = angular;
  				vel_pub.publish(vel);
  			}
  			angular = 0.0; //stop spinning momentarily
  			vel.angular.z = angular;
  			vel_pub.publish(vel);
  		
  			while(laserRange < 2 && !bumperRight && !bumperCentre && !bumperLeft){ //keep turning left if first 45deg CW doesn't have any space
  				ros::spinOnce();
  				angular = -pi/4;
  				vel.angular.z = angular;
  				vel_pub.publish(vel);
  			}
  			angular = 0.0; //stop spinning once it's free!!
  			vel.angular.z = angular;
  			vel_pub.publish(vel);
  		}
  	}
  		//bumper conditions
  	if(!bumperCentre && !bumperLeft && !bumperRight){ //for no bumper hits 
  	}	
  	else if(bumperCentre && !bumperLeft && !bumperRight){ //front bumper only
  		linear = 0.0; //stop immediately
  		angular = 0.0; 
  		vel.linear.x = linear;
  		vel.angular.z = angular;
  		vel_pub.publish(vel);

  		linear = -0.1; //move back 0.6m (to start turning algorithm)
  		seconds = time(&timer);
  		while(difftime(time(&timer),seconds) < 6){
  			vel.linear.x = linear;
  			vel_pub.publish(vel);
  			//ROS_INFO("front only");
  		}
  		linear = 0.0; //stop movement
  		vel.linear.x = linear;
  		vel_pub.publish(vel);
  	}
  	else if(!bumperCentre && bumperLeft && !bumperRight){ //left bumper only
  		linear = 0.0; //stop immediately
  		angular = 0.0; 
  		vel.linear.x = linear;
  		vel.angular.z = angular;
  		vel_pub.publish(vel);
  		linear = -0.1; //move back 0.2m, turn CW 30deg
  		seconds = time(&timer);
  		while(difftime(time(&timer),seconds) < 2){
  			vel.linear.x = linear;
  			vel_pub.publish(vel);
  			//ROS_INFO("left only back");
  		}
  		linear = 0.0;
  		angular = -pi/6;
  		seconds = time(&timer);
  		while(difftime(time(&timer),seconds) < 1){
  			vel.angular.z = angular;
  			vel_pub.publish(vel);
  			//ROS_INFO("left -turn only"); 
  		}
  		angular = 0.0; //stop rotation
  		vel.angular.z = angular;
  		vel_pub.publish(vel);
  	}
  	else if(!bumperCentre && !bumperLeft && bumperRight){ //right bumper only
  		linear = 0.0; //stop immediately
  		angular = 0.0; 
  		vel.linear.x = linear;
  		vel.angular.z = angular;
  		vel_pub.publish(vel);
  		linear = -0.1; //move back 0.2m, turn CCW 30deg
  		seconds = time(&timer);
  		while(difftime(time(&timer),seconds) < 2){
  			vel.linear.x = linear;
  			vel_pub.publish(vel);
  			//ROS_INFO("right only");
  		}
  		linear = 0.0;
  		angular = pi/6;
  		seconds = time(&timer);
  		while(difftime(time(&timer),seconds) < 1){
  			vel.angular.z = angular;
  			vel_pub.publish(vel);
  			//ROS_INFO("right -turn only");
  		}
  		angular = 0.0; //stop rotation
  		vel.angular.z = angular;
  		vel_pub.publish(vel);
  	}	
  	else{ //for any other potential scenario --> keep rotating and look for a way out
  		angular = pi/6;		
  		while(laserRange < 0.8){
  			ros::spinOnce();
  			vel.angular.z = angular;
  			vel_pub.publish(vel);
  			//ROS_INFO("last bumper only");
  		}
  		angular = 0.0; //stop rotating once a way out is found
  		vel.angular.z = angular;
  		vel_pub.publish(vel);
  	}
  }

  //360 spin, time may need tweaking
	if(difftime(time(&timer),lastSpun) > 60){
		seconds = time(&timer);
		while(difftime(time(&timer),seconds) < 8){
			ros::spinOnce();
			linear = 0.0;
			angular = -pi/3;
			vel.linear.x = linear;
			vel.angular.z = angular;
			vel_pub.publish(vel);
			ROS_INFO("360 loop");
		}
		angular = 0;
		vel.angular.z = angular;
		vel_pub.publish(vel);
		lastSpun = time(&timer); //record new last spun time
	}
  

	// leave below
	vel.angular.z = angular;
	vel.linear.x = linear;
	vel_pub.publish(vel);

	}

	return 0;
}
