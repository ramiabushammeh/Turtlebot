#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "nav_header.h"
#include <eStop.h>
#include <math.h>
#include <time.h>
#include <kobuki_msgs/BumperEvent.h>

using namespace std;
ros::Publisher vel_pub;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

float x=0;
float y=0;
float phi=0;

time_t timer; //set up timer
struct tm y2k = {0};
double seconds;

double pi = 3.14159;

//bumper variables
bool bumperLeft = 0, bumperCentre = 0, bumperRight = 0;

void poseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg){
	phi = tf::getYaw(msg.pose.pose.orientation);
    x = msg.pose.pose.position.x;
    y = msg.pose.pose.position.y;
}

void bumperCallback(const kobuki_msgs::BumperEvent msg){
	if(msg.bumper == 0)
		bumperLeft = !bumperLeft;
	else if(msg.bumper == 1)
		bumperCentre = !bumperCentre;
	else if(msg.bumper == 2)
		bumperRight = !bumperRight;	
}

//-------------------------move robot function---------------
bool moveToGoal(float xGoal, float yGoal, float phiGoal){

	//define a client for to send goal requests to the move_base server through a SimpleActionClient
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

	//wait for the action server to come up
	while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
	}

	move_base_msgs::MoveBaseGoal goal;

	//set up the frame parameters
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();

	/* moving towards the goal*/
    geometry_msgs::Quaternion phi = tf::createQuaternionMsgFromYaw(phiGoal);

	goal.target_pose.pose.position.x =  xGoal;
	goal.target_pose.pose.position.y =  yGoal;
	goal.target_pose.pose.position.z =  0.0;
	goal.target_pose.pose.orientation.x = 0.0;
	goal.target_pose.pose.orientation.y = 0.0;
	goal.target_pose.pose.orientation.z = phi.z;
	goal.target_pose.pose.orientation.w = phi.w;

	ROS_INFO("Sending goal location ...");
	ac.sendGoal(goal);

	ac.waitForResult();

	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
		ROS_INFO("You have reached the destination");
		return true;
	}
	else{
		ROS_INFO("The robot failed to reach the destination");
		return false;
	}

}

int main(int argc, char** argv){
	cv::initModule_nonfree();
	ros::init(argc, argv, "map_navigation_node");
	ros::NodeHandle n;
	ros::spinOnce();
    teleController eStop;

	ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, poseCallback);
	
	vector<vector<float> > coord;
	vector<vector<float> > orient;
	std::vector<cv::Mat> imgs_track;	
	init(coord, orient, imgs_track);

    while(ros::ok()){
        //.....**E-STOP DO NOT TOUCH**.......
        eStop.block();
        //...................................

        //fill with your code

	ros::NodeHandle nh;
	vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop",1);
	geometry_msgs::Twist vel;

	while (x == 0 && y == 0 && phi == 0){       //spinOnce to call latest position and orientation 
		ros::spinOnce();
	}
	
	float initialPos[3] = {x,y,phi};		//save starting position
	//coord.push_back(vector<float>(initialPos, initialPos + sizeof initialPos / sizeof initialPos[0]));
	//float initialOrient[2] = {phi};		//save starting orientation
	//orient.push_back(vector<float>(initialOrient, initialOrient + sizeof initialOrient / sizeof initialOrient[0]));

	//double oldShort = pow(pow(coord[0][0] - initialPos[0][0],2) + pow(coord[0][1] - initialPos[0][1],2), 0.5);
	int moveTo = 0;
	double newShort = 0.0;
	int movement = 0;
	float finished[5] = {0,0,0,0,0};
	double looking = 0.0;
	float xMove[5] = {0,0,0,0,0};
	float yMove[5] = {0,0,0,0,0};
	float phiMove[5] = {0,0,0,0,0};
	double linear = 0.0;
	double angular = 0.0;
	double tim = 0.4;
	int success = 0;

	
	for(int i = 0; i < 5; i++){						// make go-to positions tim(distance) in front of the pictures
		if(orient[i][0] >= 0 && orient[i][0] < pi/2){
			xMove[i] = coord[i][0] + tim*cos(orient[i][0]);
			yMove[i] = coord[i][1] + tim*sin(orient[i][0]);
		}
		else if(orient[i][0] >= pi/2 && orient[i][0] < pi){
			xMove[i] = coord[i][0] - tim*cos(pi - orient[i][0]);
			yMove[i] = coord[i][1] + tim*sin(pi - orient[i][0]);
		}
		else if(orient[i][0] >= -pi && orient[i][0] < -pi/2){
			xMove[i] = coord[i][0] - tim*cos(-pi - orient[i][0]);
			yMove[i] = coord[i][1] + tim*sin(-pi - orient[i][0]);
		}
		else if(orient[i][0] >= -pi/2 && orient[i][0] < 0){
			xMove[i] = coord[i][0] + tim*cos(orient[i][0]);
			yMove[i] = coord[i][1] + tim*sin(orient[i][0]);
		}
		else{
		}

		if(orient[i][0] >= 0){			// ensure that the angle is within -pi to +pi
			phiMove[i] = orient[i][0] - pi;
		} 
		else{
			phiMove[i] = orient[i][0] + pi;
		}
	}

	ROS_INFO ("%f %f", initialPos[0], initialPos[1]);              //print initial position, desired positions/orientation
	ROS_INFO ("%f %f %f %f %f", xMove[0], xMove[1], xMove[2], xMove[3], xMove[4]);
	ROS_INFO ("%f %f %f %f %f", yMove[0], yMove[1], yMove[2], yMove[3], yMove[4]);
	ROS_INFO ("%f %f %f %f %f", phiMove[0], phiMove[1], phiMove[2], phiMove[3], phiMove[4]);
	ROS_INFO ("%f %f %f %f %f", orient[0][0], orient[1][0], orient[2][0], orient[3][0], orient[4][0]);
	//ROS_INFO ("%f %f %f %f %f", coord[0][0], coord[1][0], coord[2][0], coord[3][0], coord[4][0]);
	//ROS_INFO ("%f %f %f %f %f", coord[0][1], coord[1][1], coord[2][1], coord[3][1], coord[4][1]);


	while(movement < 5){
		double currentX = x;
		double currentY = y;
		double oldShort = 100;

		for(int i = 0; i < 5; i++){         // calculate shortest distance
			newShort = pow(pow(xMove[i] - currentX,2) + pow(yMove[i] - currentY,2), 0.5);
			//newShort = pow(pow(coord[i][0] - currentX,2) + pow(coord[i][1] - currentY,2), 0.5);
			if(newShort < oldShort && finished[i] == 0){
				oldShort = newShort;
				moveTo = i;
			}
		}
		
		/*if(orient[moveTo][0] >= 0){			// ensure that the angle is within -pi to +pi
			looking = orient[moveTo][0] - pi;
		} 
		else{
			looking = orient[moveTo][0] + pi;
		}*/


		success = moveToGoal(xMove[moveTo], yMove[moveTo], phiMove[moveTo]); // move to position
		//moveToGoal(coord[moveTo][0], coord[moveTo][1], looking); // move to position
		if (success == 1){		
			movement++;
	/*		linear = 0.0;
			angular = 0.0;
			vel.linear.x = linear;
			vel.angular.z = angular;
			vel_pub.publish(vel);		
			linear = 0.1;
			angular = 0.0;
			vel.linear.x = linear;
			vel.angular.z = angular;
			vel_pub.publish(vel);
			if(bumperCentre && !bumperLeft && !bumperRight){ //front bumper only
		    		linear = 0.0; //stop immediately
		    		angular = 0.0; 
		    		vel.linear.x = linear;
		    		vel.angular.z = angular;
		    		vel_pub.publish(vel);

		    		linear = -0.1; //move back 0.6m (to start turning algorithm)
		    		seconds = time(&timer);
		    		while(difftime(time(&timer),seconds) < 2){
		    			vel.linear.x = linear;
		    			vel_pub.publish(vel);
	    			}
	    		linear = 0.0; //stop movement
	    		vel.linear.x = linear;
	    		vel_pub.publish(vel);*/
			finished[moveTo] = 1;	// set location to finished
			ros::spinOnce();
			ROS_INFO("Current orientation: %f Calculated orientation: %f", phi, phiMove[moveTo]);
			ROS_INFO("%f %f %f %f %f", finished[0], finished[1], finished[2], finished[3], finished[4]);

			seconds = time(&timer);	// take current time 
			while((difftime(time(&timer),seconds) < 5)){	// let robot stabilize before taking picture
				linear = 0.0;
				vel.linear.x = linear;
				vel_pub.publish(vel);
			}
		
			int foundPic = findPic(nh, imgs_track);
			ROS_INFO("Coordinate is: %f, %f \nPicture is: %i", coord[moveTo][0], coord[moveTo][1], foundPic);
		}
		else if (success == 0) {
			seconds = time(&timer);	// take current time 
			while((difftime(time(&timer),seconds) < 2)){	// move robot back 0.5m
				linear = -0.25;
				vel.linear.x = linear;
				vel_pub.publish(vel);
			}
			linear = 0.0;
			vel.linear.x = linear;
			vel_pub.publish(vel);
		}
	}	

	moveToGoal(initialPos[0], initialPos[1], initialPos[2]);
	ROS_INFO("We are the best!!!!!");
	int completed = 1;

	while(completed = 1){
		linear = 0.0;
		angular = 0.0;
		vel.linear.x = linear;
		vel.angular.z = angular;
		vel_pub.publish(vel);
	}
	

	
	//ROS_INFO("%f",coord[1][0]-1);

	//moveToGoal(0, 0, 0.0);
	//moveToGoal(coord[0][0], coord[0][1], orient[0][0]);
	//moveToGoal(coord[1][0], coord[1][1], orient[1][0]);
	//moveToGoal(coord[2][0], coord[2][1], orient[2][0]);
	//moveToGoal(coord[3][0], coord[3][1], orient[3][0]);	
	//moveToGoal(coord[4][0], coord[4][1], orient[4][0]);
	//moveToGoal(initialPos[0], initialPos[1], initialPos[2]);

    }

	return 00;
}
