#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h> //Including the Float32 class from std_msgs
#include <std_msgs/Bool.h> // boolean message 
#include <stdlib.h>
#include <string>
#include "std_msgs/String.h"

const double MIN_SAFE_DISTANCE = 0.5; // set alarm if anything is within 0.5m of the front of robot

// these values to be set within the laser callback
float ping_dist_in_front_=3.0; // global var to hold length of a SINGLE LIDAR ping--in front
int ping_index_= -1; // NOT real; callback will have to find this
double angle_min_=0.0;
double angle_max_=0.0;
double angle_increment_=0.0;
double range_min_ = 0.0;
double range_max_ = 0.0;
bool laser_alarm_=false;

double upper = 0.0; // upper bound
double lower = 0.0; // lower bound
float dist = 3.0; // distance scanned by pings

void laserCallback(const sensor_msgs::LaserScan& laser_scan) {
    if (ping_index_<0)  {
        //for first message received, set up the desired index of LIDAR range to eval
        angle_min_ = laser_scan.angle_min;
        angle_max_ = laser_scan.angle_max;
        angle_increment_ = laser_scan.angle_increment;
        range_min_ = laser_scan.range_min;
        range_max_ = laser_scan.range_max;
        // what is the index of the ping that is straight ahead?
        // BETTER would be to use transforms, which would reference how the LIDAR is mounted;
        // but this will do for simple illustration
        ping_index_ = (int) ((0.0 -angle_min_)/angle_increment_);
        ROS_INFO("LIDAR setup: ping_index = %d",ping_index_);
        
    }

   ping_dist_in_front_ = laser_scan.ranges[ping_index_];
   ROS_INFO("ping dist in front = %f",ping_dist_in_front_);

   // scan up to 1.5 rad (around 90 degrees) on left and right side, 
   // and calculate the upper and lower bound ping index
   upper = (int) ping_index_ + 1.5 / angle_increment_; 
   lower = (int) ping_index_ - 1.5 / angle_increment_;

   laser_alarm_=false; // reset the alarm eveytime a new call comes in


   for (int i = lower; i < upper; i++) { // loop from lower bound ping to upper bound ping
     
     dist = laser_scan.ranges[i]; // calculate the wall distance at current ping
     if (dist < MIN_SAFE_DISTANCE) { // alarm if current ping distance is smaller than safety
         ROS_WARN("DANGER, WILL ROBINSON!!");
         laser_alarm_=true; 
         
     }
   }

}


int main(int argc, char **argv) {
	ros::init(argc, argv, "stdr_commander");
	ros::NodeHandle n; // two lines to create a publisher object that can talk to ROS
	std_msgs::String topic_name ; 
	std::stringstream ss ;
	ss << "/robot0/laser_1";
	topic_name.data = ss.str();

	int opt;
	while ((opt = getopt(argc, (argv), "n:")) != -1) {
	switch (opt) {
	case 'n':
	topic_name.data = optarg;
	break;
	default:
	printf("The -%c is not a recognized parameter\n", opt);
	break;
	}
	}
	ros::Publisher twist_commander = n.advertise<geometry_msgs::Twist>( "/robot0/cmd_vel", 1);
	ros::Subscriber sub = n.subscribe ( topic_name.data.c_str(), 1 , laserCallback );
	double sample_dt = 0.01; // specify a sample period of 10 ms
	double speed = 0.5; // 1 m / s speed command
	double yaw_rate = 0.5; // 0.5 rad / sec yaw rate command
	double time_3_sec = 3.0; // should move 3 meters or 1.5 rad in 3 seconds
	geometry_msgs::Twist twist_cmd; 
	twist_cmd.linear.x =0.0;
	twist_cmd.linear.y =0.0;
	twist_cmd.linear.z =0.0;
	twist_cmd.angular.x =0.0;
	twist_cmd.angular.y =0.0;
	twist_cmd.angular.z =0.0;
	ros::Rate loop_timer(1/sample_dt);
	double timer=0.0;
	for(int i=0; i<10; i++){
	twist_commander.publish(twist_cmd) ;
	loop_timer.sleep() ;
	}

	while(ros::ok()) { // do forever
        twist_cmd.angular.z=0.0; // do not spin 
        twist_cmd.linear.x=speed; //command to move forward
        while(!laser_alarm_) { // keep moving forward until get an alarm signal
          twist_commander.publish(twist_cmd);
          timer+=sample_dt;
          ros::spinOnce();
          loop_timer.sleep();
          }
        //here if got an alarm; turn CCW until alarm clears
        twist_cmd.linear.x=0.0; //stop moving forward
        twist_cmd.angular.z=yaw_rate; //and start spinning in place
        timer=0.0; //reset the timer
        while(laser_alarm_) {
          twist_commander.publish(twist_cmd);
          timer+=sample_dt;
          ros::spinOnce();
          loop_timer.sleep();
          }
	} 
	
}
