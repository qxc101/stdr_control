#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h> 
#include <std_msgs/Bool.h> 
#include <stdlib.h>
#include <string>
#include "std_msgs/String.h"

const double MIN_SAFE_DISTANCE = 0.5; 
float ping_dist_in_front_=3.0; 
int ping_index_= -1; 
double angle_min_=0.0;
double angle_max_=0.0;
double angle_increment_=0.0;
double range_min_ = 0.0;
double range_max_ = 0.0;
bool laser_alarm_=false;
double upper = 0.0; 
double lower = 0.0; 
float dist = 3.0; 

void laserCallback(const sensor_msgs::LaserScan& laser_scan) {
    if (ping_index_<0)  {
        angle_min_ = laser_scan.angle_min;
        angle_max_ = laser_scan.angle_max;
        angle_increment_ = laser_scan.angle_increment;
        range_min_ = laser_scan.range_min;
        range_max_ = laser_scan.range_max;
        ping_index_ = (int) ((0.0 -angle_min_)/angle_increment_); 
    	}

   	ping_dist_in_front_ = laser_scan.ranges[ping_index_];
  	upper = (int) ping_index_ + 1.5 / angle_increment_; 
   	lower = (int) ping_index_ - 1.5 / angle_increment_;
   	laser_alarm_=false; 
	for (int i = lower; i < upper; i++) { 
     	dist = laser_scan.ranges[i]; 
     	if (dist < MIN_SAFE_DISTANCE) { 
  	laser_alarm_=true; 
     	}
   	}
}


int main(int argc, char **argv) {
	std_msgs::String node_name; 
	std::stringstream nts;
	node_name.data = nts.str();
	std_msgs::String laser_topic_name ; 
	std_msgs::String vel_topic_name ; 
	std::stringstream vts ;
	std::stringstream lts ;
	vts << "/";
	lts << "/";
	
	int opt;
	while ((opt = getopt(argc, (argv), "n:")) != -1) {
	switch (opt) {
	case 'n':
	vts << optarg;
	lts << optarg;
	nts << optarg;
	break;
	default:
	printf("The -%c is not a recognized parameter\n", opt);
	break;
	}
	}
	vts << "/cmd_vel";
	lts << "/laser_1";
	vel_topic_name.data = vts.str();
	laser_topic_name.data = lts.str();
	node_name.data = nts.str();
	std::string n_name (node_name.data.c_str());
	std::string v_topic (vel_topic_name.data.c_str());
	std::string l_topic (laser_topic_name.data.c_str());
	if(n_name.empty()){
		n_name = "robot0";
		v_topic = "/robot0/cmd_vel";	
		l_topic = "/robot0/laser_1";
	}

	ros::init(argc, argv, n_name);
	ros::NodeHandle n; 
	printf("%s", vel_topic_name.data.c_str());
	printf("%s", laser_topic_name.data.c_str());
	ROS_INFO("subscirbed to = %s",vel_topic_name.data.c_str());
	ROS_INFO("subscirbed to = %s",laser_topic_name.data.c_str());
	ros::Publisher twist_commander = n.advertise<geometry_msgs::Twist>(v_topic, 1);
	ros::Subscriber sub = n.subscribe ( l_topic, 1 , laserCallback );
	double sample_dt = 0.01; 
	double speed = 0.5; 
	double yaw_rate = 0.5; 
	double time_3_sec = 3.0; 
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

	while(ros::ok()) { 
        twist_cmd.angular.z=0.0; 
        twist_cmd.linear.x=speed; 
        while(!laser_alarm_) { 
          twist_commander.publish(twist_cmd);
          timer+=sample_dt;
          ros::spinOnce();
          loop_timer.sleep();
          }
        
        twist_cmd.linear.x=0.0; 
        twist_cmd.angular.z=yaw_rate; 
        timer=0.0; 
        while(laser_alarm_) {
          twist_commander.publish(twist_cmd);
          timer+=sample_dt;
          ros::spinOnce();
          loop_timer.sleep();
          }
	} 
	
}
