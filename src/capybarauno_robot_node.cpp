
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include "capybarauno_robot.h"
using namespace std;

volatile double tv = 0, rv = 0;



void commandVelCallback(const geometry_msgs::TwistConstPtr twist){
	tv = twist->linear.x;
	rv = twist->angular.z;
}



int main(int argc, char** argv) {
	CapybaraunoConfig config;
	
	std::string comm_port;
	std::string comm_ascii;
	std::string n_encoder;
	std::string wheel_radius;
	std::string wheel_distance;
	std::string odom_topic;
	std::string odom_frame_id;
	std::string command_vel_topic;
	std::string n_pub_updates;
	std::string publish_tf;
	std::string debug;
	std::string trans_multiplier;
	std::string rot_multiplier;
	std::string boost_multiplier;
	
	// init ROS
	ros::init(argc, argv, "capybarauno_node");
	ros::NodeHandle nh("~");
	
	// read parameters
	nh.param("comm_port", comm_port, std::string("/dev/ttyACM0"));
	nh.param("comm_ascii", comm_ascii, std::string("0"));
	nh.param("n_encoder", n_encoder, std::string("3600"));
	nh.param("wheel_radius", wheel_radius, std::string("0.1"));
	nh.param("wheel_distance", wheel_distance, std::string("0.4"));
	nh.param("odom_topic", odom_topic, std::string("/odom"));
	nh.param("odom_frame_id", odom_frame_id, std::string("/odom"));
	nh.param("n_pub_updates", n_pub_updates, std::string("50"));
	nh.param("publish_tf", publish_tf, std::string("0"));
	nh.param("debug", debug, std::string("0"));
	nh.param("trans_multiplier", trans_multiplier, std::string("1"));
	nh.param("rot_multiplier", rot_multiplier, std::string("1"));
	nh.param("boost_multiplier", boost_multiplier, std::string("2"));
	
	nh.param("command_vel_topic", command_vel_topic, std::string("/cmd_vel"));
	
	// set config object
	config.MoveConfig::comm_port_ = comm_port;
	config.OdomConfig::comm_port_ = comm_port;
	config.MoveConfig::comm_ascii_ = atoi( comm_ascii.c_str() );
	config.OdomConfig::comm_ascii_ = atoi( comm_ascii.c_str() );
	config.n_encoder_ = atoi( n_encoder.c_str() );
	config.wheel_radius_ = atoi( wheel_radius.c_str() );
	config.wheel_distance_ = atoi( wheel_distance.c_str() );
	config.odom_topic_ = odom_topic;
	config.n_pub_updates_ = atoi( n_pub_updates.c_str() );
	config.publish_tf_ = atoi( publish_tf.c_str() );
	config.MoveConfig::debug_ = atoi( debug.c_str() );
	config.OdomConfig::debug_ = atoi( debug.c_str() );
	config.trans_multiplier_ = atoi( trans_multiplier.c_str() );
	config.rot_multiplier_ = atoi( rot_multiplier.c_str() );
	config.boost_multiplier_ = atoi( boost_multiplier.c_str() );
	
	config.printParameters();
	printf( "  %s %s\n", "odom_frame_id", odom_frame_id.c_str() );
	printf( "  %s %s\n", "command_vel_topic", command_vel_topic.c_str() );
	
	// subscribe and advertise
	ros::Subscriber command_vel_subscriber = nh.subscribe<geometry_msgs::TwistConstPtr>(command_vel_topic, 1, &commandVelCallback);
	ros::Publisher odom_publisher = nh.advertise<nav_msgs::Odometry>(odom_topic, 1);
	
	// create robot object & init
	CapybaraunoRobot robot( config );
	robot.init();
	
	nav_msgs::Odometry odom;
	odom.header.frame_id = odom_frame_id;
	int seq = 0;
	int _packet_count = 0;
	
	while(ros::ok()){
		ros::spinOnce();
		robot.spinOnce();
		double v = tv;
		double r = rv;
		robot.setSpeed( v, r );
		// send the odometry
		double x,y,theta;
		robot.getOdometry(x,y,theta);
		odom.header.seq = seq;		// ignored/overwritten by ROS
		odom.header.stamp = ros::Time::now();
		odom.pose.pose.position.x = x;
		odom.pose.pose.position.y = y;
		odom.pose.pose.position.z = 0;
		double s = sin (theta/2);
		double c = cos (theta/2);
		odom.pose.pose.orientation.x = 0;
		odom.pose.pose.orientation.y = 0;
		odom.pose.pose.orientation.z = s;
		odom.pose.pose.orientation.w = c;
		odom_publisher.publish(odom);
		seq++;
	}
	//robot.disconnect();
}
