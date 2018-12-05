/*
 *
 */
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>

using namespace std;

#define GPS_change 100

const string pass = "/home/amsl/toyota/dataset11/car[5]/1/vision/pose/";

class Pose{
	private:
		ros::NodeHandle n;
		ros::Publisher pose_pub;

		tf::TransformBroadcaster br;
		tf::Transform transform;

		nav_msgs::Odometry pose_result;

		bool flag;
	public:
		Pose(ros::NodeHandle n);
		
		void ndt_result(nav_msgs::Odometry msg);
		void gps_result(nav_msgs::Odometry msg);
		void tf_broad(nav_msgs::Odometry msg);
		void process();

};

Pose::Pose(ros::NodeHandle n) :
	flag(false)
{
	map_matching_pub = n.advertise<nav_msgs::Odometry>("/mapmatching_result", 10);

}


int main (int argc, char** argv){
	ros::init(argc,argv,"tf_map2gps");
	ros::NodeHandle n;

	Pose pose(n);
	ros::Rate loop_rate(10);

	cout <<"----- tf_pub ok ------" <<endl;

	while(ros::ok()){
		pose.process();

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

void 
Pose::input_odom(){
	

}


void 
Pose::tf_broad(nav_msgs::Odometry msg){


	transform.setOrigin( tf::Vector3(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z) );
	tf::Quaternion q;
	q.setRPY(0, 0, msg.pose.pose.orientation.z);

	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, msg.header.stamp , "/map", "/matching_base_link"));

}

void 
Pose::ndt_result(nav_msgs::Odometry msg){

	nav_msgs::Odometry lcl_ndt;
	lcl_ndt.header.frame_id = "/map";
	lcl_ndt.child_frame_id = "/matching_base_link";
	lcl_ndt.header.stamp = ros::Time::now();
	lcl_ndt.pose.pose.position.x = msg.pose.pose.position.x;
	lcl_ndt.pose.pose.position.y = msg.pose.pose.position.y;
	lcl_ndt.pose.pose.position.z = 20.0;	
	lcl_ndt.pose.pose.orientation.z = sin(msg.pose.pose.orientation.z*0.5);
	lcl_ndt.pose.pose.orientation.w = cos(msg.pose.pose.orientation.z*0.5);

	map_matching_pub.publish(lcl_ndt);

}

void
Pose::process(){
	if(start_flag){
		input_pose();
		publish_pose();
	}

}
