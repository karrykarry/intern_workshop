#ifndef _IMAGE2VELODYNE_HPP_
#define _IMAGE2VELODYNE_HPP_

#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <string.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;


class Image2velodyne
{
	private:
		ros::NodeHandle n;
		ros::Publisher pc_publisher;
		ros::Publisher odom_publisher;
		tf::TransformBroadcaster br;
		tf::Transform transform;
		double pos_init[3];

		bool flag;

		cv::Mat depth_png;
		cv::Mat intensity_png;
		cv::Mat normal_png;
		pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc;

		geometry_msgs::Pose geo_pose;


		double dx;
		double dy;
		double dz;


	public:

	Image2velodyne();

	int load_data(string path,int num);
	void json2msg(string filename, geometry_msgs::Pose& pose, int flag_init);
	int calc_dv(string before_file,string after_file);
	void calcCurv(cv::Vec3b center, cv::Vec3b neighbor, double &curv);
	int	image2point(cv::Mat depth_image_, cv::Mat intensity_image_ ,cv::Mat normal_image_);
	int calc(int row, int col, ushort distance,float &x,float &y,float &z);
	int complement(int col);
	void tf_broad(nav_msgs::Odometry msg);
	int point_pub();
	int odom_pub();


};

#endif

