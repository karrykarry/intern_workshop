#ifndef _COLOR2PC_HPP_
#define _COLOR2PC_HPP_

#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <string.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;


class Color2pc
{
	private:
		ros::NodeHandle n;
		ros::Publisher pc_publisher;
		ros::Publisher image_pub_;
		ros::Subscriber pc_sub;
		ros::Subscriber image_sub;

		bool flag;

		int vision_width;
		cv::Mat depth_png;
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pc;

	public:

		Color2pc();
		int load_data(string path, int num);
		
		void imageCb(const sensor_msgs::ImageConstPtr& msg);
		
		void calibration(const sensor_msgs::ImageConstPtr& msg,cv::Mat depth_png);
		void point_pub();



};

#endif


