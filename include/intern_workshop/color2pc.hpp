#ifndef _COLOR2PC_HPP_
#define _COLOR2PC_HPP_

#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <string.h>
#include <sensor_msgs/Image.h>
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

		bool flag;

		cv::Mat depth_png;

	public:

		Color2pc();
		int load_data(string path, int num);
		void point_pub();



};

#endif


