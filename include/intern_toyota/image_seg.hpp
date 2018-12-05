#ifndef _IMAGE_SEG_HPP_
#define _IMAGE_SEG_HPP_

#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <string.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

using namespace std;


class Image_seg
{
	private:
		ros::NodeHandle n;
		ros::Publisher pub;

		bool flag;

		cv::Mat vision_image;

	public:

		Image_seg();
		int load_data(string path, int num);
		void point_pub();



};

#endif


