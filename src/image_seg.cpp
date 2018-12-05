#include "image_seg.hpp"

Image_seg::Image_seg(){
	pub	= n.advertise<sensor_msgs::Image>("/camera/vision", 100);
	flag = false;
}

int
Image_seg::load_data(string path, int num){

	vision_image = cv::imread(path + "/vision/" + to_string(num) +"_camera.png" , 1);
	if(vision_image.empty()) return -1;

	cv::waitKey(1);
}


void
Image_seg::point_pub()
{
	sensor_msgs::ImagePtr msg_image;

	msg_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", vision_image).toImageMsg();

    msg_image->header.stamp = ros::Time::now();
    msg_image->header.frame_id = "/camera";

    pub.publish(msg_image);
}

