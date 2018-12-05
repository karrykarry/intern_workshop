#include "image_seg.hpp"

Image_seg::Image_seg(){
	pub	= n.advertise<sensor_msgs::Image>("/camera/vision", 100);
	info_pub	= n.advertise<sensor_msgs::CameraInfo>("/camera/info", 100);
	flag = false;

	msg_info.header.frame_id = "/camera";
	msg_info.height = 614;
	msg_info.width = 808;
	

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


	msg_info.header.stamp = ros::Time::now();
	info_pub.publish(msg_info);

}

