#include "color2pc.hpp"

Color2pc::Color2pc(){

}


int
Color2pc::load_data(string path, int num){

	depth_png = cv::imread(path + "/depth/" + to_string(num) +"_depth.png" , -1);
	if(depth_png.empty()) return -1;

	// cv::namedWindow("Image", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);
	// cv::imshow("Image", depth_image);

	cv::waitKey(1);
	
}




// void
// Color2pc::point_pub()
// {
// 	sensor_msgs::ImagePtr msg_image;
//
// 	msg_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", depth_png).toImageMsg();
//
//     msg_image->header.stamp = ros::Time::now();
//     msg_image->header.frame_id = "/camera";
//
//     pub.publish(msg_image);
// }


