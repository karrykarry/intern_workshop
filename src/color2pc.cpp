#include "color2pc.hpp"


Color2pc::Color2pc()
	:pc(new pcl::PointCloud<pcl::PointXYZRGBNormal>),vision_width(808)
{
	pc_publisher = n.advertise<sensor_msgs::PointCloud2>("/velodyne_rgb", 100);
	image_pub_ = n.advertise<sensor_msgs::Image>("/camera/vision/test", 100);
	// pc_sub = n.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, &Color2pc::pcCallback, this);
	image_sub = n.subscribe<sensor_msgs::Image>("/grass_image", 1, &Color2pc::imageCb, this);
	
	flag  = true;
}


int
Color2pc::load_data(string path, int num){

	depth_png = cv::imread(path + "/depth/" + to_string(num) +"_depth.png" , -1);
	if(depth_png.empty()) return -1;

	// cv::namedWindow("Image", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);
	// cv::imshow("Image", depth_image);

	cv::waitKey(1);
	
}



	void 
Color2pc::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}


	for(int row=0;cv_ptr->image.rows;row++){
		cv::Vec3b* n_ptr = cv_ptr->image.ptr<cv::Vec3b>(row);
		for(int col=0;cv_ptr->image.cols;col++){
    
			cv::Vec3b normal = n_ptr[col];
			// if((normal[0]==255)&&(normal[1]==255)&&(normal[2]==255))
				// cout<<col<<","<<row;
		}
	}


	// Update GUI Window
	// cv::imshow(OPENCV_WINDOW, cv_ptr->image);
	cv::waitKey(3);

	// Output modified video stream
	image_pub_.publish(cv_ptr->toImageMsg());
}

void
Color2pc::calibration(const sensor_msgs::ImageConstPtr& msg,cv::Mat depth_png){

	// pcl::PointXYZINormal cloud;
	// float x,y,z;
	// double curv = 0.0;
	//
	// pc->points.clear();
	// int start_id = 7*depth_png.cols/16;
	// int end_id = 9*depth_png.cols/16;
    //
	// for(int row=0;row < depth_image_.rows;row++){
	// 	for(int col=start_id;col<end_id;col++){
	// 		unsigned short distance = depth_png.at<unsigned short>(row,col);
	// 		if(!distance) continue;	
	// 		// calc(row,col,distance,x,y,z);
    //
	// 		double ratio = (float) col/vision_width;
	// 		
	// 		cloud.x = x;
	// 		cloud.y = y;
	// 		cloud.z = z;
	// 		// cloud.intensity = (int) intensity;
	// 		cloud.normal_x = normal[2];
	// 		cloud.normal_y = normal[1];
	// 		cloud.normal_z = normal[0];
	// 		cloud.curvature = curv;
	// 		
	// 		pc->points.push_back(cloud);
	// 	}
	// }
    //

}

// void 
// Color2pc::pcCallback(sensor_msgs::PointCloud2 msg){
// 	cloud = msg;
// }




// Color2pc::calibration(sensor_msgs::Image image_msg,sensor_msgs::PointCloud pc_msg){
// void 
// Color2pc::calibration( ){
//
// 	float x,y,z;
// 	double curv = 0.0;
// 	
// 	pc->points.clear();
// 	
// 	pcl::fromROSMsg(*input,*input_pc);
// 	size_t pc_size = input_pc->points.size();
// 	for(size_t i = 0 < i <pc_size<i++){
// 		pcl::PointXYZINormal cloud;
// 			
// 		pc->points.push_back(cloud);
// 	}
// }

//point_pub/*{{{*/
void 
Color2pc::point_pub()
{
	sensor_msgs::PointCloud2 pub_points;

    pcl::toROSMsg(*pc,pub_points);

    pub_points.header.stamp = ros::Time::now();
    pub_points.header.frame_id = "/velodyne";

    pc_publisher.publish(pub_points);
}/*}}}*/

