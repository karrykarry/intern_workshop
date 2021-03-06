/* pub_data.cpp
 *
 * 2018.12.7
 *
 * author : R.Kusakari
 *
 * depth intensity normalのpngから3次元点群に変換
*/ 
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include "image2velodyne.hpp"
#include "image_seg.hpp"

using namespace std;

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "pub_data");
	ros::NodeHandle n;

	ros::Publisher flag_pub = n.advertise<std_msgs::Bool>("scene_flag", 10);

	string root_dir_path,work_num,scene_path,scene_num;

	n.getParam("/root_dir_path",root_dir_path);
	n.getParam("/work_num",work_num);
	n.getParam("/scene_path",scene_path);
	n.getParam("/scene_num",scene_num);

	string path = root_dir_path + work_num + scene_path +scene_num;

	Image2velodyne i2velo;	//velodyneに付加する系
	Image_seg i_seg;	//visionによる処理系

    ros::Rate loop_rate(10);

    int count = 0;
    while(ros::ok()){
		i2velo.load_data(path,count);	
		i_seg.load_data(path,count);	
		
		i2velo.point_pub();
		i2velo.odom_pub();
		i_seg.point_pub();


        count++;
        // if(count == 50){
        if(count == 49){
			std_msgs::Bool data;
			data.data = true;
			flag_pub.publish(data);
        /* if(count == 70){ */
        /* if(count == 90){ */
        //	break;
			count = 0;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }   

    return 0;
}

       
