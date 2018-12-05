#include <ros/ros.h>
#include "color2pc.hpp"

using namespace std;

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "pub_color2pc");
	ros::NodeHandle n;
  
	string root_dir_path,work_num,scene_path,scene_num;

	n.getParam("/root_dir_path",root_dir_path);
	n.getParam("/work_num",work_num);
	n.getParam("/scene_path",scene_path);
	n.getParam("/scene_num",scene_num);

	string path = root_dir_path + work_num + scene_path +scene_num;

	Color2pc c2pc;	//velodyneに付加する系

    ros::Rate loop_rate(10);

    int count = 0;
    while(ros::ok()){
		c2pc.load_data(path,count);	
		
        count++;
        
		if(count == 49){
            count = 0;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }   

    return 0;
}

       
