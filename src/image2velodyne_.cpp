#include "image2velodyne.hpp"

Image2velodyne::Image2velodyne()
	:pc(new pcl::PointCloud<pcl::PointXYZINormal>)
{
	pc_publisher = n.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 100);
	odom_publisher = n.advertise<nav_msgs::Odometry>("/lcl/odom", 100);

	flag  = true;
}

int
Image2velodyne::load_data(string path, int num){

	string json_filename = path + "/pose/" + to_string(num) +"_pose.json";
	string json_filename_next = path + "/pose/" + to_string(num+1) +"_pose.json";

	json2msg(json_filename,geo_pose,num);
	calc_dv(json_filename,json_filename_next);

	depth_png = cv::imread(path + "/depth/" + to_string(num) +"_depth.png" , -1);
	if(depth_png.empty()) return -1;

	intensity_png = cv::imread(path + "/intensity/" + to_string(num) +"_intensity.png" , 0);
	if(intensity_png.empty()) return -1;

	normal_png = cv::imread(path + "/normal/" + to_string(num) +"_normal.png" , 1);
	if(normal_png.empty()) return -1;


	// cv::namedWindow("Image", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);
	// cv::imshow("Image", depth_image);

	cv::waitKey(1);
	
	image2point(depth_png,intensity_png,normal_png);
}


//json2odom/*{{{*/
void 
Image2velodyne::json2msg(string filename, geometry_msgs::Pose& pose, int flag_init)
{
	ifstream ifs(filename);
	string str;
	vector<string> v;
	string value;
	int cnt = 0;

	if (ifs.fail())
	{
		cerr << "can't open json file [" << filename << "]" << endl;
		return;
	}

	while (getline(ifs, str) )
	{
		cnt = 0;
		value.clear();
		for(auto it = str.begin(); it != str.end(); ++it){
			if(*it == '"'){
				cnt++;
			}
			if(cnt == 3){
				value += *it;
			}
		}
		if(cnt == 4){
			value.erase(value.begin());
			v.push_back(value);
		}
	}
	if(!flag_init){
		pos_init[0] = stod(v[0]);
		pos_init[1] = stod(v[1]);
		pos_init[2] = stod(v[2]);
		pose.position.x    = 0.0;
		pose.position.y    = 0.0;
		pose.position.z    = 0.0;
		pose.orientation.x = stod(v[3]);
		pose.orientation.y = stod(v[4]);
		pose.orientation.z = stod(v[5]);
		pose.orientation.w = stod(v[6]);

	}else{
		pose.position.x    = stod(v[0]) - pos_init[0];
		pose.position.y    = stod(v[1]) - pos_init[1];
		pose.position.z    = stod(v[2]) - pos_init[2];
		pose.orientation.x = stod(v[3]);
		pose.orientation.y = stod(v[4]);
		pose.orientation.z = stod(v[5]);
		pose.orientation.w = stod(v[6]);
	}

}/*}}}*/


//相対移動量/*{{{*/
int
Image2velodyne::calc_dv(string before_file,string after_file){
	ifstream ifs(before_file);
	string str;
	vector<string> v;
	string value;
	int cnt = 0;
	double before_pos[3];
	double after_pos[3];

	if (ifs.fail())
	{
		cerr << "can't open json file [" << before_file << "]" << endl;
		return 1;
	}

	while (getline(ifs, str) )
	{
		cnt = 0;
		value.clear();
		for(auto it = str.begin(); it != str.end(); ++it){
			if(*it == '"'){
				cnt++;
			}
			if(cnt == 3){
				value += *it;
			}
		}
		if(cnt == 4){
			value.erase(value.begin());
			v.push_back(value);
		}
	}
	before_pos[0] = stod(v[0]);
	before_pos[1] = stod(v[1]);
	before_pos[2] = stod(v[2]);
	

	ifstream ifs_(after_file);
	vector<string> v_;
	
	if (ifs_.fail())
	{
		cerr << "can't open json file [" << after_file << "]" << endl;
		return 1;
	}

	while (getline(ifs_, str) )
	{
		cnt = 0;
		value.clear();
		for(auto it = str.begin(); it != str.end(); ++it){
			if(*it == '"'){
				cnt++;
			}
			if(cnt == 3){
				value += *it;
			}
		}
		if(cnt == 4){
			value.erase(value.begin());
			v_.push_back(value);
		}
	}
	after_pos[0] = stod(v_[0]);
	after_pos[1] = stod(v_[1]);
	after_pos[2] = stod(v_[2]);

	dx = ( after_pos[0] - before_pos[0] ); // m 
	dy = ( after_pos[1] - before_pos[1] ); // m
	dz = ( after_pos[2] - before_pos[2] ); // m
}/*}}}*/


void 
Image2velodyne::calcCurv(cv::Vec3b center, cv::Vec3b neighbor, double &curv)
{
	double  cx, cy, cz, nx, ny, nz;
	double dot;// = center[0] * neighbor[0] + center[1] * neighbor[1] + center[2] * neighbor[2];

	cx = neighbor[2] / 255.0;
	cy = neighbor[1] / 255.0;
	cz = neighbor[0] / 255.0;
	nx = center[2] / 255.0;
	ny = center[1] / 255.0;
	nz = center[0] / 255.0;

	dot = cx * nx + cy * ny + cz * nz;

	if(cx > 0.1 || cy > 0.1 && nz < 1.0){
		curv = 0.6;
	}else{
		curv = 0.0;
	}
}



int
Image2velodyne::image2point(cv::Mat depth_image_, cv::Mat intensity_image_ ,cv::Mat normal_image_){

	pcl::PointXYZINormal cloud;
	float x,y,z;
	double curv = 0.0;
	
	pc->points.clear();

	for(int row=0;row < depth_image_.rows;row++){
		cv::Vec3b* n_ptr = normal_image_.ptr<cv::Vec3b>(row);
		for(int col=0;col<depth_image_.cols;col++){
			unsigned short distance = depth_image_.at<unsigned short>(row,col);
			if(!distance) continue;	
			calc(row,col,distance,x,y,z);
			unsigned char intensity = intensity_image_.at<unsigned char>(row,col);
			// int intensity = intensity_image_.data[row * intensity_image_.cols + col];

			//normal
			cv::Vec3b normal = n_ptr[col];

			for(int i = 0; i < 3; i++){
				cv::Vec3b* n_ptr9 = normal_image_.ptr<cv::Vec3b>(row - 1 + i);
				for(int j = 0; j < 3; j++){
					if(i != 2 || j != 2){
						cv::Vec3b n9 = n_ptr[(col - 1 + j + depth_image_.cols) % depth_image_.cols];
						calcCurv(normal, n9, curv);
						if(curv > 0.5){
							break;
						}
					}
				}
				if(curv > 0.5){
					break;
				}
			}
			
			cloud.x = x;
			cloud.y = y;
			cloud.z = z;
			cloud.intensity = (int) intensity;
			cloud.normal_x = normal[2];
			cloud.normal_y = normal[1];
			cloud.normal_z = normal[0];
			cloud.curvature = curv;
			
			pc->points.push_back(cloud);
		}
	}

}

//相対移動量を考慮/*{{{*/
int
Image2velodyne::complement(int col){

	double dt;

	if(col<1024) dt = (1023.0-col)/2048.0;
	
	else dt = 1 + (1023.0-col)/2048.0; 

	// return pow(dx*dt,2) + pow(dy*dt,2) + pow(dz*dt,2);
	return pow(dx*dt,2) + pow(dy*dt,2);
	// return pow(dx*dt,2);

}/*}}}*/



int
Image2velodyne::calc(int row, int col, ushort distance,float &x,float &y,float &z){

	double horizon,vertical;

	// horizon = (1023.5 - col) * 2 * M_PI / 2048 ; //rad
	horizon = (1024 - col) * 2 * M_PI / 2048.0 ; //rad

	if(row < 32){	//真ん中より上
		vertical = (-8.33 + 1.0/3.0*(31 - row)) * M_PI /180.0; //deg2rad
	}
	else vertical = (-8.53 + 1.0/2.0*(32 - row)) * M_PI /180.0; //deg2rad


	double dist = distance * 100.0 / pow(2,16);
	double d = complement(col);
    
	double theta = 2*M_PI *(1 - (2047.0-col)/2048.0);  
   
	double dist_ = sqrt(d + dist*dist - 2 * sqrt(d)*dist*cos(theta));


	x = dist_*cos(vertical)*cos(horizon);
	y = dist_*cos(vertical)*sin(horizon);
	z = dist_*sin(vertical) ;
}



//tf_broad/*{{{*/
void 
Image2velodyne::tf_broad(nav_msgs::Odometry msg){


	transform.setOrigin( tf::Vector3(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z) );
	tf::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);

	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now() , "/map", "/matching_base_link"));

}/*}}}*/

//point_pub/*{{{*/
int 
Image2velodyne::point_pub()
{
	sensor_msgs::PointCloud2 pub_points;

    pcl::toROSMsg(*pc,pub_points);

    pub_points.header.stamp = ros::Time::now();
    pub_points.header.frame_id = "/velodyne";

    pc_publisher.publish(pub_points);
}/*}}}*/

//odom_pub/*{{{*/
int 
Image2velodyne::odom_pub()
{
	nav_msgs::Odometry odom;
	odom.header.stamp = ros::Time::now();
	odom.header.frame_id = "/map";
	odom.child_frame_id = "/matching_base_link";
	
	odom.pose.pose = geo_pose;
	tf_broad(odom);

    odom_publisher.publish(odom);
}/*}}}*/
