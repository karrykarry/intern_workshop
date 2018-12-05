/*
 * argv[1] 1 or 2 (dataset) 
 * argv[2] pass1 ~ 10 (pass1~pass10) 
 * argv[3] 1 ~ 5 (1~5) 
 *
 */
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/core.hpp>
#include <iostream>
#include <string>

using namespace std;

const string const_pass = "/home/amsl/toyota/dataset";

int main (int argc, char** argv)
{

	// "/home/amsl/toyota/dataset1_ansewer/car[5]/movie1.mp4"

	// cv::VideoWriter writer(const_pass +  argv[1] + "_answer/" +  argv[2]  + "/check_image"  +  "/movie" + argv[3] +".mp4",cv::VideoWriter::fourcc('m', 'p', '4', 'v') , 10, cv::Size(768, 966), true);
	cv::VideoWriter writer(const_pass +  argv[1] + "_answer/" +  argv[2]  + "/vision"  +  "/movie" + argv[3] +".mp4",cv::VideoWriter::fourcc('m', 'p', '4', 'v') , 10, cv::Size(808, 614), true);


	cv::Mat src_img;
	int a=0;

	while(1){
	
		// "/home/amsl/toyota/dataset1/car[5]/1/check_image/1.png"
		
		// src_img = cv::imread(const_pass + argv[1] + "/" + argv[2] + "/" + argv[3] + "/check_image/"  + to_string(a) +".png", 1);
		src_img = cv::imread(const_pass + argv[1] + "/" + argv[2] + "/" + argv[3] + "/vision/"  + to_string(a)  +"_camera.png", 1);
		// 画像が読み込まれなかったらプログラム終了
		if(src_img.empty()) return -1;

		// 結果画像表示
		cv::namedWindow("Image", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);
		cv::imshow("Image", src_img);
		writer << src_img;

		cv::waitKey(1);
		a++;
		if(a>1000){
			cout<<"アカン"<<endl;
			break;
		}
	}

	return 0;
}
