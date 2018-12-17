/*赤認識
 *
 */
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace std;
using namespace cv;

// 抽出する画像の輝度値の範囲を指定
#define B_MAX 70
#define B_MIN 0
#define G_MAX 70
#define G_MIN 0
#define R_MAX 255
#define R_MIN 100

// メイン関数
int main(void)
{
	// 入力画像名(ファイルパス)
	string input_filename = "/home/amsl/3.jpg";

	// 画像を3チャンネル(BGR)で読み込む
	Mat input_image_rgb = imread(input_filename, CV_LOAD_IMAGE_COLOR);
	if (input_image_rgb.empty()) {
		cerr << "入力画像が見つかりません" << endl;
		return -1;
	}

	// 表示して確認
	namedWindow("input_RGB");
	imshow("input_RGB", input_image_rgb);

	// 結果保存用Matを定義
	Mat mask_image, output_image_rgb;

	// inRangeを用いてフィルタリング
	Scalar s_min = Scalar(B_MIN, G_MIN, R_MIN);
	Scalar s_max = Scalar(B_MAX, G_MAX, R_MAX);
	inRange(input_image_rgb, s_min, s_max, mask_image);

	// マスク画像を表示
	namedWindow("mask");
	imshow("mask", mask_image);
	imwrite("mask.jpg", mask_image);

	// マスクを基に入力画像をフィルタリング
	input_image_rgb.copyTo(output_image_rgb, mask_image);

	// 結果の表示と保存
	namedWindow("output");
	imshow("output", output_image_rgb);
	imwrite("output.jpg", output_image_rgb);
	waitKey(0);

	return 0;
}
