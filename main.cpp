//
// Created by jiang on 2020/4/29.
//
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;

const cv::Mat K = ( cv::Mat_<double> ( 3,3 ) << 379.84453644f, 0, 860.07374137f, 0,
						380.01995737f, 610.34223052f, 0, 0, 1.0f );
const cv::Mat D = ( cv::Mat_<double> ( 4,1 ) << -0.02985565f, -0.00115256f, -0.00096303f, -0.00002061f);
const string str = "/Users/jim/workspace/opencv/undistor/Pictures/";
const int nImage = 1;
const int ImgWidth = 1600;
const int ImgHeight = 1200;

int main()
{
	cv::Mat map1, map2;
	cv::Mat mNewCameraMatrix;
	cv::Mat mCameraMatrix = K;
	cv::Size sImgSize(ImgWidth, ImgHeight);
	cv::Mat mDistCoeffs = D;
	const double balance = 1;
	const double fov_scalar = 1.0;
	cv::fisheye::estimateNewCameraMatrixForUndistortRectify(mCameraMatrix, mDistCoeffs, sImgSize, cv::Matx33d::eye(), mNewCameraMatrix, balance, sImgSize, fov_scalar);
	cout << "new camera Matrix: " << mNewCameraMatrix << endl;
	cv::fisheye::initUndistortRectifyMap(mCameraMatrix, mDistCoeffs, cv::Matx33d::eye(), mNewCameraMatrix, sImgSize, CV_32FC1, map1, map2);
	cv::Mat correct_image;
	string InputPath = str + to_string(0) + ".jpeg";
	cv::Mat RawImage = cv::imread(InputPath);
	cv::remap(RawImage, correct_image, map1, map2, cv::INTER_LINEAR);
	cv::imwrite("undistor.jpg", correct_image);
	return 0;
}


