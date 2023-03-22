//
// Created by jiang on 2020/4/29.
//
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;

const cv::Mat K = ( cv::Mat_<double> ( 3,3 ) << 
	379.84453644f,		0,			860.07374137f,
	0,			380.01995737f,		610.34223052f,
	0,			0,				1.0f );
const cv::Mat D = ( cv::Mat_<double> ( 4,1 ) << -0.02985565f, -0.00115256f, -0.00096303f, -0.00002061f);
const string str = "/Users/jim/workspace/opencv/undistor/Pictures/";
const int nImage = 1;
const int ImgWidth = 1600;
const int ImgHeight = 1200;
#include <opencv2/opencv.hpp>

// 文件路径，如果不对，请调整
std::string input_file = "/Users/jim/workspace/opencv/undistor/Pictures/0.jpeg";
#if 1

int main()
{
#if 1
	cv::Mat fpCameraMatrix = K;
	cv::Mat fpDistCoeffs = D;
	cv::Mat map1, map2;
	cv::Mat mNewCameraMatrix;
	cv::Mat mCameraMatrix = K;//(3, 3, CV_32FC1, fpCameraMatrix);
	cv::Size sImgSize(ImgWidth, ImgHeight);
	cv::Mat mDistCoeffs = D; //(1, 4, CV_32FC1, fpDistCoeffs);
	// È¥»û±äÍ¼Æ¬ÖÐ£¬»á¼ÆËã³öÄÚÇÐ¾ØÐÎºÍÍâÇÐ¾ØÐÎ¡£balanceÓÃÀ´¿ØÖÆÐÂµÄROIÇøÓòÎªÕâÁ½¸ö¾ØÐÎ¼äµÄ¹ý¶É
	const double balance = 1; // µ±balance=0Ê±£¬ROIÎªÄÚÇÐ¾ØÐÎ£¬balance=1Ê±£¬ROIÎªÍâÇÐ¾ØÐÎ
	const double fov_scalar = 1.0; // Í¨¹ýµ÷Õû½¹¾àÀ´µ÷Õûfov£¿
	cv::fisheye::estimateNewCameraMatrixForUndistortRectify(mCameraMatrix, mDistCoeffs, sImgSize, cv::Matx33d::eye(), mNewCameraMatrix, balance, sImgSize, fov_scalar);
	cout << "new camera Matrix: " << mNewCameraMatrix << endl;
	// cv::cuda::remapÖÐmap1ºÍmap2ÐèÒªCV_32FC1£¬cv::remapÔòCV_32FC1ºÍCV_16SC2¶¼ÐÐ
	cv::fisheye::initUndistortRectifyMap(mCameraMatrix, mDistCoeffs, cv::Matx33d::eye(), mNewCameraMatrix, sImgSize, CV_32FC1, map1, map2);
	//cout << undistortRectifyMapX.size() << endl; // [2592 x 1944] 
	//cout << undistortRectifyMapX.type() << endl; // 5: CV_32FC1, 11: CV_16SC2
	//cout << undistortRectifyMapY.size() << endl; // [2592 x 1944] 
	//cout << undistortRectifyMapY.type() << endl; // 5: CV_32FC1, 11: CV_16SC2

	//cv::fisheye::initUndistortRectifyMap(mCameraMatrix, mDistCoeffs, cv::Matx33d::eye(), this->mNewCameraMatrix, sImgSize, CV_16SC2, undistortRectifyMapX, undistortRectifyMapY);
	//gmUndistortRectifyMapX.upload(undistortRectifyMapX);
	//gmUndistortRectifyMapY.upload(undistortRectifyMapY);
#endif
	cv::Mat correct_image;
	string InputPath = str + to_string(0) + ".jpeg";
	cv::Mat RawImage = cv::imread(InputPath);
	cv::remap(RawImage, correct_image, map1, map2, cv::INTER_LINEAR);
	//cv::Mat mUndistortedImg = cv::Mat(ImgHeight, ImgWidth, CV_8UC3, mNewCameraMatrix, 0);
	cv::imwrite("undistor.jpg", correct_image);
	return 0;
}
#if 0
void hello()
{
}
int main()
{
	hello();
	cv::Mat map1, map2;
	cv::Size imageSize(ImgWidth, ImgHeight);
	const double alpha = 1;
	cv::Mat NewCameraMatrix = getOptimalNewCameraMatrix(K, D, imageSize, alpha, imageSize, 0);
	initUndistortRectifyMap(K, D, cv::Mat(), NewCameraMatrix, imageSize, CV_16SC2, map1, map2);

	for(int i=0; i<nImage; i++)
	{
		string InputPath = str + to_string(i) + ".jpeg";
		cv::Mat RawImage = cv::imread(InputPath);
		cv::imshow("RawImage", RawImage);

		cv::Mat UndistortImage;
		remap(RawImage, UndistortImage, map1, map2, cv::INTER_LINEAR);
		cv::imshow("UndistortImage", UndistortImage);

		string OutputPath = str + to_string(i) + "_un" + ".png";
		cv::imwrite(OutputPath, UndistortImage);
		cv::waitKey(1);
	}

	return 0;
}
#endif



#endif
#if 0
int main()
{
	//已知相机内参和畸变系数
	//step1.估计新矩阵
	cv::Mat newCamMat;
	// 估计新的相机内参矩阵,无畸变后的
	cv::fisheye::estimateNewCameraMatrixForUndistortRectify(
			camera_intrinsic_matrix, distort_coeff_matrix, img_size,
			cv::Matx33d::eye(), newCamMat, 1);
	//step2.计算map1,map2
	cv::Mat map1, map2;
	cv::fisheye::initUndistortRectifyMap(
			camera_intrinsic_matrix, 
			distort_coeff_matrix, 
			cv::Matx33d::eye(), newCamMat, img_size,
			CV_16SC2, map1, map2);

	// step3.remap图像去畸变
	cv::Mat cam_im = imread("1.png");
	cv::Mat correct_image;
	cv::remap(cam_im, correct_image, map1, map2, cv::INTER_LINEAR);

	//step4. undistortImage图像去畸变
	cv::Mat undistort_im;
	cv::fisheye::undistortImage(
			cam_im,undistort_im,
			camera_intrinsic_matrix,
			distort_coeff_matrix,
			newCamMat,
			cam_im.size());
	//step5.比较一下这两个图像是否一直
	cv::Mat substrct_im;
	cv::subtract(undistort_im,correct_image,substrct_im);
	cv::imwrite("substrct_im.jpg",substrct_im);

	//step6.undistortPoints图像点去畸变
	std::vector<cv::Point2f> src_pts{ cv::Point2f(500,500)};
	std::vector<cv::Point2f> dst_pts;
	cv::fisheye::undistortPoints(
			src_pts,dst_pts,
			camera_intrinsic_matrix,
			distort_coeff_matrix,
			cv::noArray(),
			newCamMat);
	cout<<"dst_pts= "<<dst_pts[0]<<endl;
}
#endif
#if 0
void UndistortKeyPoints(vector<cv::Point2f> &points);

int main()
{
	const int MAX_CNT = 150;
	const int MIN_DIST = 30;

	for(int i=0; i<nImage; i++)
	{
		string InputPath = str + to_string(i) + ".jpeg";
		cv::Mat RawImage = cv::imread(InputPath);

		vector<cv::Point2f> pts;
		cv::Mat RawImage_Gray;
		cv::cvtColor(RawImage, RawImage_Gray, cv::COLOR_RGB2GRAY);

		cv::goodFeaturesToTrack(RawImage_Gray, pts, MAX_CNT, 0.01, MIN_DIST);

		for(auto& pt:pts)
			circle(RawImage, pt, 2, cv::Scalar(255, 0, 0), 2);
		cv::imshow("pts", RawImage);

		UndistortKeyPoints(pts);

		cv::Mat UndistortImage;
		cv::undistort(RawImage, UndistortImage, K, D, K);

		for(auto& pt:pts)
			circle(UndistortImage, pt, 2, cv::Scalar(0, 0, 255), 2);
		cv::imshow("pts_un", UndistortImage);

		string OutputPath = str + to_string(i) + "_pts_un" + ".png";
		cv::imwrite(OutputPath, UndistortImage);
		cv::waitKey(0);
	}

	return 0;
}

void UndistortKeyPoints(vector<cv::Point2f> &points)
{
	if(D.at<float>(0)==0.0)    // 图像矫正过
		return;

	// N为提取的特征点数量，将N个特征点保存在N*2的mat中
	uint N = points.size();
	cv::Mat mat(N,2,CV_32F);
	for(int i=0; i<N; i++)
	{
		mat.at<float>(i,0)=points[i].x;
		mat.at<float>(i,1)=points[i].y;
	}

	// 调整mat的通道为2，矩阵的行列形状不变
	mat=mat.reshape(2);
	cv::undistortPoints(mat, mat, K, D, cv::Mat(), K);
	mat=mat.reshape(1);

	// 存储校正后的特征点
	for(int i=0; i<N; i++)
	{
		cv::Point2f kp = points[i];
		kp.x=mat.at<float>(i,0);
		kp.y=mat.at<float>(i,1);
		points[i] = kp;
	}
}
#endif

