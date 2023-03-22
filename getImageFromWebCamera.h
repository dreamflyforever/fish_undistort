  #pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "debugLog.h"

using namespace std;

class WebCamera
{
private:
	int iInWidth;
	int iInHeight;
	int iFPS;
	int iOutWidth;
	int iOutHeight;
	bool isFisheye;
	cv::VideoCapture cap;
	cv::Mat mCameraMatrix;
	cv::Mat mDistCoeffs;
	cv::Mat mNewCameraMatrix;
	cv::Mat undistortRectifyMapX;
	cv::Mat undistortRectifyMapY;
	cv::cuda::GpuMat gmUndistortRectifyMapX;
	cv::cuda::GpuMat gmUndistortRectifyMapY;
public:
	WebCamera() {};
	~WebCamera() {};
	bool TurnOnCamera(int ID, int width, int height, int fps, bool isFisheye);
	bool TurnOffCamera();
	void FindCameraID();
	bool GetRawImg(uchar* rawImgData);
	void initUndistortRectifyMap(float* fpCameraMatrix, float* fpDistCoeffs);
	bool GetUndistortedImg(uchar* undistortedImgData);
	void PixelConvert2NormalizedPlane(cv::Point2f&, cv::Point3f&);
};