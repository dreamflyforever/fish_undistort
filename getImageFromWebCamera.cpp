#include "getImageFromWebCamera.h"

bool WebCamera::TurnOnCamera(int ID, int width, int height, int fps, bool isFisheye)
{
	iInWidth = width;
	iInHeight = height;
	iFPS = fps;
	this->isFisheye = isFisheye;
	cap = cv::VideoCapture(ID);
	if (!cap.isOpened())
	{
		Debug::Log("WebCam Failed to Open!");
		return EXIT_FAILURE;
	}
	cap.set(CV_CAP_PROP_FRAME_WIDTH, width);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, height);
	cap.set(CV_CAP_PROP_FPS, fps);
	return EXIT_SUCCESS;
}

bool WebCamera::TurnOffCamera()
{
	if (cap.isOpened())
	{
		cap.release();
	}
	return EXIT_SUCCESS;
}

bool WebCamera::GetRawImg(uchar* rawImgData)
{
	cv::Mat mRawImg = cv::Mat(iInHeight, iInWidth, CV_8UC3, rawImgData, 0);
	//cap >> mRawImg;
	bool bGetImgFailure = cap.read(mRawImg);
	if (bGetImgFailure == false)
	{
		return EXIT_FAILURE;
	}
	//memcpy(rawImgData, mRawImg.data, iInWidth * iInHeight * 3);
	return EXIT_SUCCESS;
}

void WebCamera::initUndistortRectifyMap(float* fpCameraMatrix, float* fpDistCoeffs)
{
	cv::Mat mCameraMatrix = cv::Mat(3, 3, CV_32FC1, fpCameraMatrix);
	this->mCameraMatrix = mCameraMatrix;
	cv::Size sImgSize(this->iInWidth, this->iInHeight);
	if (this->isFisheye)
	{
		cv::Mat mDistCoeffs = cv::Mat(1, 4, CV_32FC1, fpDistCoeffs);
		this->mDistCoeffs = mDistCoeffs;
		// ȥ����ͼƬ�У����������о��κ����о��Ρ�balance���������µ�ROI����Ϊ���������μ�Ĺ���
		const double balance = 1; // ��balance=0ʱ��ROIΪ���о��Σ�balance=1ʱ��ROIΪ���о���
		const double fov_scalar = 1.0; // ͨ����������������fov��
		cv::fisheye::estimateNewCameraMatrixForUndistortRectify(mCameraMatrix, mDistCoeffs, sImgSize, cv::Matx33d::eye(), this->mNewCameraMatrix, balance, sImgSize, fov_scalar);
		cout << "new camera Matrix: " << this->mNewCameraMatrix << endl;
		// cv::cuda::remap��map1��map2��ҪCV_32FC1��cv::remap��CV_32FC1��CV_16SC2����
		cv::fisheye::initUndistortRectifyMap(mCameraMatrix, mDistCoeffs, cv::Matx33d::eye(), this->mNewCameraMatrix, sImgSize, CV_32FC1, undistortRectifyMapX, undistortRectifyMapY);
		//cout << undistortRectifyMapX.size() << endl; // [2592 x 1944] 
		//cout << undistortRectifyMapX.type() << endl; // 5: CV_32FC1, 11: CV_16SC2
		//cout << undistortRectifyMapY.size() << endl; // [2592 x 1944] 
		//cout << undistortRectifyMapY.type() << endl; // 5: CV_32FC1, 11: CV_16SC2

		//cv::fisheye::initUndistortRectifyMap(mCameraMatrix, mDistCoeffs, cv::Matx33d::eye(), this->mNewCameraMatrix, sImgSize, CV_16SC2, undistortRectifyMapX, undistortRectifyMapY);
	}
	else {
		cv::Mat mDistCoeffs = cv::Mat(1, 5, CV_32FC1, fpDistCoeffs);
		this->mDistCoeffs = mDistCoeffs;
		// ȥ����ͼƬ�У����������о��κ����о��Ρ�balance���������µ�ROI����Ϊ���������μ�Ĺ���
		const double balance = 0; // ��balance=0ʱ��ROIΪ���о��Σ�balance=1ʱ��ROIΪ���о���
		this->mNewCameraMatrix = cv::getOptimalNewCameraMatrix(mCameraMatrix, mDistCoeffs, sImgSize, balance, sImgSize, 0);
		cout << "new camera Matrix: " << this->mNewCameraMatrix << endl;
		cv::initUndistortRectifyMap(mCameraMatrix, mDistCoeffs, cv::Matx33d::eye(), this->mNewCameraMatrix, sImgSize, CV_16SC2, undistortRectifyMapX, undistortRectifyMapY);
	}
	gmUndistortRectifyMapX.upload(undistortRectifyMapX);
	gmUndistortRectifyMapY.upload(undistortRectifyMapY);
}

// cpu�汾��cv::remap()
//bool WebCamera::GetUndistortedImg(uchar* undistortedImgData)
//{
//	cv::Mat mRawImg;
//	bool bGetImgFailure = cap.read(mRawImg);
//	if (bGetImgFailure == false)
//	{
//		return EXIT_FAILURE;
//	}
//	cv::Mat mUndistortedImg = cv::Mat(iInHeight, iInWidth, CV_8UC3, undistortedImgData, 0);
//	//// ����remapȥ����
//	//if (isFisheye)
//	//{
//	//	cv::fisheye::undistortImage(mRawImg, mUndistortedImg, this->mCameraMatrix, this->mDistCoeffs, this->mNewCameraMatrix, cv::Size(iInWidth, iInHeight));
//	//}
//	//else {
//	//	cv::undistort(mRawImg, mUndistortedImg, this->mCameraMatrix, this->mDistCoeffs, this->mNewCameraMatrix);
//	//}
//	// ʹ��remapȥ����
//	cv::remap(mRawImg, mUndistortedImg, undistortRectifyMapX, undistortRectifyMapY, cv::INTER_LINEAR); // ˫���Բ�ֵ
//	return EXIT_SUCCESS;
//}

// gpu�汾��cv::cuda::remap()
bool WebCamera::GetUndistortedImg(uchar* undistortedImgData)
{
	cv::Mat mRawImg;
	bool bGetImgFailure = cap.read(mRawImg);
	if (bGetImgFailure == false)
	{
		return EXIT_FAILURE;
	}
	cv::cuda::GpuMat gmRawImg(mRawImg);
	cv::cuda::GpuMat gmUndistortedImg(iInHeight, iInWidth, CV_8UC3);
	cv::cuda::remap(gmRawImg, gmUndistortedImg, gmUndistortRectifyMapX, gmUndistortRectifyMapY, cv::INTER_LINEAR);
	cv::Mat mUndistortedImg = cv::Mat(iInHeight, iInWidth, CV_8UC3, undistortedImgData, 0);
	gmUndistortedImg.download(mUndistortedImg);
	return EXIT_SUCCESS;
}

void WebCamera::PixelConvert2NormalizedPlane(cv::Point2f& pIn, cv::Point3f& pOut)
{
	float fx = mNewCameraMatrix.at<float>(0, 0);
	float fy = mNewCameraMatrix.at<float>(1, 1);
	float cx = mNewCameraMatrix.at<float>(0, 2);
	float cy = mNewCameraMatrix.at<float>(1, 2);
	pOut.x = (pIn.x - cx) / fx;
	pOut.y = (pIn.y - cy) / fy;
	pOut.z = 1.0f;
}

void WebCamera::FindCameraID()
{
	int ID = 0;
	while (1)
	{
		cv::VideoCapture cap = cv::VideoCapture(ID);
		cv::Mat frame;
		bool b = cap.read(frame);
		if (b == false)
		{
			ID += 1;
		}
		else
		{
			cv::imshow("camera", frame);
			cout << "ID: " << ID << endl;
			if (cv::waitKey(1) == 'q')
			{
				break;
			}
			break;
		}
	}
}
