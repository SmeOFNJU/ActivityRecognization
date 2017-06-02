         
#include <opencv2/opencv.hpp>
#include <Windows.h>
#include <Kinect.h>
#include<iostream>
#include<time.h>

using namespace std;
using namespace cv;

//释放接口需要自己定义
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL) {
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}	
}
 

void DrawBone(Mat& SkeletonImage, CvPoint pointSet[], const Joint* pJoints, int whichone, JointType joint0, JointType joint1);

void drawSkeleton(Mat& SkeletonImage, CvPoint pointSet[], const Joint* pJoints, int whichone);


int main(int argc, char **argv[])
{

	//OpenCV中开启CPU的硬件指令优化功能函数
	setUseOptimized(true);

	// Sensor
	IKinectSensor* pSensor;
	HRESULT hResult = S_OK;
	hResult = GetDefaultKinectSensor(&pSensor);
	hResult = pSensor->Open();
	if (FAILED(hResult)) {
		std::cerr << "Error : IKinectSensor::Open()" << std::endl;
		return -1;
	}
	//Source
	IColorFrameSource* pColorSource;
	hResult = pSensor->get_ColorFrameSource(&pColorSource);
	if (FAILED(hResult)) {
		std::cerr << "Error : IKinectSensor::get_ColorFrameSource()" << std::endl;
		return -1;
	}

	IBodyFrameSource* pBodySource;
	hResult = pSensor->get_BodyFrameSource(&pBodySource);
	if (FAILED(hResult)) {
		std::cerr << "Error : IKinectSensor::get_BodyFrameSource()" << std::endl;
		return -1;
	}

	// Reader
	IColorFrameReader* pColorReader;
	hResult = pColorSource->OpenReader(&pColorReader);
	if (FAILED(hResult)) {
		std::cerr << "Error : IColorFrameSource::OpenReader()" << std::endl;
		return -1;
	}

	IBodyFrameReader* pBodyReader;
	hResult = pBodySource->OpenReader(&pBodyReader);
	if (FAILED(hResult)) {
		std::cerr << "Error : IBodyFrameSource::OpenReader()" << std::endl;
		return -1;
	}

	// Description
	IFrameDescription* pDescription;
	hResult = pColorSource->get_FrameDescription(&pDescription);
	if (FAILED(hResult)) {
		std::cerr << "Error : IColorFrameSource::get_FrameDescription()" << std::endl;
		return -1;
	}

	int width = 0;
	int height = 0;
	pDescription->get_Width(&width); // 1920
	pDescription->get_Height(&height); // 1080
	unsigned int bufferSize = width * height * 4 * sizeof(unsigned char);

	cv::Mat bufferMat(height, width, CV_8UC4);
	cv::Mat bodyMat(height / 2, width / 2, CV_8UC4);
	cv::namedWindow("Body");

	// Color Table
	cv::Vec3b color[BODY_COUNT];
	color[0] = cv::Vec3b(255, 0, 0);
	color[1] = cv::Vec3b(0, 255, 0);
	color[2] = cv::Vec3b(0, 0, 255);
	color[3] = cv::Vec3b(255, 255, 0);
	color[4] = cv::Vec3b(255, 0, 255);
	color[5] = cv::Vec3b(0, 255, 255);

	float Skeletons[6][25][3];
	// Coordinate Mapper
	ICoordinateMapper* pCoordinateMapper;
	hResult = pSensor->get_CoordinateMapper(&pCoordinateMapper);
	if (FAILED(hResult)) {
		std::cerr << "Error : IKinectSensor::get_CoordinateMapper()" << std::endl;
		return -1;
	}


	while (1) {
		// Frame
		IColorFrame* pColorFrame = nullptr;
		hResult = pColorReader->AcquireLatestFrame(&pColorFrame);
		if (SUCCEEDED(hResult)) {
			hResult = pColorFrame->CopyConvertedFrameDataToArray(bufferSize, reinterpret_cast<BYTE*>(bufferMat.data), ColorImageFormat::ColorImageFormat_Bgra);
			if (SUCCEEDED(hResult)) {
				cv::resize(bufferMat, bodyMat, cv::Size(), 0.5, 0.5);
			}

		}

		//更新骨骼帧
		IBodyFrame* pBodyFrame = nullptr;
		hResult = pBodyReader->AcquireLatestFrame(&pBodyFrame);
		if (SUCCEEDED(hResult)) {
			IBody* pBody[BODY_COUNT] = { 0 };
			//更新骨骼数据
			hResult = pBodyFrame->GetAndRefreshBodyData(BODY_COUNT, pBody);
			if (SUCCEEDED(hResult)) {
				for (int count = 0; count < BODY_COUNT; count++) {
					BOOLEAN bTracked = false;
					hResult = pBody[count]->get_IsTracked(&bTracked);
					if (SUCCEEDED(hResult) && bTracked) {
						Joint joint[JointType::JointType_Count];
	/////////////////////////////					
						hResult = pBody[count]->GetJoints(JointType::JointType_Count, joint);//joint
						if (SUCCEEDED(hResult)) {
							// Left Hand State
							HandState leftHandState = HandState::HandState_Unknown;
							hResult = pBody[count]->get_HandLeftState(&leftHandState);
							if (SUCCEEDED(hResult)) {
								ColorSpacePoint colorSpacePoint = { 0 };
								hResult = pCoordinateMapper->MapCameraPointToColorSpace(joint[JointType::JointType_HandLeft].Position, &colorSpacePoint);
								if (SUCCEEDED(hResult)) {
									int x = static_cast<int>(colorSpacePoint.X);
									int y = static_cast<int>(colorSpacePoint.Y);


									if ((x >= 0) && (x < width) && (y >= 0) && (y < height)) {
										if (leftHandState == HandState::HandState_Open) {
											cv::circle(bufferMat, cv::Point(x, y), 75, cv::Scalar(0, 128, 0), 5, CV_AA);

										}
										else if (leftHandState == HandState::HandState_Closed) {
											cv::circle(bufferMat, cv::Point(x, y), 75, cv::Scalar(0, 0, 128), 5, CV_AA);
										}
										else if (leftHandState == HandState::HandState_Lasso) {
											cv::circle(bufferMat, cv::Point(x, y), 75, cv::Scalar(128, 128, 0), 5, CV_AA);
										}
									}
								}
							}

						HandState rightHandState = HandState::HandState_Unknown;
                        ColorSpacePoint colorSpacePoint = { 0 };
						hResult = pBody[count]->get_HandRightState(&rightHandState);
							if (SUCCEEDED(hResult)) {
								hResult = pCoordinateMapper->MapCameraPointToColorSpace(joint[JointType::JointType_HandRight].Position, &colorSpacePoint);
								if (SUCCEEDED(hResult)) {
									int x = static_cast<int>(colorSpacePoint.X);
									int y = static_cast<int>(colorSpacePoint.Y);

									if ((x >= 0) && (x < width) && (y >= 0) && (y < height)) {
										if (rightHandState == HandState::HandState_Open) {
											cv::circle(bufferMat, cv::Point(x, y), 75, cv::Scalar(0, 128, 0), 5, CV_AA);
										}
										else if (rightHandState == HandState::HandState_Closed) {
											cv::circle(bufferMat, cv::Point(x, y), 75, cv::Scalar(0, 0, 128), 5, CV_AA);

										}
										else if (rightHandState == HandState::HandState_Lasso) {
											cv::circle(bufferMat, cv::Point(x, y), 75, cv::Scalar(128, 128, 0), 5, CV_AA);
										}
									}
								}
							}
							CvPoint skeletonPoint[BODY_COUNT][JointType_Count] = { cvPoint(0,0) };
							// Joint
							for (int type = 0; type < JointType::JointType_Count; type++) {
								ColorSpacePoint colorSpacePoint = { 0 };
								pCoordinateMapper->MapCameraPointToColorSpace(joint[type].Position, &colorSpacePoint);
								int x = static_cast<int>(colorSpacePoint.X);
								int y = static_cast<int>(colorSpacePoint.Y);
								skeletonPoint[count][type].x = x;
								skeletonPoint[count][type].y = y;
								if ((x >= 0) && (x < width) && (y >= 0) && (y < height)) {
									cv::circle(bufferMat, cv::Point(x, y), 5, static_cast< cv::Scalar >(color[count]), -1, CV_AA);
								}
							}
							for (int i = 0; i < 25; i++) {
								Skeletons[count][i][0] = joint[i].Position.X;
								Skeletons[count][i][1] = joint[i].Position.Y;
								Skeletons[count][i][2] = joint[i].Position.Z;
							}
								drawSkeleton(bufferMat, skeletonPoint[count], joint, count);			           		
						}
					}
				}
				cv::resize(bufferMat, bodyMat, cv::Size(), 0.5, 0.5);
			}
			for (int count = 0; count < BODY_COUNT; count++) {
				SafeRelease(pBody[count]);
			}
		}
		SafeRelease(pColorFrame);
		SafeRelease(pBodyFrame);

		waitKey(1);
		cv::imshow("Body", bodyMat);

	}


	SafeRelease(pColorSource);
	SafeRelease(pColorReader);
	SafeRelease(pDescription);
	SafeRelease(pBodySource);
	// done with body frame reader
	SafeRelease(pBodyReader);

	SafeRelease(pDescription);
	// done with coordinate mapper
	SafeRelease(pCoordinateMapper);

	if (pSensor) {
		pSensor->Close();
	}
	SafeRelease(pSensor);

	return 0;
}


void DrawBone(Mat& SkeletonImage, CvPoint pointSet[], const Joint* pJoints, int whichone, JointType joint0, JointType joint1)
{
	TrackingState joint0State = pJoints[joint0].TrackingState;
	TrackingState joint1State = pJoints[joint1].TrackingState;

	// If we can't find either of these joints, exit
	if ((joint0State == TrackingState_NotTracked) || (joint1State == TrackingState_NotTracked))
	{
		return;
	}

	// Don't draw if both points are inferred
	if ((joint0State == TrackingState_Inferred) && (joint1State == TrackingState_Inferred))
	{
		return;
	}


	CvScalar color;
	switch (whichone) //跟踪不同的人显示不同的颜色   
	{
	case 0:
		color = cvScalar(255);
		break;
	case 1:
		color = cvScalar(0, 255);
		break;
	case 2:
		color = cvScalar(0, 0, 255);
		break;
	case 3:
		color = cvScalar(255, 255, 0);
		break;
	case 4:
		color = cvScalar(255, 0, 255);
		break;
	case 5:
		color = cvScalar(0, 255, 255);
		break;
	} 


	// We assume all drawn bones are inferred unless BOTH joints are tracked
	if ((joint0State == TrackingState_Tracked) && (joint1State == TrackingState_Tracked))
	{
		line(SkeletonImage, pointSet[joint0], pointSet[joint1], color, 2);
	}
	else
	{
		line(SkeletonImage, pointSet[joint0], pointSet[joint1], color, 2);
	}
}

void drawSkeleton(Mat& SkeletonImage, CvPoint pointSet[], const Joint* pJoints, int whichone)
{

	// Draw the bones

	// Torso
	DrawBone(SkeletonImage, pointSet, pJoints, whichone, JointType_Head, JointType_Neck);
	DrawBone(SkeletonImage, pointSet, pJoints, whichone, JointType_Neck, JointType_SpineShoulder);
	DrawBone(SkeletonImage, pointSet, pJoints, whichone, JointType_SpineShoulder, JointType_SpineMid);
	DrawBone(SkeletonImage, pointSet, pJoints, whichone, JointType_SpineMid, JointType_SpineBase);
	DrawBone(SkeletonImage, pointSet, pJoints, whichone, JointType_SpineShoulder, JointType_ShoulderRight);
	DrawBone(SkeletonImage, pointSet, pJoints, whichone, JointType_SpineShoulder, JointType_ShoulderLeft);
	DrawBone(SkeletonImage, pointSet, pJoints, whichone, JointType_SpineBase, JointType_HipRight);
	DrawBone(SkeletonImage, pointSet, pJoints, whichone, JointType_SpineBase, JointType_HipLeft);

	// Right Arm    
	DrawBone(SkeletonImage, pointSet, pJoints, whichone, JointType_ShoulderRight, JointType_ElbowRight);
	DrawBone(SkeletonImage, pointSet, pJoints, whichone, JointType_ElbowRight, JointType_WristRight);
	DrawBone(SkeletonImage, pointSet, pJoints, whichone, JointType_WristRight, JointType_HandRight);
	DrawBone(SkeletonImage, pointSet, pJoints, whichone, JointType_HandRight, JointType_HandTipRight);
	DrawBone(SkeletonImage, pointSet, pJoints, whichone, JointType_WristRight, JointType_ThumbRight);

	// Left Arm
	DrawBone(SkeletonImage, pointSet, pJoints, whichone, JointType_ShoulderLeft, JointType_ElbowLeft);
	DrawBone(SkeletonImage, pointSet, pJoints, whichone, JointType_ElbowLeft, JointType_WristLeft);
	DrawBone(SkeletonImage, pointSet, pJoints, whichone, JointType_WristLeft, JointType_HandLeft);
	DrawBone(SkeletonImage, pointSet, pJoints, whichone, JointType_HandLeft, JointType_HandTipLeft);
	DrawBone(SkeletonImage, pointSet, pJoints, whichone, JointType_WristLeft, JointType_ThumbLeft);

	// Right Leg
	DrawBone(SkeletonImage, pointSet, pJoints, whichone, JointType_HipRight, JointType_KneeRight);
	DrawBone(SkeletonImage, pointSet, pJoints, whichone, JointType_KneeRight, JointType_AnkleRight);
	DrawBone(SkeletonImage, pointSet, pJoints, whichone, JointType_AnkleRight, JointType_FootRight);

	// Left Leg
	DrawBone(SkeletonImage, pointSet, pJoints, whichone, JointType_HipLeft, JointType_KneeLeft);
	DrawBone(SkeletonImage, pointSet, pJoints, whichone, JointType_KneeLeft, JointType_AnkleLeft);
	DrawBone(SkeletonImage, pointSet, pJoints, whichone, JointType_AnkleLeft, JointType_FootLeft);
}

