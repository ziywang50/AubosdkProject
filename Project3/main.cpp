#include "MvCameraControl.h"
#include "yolo.h"
#include <iostream>
#include<opencv2//opencv.hpp>
#include<math.h>
#include<string>
#include "example.h"
#include <armadillo>
#include <windows.h>
#include <opencv2/core/utils/logger.hpp>
#include <csignal>
//#include <opencv2/core.hpp>
//#include <opencv2/videoio.hpp>
//#include <opencv2/highgui.hpp>

using namespace std;
using namespace cv;
using namespace dnn;
#define ROBOT_ADDR "192.168.214.6"
#define ROBOT_PORT 8899

static volatile int keepRunning = 1;

//注释了海康摄像机sdk部分
/*void sig_handler(int sig)
{
	if (sig == SIGINT)
	{
		keepRunning = 0;
	}
}*/

/*bool Convert2Mat(MV_FRAME_OUT_INFO_EX* pstImageInfo, unsigned char* pData)
{
	//monochrome
	cv::Mat srcImage;
	if (pstImageInfo->enPixelType == PixelType_Gvsp_Mono8)
	{
		srcImage = cv::Mat(pstImageInfo->nHeight, pstImageInfo->nWidth, CV_8UC1, pData);
	}
	else
	{
		printf("unsupported pixel format\n");
		return false;
	}

	if (NULL == srcImage.data)
	{
		return false;
	}

	//save converted image in a local file
	try {
		cv::imwrite("C:/Users/win10/Downloads/yolo/Project3/imagefile.jpg", srcImage);
	}
	catch (cv::Exception& ex) {
		fprintf(stderr, "Exception saving image to bmp format: %s\n", ex.what());
	}

	srcImage.release();

	return true;
}

bool PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo)
{
	if (NULL == pstMVDevInfo)
	{
		printf("The Pointer of pstMVDevInfo is NULL!\n");
		return false;
	}
	if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
	{
		int nIp1 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
		int nIp2 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
		int nIp3 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
		int nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);

		// print current ip and user defined name
		printf("CurrentIp: %d.%d.%d.%d\n", nIp1, nIp2, nIp3, nIp4);
		printf("UserDefinedName: %s\n\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
	}
	else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
	{
		printf("UserDefinedName: %s\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
		printf("Serial Number: %s\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chSerialNumber);
		printf("Device Number: %d\n\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.nDeviceNumber);
	}
	else
	{
		printf("Not support.\n");
	}

	return true;
}*/

int main()
{
	//海康相机部分
	/*int nRet = MV_OK;
	void* handle = NULL;
	unsigned int g_nPayloadSize = 0;
	static volatile int keepRunning = 1;
	signal(SIGINT, sig_handler);
	while (keepRunning)
	{
		cv::Mat frame;
		// Enum device
		MV_CC_DEVICE_INFO_LIST stDeviceList;
		memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
		nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
		if (MV_OK != nRet)
		{
			printf("Enum Devices fail! nRet [0x%x]\n", nRet);
			break;
		}

		if (stDeviceList.nDeviceNum > 0)
		{
			for (unsigned int i = 0; i < stDeviceList.nDeviceNum; i++)
			{
				printf("[device %d]:\n", i);
				MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
				if (NULL == pDeviceInfo)
				{
					break;
				}
				PrintDeviceInfo(pDeviceInfo);
			}
		}
		else
		{
			printf("Find No Devices!\n");
			break;
		}
		// Select device and create handle
		nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[0]);
		if (MV_OK != nRet)
		{
			printf("Create Handle fail! nRet [0x%x]\n", nRet);
			break;
		}

		// open device
		nRet = MV_CC_OpenDevice(handle);
		if (MV_OK != nRet)
		{
			printf("Open Device fail! nRet [0x%x]\n", nRet);
			break;
		}

		// Detection network optimal package size(It only works for the GigE camera)
		if (stDeviceList.pDeviceInfo[0]->nTLayerType == MV_GIGE_DEVICE)
		{
			int nPacketSize = MV_CC_GetOptimalPacketSize(handle);
			if (nPacketSize > 0)
			{
				nRet = MV_CC_SetIntValue(handle, "GevSCPSPacketSize", nPacketSize);
				if (nRet != MV_OK)
				{
					printf("Warning: Set Packet Size fail nRet [0x%x]!", nRet);
				}
			}
			else
			{
				printf("Warning: Get Packet Size fail nRet [0x%x]!", nPacketSize);
			}
		}

		// Set trigger mode as off
		nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 0);
		if (MV_OK != nRet)
		{
			printf("Set Trigger Mode fail! nRet [0x%x]\n", nRet);
			break;
		}

		// Get payload size
		MVCC_INTVALUE stParam;
		memset(&stParam, 0, sizeof(MVCC_INTVALUE));
		nRet = MV_CC_GetIntValue(handle, "PayloadSize", &stParam);
		if (MV_OK != nRet)
		{
			printf("Get PayloadSize fail! nRet [0x%x]\n", nRet);
			break;
		}
		g_nPayloadSize = stParam.nCurValue;

		// Start grab image
		nRet = MV_CC_StartGrabbing(handle);
		if (MV_OK != nRet)
		{
			printf("Start Grabbing fail! nRet [0x%x]\n", nRet);
			break;
		}

		MV_FRAME_OUT_INFO_EX stImageInfo = { 0 };
		memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
		unsigned char* pData = (unsigned char*)malloc(sizeof(unsigned char) * (g_nPayloadSize));
		if (pData == NULL)
		{
			printf("Allocate memory failed.\n");
			break;
		}

		// get one frame from camera with timeout=1000ms
		nRet = MV_CC_GetOneFrameTimeout(handle, pData, g_nPayloadSize, &stImageInfo, 1000);
		if (nRet == MV_OK)
		{
			printf("Get One Frame: Width[%d], Height[%d], nFrameNum[%d]\n",
				stImageInfo.nWidth, stImageInfo.nHeight, stImageInfo.nFrameNum);
		}
		else
		{
			printf("No data[0x%x]\n", nRet);
			free(pData);
			pData = NULL;
			break;
		}

		// 数据去转换
		bool bConvertRet = false;
		// if ( 0 == nFormat )
		// {
		bConvertRet = Convert2Mat(&stImageInfo, pData);
		//}
		//else
		//{
			//bConvertRet = Convert2Ipl(&stImageInfo, pData);
		//}
		// print result
		if (bConvertRet)
		{
			printf("OpenCV format convert finished.\n");
			free(pData);
			pData = NULL;
		}
		else
		{
			printf("OpenCV format convert failed.\n");
			free(pData);
			pData = NULL;
			break;
		}*/
		//..............................................................................................
		cv::utils::logging::setLogLevel(utils::logging::LOG_LEVEL_SILENT);
		Mat frame;
		//--- INITIALIZE VIDEOCAPTURE
		VideoCapture cap(0, cv::CAP_DSHOW);
		if (!cap.isOpened()) {
			cerr << "ERROR! Unable to open camera\n";
			return -1;
		}
		Sleep(500);
		cap.read(frame);
		imshow("CAMERA 1", frame);
 		waitKey(200);
		//cap.open(0);
		// const char* img_path = "C:\\Users\\win10\\Downloads\\yolo\\Project3\\imagefile.jpg";
		const char* img_path = "C:/Users/win10/Downloads/yolo/Project3/imagefile.jpg";
		// frame = imread(img_path);
		imwrite(img_path, frame);
		const char* model_path = "C:\\Users\\win10\\yolov5s-sim.onnx";
		//int num_devices = cv::cuda::getCudaEnabledDeviceCount();
		//if (num_devices <= 0) {
			//cerr << "There is no cuda." << endl;
			//return -1;
		//}
		//else {
			//cout << num_devices << endl;
		//}

		Yolo test;
		Net net;
		if (test.readModel(model_path, true)) {
			cout << "read net ok!" << endl;
		}
		else {
			return -1;
		}
		Output outputList1[2];
		auto start = std::chrono::system_clock::now();
		StructInferYolo yoloStrcut;
		yoloStrcut.boxThreshold = 0.25;
		yoloStrcut.classThreshold = 0.25;
		yoloStrcut.nmsThreshold = 0.45;
		int classNum = 80;
		std::vector<Output> result;
		if (test.Detect(frame, yoloStrcut, classNum, result)) {
			cout << "Detect success!" << endl;
			RSHD g_rshd = -1;
			if (example_login(g_rshd, ROBOT_ADDR, ROBOT_PORT)) {
				aubo_robot_namespace::Pos pos = convertcoordinates(g_rshd, result);
				example_moveJ(g_rshd);
				example_moveL(g_rshd, pos);
				result.clear();
				while (true) {
					auto start = std::chrono::system_clock::now();
					frame = Mat::zeros(frame.rows, frame.cols, CV_64FC1);
					//--- INITIALIZE VIDEOCAPTURE
					if (!cap.isOpened()) {
						cerr << "ERROR! Unable to open camera\n";
						return -1;
					}
					cap.open(0);
					//sleep 500s is neccessary for this USB camera. Other cameras may be different.
					Sleep(500);
					cap.read(frame);
					imshow("1", frame);
					waitKey(200);
					//海康相机部分
					// get one frame from camera with timeout=1000ms
					/*nRet = MV_CC_GetOneFrameTimeout(handle, pData, g_nPayloadSize, &stImageInfo, 1000);
					if (nRet == MV_OK)
					{
						printf("Get One Frame: Width[%d], Height[%d], nFrameNum[%d]\n",
							stImageInfo.nWidth, stImageInfo.nHeight, stImageInfo.nFrameNum);
					}
					else
					{
						printf("No data[0x%x]\n", nRet);
						free(pData);
						pData = NULL;
						break;
					}

					// 数据去转换
					bool bConvertRet = false;
					bConvertRet = Convert2Mat(&stImageInfo, pData);
					if (bConvertRet)
					{
						printf("OpenCV format convert finished.\n");
						free(pData);
						pData = NULL;
					}
					else
					{
						printf("OpenCV format convert failed.\n");
						free(pData);
						pData = NULL;
						break;
					}*/
					//end camera
					auto end = std::chrono::system_clock::now();
					std::cout << "inference time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;
					if (test.Detect(frame, yoloStrcut, classNum, result)) {
						aubo_robot_namespace::Pos pos = convertcoordinates(g_rshd, result);
						example_moveL(g_rshd, pos);
					}
					result.clear();
				}
			}
		}
		else {
			cout << "Detect Failed!" << endl;
		}
	//}

	// 海康Stop grab image
	/*nRet = MV_CC_StopGrabbing(handle);
	if (MV_OK != nRet)
	{
		printf("Stop Grabbing fail! nRet [0x%x]\n", nRet);
	}

	// Close device
	nRet = MV_CC_CloseDevice(handle);
	if (MV_OK != nRet)
	{
		printf("ClosDevice fail! nRet [0x%x]\n", nRet);
	}

	// Destroy handle
	nRet = MV_CC_DestroyHandle(handle);
	if (MV_OK != nRet)
	{
		printf("Destroy Handle fail! nRet [0x%x]\n", nRet);
	}*/
	return 0;
}

//yolo2