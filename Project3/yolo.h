#pragma once
#pragma once
#include<iostream>
#include<opencv2/opencv.hpp>
#include<string>
#define DLL_API extern "C" _declspec(dllexport)


#define YOLO_P6 false //是否使用P6模型
struct MyRect {
	int x;
	int y;
	int width;
	int height;
};
struct Output {
	int id;             //结果类别id
	float confidence;   //结果置信度
	MyRect box;       //矩形框
};
struct StructInferYolo {
	float boxThreshold;
	float classThreshold;
	float nmsThreshold;
};
class Yolo {
public:
	cv::dnn::Net net;
	Yolo() {

	}
	~Yolo() {}

	int readModel(const char* netPath, int isCuda);
	int Detect(cv::Mat img, StructInferYolo yoloStrcut, int classNum, std::vector<Output>& output);
	void drawPred(cv::Mat img_path, std::vector<std::string> className, std::vector<Output> result, int length);

private:

#if(defined YOLO_P6 && YOLO_P6==true)
	const float netAnchors[4][6] = { { 19,27, 44,40, 38,94 },{ 96,68, 86,152, 180,137 },{ 140,301, 303,264, 238,542 },{ 436,615, 739,380, 925,792 } };

	const int netWidth = 1280;  //ONNX图片输入宽度
	const int netHeight = 1280; //ONNX图片输入高度

	const int strideSize = 4;  //stride size
#else
	const float netAnchors[3][6] = { { 10,13, 16,30, 33,23 },{ 30,61, 62,45, 59,119 },{ 116,90, 156,198, 373,326 } };

	const int netWidth = 640;   //ONNX图片输入宽度
	const int netHeight = 640;  //ONNX图片输入高度

	const int strideSize = 3;   //stride size
#endif // YOLO_P6

	const float netStride[4] = { 8, 16.0,32,64 };


};



