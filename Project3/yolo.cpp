#include"yolo.h"
#include <iostream>
#include <string>
#include <fstream>
#include <comdef.h>
using namespace std;
using namespace cv;
using namespace cv::dnn;
int Yolo::readModel(const char* netPath, int isCuda = 0) {
	try {

		this->net = readNet(netPath);
	}
	catch (const std::exception&) {
		return 0;
	}
	//cuda
	if (isCuda) {
		this->net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
		this->net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA_FP16);
	}
	//cpu
	else {
		this->net.setPreferableBackend(cv::dnn::DNN_BACKEND_DEFAULT);
		this->net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
	}
	return 1;
}
int Yolo::Detect(cv::Mat img, StructInferYolo yoloStrcut, int classNum, std::vector<Output>& output) {
	
	vector<string> receiveStrs;

	/*for (int i = 0; i < classNum; i++)
	{
		string str = *(receiveclassName + i);
		receiveStrs.push_back(str);
	}*/

	vector<Scalar> color;
	color.push_back(Scalar(0, 0, 255));
	color.push_back(Scalar(0, 0, 255));
	Mat SrcImg = img;
	Mat blob;
	int col = SrcImg.cols;
	int row = SrcImg.rows;
	int maxLen = MAX(col, row);
	Mat netInputImg = SrcImg.clone();
	if (maxLen > 1.2 * col || maxLen > 1.2 * row) {
		Mat resizeImg = Mat::zeros(maxLen, maxLen, CV_8UC3);
		SrcImg.copyTo(resizeImg(Rect(0, 0, col, row)));
		netInputImg = resizeImg;
	}
	blobFromImage(netInputImg, blob, 1 / 255.0, cv::Size(netWidth, netHeight), cv::Scalar(0, 0, 0), true, false);
	//如果在其他设置没有问题的情况下但是结果偏差很大，可以尝试下用下面两句语句
	//blobFromImage(netInputImg, blob, 1 / 255.0, cv::Size(netWidth, netHeight), cv::Scalar(104, 117, 123), true, false);
	//blobFromImage(netInputImg, blob, 1 / 255.0, cv::Size(netWidth, netHeight), cv::Scalar(114, 114,114), true, false);
	this->net.setInput(blob);
	std::vector<cv::Mat> netOutputImg;
	//vector<string> outputLayerName{"345","403", "461","output" };
	//net.forward(netOutputImg, outputLayerName[3]); //获取output的输出
	//while(true){
	auto start = std::chrono::system_clock::now();
	this->net.forward(netOutputImg, net.getUnconnectedOutLayersNames());
	auto end = std::chrono::system_clock::now();
	//std::cout << "inference time++++++++++++++++: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;
	//}
	std::vector<int> classIds;//结果id数组
	std::vector<float> confidences;//结果每个id对应置信度数组
	std::vector<cv::Rect> boxes;//每个id矩形框
	float ratio_h = (float)netInputImg.rows / netHeight;
	float ratio_w = (float)netInputImg.cols / netWidth;
	int net_width = classNum + 5;  //输出的网络宽度是类别数+5
	float* pdata = (float*)netOutputImg[0].data;
	for (int stride = 0; stride < strideSize; stride++) {    //stride
		int grid_x = (int)(netWidth / netStride[stride]);
		int grid_y = (int)(netHeight / netStride[stride]);
		for (int anchor = 0; anchor < 3; anchor++) {	//anchors
			const float anchor_w = netAnchors[stride][anchor * 2];
			const float anchor_h = netAnchors[stride][anchor * 2 + 1];
			for (int i = 0; i < grid_y; i++) {
				for (int j = 0; j < grid_x; j++) {
					if (pdata != NULL) {
						float box_score = pdata[4]; ;//获取每一行的box框中含有某个物体的概率
						if (box_score >= yoloStrcut.boxThreshold) {
							cv::Mat scores(1, classNum, CV_32FC1, pdata + 5);
							Point classIdPoint;
							double max_class_socre;
							minMaxLoc(scores, 0, &max_class_socre, 0, &classIdPoint);
							max_class_socre = (float)max_class_socre;
							if (max_class_socre >= yoloStrcut.classThreshold) {
								//rect [x,y,w,h]
								float x = pdata[0];  //x
								float y = pdata[1];  //y
								float w = pdata[2];  //w
								float h = pdata[3];  //h
								int left = (x - 0.5 * w) * ratio_w;
								int top = (y - 0.5 * h) * ratio_h;
								classIds.push_back(classIdPoint.x);
								confidences.push_back(max_class_socre * box_score);
								boxes.push_back(Rect(left, top, int(w * ratio_w), int(h * ratio_h)));
							}
						}

						pdata += net_width;//下一行
					}
					else

					{
						break;
					}
				}
			}
		}
	}

	//执行非最大抑制以消除具有较低置信度的冗余重叠框（NMS）
	
	vector<int> nms_result;
	//vector<Output> outputList;
	float nmsScoreThreshold = yoloStrcut.boxThreshold * yoloStrcut.classThreshold;
	NMSBoxes(boxes, confidences, nmsScoreThreshold, yoloStrcut.nmsThreshold, nms_result);
	vector<Output> outputList;
	for (int i = 0; i < nms_result.size(); i++) {
		int idx = nms_result[i];
		Output result;
		result.id = classIds[idx];
		result.confidence = confidences[idx];
		result.box.x = boxes[idx].x;
		result.box.y = boxes[idx].y;
		result.box.width = boxes[idx].width;
		result.box.height = boxes[idx].height;
		if (result.id == 0 || result.id == 67) {
			output.push_back(result);
		}
		//std::cout << "x:" << result.box.x << "y:" << result.box.y << "w:" << result.box.width << "h:" << result.box.height << "confidence" << result.confidence;

			/*outputList[i].id = classIds[idx];
			outputList[i].confidence = confidences[idx];
			outputList[i].box.x = boxes[idx].x;
			outputList[i].box.y = boxes[idx].y;
			outputList[i].box.width = boxes[idx].width;
			outputList[i].box.height = boxes[idx].height;*/
	}
	int nmsLength = output.size();
	std::vector<std::string> className = { "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light",
  "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
  "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
  "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
  "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
  "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
  "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone",
  "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
  "hair drier", "toothbrush" };
	if (nms_result.size())
	{
		drawPred(SrcImg, className, output, nmsLength);
		return 1;
	}
	else
		return 0;
	
	return 1;
}

std::string wstring2string(std::wstring wstr)
{
	// support chinese
	std::string res;
	int len = WideCharToMultiByte(CP_ACP, 0, wstr.c_str(), wstr.size(), nullptr, 0, nullptr, nullptr);
	if (len <= 0) {
		return res;
	}
	char* buffer = new char[len + 1];
	if (buffer == nullptr) {
		return res;
	}
	WideCharToMultiByte(CP_ACP, 0, wstr.c_str(), wstr.size(), buffer, len, nullptr, nullptr);
	buffer[len] = '\0';
	res.append(buffer);
	delete[] buffer;
	return res;
}



void Yolo::drawPred(cv::Mat img, std::vector<std::string> className, std::vector<Output> result, int length) {
	vector<Scalar> color;
	color.push_back(Scalar(0, 0, 255));
	for (size_t i = 0; i < length; i++) {
		int left, top;
		left = result[i].box.x;
		top = result[i].box.y;
		int color_num = i;
		cv::Rect a;
		a.x = result[i].box.x;
		a.y = result[i].box.y;
		a.width = result[i].box.width;
		a.height = result[i].box.height;
		rectangle(img, a, color[0], 2, 8);
		//std::wstring wStr = className[result[i].id];
		////std::string a= wstring2string(wStr);
		//std::string a(wStr.begin(), wStr.end());
		

		//string label = className[result[i].id] + ":" + to_string(result[i].confidence);
		string label = className[result[i].id] + ":" + to_string(result[i].confidence);
		int baseLine;
		Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
		top = max(top, labelSize.height);
		//rectangle(frame, Point(left, top - int(1.5 * labelSize.height)), Point(left + int(1.5 * labelSize.width), top + baseLine), Scalar(0, 255, 0), FILLED);
		putText(img, label, Point(left, top), FONT_HERSHEY_SIMPLEX, 1, color[0], 2);
		imshow("1", img);
		waitKey(800);
		destroyAllWindows();
	}



	//size = buf.size();
	//for (uchar& var : buf)
	//{
	//	*data = var;
	//	data++;
	//}

	//imshow("1", img);
	////imwrite("out.bmp", img);
	//waitKey(0);
	//destroyAllWindows();

}
