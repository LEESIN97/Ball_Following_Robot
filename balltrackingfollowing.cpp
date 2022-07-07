#define brightness 40
#define ga_in 20

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <time.h>
#include <cmath>
#include "serialcomm.h"

using namespace cv;
using namespace std;

int threshold1 = 100;
int tolerance = 50;
Vec3b lower_1, upper_1, lower_2, upper_2, lower_3, upper_3;
CSerialComm serialComm;

void mouse_callback(int event, int x, int y, int flags, void* param);

void dummy(int, void*);

void Send_data(BYTE data);

void Connect_Uart_Port(const char* _portNum);

void DisConnect_Uart_Port();

int main()
{
	Connect_Uart_Port("COM3");

	Mat img_hsv;
	Mat img_color;
	VideoCapture cap(2);

	/*double fps = cap.get(CAP_PROP_FPS);
	cout << "Frames per second camera : " << fps << endl;*/

	int num_frames = 1;

	clock_t start;
	clock_t end;
	double fpsLive;

	cap.set(CAP_PROP_FRAME_HEIGHT, 480);
	cap.set(CAP_PROP_FRAME_WIDTH, 640);
	cap.set(CAP_PROP_BRIGHTNESS, brightness);
	cap.set(CAP_PROP_GAIN, ga_in);

	int key = -1;

	if (!cap.isOpened()) {
		cout << "error!" << endl;
		return -1;
	}

	namedWindow("img_color");
	setMouseCallback("img_color", mouse_callback, (void*)&img_color);
	createTrackbar("threshold", "img_color", &threshold1, 255, dummy);
	setTrackbarPos("threshold", "img_color", 30);

	while (1) {

		start = clock();

		cap.read(img_color);

		threshold1 = getTrackbarPos("threshold", "img_color");

		if (img_color.empty())
		{
			cout << "over" << endl;
			break;
		}

		cvtColor(img_color, img_hsv, COLOR_BGR2HSV);

		Mat img_mask1, img_mask2, img_mask3, img_mask;
		inRange(img_hsv, lower_1, upper_1, img_mask1);
		inRange(img_hsv, lower_2, upper_2, img_mask2);
		inRange(img_hsv, lower_3, upper_3, img_mask3);
		img_mask = img_mask1 | img_mask2 | img_mask3;

		int morph_size = 2;
		Mat element = getStructuringElement(MORPH_RECT, Size(2 * morph_size + 1, 2 * morph_size + 1),
			Point(morph_size, morph_size));

		morphologyEx(img_mask, img_mask, MORPH_OPEN, element);
		morphologyEx(img_mask, img_mask, MORPH_CLOSE, element);


		Mat img_result;
		bitwise_and(img_color, img_color, img_result, img_mask);

		Mat img_labels, stats, centroids;
		int numOfLabels = connectedComponentsWithStats(img_mask, img_labels, stats, centroids, 8, CV_32S);
		int plus_numOflabels = numOfLabels + 1;
		int* area = new int[plus_numOflabels];
		int* left = new int[plus_numOflabels];
		int* top = new int[plus_numOflabels];
		int* width = new int[plus_numOflabels];
		int* height = new int[plus_numOflabels];
		double* centerX = new double[plus_numOflabels];
		double* centerY = new double[plus_numOflabels];

		for (int i = 1; i < plus_numOflabels; i++) {
			area[i] = stats.at<int>(i, CC_STAT_AREA);
			left[i] = stats.at<int>(i, CC_STAT_LEFT);
			top[i] = stats.at<int>(i, CC_STAT_TOP);
			width[i] = stats.at<int>(i, CC_STAT_WIDTH);
			height[i] = stats.at<int>(i, CC_STAT_HEIGHT);

			centerX[i] = centroids.at<double>(i, 0);
			centerY[i] = centroids.at<double>(i, 1);

			if (area[i] > 200) {
				circle(img_color, Point(centerX[i], centerY[i]), 5, Scalar(255, 0, 0), 1);
				rectangle(img_color, Point(left[i], top[i]), Point(left[i] + width[i], top[i] + height[i]), Scalar(0, 0, 255), 1);
			}

		}

		double max_centerX = centerX[1]; // 판단 하는 객체중 가장 넓이가 큰 객체의 x 좌표
		double max_centerY = centerY[1]; // y좌표
		int max_width = width[1];
		int max_height = height[1];
		int max_area = area[1];

		for (int i = 1; i < plus_numOflabels; i++) {
			if (area[i] >= area[1]) {
				max_area = area[i];
				max_centerX = centerX[i];
				max_centerY = centerY[i];
				max_width = width[i];
				max_height = height[i];
			}
		}


		::imshow("img_color", img_color);
		::imshow("img_mask", img_mask);

		int windowCenter = 320;
		int centerBuffer = 10;
		int leftBound, rightBound;
		leftBound = int(windowCenter - centerBuffer);
		rightBound = int(windowCenter + centerBuffer);
		char error = 0;
		char mode;


		if (numOfLabels == 1) {
			mode = 's'; //stop
			error = 0;
		}
		else {
			if (max_width < 550 && max_height < 350) {


				if (max_centerX < leftBound || max_centerX > rightBound) {

					error = abs(windowCenter - max_centerX); // default 속도에 error에 weight 를 곱해줘서 속도 조절

					if (max_centerX < leftBound) { //left side
						if ((max_height / 2 > 50 || max_width / 2 > 50) && max_centerX < 110)
							mode = 'r'; // 우회전 오른쪽 바퀴는 dafault speed의 2/5
						else
							mode = 'R'; // 우회전이긴 한데 좀 더 빠르게 우회전 하게 끔 오른쪽은 default pwm 왼쪽은 error에 가중치 곱한 거 더한 값 
					}
					else if (max_centerX > rightBound) { //right side
						if ((max_height / 2 > 50 || max_width / 2 > 50) && max_centerX > 540)
							mode = 'l'; // 'r'과 마찬가지로 약한 좌회전
						else
							mode = 'L'; // 'R'과 마찬가지로 강한 좌회전
					}
				}


				else// 중간지점
					if (max_height / 2 < 40 || max_width / 2 < 40) { //설정한 크기보다 작으면 멀어서 가까이 가기 위해
						mode = 'd'; //default speed 모드
						error = 0;
					}
					else {
						mode = 'f'; // 정지
						error = 0;
					}

			}

			else mode = 'b';

		}


		char error_c = char(error / 3);



		key = waitKeyEx(1);
		if (key != -1) {
			if (key == 2424832) {
				cap.set(CAP_PROP_BRIGHTNESS, cap.get(CAP_PROP_BRIGHTNESS) + 1);
			}
			if (key == 2555904) {
				cap.set(CAP_PROP_BRIGHTNESS, cap.get(CAP_PROP_BRIGHTNESS) - 1);
			}

			if (key == 0x230000) break;
		}

		Send_data('t'); //시작
		Send_data(mode);
		Send_data(error_c);
		Send_data('e'); //끝


		end = clock();

		//cout << mode << endl;
		double seconds = (double(end) - double(start)) / double(CLOCKS_PER_SEC);
		//cout << "Time taken : " << seconds << " seconds" << endl;

		fpsLive = double(num_frames) / double(seconds);
		//cout << "Estimated frames per second : " << fpsLive << endl;

		::putText(img_result, "FPS: " + to_string(fpsLive), { 50, 50 }, FONT_HERSHEY_PLAIN, 1.5, (255, 255, 255), 1, 8);
		::imshow("img_result", img_result);

		delete[] area;
		delete[] left;
		delete[] top;
		delete[] width;
		delete[] height;
		delete[] centerX;
		delete[] centerY;
	}
	DisConnect_Uart_Port();


	return 0;
}

void mouse_callback(int event, int x, int y, int flags, void* param) //마우스 클릭시 호출
{
	Mat img_color = *(Mat*)param;

	if (event == EVENT_LBUTTONDOWN) // 마우스 왼쪽 클릭시
	{
		Vec3b color_pixel = img_color.at<Vec3b>(y, x); //클릭한 위치의 픽셀값 읽어옴

		Mat bgr_color = Mat(1, 1, CV_8UC3, color_pixel); //cvt를 적용하기 위해 이미지로 변환

		Mat hsv_color;

		cvtColor(bgr_color, hsv_color, COLOR_BGR2HSV); //색공간 변환

		int hue = hsv_color.at<Vec3b>(0, 0)[0]; //hue 값을 대입
		int saturation = hsv_color.at<Vec3b>(0, 0)[1]; // saturation 값 대입
		int value = hsv_color.at<Vec3b>(0, 0)[2]; //value 값 대입

		if (hue < 4) {
			lower_1 = Vec3b(hue - 4 + 180, threshold1, threshold1);
			upper_1 = Vec3b(180, 255, 255);
			lower_2 = Vec3b(0, threshold1, threshold1);
			upper_2 = Vec3b(hue, 255, 255);
			lower_3 = Vec3b(hue, threshold1, threshold1);
			upper_3 = Vec3b(hue + 4, 255, 255);
		}

		else if (hue > 176) {
			lower_1 = Vec3b(hue, threshold1, threshold1);
			upper_1 = Vec3b(180, 255, 255);
			lower_2 = Vec3b(0, threshold1, threshold1);
			upper_2 = Vec3b(hue + 4 - 180, 255, 255);
			lower_3 = Vec3b(hue - 4, threshold1, threshold1);
			upper_3 = Vec3b(hue, 255, 255);
		}

		else {
			lower_1 = Vec3b(hue, threshold1, threshold1);
			upper_1 = Vec3b(hue + 4, 255, 255);
			lower_2 = Vec3b(hue - 4, threshold1, threshold1);
			upper_2 = Vec3b(hue, 255, 255);
			lower_3 = Vec3b(hue - 4, threshold1, threshold1);
			upper_3 = Vec3b(hue, 255, 255);
		}

		cout << "hue = " << hue << endl;
		cout << "#1 = " << lower_1 << "~" << upper_1 << endl;
		cout << "#2 = " << lower_2 << "~" << upper_2 << endl;
		cout << "#3 = " << lower_3 << "~" << upper_3 << endl;

	}
}

void dummy(int, void*) {
}

void Send_data(BYTE data) {
	if (!serialComm.sendCommand(data)) {
	}
}

void Connect_Uart_Port(const char* _portNum) {
	if (!serialComm.connect(_portNum))
	{
		cout << "connect faliled";
		return;
	}
}
void DisConnect_Uart_Port() {
	serialComm.disconnect();
}

