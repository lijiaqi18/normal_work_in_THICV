#include <opencv2\opencv.hpp>
#include<opencv2\highgui.hpp>
#include<thread>
#include<windows.h>

using namespace cv;
using namespace std;

VideoCapture capture1, capture2,capture3,capture4;
Mat frame1, frame2, frame3, frame4;

void camera1();
void camera2();
void camera3();
void camera4();

int main() {
	namedWindow("Camera-1", 0);
	moveWindow("Camera-1", 0, 0);
	resizeWindow("Camera-1", 320, 240);
	namedWindow("Camera-2", 0);
	moveWindow("Camera-2", 480, 0);
	resizeWindow("Camera-2", 320, 240);
	namedWindow("Camera-3", 0);
	moveWindow("Camera-3", 0, 360);
	resizeWindow("Camera-3", 320, 240);
	namedWindow("Camera-4", 0);
	moveWindow("Camera-4", 480, 360);
	resizeWindow("Camera-4", 320, 240);
	thread t1(camera1);
	t1.detach();
	thread t2(camera2);
	t2.detach();
	thread t3(camera3);
	t3.detach();
	thread t4(camera4);
	t4.detach();
	system("pause");

		/*waitKey(20);

		if (waitKey(20) == 'q') {
			break;
		}
	*/	
	
	cout << "camera read end" << endl;
	return 0;
}

void camera1() {
	while (true) {
		capture1.open(1,CAP_DSHOW);
		capture1.read(frame1);
		
		imshow("Camera-1", frame1);
		waitKey(20);
		Sleep(100);
		if (waitKey(20) == 'q') {
			break;
		}
	}
}

void camera2() {
	while (true) {
		capture1.open(2, CAP_DSHOW);
		capture1.read(frame2);
		
		imshow("Camera-2", frame2);
		waitKey(20);
		Sleep(100);
		if (waitKey(20) == 'q') {
			break;
		}
	}
}

void camera3() {
	while (true) {
		capture1.open(3, CAP_DSHOW);
		capture1.read(frame3);
		
		imshow("Camera-3", frame3);
		waitKey(20);
		Sleep(100);
		if (waitKey(20) == 'q') {
			break;
		}
	}
}

void camera4() {
	while (true) {
		capture1.open(4, CAP_DSHOW);
		capture1.read(frame4);
		
		imshow("Camera-4", frame4);
		waitKey(20);
		Sleep(100);
		if (waitKey(20) == 'q') {
			break;
		}
	}
}