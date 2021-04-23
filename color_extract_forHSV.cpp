#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

#define WINDOW_NAME "��Ч��ͼ���ڡ�"        //Ϊ���ڱ��ⶨ��ĺ� 

//void pickHighLightFire(Mat& inputFrame, Mat& outputFrame);
void on_MouseHandle(int event, int x, int y, int flags, void* param);
int main()
{
	int multiple = 1;//ͼƬ�ķŴ���
	Mat inputImage = imread(".jpg");//��������Լ����ļ�·����
	Mat outputImage;
	resize(inputImage, inputImage, Size(multiple * inputImage.cols, multiple * inputImage.rows));
	cvtColor(inputImage, outputImage, COLOR_BGR2HSV);

	//�����������ص�����
	namedWindow(WINDOW_NAME);
	setMouseCallback(WINDOW_NAME, on_MouseHandle, (void*)&outputImage);
	imshow(WINDOW_NAME, inputImage);
	while (1)
	{
		if (waitKey(10) == 27) break;//����ESC���������˳�
	}
	waitKey();
	return 0;
}

void on_MouseHandle(int event, int x, int y, int flags, void* param)
{

	Mat& image = *(cv::Mat*) param;
	switch (event)
	{
		//���������Ϣ
	case EVENT_LBUTTONDOWN:
	{

		cout << static_cast<int>(image.at<Vec3b>(y, x)[0]) << ",";
		cout << static_cast<int>(image.at<Vec3b>(y, x)[1]) << ",";
		cout << static_cast<int>(image.at<Vec3b>(y, x)[2]) << endl;
	}
	break;
	}
}