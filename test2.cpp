/*
By longxiaoyue wunnnn@hotmail.com
ʹ��directshow�����Զ�ȡ100fps640*480��MJPGѹ����Ƶ,���ǲ��ܼ�����ʾ������ֻ�ܵ�60֡
����Ҫ�����Լ�������ͷ����Ч
�ŵ㣺�����������࣬���ϰ汾���㡣
���Ի���  i7 5557u vs2017  DebugX64 OpenCV3.3
*/

#include<opencv2\core.hpp>
#include<opencv2\opencv.hpp>
#include<opencv2\highgui\highgui_c.h>
#include<opencv2/imgproc.hpp>
using namespace cv;
using namespace std;

int _main()
{
	VideoCapture cap;
	cap = VideoCapture(CAP_DSHOW);	//ʹ��DirectShow
	cap.open(1);						//�����ҵ�USB����ͷ
	if (!cap.isOpened())
		return -1;
	cap.set(CAP_PROP_FOURCC, CAP_OPENCV_MJPEG);//����ΪMJPG��ʽ
	cap.set(CAP_PROP_FRAME_HEIGHT, 640);
	cap.set(CAP_PROP_FRAME_WIDTH, 480);//�°�opencvȥ����cv
	TickMeter tm;//��ʱ

	while (1)
	{

		tm.reset();
		tm.start();
		Mat frame;
		for (int i = 0; i < 100; i++)//Is not accuracy when I count once.
		{
			cap >> frame;
			imshow("frame", frame);
			if (waitKey(1) == 27);//ESC for quit
		}
		tm.stop();
		cout << 100 / tm.getTimeSec() << "fps" << endl;//���֡��
	}
	return 0;
}