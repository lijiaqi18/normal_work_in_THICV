/*
By longxiaoyue wunnnn@hotmail.com
使用directshow，可以读取100fps640*480，MJPG压缩视频,但是不能加上显示，否则只能到60帧
设置要符合自己的摄像头才有效
优点：读出来的是类，比老版本方便。
测试环境  i7 5557u vs2017  DebugX64 OpenCV3.3
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
	cap = VideoCapture(CAP_DSHOW);	//使用DirectShow
	cap.open(1);						//这是我的USB摄像头
	if (!cap.isOpened())
		return -1;
	cap.set(CAP_PROP_FOURCC, CAP_OPENCV_MJPEG);//设置为MJPG格式
	cap.set(CAP_PROP_FRAME_HEIGHT, 640);
	cap.set(CAP_PROP_FRAME_WIDTH, 480);//新版opencv去掉了cv
	TickMeter tm;//计时

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
		cout << 100 / tm.getTimeSec() << "fps" << endl;//输出帧率
	}
	return 0;
}