#include<iostream>
#include <opencv2\opencv.hpp>
#include<opencv2\highgui.hpp>
using namespace std;
using namespace cv;

enum class COLOR { BLUE, GREEN, RED, YELLOW, BLACK, PURPLE, ORANGE };//枚举类型
vector<float> areaRatios;
vector<RotatedRect> rects;
struct BlockData//矩形块参数结构体
{
	COLOR color;
	float centerX;
	float centerY;
	float edgeLength;
	int flag;
};
struct CircleData{
	Point2f center;
	float radius;
};
struct FilterBlock
{
	RotatedRect rect;
	COLOR color;
};
COLOR color = COLOR::RED;
BlockData blockData;

Mat colorDetect(Mat srcImageBGR, COLOR color)
{
	Mat _srcImageBGR = srcImageBGR.clone();//输入图像另存到变量_srcImageBGR中
	Mat srcImageHSV, channelsHSV[3];
	Mat dstImageGray;
	//resize(_srcImageBGR, _srcImageBGR, Size(640, 480));
	cvtColor(_srcImageBGR, srcImageHSV, COLOR_BGR2HSV);//输入_srcImageBGR，输出srcImageHSV，转换色彩空间
	split(srcImageHSV, channelsHSV);
	int h, s, v;
	int H, S, V;
	int h1 = 156, H1 = 180;
	switch (color)
	{
	case COLOR::BLUE:
		h = 100; H = 124;
		s = 43; S = 255;
		v = 46; V = 255;
		break;
	case COLOR::GREEN:
		h = 35; H = 77;
		s = 120; S = 255;
		v = 80; V = 255;
		break;
	case COLOR::RED:
		h = 0; H = 10;
		s = 43; S = 255;
		v = 46; V = 255;
		break;
	case COLOR::YELLOW:
		h = 26; H = 34;
		s = 43; S = 255;
		v = 46; V = 255;
		break;
	case COLOR::BLACK:
		h = 0; H = 180;
		s = 0; S = 255;
		v = 0; V = 80;
	case COLOR::PURPLE:
		h = 125; H = 155;
		s = 43; S = 255;
		v = 46; V = 255;
		break;
	case COLOR::ORANGE:
		h = 11; H = 25;
		s = 43; S = 255;
		v = 46; V = 255;
		break;
	default:
		h = 0; H = 10;
		s = 43; S = 255;
		v = 46; V = 255;
		break;
	}
	inRange(srcImageHSV, Scalar(h, s, v), Scalar(H, S, V), dstImageGray);//按HSV上下限分割图像，输出为dstImageGray，三个通道均在上下限之内时输出为255，否则为0
	if (color == COLOR::RED)//针对红色的再处理，后续可调整
	{
		Mat dstImageGray1;//灰度图
		inRange(srcImageHSV, Scalar(h1, s, v), Scalar(H1, S, V), dstImageGray1);
		dstImageGray = dstImageGray + dstImageGray1;//两次分割结果作“或”运算
	}
	//blur(dstImageGray, dstImageGray, Size(9, 9));
	Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
	erode(dstImageGray, dstImageGray, element);
	erode(dstImageGray, dstImageGray, element);//腐蚀两次
	//erode(dstImageGray, dstImageGray, element);
	dilate(dstImageGray, dstImageGray, element);//膨胀一次
	//GaussianBlur(dstImageGray, dstImageGray, Size(9, 9), 0);
	return dstImageGray;
}

float computeAreaRatio(Mat imgBin, RotatedRect rotateRect)//计算轮廓区域的比例,imgBin是colorDetect的输出灰度图
{
	Point2f recPoint[4];
	Point centerPoint;//矩阵中心点
	rotateRect.points(recPoint);//将参数中旋转矩阵的四个角的坐标写入recPoint中
	float countValue = 0;
	int rectWidth = (int)(sqrt(pow((recPoint[0].x - recPoint[1].x), 2) + pow((recPoint[0].y - recPoint[1].y), 2)));
	int rectHeight = (int)(sqrt(pow((recPoint[1].x - recPoint[2].x), 2) + pow((recPoint[1].y - recPoint[2].y), 2)));
	centerPoint.x = (recPoint[0].x + recPoint[2].x) / 2;
	centerPoint.y = (recPoint[0].y + recPoint[2].y) / 2;
	int initX = (int)(centerPoint.x - rectWidth / 8.0);
	int initY = (int)(centerPoint.y - rectHeight / 8.0);
	initX = initX >= 0 ? initX : 0;
	initY = initY >= 0 ? initY : 0;
	int endX = initX + rectWidth / 4.0;
	int endY = initY + rectHeight / 4.0;
	endX = endX < imgBin.cols ? endX : imgBin.cols;
	endY = endY < imgBin.rows ? endY : imgBin.rows;
	for (int j = initX; j < endX; j++)
	{
		for (int i = initY; i < endY; i++)
		{
			float binValue = imgBin.at<unsigned char>(i, j);
			if (binValue == 255)   countValue++;
		}
	}
	if (rectWidth * rectHeight == 0) return 1.0;
	else
	{
		return countValue / (rectWidth * rectHeight / 16.0);
	}
}

vector<RotatedRect> blockDetect(Mat dstImageGray)//输出类型为旋转矩阵组成的向量
{
	vector<vector<Point>> contours;//存放所有轮廓（Point的向量）的向量
	vector<Vec4i> hieracy;//Vec4i，4个int组成的向量
	findContours(dstImageGray, contours, hieracy, RETR_EXTERNAL, CHAIN_APPROX_NONE);
	vector<vector<Point>> contours_poly(contours.size());
	vector<RotatedRect> rotateRects;

	for (size_t i = 0; i < contours.size(); i++)
	{
		float contLength = arcLength(contours[i], true);//第二个参数为ture，输出的是封闭曲线的周长
		float contArea = contourArea(contours[i]);//像素点数
		//多边拟合函数
		approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);//相差3以内的点保留下来
		if ((contLength > 10) && (contArea > 10) && (contours_poly[i].size()<5)) // 待调节,角点数量判断，区分圆形与矩形
		{
			RotatedRect rect;
			Point2f recPoint[4];
			
			//返回最小区域边界斜矩形
			rect = minAreaRect(Mat(contours_poly[i]));
			rotateRects.push_back(rect);
		}
	}
	return rotateRects;
}

vector<RotatedRect> blockSelect(vector<RotatedRect> rotateRects, Mat binImage)
{
	if (rotateRects.size() <= 1)
	{
		return rotateRects;
	}
	vector<RotatedRect> selectedRects;
	/*float centerX, centerY;
	float rectWidth, rectHeight;
	float maxDisToCenter = 1000, currentDis;
	float maxEdge = 500, currentMaxEdge;
	float maxCenterX = 0, maxCenterY = 0;
	for (size_t i = 0; i < rotateRects.size(); i++)
	{
		RotatedRect rect = rotateRects[i];
		Point2f recPoint[4];
		rect.points(recPoint);
		if (recPoint[0].x < 0 || recPoint[0].y < 0 || recPoint[1].x>640 || recPoint[1].y < 0 ||
			recPoint[2].x>640 || recPoint[0].y>480 || recPoint[3].x < 0 || recPoint[3].y>480)
		{
			continue;
		}
		centerX = (recPoint[0].x + recPoint[2].x) / 2;
		centerY = (recPoint[0].y + recPoint[2].y) / 2;
		rectWidth = sqrt(pow((recPoint[0].x - recPoint[1].x), 2) + pow((recPoint[0].y - recPoint[1].y), 2));
		rectHeight = sqrt(pow((recPoint[1].x - recPoint[2].x), 2) + pow((recPoint[1].y - recPoint[2].y), 2));

		if ((centerX > maxCenterX) && (centerY > maxCenterY))
		{
			temRects.clear();
			temRects.push_back(rect);
			maxCenterX = centerX;
			maxCenterY = centerY;
		}
	}*/
	for (int i = 0; i < rotateRects.size(); i++)
	{
		Point2f recPoint[4];
		rotateRects[i].points(recPoint);
		float rectWidth, rectHeight, H_W, W_H, areaRatio, ratio;
		rectWidth = sqrt(pow((recPoint[0].x - recPoint[1].x), 2) + pow((recPoint[0].y - recPoint[1].y), 2));
		rectHeight = sqrt(pow((recPoint[1].x - recPoint[2].x), 2) + pow((recPoint[1].y - recPoint[2].y), 2));
		H_W = abs(rectHeight / rectWidth);
		W_H = abs(rectWidth / rectHeight);
		areaRatio = computeAreaRatio(binImage, rotateRects[i]);
		ratio = H_W > W_H ? W_H : H_W;
		if (ratio >= 0.6 && areaRatio >= 0.4)
		{
			selectedRects.push_back(rotateRects[i]);
			areaRatios.push_back(areaRatio);
		}
	}
	cout << "running blockselect " << rotateRects.size() << "   " << "   " << selectedRects.size() << endl;
	return selectedRects;
}

/*
Description: draw rotate rectangles surrounding the blocks
Arguments:
srcImage - the source image where the rectangles are drawn on.
rotateRects - rectangles to be drawn.
Return: None
*/
void drawRect(Mat srcImage, vector<RotatedRect> rects, string windowName)
{
	Mat _srcImage = srcImage.clone();
	Point2f rectPoint[4];
	Point centerPoint;
	//for (size_t i = 0; i < rects.size(); i++)
	//{
	//	Scalar drawColor = Scalar(255,255,255);
	//	rects[i].points(rectPoint);
	//	for (int i = 0; i < 4; i++)
	//	{
	//		line(_srcImage, rectPoint[i], rectPoint[(i + 1) % 4], drawColor, 4);

	//	}
	//	centerPoint.x = (rectPoint[0].x + rectPoint[2].x) / 2.0; 
	//	centerPoint.y = (rectPoint[0].y + rectPoint[2].y) / 2.0;	
	//	//putText(_srcImage,to_string(areaRatios[i]) , centerPoint, 1, 1, drawColor);
	//	//circle(_srcImage, centerPoint, 4, drawColor, -1, 8, 0);
	//}
	if (rects.size() == 2)
	{
		Point2f center1, center2, center1s[4], center2s[4];
		rects[0].points(center1s);
		rects[1].points(center2s);
		center1.x = (int)((center1s[0].x + center1s[2].x) / 2);
		center2.x = (int)((center2s[0].x + center2s[2].x) / 2);
		center1.y = (int)((center1s[0].y + center1s[2].y) / 2);
		center2.y = (int)((center2s[0].y + center2s[2].y) / 2);
		line(_srcImage, center1, center2, Scalar(255, 255, 85), 2, 8, 0);
		cout << center1.x << "   " << center1.y << "  " << center2.x << "  " << center2.y << endl;
	}
	else
	{

	}
	imshow(windowName, _srcImage);
}

vector<CircleData> circleDetect(Mat dstImageGray) {//检测灰度图中的圆形
	vector<vector<Point>> contours;//存放所有轮廓（Point的向量）的向量
	vector<Vec4i> hieracy;//Vec4i，4个int组成的向量，图形拓扑结构信息，同一等级下的下一个、上一个轮廓，子轮廓、父轮廓
	findContours(dstImageGray, contours, hieracy, RETR_EXTERNAL, CHAIN_APPROX_NONE);
	vector<vector<Point>> contours_poly(contours.size());//存放拟合后的多边形
	vector<CircleData> circles;

	for (size_t i = 0; i < contours.size(); i++)
	{
		float contLength = arcLength(contours[i], true);//第二个参数为ture，输出的是封闭曲线的周长
		float contArea = contourArea(contours[i]);//像素点数
		//多边拟合函数
		approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);//角点很多时认为是圆
		if ((contLength > 10) && (contArea > 10) && (contours_poly[i].size() > 10)) // 待调节
		{
			CircleData circle;
			Point2f recPoint[4];

			//返回最小区域边界圆形
			minEnclosingCircle(contours_poly[i], circle.center, circle.radius);
			circles.push_back(circle);
		}
	}
	return circles;
}

void drawCircle(Mat srcImage, vector<CircleData> circles, string windowName) {
	Mat _srcImage = srcImage.clone();
	for (int i = 0; i < circles.size(); i++) {
		circle(_srcImage, circles[i].center, circles[i].radius, Scalar(255, 0, 0), 4);
	}
	imshow(windowName, _srcImage);
}

/*
* +-
* +-
Description: get block data with specified color.
Arguments:
srcImage - source BGR-format image.
color - an enum type specifying the color amoung BLUE, GREEN, RED and YELLOW.
drawResult - draw the rectangle surrounding the block (default: true).
windowName - window name of the result (default: "Result").
Return: struct type used for saving the block data.
*/
BlockData getBlockData(Mat srcImage, COLOR color, bool drawResult, string windowName)//主函数
{
	BlockData blockData;
	blockData.centerX = 0;
	blockData.centerY = 0;
	blockData.color = color;
	blockData.edgeLength = 0;
	blockData.flag = 0;
	Mat binImage = colorDetect(srcImage, color);
	imshow("Gray Image", binImage);
	vector<RotatedRect> rotateRects = blockDetect(binImage);
	vector<RotatedRect> selectedRects = blockSelect(rotateRects, binImage);
	vector<CircleData> circles = circleDetect(binImage);
	if (drawResult && (selectedRects.size() >= 1))
	{
		drawRect(srcImage, selectedRects, windowName);
	}
	if (drawResult && (circles.size() >= 1))
	{
		drawCircle(srcImage, circles, "circle_test");
	}
	return blockData;
}

float computeDistance(RotatedRect rect1, RotatedRect rect2) {
	Point2f p1[4], p2[4], center1, center2;
	rect1.points(p1);
	rect2.points(p2);
	center1.x = (int)((p1[0].x + p1[3].x) / 2);
	center1.y = (int)((p1[0].y + p1[3].y) / 2);
	center2.x = (int)((p2[0].x + p2[3].x) / 2);
	center2.y = (int)((p2[0].y + p2[3].y) / 2);
	float distance = sqrt(pow((center1.x - center2.x), 2) + pow((center2.y - center2.y), 2));
	return distance;
}

void carDetect(Mat srcimage, COLOR color1, COLOR color2, bool drawresult, string windowname) {
	rects.clear();
	Mat img1 = colorDetect(srcimage, color1);
	vector<RotatedRect> rects1 = blockSelect(blockDetect(img1), img1);
	vector<FilterBlock> blocks1, blocks2;
	Mat img2 = colorDetect(srcimage, color2);
	vector<RotatedRect> rects2 = blockSelect(blockDetect(img2), img2);
	for (int x = 0; x < rects1.size(); x++)
	{
		for (int y = 0; y < rects2.size(); y++) {
			float dis = computeDistance(rects1[x], rects2[y]);
			cout << "distance between " << x << " and " << y << " is : " << dis << endl;
			if (dis < 20)
			{
				rects.push_back(rects1[x]);
				rects.push_back(rects2[y]);
			}
		}
	}
	drawRect(srcimage, rects, windowname);

}



int main() {
	VideoCapture capture;
	capture.open(1);
	if (!capture.isOpened())
		return 0;
	Mat frame_input;

	while (1) {
		capture >> frame_input;
		if (frame_input.empty())
			break;


		//color_detect(frame_input);
		namedWindow("test", 0);
		//moveWindow("cloudcars - Camera 1", 0, 0);
		//resizeWindow("Camera-1", 640, 480);

		if (frame_input.empty()) {
			return 0;
		}

		blockData = getBlockData(frame_input, color, true, "test");
		imshow("test", frame_input);

		waitKey(100);

		if (waitKey(20) == 'q') {
			//imwrite("30.jpg",frame_input);
			break;
		}

	}
	return 0;
}