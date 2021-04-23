#include <opencv2\opencv.hpp>
#include<opencv2\highgui.hpp>
 

using namespace cv;
using namespace std;

struct CircleData {
	Point2f center;
	float radius;
};

void color_detect(Mat const &frame_input) {
	Mat channelbgr[3];
	Mat im_blue, im_green, im_red, blue_blob, green_blob, red_blob;
	Mat element;
	Mat red_temp, blue_temp, green_temp;

	vector<vector<Point>> contours[3];//每个contour[i]都是存储边缘的双重向量
	vector<RotatedRect> current_frame_blobs[3];//三种颜色的块集，//car_data_structure.h引入Blob结构体，某一颜色的“块集”

	//找最小斜矩形
	vector<Point> current_contour;
	double pixel_count;//面积绝对值
	RotatedRect current_bounding_rect;
	vector<Point> center_positions;
	double rect_diagonal_size;
	double rect_size;//矩形面积
	double aspect_ratio;//宽高比
	if (frame_input.empty()) {
		return;
	}
	//前面还有一步透视变换，畸变之后确保摄像头镜头平行路面，和摄像头标定一样需要实际场地操作
	//这里从色彩分割开始

	split(frame_input,channelbgr);//分离通道
	im_blue = channelbgr[0];
	im_green = channelbgr[1];
	im_red = channelbgr[2];

	//确保颜色所属
	blue_blob = (im_blue - im_green) + (im_blue - im_red);
	green_blob = (im_green - im_blue) + (im_green - im_red);
	red_blob = (im_red - im_blue) + (im_red - im_green);

	//形态学运算内核
	element = getStructuringElement(MORPH_RECT,Size(3,3),Point(2,2));//内核形状为矩形
	morphologyEx(blue_blob, blue_temp, MORPH_OPEN, element);//开运算，先腐蚀后膨胀
	morphologyEx(blue_temp, blue_blob, MORPH_CLOSE, element);//闭运算
	morphologyEx(green_blob, green_temp, MORPH_OPEN, element);
	morphologyEx(green_temp, green_blob, MORPH_CLOSE, element);
	morphologyEx(red_blob, red_temp, MORPH_OPEN, element);
	morphologyEx(red_temp, red_blob, MORPH_CLOSE, element);

	inRange(blue_blob, 150, 255, blue_blob);
	inRange(green_blob, 200, 255, green_blob);    //形态学处理与彩色图像分割之后的blob
	inRange(red_blob, 200, 255, red_blob);

	findContours(blue_blob, contours[0], RETR_EXTERNAL, CHAIN_APPROX_NONE);
	findContours(red_blob, contours[2], RETR_EXTERNAL, CHAIN_APPROX_NONE);

	//计算每条轮廓的凸包
	vector<vector<Point>> convexHulls[3];
	for (int i = 0; i < 3; i++) {
		convexHulls[i].resize(contours[i].size());
		for (int j = 0; j < contours[i].size(); j++) {
			convexHull(contours[i][j],convexHulls[i][j]);  //convexHull函数，输出为convesHulls默认凸包逆时针
		}
	}
		
	for (int i = 0; i < 3; i++) {
		current_frame_blobs[i].resize(0);
		int len = convexHulls[i].size();
		for (int j = 0; j < len; j++) {
			current_contour = convexHulls[i][j];
			pixel_count = contourArea(current_contour);
			current_bounding_rect = minAreaRect(current_contour);
			rect_diagonal_size = sqrt(pow(current_bounding_rect.size.width, 2) + pow(current_bounding_rect.size.height, 2));
			rect_size = (double)current_bounding_rect.size.width * (double)current_bounding_rect.size.height;
			aspect_ratio = current_bounding_rect.size.width / current_bounding_rect.size.height;
			

			if (rect_size > 150 && rect_size < 450)
				current_frame_blobs[i].emplace_back(current_bounding_rect);
			else if (pixel_count > 250 && pixel_count < 900)
				current_frame_blobs[i].emplace_back(current_bounding_rect);

			int length = current_frame_blobs[i].size();
			for (int k = 0; k < length; k++) {
				Point2f rect_points[4];
				current_frame_blobs[i][k].points(rect_points);
				for (int l = 0; l < 4; l++) {
					if (i == 0)
						line(frame_input, rect_points[l], rect_points[(l + 1) % 4], Scalar(255, 0, 0), 1, 8);

					else if (i == 2)
						line(frame_input, rect_points[l], rect_points[(l + 1) % 4], Scalar(0, 0, 255), 1, 8);
				}
			}
		}
		
		
	}

	vector<Point> red_centers;
	vector<Point> blue_centers;

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

int main() {
	/*
	VideoCapture capture;
	capture.open("r+p.mp4");
	if (!capture.isOpened())
		return 0;
	*/
	Mat frame_input = imread("test_circle.png");

	while (1) {
		//capture >> frame_input;
		if (frame_input.empty())
			break;


		//color_detect(frame_input);
		namedWindow("test_circle", 0);
		//moveWindow("cloudcars - Camera 1", 0, 0);
		//resizeWindow("Camera-1", 640, 480);

		color_detect(frame_input);
		imshow("Camera-1", frame_input);

		waitKey(100);

		if (waitKey(20) == 'q') {
			//imwrite("30.jpg",frame_input);
			break;
		}
		
	}
	return 0;
}