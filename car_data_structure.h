#pragma once
#include <opencv2\opencv.hpp>
#include<opencv2\highgui.hpp>
#include <sys/timeb.h>

using namespace cv;
using namespace std;

struct Blob
{
	// uint x_seed, y_seed;   //pixel location that can be used as seed for object
	float x_center, y_center;	  //center of blob (in pixel coords)
	// float x_floor, y_floor;		   //center of blob (in floor coords)
	double pixel_count;	  //number of pixels in blob
	// uint edge_pixel_count; //number of pixel on edge of image
	int color_id;
	uchar color;		   // color index,uchar即unsigned char表示整数时0~255
	// float min_x_dist;			   //number of pixels center of blob is from image right/left edge
	// uchar y_min, u_min, v_min;
	// uchar y_max, u_max, v_max;
	// float y_avg, u_avg, v_avg;
	// float y_std, u_std, v_std;
	int ID;
	double angle;
	// bool isattached;

	// For tracking
	// float boundingRectX, boundingRectY, boundingRectWidth, boundingRectHeight;
	vector<Point> currentContour;
	RotatedRect currentBoundingRect;
	vector<Point> centerPositions;
	vector<uint64_t> timestamps;
	vector<uint64_t> timestampsChangeSides;
	double dblCurrentDiagonalSize;
	double dblCurrentRectSize;
	double dblCurrentAspectRatio;
	//bool blnCurrentMatchFoundOrNewBlob;
	//bool blnStillBeingTracked;
	//int intNumOfConsecutiveFramesWithoutAMatch;
	cv::Point predictedNextPosition;
	cv::Point globalPosition;
	//bool blnID;
	//int attemptsOfIdentification;
	//int delayIdentification;
	//int intNumOfConsecutiveIdentification;
	//bool refuseMatchOrIdentify;
	// std::string strID;
	// cv::Point globalPosition;
	uint64_t getSystemTime2()
	{
		struct timeb t;

		ftime(&t);

		return 1000 * t.time + t.millitm;
	}
	// functions
	void initBlob(std::vector<cv::Point> _contour) {

		currentContour = _contour;

		pixel_count = contourArea(_contour, false);
		// create a minimum rectangle area from the contour using minAreaRect/create a bounding rectangle using boundingRect= bigger rectangle
		currentBoundingRect = minAreaRect(_contour);

		cv::Point currentCenter;
		// find the center point of the blob
		currentCenter = currentBoundingRect.center;
		// currentCenter.x = (currentBoundingRect.x + currentBoundingRect.x + currentBoundingRect.size.width) / 2;
		// currentCenter.y = (currentBoundingRect.y + currentBoundingRect.y + currentBoundingRect.size.height) / 2;

		centerPositions.emplace_back(currentCenter);

		dblCurrentDiagonalSize = sqrt(pow((float)currentBoundingRect.size.width, 2) + pow((float)currentBoundingRect.size.height, 2));
		dblCurrentRectSize = (double)currentBoundingRect.size.width * (double)currentBoundingRect.size.height;

		dblCurrentAspectRatio = (float)currentBoundingRect.size.width / (float)currentBoundingRect.size.height;

		/*
		blnStillBeingTracked = true;
		blnCurrentMatchFoundOrNewBlob = true;

		intNumOfConsecutiveFramesWithoutAMatch = 0;
		attemptsOfIdentification = 0;
		delayIdentification = 0;
		intNumOfConsecutiveIdentification = 0;

		// new attributes
		blnID = false;
		*/

		Point2f rect_points[4];
		currentBoundingRect.points(rect_points);//依次是左下，左上，右上，右下
		float dist1 = pow(rect_points[0].x - rect_points[1].x, 2) + pow(rect_points[0].y - rect_points[1].y, 2);//勾股计算宽高
		float dist2 = pow(rect_points[2].x - rect_points[1].x, 2) + pow(rect_points[2].y - rect_points[1].y, 2);
		if (dist1 > dist2) {
			float xd = rect_points[0].x - rect_points[1].x;
			float yd = rect_points[0].y - rect_points[1].y;
			if (xd < 0) {
				xd = -xd;
				yd = -yd;
			}
			angle = 180.0 / 3.1415926 * atan2f(yd, xd);//arctan(y/x)
			//自己定义了角度参数
		}
		else {
			float xd = rect_points[2].x - rect_points[1].x;
			float yd = rect_points[2].y - rect_points[1].y;
			if (xd < 0) {
				xd = -xd;
				yd = -yd;
			}
			angle = 180.0 / 3.1415926 * atan2f(yd, xd);
		}

		//refuseMatchOrIdentify = false;

	}

	/*
	void predictNextPosition(void) {

		int numPositions = (int)centerPositions.size();

		if (numPositions == 1) {

			predictedNextPosition.x = centerPositions.back().x;
			predictedNextPosition.y = centerPositions.back().y;

		}
		else if (numPositions == 2) {

			int deltaX = centerPositions[1].x - centerPositions[0].x;
			int deltaY = centerPositions[1].y - centerPositions[0].y;

			predictedNextPosition.x = centerPositions.back().x + deltaX;
			predictedNextPosition.y = centerPositions.back().y + deltaY;

		}
		else if (numPositions == 3) {

			int sumOfXChanges = ((centerPositions[2].x - centerPositions[1].x) * 2) +
				((centerPositions[1].x - centerPositions[0].x) * 1);

			int deltaX = (int)floor((float)sumOfXChanges / 3.0);

			int sumOfYChanges = ((centerPositions[2].y - centerPositions[1].y) * 2) +
				((centerPositions[1].y - centerPositions[0].y) * 1);

			int deltaY = (int)floor((float)sumOfYChanges / 3.0);

			predictedNextPosition.x = centerPositions.back().x + deltaX;
			predictedNextPosition.y = centerPositions.back().y + deltaY;

		}
		else if (numPositions == 4) {

			int sumOfXChanges = ((centerPositions[3].x - centerPositions[2].x) * 3) +
				((centerPositions[2].x - centerPositions[1].x) * 2) +
				((centerPositions[1].x - centerPositions[0].x) * 1);

			int deltaX = (int)floor((float)sumOfXChanges / 6.0);

			int sumOfYChanges = ((centerPositions[3].y - centerPositions[2].y) * 3) +
				((centerPositions[2].y - centerPositions[1].y) * 2) +
				((centerPositions[1].y - centerPositions[0].y) * 1);

			int deltaY = (int)floor((float)sumOfYChanges / 6.0);

			predictedNextPosition.x = centerPositions.back().x + deltaX;
			predictedNextPosition.y = centerPositions.back().y + deltaY;

		}
		else if (numPositions >= 5) {

			double sumOfXChanges = ((centerPositions[numPositions - 1].x - centerPositions[numPositions - 2].x) / ((double)timestamps[numPositions - 1] - (double)timestamps[numPositions - 2]) * 4) +
				((centerPositions[numPositions - 2].x - centerPositions[numPositions - 3].x) / ((double)timestamps[numPositions - 2] - (double)timestamps[numPositions - 3]) * 3) +
				((centerPositions[numPositions - 3].x - centerPositions[numPositions - 4].x) / ((double)timestamps[numPositions - 3] - (double)timestamps[numPositions - 4]) * 2) +
				((centerPositions[numPositions - 4].x - centerPositions[numPositions - 5].x) / ((double)timestamps[numPositions - 4] - (double)timestamps[numPositions - 5]) * 1);

			double deltaX = sumOfXChanges / 10.0;

			double sumOfYChanges = ((centerPositions[numPositions - 1].y - centerPositions[numPositions - 2].y) / ((double)timestamps[numPositions - 1] - (double)timestamps[numPositions - 2]) * 4) +
				((centerPositions[numPositions - 2].y - centerPositions[numPositions - 3].y) / ((double)timestamps[numPositions - 2] - (double)timestamps[numPositions - 3]) * 3) +
				((centerPositions[numPositions - 3].y - centerPositions[numPositions - 4].y) / ((double)timestamps[numPositions - 3] - (double)timestamps[numPositions - 4]) * 2) +
				((centerPositions[numPositions - 4].y - centerPositions[numPositions - 5].y) / ((double)timestamps[numPositions - 4] - (double)timestamps[numPositions - 5]) * 1);

			double deltaY = sumOfYChanges / 10.0;

			predictedNextPosition.x = centerPositions.back().x + (int)floor(deltaX * ((double)getSystemTime2() - (double)timestamps.back()));
			predictedNextPosition.y = centerPositions.back().y + (int)floor(deltaY * ((double)getSystemTime2() - (double)timestamps.back()));

		}
		

		else {
			// should never get here
		}
		
		// else if (numPositions >= 5) {

		// 	int sumOfXChanges = ((centerPositions[numPositions - 1].x - centerPositions[numPositions - 2].x) * 4) +
		// 		((centerPositions[numPositions - 2].x - centerPositions[numPositions - 3].x) * 3) +
		// 		((centerPositions[numPositions - 3].x - centerPositions[numPositions - 4].x) * 2) +
		// 		((centerPositions[numPositions - 4].x - centerPositions[numPositions - 5].x) * 1);

		// 	int deltaX = (int)floor((float)sumOfXChanges / 10.0);

		// 	int sumOfYChanges = ((centerPositions[numPositions - 1].y - centerPositions[numPositions - 2].y) * 4) +
		// 		((centerPositions[numPositions - 2].y - centerPositions[numPositions - 3].y) * 3) +
		// 		((centerPositions[numPositions - 3].y - centerPositions[numPositions - 4].y) * 2) +
		// 		((centerPositions[numPositions - 4].y - centerPositions[numPositions - 5].y) * 1);

		// 	int deltaY = (int)floor((float)sumOfYChanges / 10.0);

		// 	predictedNextPosition.x = centerPositions.back().x + deltaX;
		// 	predictedNextPosition.y = centerPositions.back().y + deltaY;

		// }

	}
	*/

};