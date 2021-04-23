#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include <opencv2/highgui.hpp>
#include <string>
#include <vector>
#include <fstream>
using namespace std;
using namespace cv;
#define PATH "D:\\OpenCV_text\\opencv_test\\"
#define NUM 7
int main() {
    // �����������浼���ͼƬ
    Mat image_in;
    // �������������ļ�·��������
    vector<string> filelist;
    // ��������������ת��ƽ�ƾ��������
    vector<Mat> rvecs, tvecs;
    // ����������󣬻������
    Mat cameraMatrix;
    Mat distCoeffs;
    int flags = 0;
    // ���屣��ͼ���ά�ǵ������
    vector<Point2f> corners;
    // ���屣��ͼ����ά�ǵ������
    vector<vector<Point2f> > corners2;
    // ���屣��ͼ���ά����ά�ǵ������
    vector<Point3f> worldPoints;
    vector<vector<Point3f> > worldPoints2;
    //***************��ȡһ���ļ����е�����ͼƬ�����б궨ͼƬ��**********************
    for (int i = 1; i < NUM; i++) {
        stringstream str;
        str << PATH << setw(2) << setfill('0') << i << ".jpg";
        // ��������ͼƬ��·������������filelist��
        filelist.push_back(str.str());
        image_in = imread(str.str());
    }
    //imshow("hahaha", image_in);
    //waitKey(20);

    //***********************����һ��object_points*************************
    for (int j = 0; j < 5; j++) {
        for (int k = 0; k < 9; k++) {
            worldPoints.push_back(Point3f(j * 1.0, k * 1.0, 0.0f));
        }
    }
    //***************************�ҽǵ����������������������������������������������������������������
    for (int i = 0; i < filelist.size(); i++) {
        //cout <<filelist[i]<<endl;
        // һ���Ŷ���ͼƬ��
        image_in = imread(filelist[i]);
        // ��ͼƬ�Ľǵ㣬�����ֱ�Ϊ��
        // ����ͼƬ��ͼƬ�ڽǵ������������̸������Ľǵ㣩������ǵ㣬��ⷽʽ
        bool found = findChessboardCorners(image_in, Size(9, 5), corners, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
        // ���ҵ��Ľǵ���������У�
        corners2.push_back(corners);
        //�����ǵ�
        drawChessboardCorners(image_in, Size(9, 5), corners, found);
        //��ʾͼ��
        imshow("test", image_in);
        // ͼ��ˢ�µȴ�ʱ�䣬��λms
        waitKey(100);
        // ��������ϵ�Ķ�άvector ������άvector
        worldPoints2.push_back(worldPoints);
    }

    calibrateCamera(worldPoints2, corners2, image_in.size(), cameraMatrix, distCoeffs, rvecs, tvecs, CV_CALIB_RATIONAL_MODEL);
    //*************************************�鿴����*****************************************
    cout << " Camera intrinsic: " << cameraMatrix.rows << "x" << cameraMatrix.cols << endl;
    cout << cameraMatrix.at<double>(0, 0) << " " << cameraMatrix.at<double>(0, 1) << " " << cameraMatrix.at<double>(0, 2) << endl;
    cout << cameraMatrix.at<double>(1, 0) << " " << cameraMatrix.at<double>(1, 1) << " " << cameraMatrix.at<double>(1, 2) << endl;
    cout << cameraMatrix.at<double>(2, 0) << " " << cameraMatrix.at<double>(2, 1) << " " << cameraMatrix.at<double>(2, 2) << endl;
    cout << distCoeffs.rows << "x" << distCoeffs.cols << endl;
    cout << distCoeffs << endl;
    for (int i = 0; i < distCoeffs.cols; i++)
    {
        cout << distCoeffs.at<double>(0, i) << " ";
    }
    cout << endl;
    worldPoints.clear();
    //*********************�������**************************
// ����Ҫ������ͼƬ
    /*
    Mat test_image2 = imread("D:\\OpenCV_text\\opencv_test\\02.jpg");
    Mat show_image;
    undistort(test_image2, show_image, cameraMatrix, distCoeffs);
    imwrite("corrected.jpg", show_image);
    waitKey(20);
    */
    return 0;
}