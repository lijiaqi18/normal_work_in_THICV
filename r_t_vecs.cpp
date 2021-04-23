#include <iostream>
#include <iomanip>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include "opencv2/calib3d.hpp"
using namespace std;
using namespace cv;

//将相机标定过程封装到CameraCalibrator类中
class CameraCalibrator {
private:
	// 输入点：
	// 世界坐标系中的点
	//（每个正方形为一个单位）
	std::vector<std::vector<cv::Point3f>> objectPoints;
	// 点在图像中的位置（以像素为单位）
	std::vector<std::vector<cv::Point2f>> imagePoints;
	// 输出矩阵
	cv::Mat cameraMatrix;//相机矩阵
	cv::Mat distCoeffs;//畸变系数摄像机的5个畸变系数，(k1,k2,p1,p2[,k3[,k4,k5,k6]])
	// 输出旋转量和平移量
	std::vector<cv::Mat> rvecs, tvecs;
	//x 和y 映射功能
	Mat map1;
	Mat map2;
	// 指定标定方式的标志
	int flag;
	bool mustInitUndistort;
	//保存点数据
	void addPoints(const std::vector<cv::Point2f>& imageCorners, const std::vector<cv::Point3f>& objectCorners)
	{
		//一个视图中的二维图像
		imagePoints.push_back(imageCorners);
		// 对应的三维场景点
		objectPoints.push_back(objectCorners);
	}
public:
	CameraCalibrator() :flag(0), mustInitUndistort(true) {}
	// 打开棋盘图像，提取角点
	int addChessboardPoints(
		const std::vector<std::string>& filelist1, // 文件名列表
		cv::Size& boardSize)
	{ // 标定面板的大小
		// 棋盘上的角点
		std::vector<cv::Point2f> imageCorners;
		std::vector<cv::Point3f> objectCorners;
		// 场景中的三维点：
		// 在棋盘坐标系中，初始化棋盘中的角点
		// 角点的三维坐标(X,Y,Z)= (i,j,0)
		for (int i = 0; i < boardSize.height; i++) {
			for (int j = 0; j < boardSize.width; j++) {
				objectCorners.push_back(cv::Point3f(i, j, 0.0f));
			}
		}
		// 图像上的二维点：
		cv::Mat image; // 用于存储棋盘图像
		int successes = 0;
		// 处理所有视角
		for (int i = 1; i < filelist1.size(); i++) {
			// 打开图像
			image = cv::imread(filelist1[i], 0);
			// 取得棋盘中的角点
			bool found = cv::findChessboardCorners(
				image, // 包含棋盘图案的图像
				boardSize, // 图案的大小
				imageCorners); // 检测到角点的列表
				// 取得角点上的亚像素级精度
			if (found) {
				cv::cornerSubPix(image, imageCorners,
					cv::Size(5, 5), // 搜索窗口的半径
					cv::Size(-1, -1),
					cv::TermCriteria(cv::TermCriteria::MAX_ITER +
						cv::TermCriteria::EPS, 30, // 最大迭代次数
						0.1)); // 最小精度
						// 如果棋盘是完好的，就把它加入结果
				if (imageCorners.size() == boardSize.area()) {
					// 加入从同一个视角得到的图像和场景点
					addPoints(imageCorners, objectCorners);
					successes++;
				}
			}
			//画出角点
			cv::drawChessboardCorners(image, boardSize, imageCorners, found);
			cv::imshow("Corners on Chessboard", image);
			cv::waitKey(100);
		}
		return successes;
	}

	// 标定相机
// 返回重投影误差
	double calibrate(const cv::Size& imageSize) {
		mustInitUndistort = true;

		// 开始标定
		return
			calibrateCamera(objectPoints, // 三维点
				imagePoints, // 图像点
				imageSize, // 图像尺寸
				cameraMatrix, // 输出相机矩阵
				distCoeffs, // 输出畸变矩阵
				rvecs, tvecs, // Rs、Ts（外参）
				flag); // 设置选项

	}
	// 去除图像中的畸变（标定后）
	cv::Mat remap(const cv::Mat& image) {
		cv::Mat undistorted;
		if (mustInitUndistort) { // 每个标定过程调用一次
			cv::initUndistortRectifyMap(
				cameraMatrix, // 计算得到的相机矩阵
				distCoeffs, // 计算得到的畸变矩阵
				cv::Mat(), // 可选矫正项（无）
				cv::Mat(), // 生成无畸变的相机矩阵
				image.size(), // 无畸变图像的尺寸
				CV_32FC1, // 输出图片的类型
				map1, map2); // x 和y 映射功能
			mustInitUndistort = false;
		}
		// 应用映射功能
		cv::remap(image, undistorted, map1, map2,
			cv::INTER_LINEAR); // 插值类型
		return undistorted;
	}
	void setCalibrationFlag(bool radial8CoeffEnabled, bool tangentialParamEnabled) {

		// Set the flag used in cv::calibrateCamera()
		flag = 0;
		if (!tangentialParamEnabled) flag += CV_CALIB_ZERO_TANGENT_DIST;//设定切向畸变参数（p1,p2）为零。 
		if (radial8CoeffEnabled) flag += CV_CALIB_RATIONAL_MODEL;//计算k4，k5，k6三个畸变参数。如果没有设置，则只计算其它5个畸变参数。标定函数使用倾斜传感器模型并返回14个系数
	}
	// 常成员函数，获得相机内参数矩阵、投影矩阵数据
	cv::Mat getCameraMatrix() const { return cameraMatrix; }
	cv::Mat getDistCoeffs() const { return distCoeffs; }
	std::vector<cv::Mat> getrvecs() { return rvecs; }
	std::vector<cv::Mat> gettvecs() { return tvecs; }
};
//主程序
int main()
{
	cv::Mat image;
	vector<string> filelist;//vector是容器

	// 生成棋盘图像文件名列表
	// 命名棋盘01至棋盘27棋盘子目录
	for (int i = 0; i < 20; i++) {

		std::stringstream str;//批量读取图片
		str << "D:\\vs2017project\\ConsoleApplication1\\ConsoleApplication1\\chessboards\\chessboard" << std::setw(2) << std::setfill('0') << i << ".jpg";
		std::cout << str.str() << std::endl;

		filelist.push_back(str.str());
		image = cv::imread(str.str(), 0);

		cv::imshow("Board Image", image);
		cv::waitKey(100);

	}
	// 创造标定对象
	CameraCalibrator cameraCalibrator;
	// 在棋盘中标记角点
	cv::Size boardSize(6, 4);
	cameraCalibrator.addChessboardPoints(
		filelist,	// 棋盘图像的文件名
		boardSize);	// 棋盘大小

	// 标定相机
	cameraCalibrator.setCalibrationFlag(false, true);
	cameraCalibrator.calibrate(image.size());
	Mat rotation_matrix;
	std::vector<cv::Mat> rvecs1 = cameraCalibrator.getrvecs();
	std::vector<cv::Mat> tvecs1 = cameraCalibrator.gettvecs();
	//输出外参
	for (int i = 0; i < 19; i++)
	{

		cout << "第" << i + 1 << "幅图像的旋转向量：" << endl;
		cout << rvecs1[i] << endl;
		Rodrigues(rvecs1[i], rotation_matrix);
		cout << "第" << i + 1 << "幅图像的旋转矩阵：" << endl;
		cout << rotation_matrix << endl;

		cout << "第" << i + 1 << "幅图像的平移向量：" << endl;
		cout << tvecs1[i] << endl;

	}

	//图像失真举例
	Mat image1 = cv::imread(filelist[14], 0);
	cv::Size newSize(static_cast<int>(image1.cols * 1.5), static_cast<int>(image1.rows * 1.5));
	cv::Mat uImage = cameraCalibrator.remap(image1);

	// 显示摄像机矩阵
	cv::Mat cameraMatrix = cameraCalibrator.getCameraMatrix();
	std::cout << " Camera intrinsic: " << cameraMatrix.rows << "x" << cameraMatrix.cols << std::endl;//相机内参
	std::cout << cameraMatrix.at<double>(0, 0) << " " << cameraMatrix.at<double>(0, 1) << " " << cameraMatrix.at<double>(0, 2) << std::endl;
	std::cout << cameraMatrix.at<double>(1, 0) << " " << cameraMatrix.at<double>(1, 1) << " " << cameraMatrix.at<double>(1, 2) << std::endl;
	std::cout << cameraMatrix.at<double>(2, 0) << " " << cameraMatrix.at<double>(2, 1) << " " << cameraMatrix.at<double>(2, 2) << std::endl;

	cv::namedWindow("Original Image");
	cv::imshow("Original Image", image1);
	cv::namedWindow("Undistorted Image");
	cv::imshow("Undistorted Image", uImage);
	//打印畸变系数矩阵（1*5矩阵）
	Mat distCoeffs = cameraCalibrator.getDistCoeffs();
	std::cout << "畸变系数矩阵：" << distCoeffs.rows << "x" << distCoeffs.cols << std::endl;
	for (int i = 0; i < distCoeffs.rows; i++)
		for (int j = 0; j < distCoeffs.cols; j++)
			cout << distCoeffs.at<double>(i, j) << "\t";

	// 把所有文件存储在xml文件
	cv::FileStorage fs("calib.xml", cv::FileStorage::WRITE);
	fs << "Intrinsic" << cameraMatrix;
	fs << "Distortion" << cameraCalibrator.getDistCoeffs();


	cv::waitKey();
	return 0;
}
