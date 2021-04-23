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

//������궨���̷�װ��CameraCalibrator����
class CameraCalibrator {
private:
	// ����㣺
	// ��������ϵ�еĵ�
	//��ÿ��������Ϊһ����λ��
	std::vector<std::vector<cv::Point3f>> objectPoints;
	// ����ͼ���е�λ�ã�������Ϊ��λ��
	std::vector<std::vector<cv::Point2f>> imagePoints;
	// �������
	cv::Mat cameraMatrix;//�������
	cv::Mat distCoeffs;//����ϵ���������5������ϵ����(k1,k2,p1,p2[,k3[,k4,k5,k6]])
	// �����ת����ƽ����
	std::vector<cv::Mat> rvecs, tvecs;
	//x ��y ӳ�书��
	Mat map1;
	Mat map2;
	// ָ���궨��ʽ�ı�־
	int flag;
	bool mustInitUndistort;
	//���������
	void addPoints(const std::vector<cv::Point2f>& imageCorners, const std::vector<cv::Point3f>& objectCorners)
	{
		//һ����ͼ�еĶ�άͼ��
		imagePoints.push_back(imageCorners);
		// ��Ӧ����ά������
		objectPoints.push_back(objectCorners);
	}
public:
	CameraCalibrator() :flag(0), mustInitUndistort(true) {}
	// ������ͼ����ȡ�ǵ�
	int addChessboardPoints(
		const std::vector<std::string>& filelist1, // �ļ����б�
		cv::Size& boardSize)
	{ // �궨���Ĵ�С
		// �����ϵĽǵ�
		std::vector<cv::Point2f> imageCorners;
		std::vector<cv::Point3f> objectCorners;
		// �����е���ά�㣺
		// ����������ϵ�У���ʼ�������еĽǵ�
		// �ǵ����ά����(X,Y,Z)= (i,j,0)
		for (int i = 0; i < boardSize.height; i++) {
			for (int j = 0; j < boardSize.width; j++) {
				objectCorners.push_back(cv::Point3f(i, j, 0.0f));
			}
		}
		// ͼ���ϵĶ�ά�㣺
		cv::Mat image; // ���ڴ洢����ͼ��
		int successes = 0;
		// ���������ӽ�
		for (int i = 1; i < filelist1.size(); i++) {
			// ��ͼ��
			image = cv::imread(filelist1[i], 0);
			// ȡ�������еĽǵ�
			bool found = cv::findChessboardCorners(
				image, // ��������ͼ����ͼ��
				boardSize, // ͼ���Ĵ�С
				imageCorners); // ��⵽�ǵ���б�
				// ȡ�ýǵ��ϵ������ؼ�����
			if (found) {
				cv::cornerSubPix(image, imageCorners,
					cv::Size(5, 5), // �������ڵİ뾶
					cv::Size(-1, -1),
					cv::TermCriteria(cv::TermCriteria::MAX_ITER +
						cv::TermCriteria::EPS, 30, // ����������
						0.1)); // ��С����
						// �����������õģ��Ͱ���������
				if (imageCorners.size() == boardSize.area()) {
					// �����ͬһ���ӽǵõ���ͼ��ͳ�����
					addPoints(imageCorners, objectCorners);
					successes++;
				}
			}
			//�����ǵ�
			cv::drawChessboardCorners(image, boardSize, imageCorners, found);
			cv::imshow("Corners on Chessboard", image);
			cv::waitKey(100);
		}
		return successes;
	}

	// �궨���
// ������ͶӰ���
	double calibrate(const cv::Size& imageSize) {
		mustInitUndistort = true;

		// ��ʼ�궨
		return
			calibrateCamera(objectPoints, // ��ά��
				imagePoints, // ͼ���
				imageSize, // ͼ��ߴ�
				cameraMatrix, // ����������
				distCoeffs, // ����������
				rvecs, tvecs, // Rs��Ts����Σ�
				flag); // ����ѡ��

	}
	// ȥ��ͼ���еĻ��䣨�궨��
	cv::Mat remap(const cv::Mat& image) {
		cv::Mat undistorted;
		if (mustInitUndistort) { // ÿ���궨���̵���һ��
			cv::initUndistortRectifyMap(
				cameraMatrix, // ����õ����������
				distCoeffs, // ����õ��Ļ������
				cv::Mat(), // ��ѡ������ޣ�
				cv::Mat(), // �����޻�����������
				image.size(), // �޻���ͼ��ĳߴ�
				CV_32FC1, // ���ͼƬ������
				map1, map2); // x ��y ӳ�书��
			mustInitUndistort = false;
		}
		// Ӧ��ӳ�书��
		cv::remap(image, undistorted, map1, map2,
			cv::INTER_LINEAR); // ��ֵ����
		return undistorted;
	}
	void setCalibrationFlag(bool radial8CoeffEnabled, bool tangentialParamEnabled) {

		// Set the flag used in cv::calibrateCamera()
		flag = 0;
		if (!tangentialParamEnabled) flag += CV_CALIB_ZERO_TANGENT_DIST;//�趨������������p1,p2��Ϊ�㡣 
		if (radial8CoeffEnabled) flag += CV_CALIB_RATIONAL_MODEL;//����k4��k5��k6����������������û�����ã���ֻ��������5������������궨����ʹ����б������ģ�Ͳ�����14��ϵ��
	}
	// ����Ա�������������ڲ�������ͶӰ��������
	cv::Mat getCameraMatrix() const { return cameraMatrix; }
	cv::Mat getDistCoeffs() const { return distCoeffs; }
	std::vector<cv::Mat> getrvecs() { return rvecs; }
	std::vector<cv::Mat> gettvecs() { return tvecs; }
};
//������
int main()
{
	cv::Mat image;
	vector<string> filelist;//vector������

	// ��������ͼ���ļ����б�
	// ��������01������27������Ŀ¼
	for (int i = 0; i < 20; i++) {

		std::stringstream str;//������ȡͼƬ
		str << "D:\\vs2017project\\ConsoleApplication1\\ConsoleApplication1\\chessboards\\chessboard" << std::setw(2) << std::setfill('0') << i << ".jpg";
		std::cout << str.str() << std::endl;

		filelist.push_back(str.str());
		image = cv::imread(str.str(), 0);

		cv::imshow("Board Image", image);
		cv::waitKey(100);

	}
	// ����궨����
	CameraCalibrator cameraCalibrator;
	// �������б�ǽǵ�
	cv::Size boardSize(6, 4);
	cameraCalibrator.addChessboardPoints(
		filelist,	// ����ͼ����ļ���
		boardSize);	// ���̴�С

	// �궨���
	cameraCalibrator.setCalibrationFlag(false, true);
	cameraCalibrator.calibrate(image.size());
	Mat rotation_matrix;
	std::vector<cv::Mat> rvecs1 = cameraCalibrator.getrvecs();
	std::vector<cv::Mat> tvecs1 = cameraCalibrator.gettvecs();
	//������
	for (int i = 0; i < 19; i++)
	{

		cout << "��" << i + 1 << "��ͼ�����ת������" << endl;
		cout << rvecs1[i] << endl;
		Rodrigues(rvecs1[i], rotation_matrix);
		cout << "��" << i + 1 << "��ͼ�����ת����" << endl;
		cout << rotation_matrix << endl;

		cout << "��" << i + 1 << "��ͼ���ƽ��������" << endl;
		cout << tvecs1[i] << endl;

	}

	//ͼ��ʧ�����
	Mat image1 = cv::imread(filelist[14], 0);
	cv::Size newSize(static_cast<int>(image1.cols * 1.5), static_cast<int>(image1.rows * 1.5));
	cv::Mat uImage = cameraCalibrator.remap(image1);

	// ��ʾ���������
	cv::Mat cameraMatrix = cameraCalibrator.getCameraMatrix();
	std::cout << " Camera intrinsic: " << cameraMatrix.rows << "x" << cameraMatrix.cols << std::endl;//����ڲ�
	std::cout << cameraMatrix.at<double>(0, 0) << " " << cameraMatrix.at<double>(0, 1) << " " << cameraMatrix.at<double>(0, 2) << std::endl;
	std::cout << cameraMatrix.at<double>(1, 0) << " " << cameraMatrix.at<double>(1, 1) << " " << cameraMatrix.at<double>(1, 2) << std::endl;
	std::cout << cameraMatrix.at<double>(2, 0) << " " << cameraMatrix.at<double>(2, 1) << " " << cameraMatrix.at<double>(2, 2) << std::endl;

	cv::namedWindow("Original Image");
	cv::imshow("Original Image", image1);
	cv::namedWindow("Undistorted Image");
	cv::imshow("Undistorted Image", uImage);
	//��ӡ����ϵ������1*5����
	Mat distCoeffs = cameraCalibrator.getDistCoeffs();
	std::cout << "����ϵ������" << distCoeffs.rows << "x" << distCoeffs.cols << std::endl;
	for (int i = 0; i < distCoeffs.rows; i++)
		for (int j = 0; j < distCoeffs.cols; j++)
			cout << distCoeffs.at<double>(i, j) << "\t";

	// �������ļ��洢��xml�ļ�
	cv::FileStorage fs("calib.xml", cv::FileStorage::WRITE);
	fs << "Intrinsic" << cameraMatrix;
	fs << "Distortion" << cameraCalibrator.getDistCoeffs();


	cv::waitKey();
	return 0;
}
