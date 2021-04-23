#include <opencv2\opencv.hpp>
#include<opencv2\highgui.hpp>

using namespace cv;
using namespace std;




//Mat camera_matrix = (Mat_<double>(3, 3) << 348.359, 0, 320, 0, 351.375, 240, 0, 0, 1);
//Mat dist_coeffs = (Mat_<double>(1, 5) << -0.3747956597241781, 0.1329811678263925, -0.001470632239131604, -0.001222345346804578, -0.02088590086179011);

//输出视场调整
//Mat new_camera_matrix = (Mat_<double>(3, 3) << 348.359, 0, 320, 0, 351.375, 240, 0, 0, 1);


Matx33d camera_intrinsic = Matx33d(408.384, 0, 342.958, 0, 407.255, 279.46, 0, 0, 1);
Vec4d distortion_coeff = Vec4d(-0.1828925183809516, 0.3280241712987954, 0.2743154114646242, 0.6893903959538769);
Mat intrinsic_new = (Mat_<double>(3, 3) << 408.384, 0, 342.958, 0, 407.255, 279.46, 0, 0, 1);



int camNO = 1;
int counter = 10;

int main()
{
    Mat frame;
    Mat show_image;

    VideoCapture capture(1);

    

    

    while (true)
    {
        
        capture >> frame;
        //capture1.grab();
        //capture.open(1);
        while (counter > 0) {
            //capture.grab();
            counter --;
        }
        //capture.retrieve(frame);
        //capture.read(frame);
        
        //fisheye::undistortImage(frame, show_image, camera_intrinsic, distortion_coeff, intrinsic_new);//, intrinsic_new

        //undistort(frame,show_image,camera_matrix, dist_coeffs, new_camera_matrix);

        namedWindow("Camera-1", 0);
        //moveWindow("cloudcars - Camera 1", 0, 0);
        //resizeWindow("Camera-1");
        imshow("Camera-1", frame);

        counter = 10;
       

       

        waitKey(20);

        if (waitKey(20) == 'q') {
            //imwrite("30.jpg",frame);
            break;
        }
        
    }
        
      

         
    capture.release();
    
    return 0;
}






// 运行程序: Ctrl + F5 或调试 >“开始执行(不调试)”菜单
// 调试程序: F5 或调试 >“开始调试”菜单

// 入门使用技巧: 
//   1. 使用解决方案资源管理器窗口添加/管理文件
//   2. 使用团队资源管理器窗口连接到源代码管理
//   3. 使用输出窗口查看生成输出和其他消息
//   4. 使用错误列表窗口查看错误
//   5. 转到“项目”>“添加新项”以创建新的代码文件，或转到“项目”>“添加现有项”以将现有代码文件添加到项目
//   6. 将来，若要再次打开此项目，请转到“文件”>“打开”>“项目”并选择 .sln 文件
