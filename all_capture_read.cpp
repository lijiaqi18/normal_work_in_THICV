#include <opencv2\opencv.hpp>
#include<opencv2\highgui.hpp>

using namespace cv;
using namespace std;

VideoCapture capture1,capture2;
Mat frame1,frame2,frame3,frame4;

int counter = 40;


//capture1.open(1,CAP_DSHOW);

int main() {
    while (true)
    {
        
        while (counter > 30) {
            //capture.grab();
            counter--;
        }
        
        capture1.open(1, CAP_DSHOW);
        capture1.read(frame1);
        namedWindow("Camera-1", 0);
        //moveWindow("cloudcars - Camera 1", 0, 0);
        resizeWindow("Camera-1", 320, 240);
        imshow("Camera-1", frame1);

        while (counter > 20) {
            //capture.grab();
            counter--;
        }
        capture1.open(2, CAP_DSHOW);
        capture1.read(frame2);
        namedWindow("Camera-2", 0);
        resizeWindow("Camera-2", 320, 240);
        imshow("Camera-2", frame2);

        while (counter > 10) {
            //capture.grab();
            counter--;
        }
        capture1.open(3, CAP_DSHOW);
        capture1 >> frame3;
        namedWindow("Camera-3", 0);
        resizeWindow("Camera-3", 320, 240);
        imshow("Camera-3", frame3);

        while (counter > 0) {
            //capture.grab();
            counter--;
        }
        capture1.open(4, CAP_DSHOW);
        capture1 >> frame4;
        namedWindow("Camera-4", 0);
        resizeWindow("Camera-4", 320, 240);
        imshow("Camera-4", frame4);

        counter = 40;


        waitKey(20);

        if (waitKey(20) == 'q') {
            //imwrite("15.jpg",frame1);
            break;
        }

    }

	return 0;
}