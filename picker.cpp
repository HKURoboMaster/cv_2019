#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
using namespace cv;
using namespace std;

int main()
{
    int h_min = 170, h_max = 190, s_min = 43, s_max = 255, v_min = 46, v_max = 255;
    VideoCapture cap(0);
    if(!cap.isOpened()) return -1;
    cap.set(CAP_PROP_FRAME_WIDTH, 640);
    cap.set(CAP_PROP_FRAME_HEIGHT, 480);
    Mat frame, hsv;
    namedWindow("camera");
    namedWindow("hsv");
    while(true)
    {
        cap >> frame;
        imshow("camera", frame);
        Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
        dilate(frame, frame, element);
        imshow("dilation", frame);
        cvtColor(frame, hsv, COLOR_BGR2HSV);
        inRange(hsv, Scalar(h_min, s_min, v_min), Scalar(h_max, s_max, v_max), hsv);
        createTrackbar("H_MIN: ", "hsv", &h_min, 255);
        createTrackbar("H_MAX: ", "hsv", &h_max, 255);
        createTrackbar("S_MIN: ", "hsv", &s_min, 255);
        createTrackbar("S_MAX: ", "hsv", &s_max, 255);
        createTrackbar("V_MIN: ", "hsv", &v_min, 255);
        createTrackbar("V_MAX: ", "hsv", &v_max, 255);
        imshow("hsv", hsv);
        if(waitKey(30) == 27) break;
    }
    return 0;
}
