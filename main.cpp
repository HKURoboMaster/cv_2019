/****************************************************************************
 *  Copyright (C) 2019 Brett Dong
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/
#include <iostream>
#include <cstdlib>
#include <cstring>
#include <cstdio>
#include "constraint_set.h"
using namespace std;
using namespace cv;

void write(cv::Mat &img, const char *str, const cv::Point &pt)
{
    cv::Size text_size = cv::getTextSize(str, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, 0);
    cv::rectangle(img, pt, pt + cv::Point(text_size.width, -text_size.height), cv::Scalar(0, 0, 0), CV_FILLED);
    cv::putText(img, str, pt, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1, 8);
}

int main()
{
    VideoCapture cap;
    Mat img, res;
    float intrinsic_matrix[] = { 1536.07f, 0.0f, 320.0f,
                                       0.0f, 1542.55f, 240.0f,
                                       0.0f, 0.0f, 1.0f };
    float distortion_coeffs[] = { 0.44686f, 15.5414f, -0.009048f, -0.009717f, -439.74f };
    constraint_set::InitializeConstraintSet(intrinsic_matrix, distortion_coeffs);
    Point3f target;
    if(!cap.open(0))
    {
        cerr << "Failed to open camera" << endl;
        return 1;
    }
    char buf[100];
    while(cap.read(img))
    {
        if(constraint_set::DetectArmor(img, target))
        {
            sprintf(buf, "Detected @(% 6.2f, % 6.2f, % 6.2f)", target.x, target.y, target.z);
            write(img, buf, cv::Point(10, 60));
            float yaw = atan2(target.x, target.z), pitch = atan2(target.y, sqrt(target.x*target.x + target.z*target.z));
            yaw = yaw / M_PI * 180; pitch = pitch / M_PI * 180;
            sprintf(buf, "YAW=% 4.2fDEG PITCH=% 4.2fDEG", yaw, pitch);
            write(img, buf, cv::Point(10, 80));
        }
        sprintf(buf, "INPUT: CAMERA %dx%d", img.cols, img.rows);
        write(img, buf, cv::Point(10, 40));
        line(img, Point(img.cols/2 - 10, img.rows/2), Point(img.cols/2 + 10, img.rows/2), Scalar(255, 0, 0));
        line(img, Point(img.cols/2, img.rows/2 - 10), Point(img.cols/2, img.rows/2 + 10), Scalar(255, 0, 0));
        imshow("constraint_set", img);
        if(waitKey(1) == 27) break;
    }
    return 0;
}
