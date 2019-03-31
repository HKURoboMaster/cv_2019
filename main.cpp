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
#include <string>
#include <cstdio>
#include <chrono>
#include <signal.h>
#include "constraint_set.h"
#include "protocol.h"
using namespace std;
using namespace cv;

void sig_handler(int sig)
{
    if(sig == SIGINT)
    {
        protocol::Disconnect();
        exit(0);
    }
}

void write(cv::Mat &img, const char *str, const cv::Point &pt)
{
    cv::Size text_size = cv::getTextSize(str, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, 0);
    cv::rectangle(img, pt, pt + cv::Point(text_size.width, -text_size.height), cv::Scalar(0, 0, 0), CV_FILLED);
    cv::putText(img, str, pt, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1, 8);
}

int main(int argc, char *argv[])
{
    int verbose = 0;
    const char *default_serial_device = "/dev/ttyUSB0";
    string serial_device(default_serial_device);
    if(argc > 1)
    {
        for(int i = 1; i < argc; i++)
        {
            if(strcmp(argv[i], "--verbose") == 0 && i+1 < argc)
            {
                verbose = argv[++i][0] - '0';
            }
            else if(strcmp(argv[i], "--serial") == 0 && i+1 < argc)
            {
                serial_device = argv[++i];
            }
        }
    }
    if(!protocol::Connect(serial_device.c_str()))
    {
        cerr << "Unable to connect " << serial_device << endl;
        return 1;
    }
    VideoCapture cap;
    Mat img, res;
    float intrinsic_matrix[] = { 1536.07f, 0.0f, 320.0f,
                                       0.0f, 1542.55f, 240.0f,
                                       0.0f, 0.0f, 1.0f };
    float distortion_coeffs[] = { 0.44686f, 15.5414f, -0.009048f, -0.009717f, -439.74f };
    constraint_set::InitializeConstraintSet(intrinsic_matrix, distortion_coeffs);
    Point3f target;
    if(!cap.open(0) || !cap.read(img))
    {
        cerr << "Unable to open camera" << endl;
        return 1;
    }
    char buf[100];
    bool running = true;
    float angle_amp = 1.0f;
    signal(SIGINT, sig_handler);
    cout << "INPUT:  CAMERA " << img.cols << "x" << img.rows << endl;
    cout << "OUTPUT: " << serial_device << endl;
    while(running)
    {
        auto Tstart = chrono::system_clock::now();
        if(!cap.read(img))
            break;
        if(constraint_set::DetectArmor(img, target))
        {
            float yaw = atan2(target.x, target.z), pitch = atan2(target.y, sqrt(target.x*target.x + target.z*target.z));
            yaw = yaw / M_PI * 180; pitch = pitch / M_PI * 180;
            protocol::Send(yaw * angle_amp, pitch * angle_amp);
            if(verbose > 0)
            {
                sprintf(buf, "Detected @(% 6.2f, % 6.2f, % 6.2f)", target.x, target.y, target.z);
                write(img, buf, cv::Point(10, 40));
                sprintf(buf, "YAW=% 4.2fDEG PITCH=% 4.2fDEG", yaw, pitch);
                write(img, buf, cv::Point(10, 60));
                sprintf(buf, "AMP%fx YAW= % 4.2fDEG PITCH=% 4.2fDEG", angle_amp, yaw * angle_amp, pitch * angle_amp);
                write(img, buf, cv::Point(10, 80));
            }
        }
        if(verbose > 0)
        {
            auto Tend = chrono::system_clock::now();
            auto duration = chrono::duration_cast<chrono::milliseconds>(Tend - Tstart);
            const float latency = static_cast<float>(duration.count());// * chrono::milliseconds::period::num / chrono::milliseconds::period::den;
            sprintf(buf, "LATENCY: % 4.2f MS | %d FPS", latency, static_cast<int>(1000.0f / latency));
            write(img, buf, cv::Point(5, 20));
            line(img, Point(img.cols/2 - 10, img.rows/2), Point(img.cols/2 + 10, img.rows/2), Scalar(255, 0, 0));
            line(img, Point(img.cols/2, img.rows/2 - 10), Point(img.cols/2, img.rows/2 + 10), Scalar(255, 0, 0));
            imshow("constraint_set", img);
            switch(waitKey(1))
            {
                case 27:
                    running = false;
                    break;
                case 'a':
                    if(angle_amp > 0.5f)
                        angle_amp -= 0.5f;
                    break;
                case 'd':
                    if(angle_amp < 9.5f)
                        angle_amp += 0.5f;
                    break;
            }
        }
    }
    protocol::Disconnect();
    return 0;
}
