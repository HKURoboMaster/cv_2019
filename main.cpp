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
#include <mutex>
#include <atomic>
#include <thread>
#include <signal.h>
#include <unistd.h>
#include "constraint_set.h"
#include "protocol.h"
using namespace std;
using namespace cv;
int verbose = 0;
bool serial_comm, recording, single_step;
atomic<bool> running;
atomic<bool> new_image;
mutex mtx_input, mtx_output;
VideoCapture cap;
VideoWriter writer;
Mat img;

enum INPUT_TYPE
{
    CAMERA = 0,
    VIDEO
};

void clean_up()
{
    running = false;
    mtx_output.lock();
    if(serial_comm)
    {
        protocol::Disconnect();
    }
    if(recording)
    {
        writer.release();
    }
    mtx_output.unlock();
}

void sig_handler(int sig)
{
    clean_up();
}

void write(cv::Mat &img, const char *str, const cv::Point &pt)
{
    cv::Size text_size = cv::getTextSize(str, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, 0);
    cv::rectangle(img, pt, pt + cv::Point(text_size.width, -text_size.height), cv::Scalar(0, 0, 0), cv::FILLED);
    cv::putText(img, str, pt, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1, 8);
}

void VideoReader()
{
    Mat raw_input;
    while(running)
    {
        if(!cap.read(raw_input))
        {
            running = false;
            break;
        }
        mtx_output.lock();
        if(running && recording)
        {
            writer.write(raw_input);
        }
        mtx_output.unlock();
        mtx_input.lock();
        img = raw_input.clone();
        new_image = true;
        mtx_input.unlock();
    }
}

void Detector()
{
    bool detected;
    float yaw, pitch;
    char buf[100];
    Point3f target;
    while(running)
    {
        mtx_input.lock();
        if(!new_image)
        {
            mtx_input.unlock();
            usleep(20000);
            continue;
        }
        auto Tstart = chrono::system_clock::now();
        detected = constraint_set::DetectArmor(img, target);
        auto Tend = chrono::system_clock::now();
        new_image = false;
        mtx_output.lock();
        if(!running)
        {
            mtx_input.unlock();
            mtx_output.unlock();
            break;
        }
        if(detected)
        {
            yaw = -atan2(target.x, target.z) / M_PI * 180;
            pitch = -atan2(target.y, sqrt(target.x*target.x + target.z*target.z)) / M_PI * 180;
            if(serial_comm)
                protocol::Send(yaw, pitch);
        }
        mtx_output.unlock();
        if(verbose > 0)
        {
            const float duration = static_cast<float>(chrono::duration_cast<chrono::milliseconds>(Tend - Tstart).count());
            sprintf(buf, "Time elapsed during detection algorithm : % 4.2f MS", duration);
            write(img, buf, cv::Point(5, 20));
            line(img, Point(img.cols/2 - 10, img.rows/2), Point(img.cols/2 + 10, img.rows/2), Scalar(255, 0, 0));
            line(img, Point(img.cols/2, img.rows/2 - 10), Point(img.cols/2, img.rows/2 + 10), Scalar(255, 0, 0));
            if(detected)
            {
                sprintf(buf, "Enemy spotted at (% 6.2f, % 6.2f, % 6.2f)", target.x, target.y, target.z);
                write(img, buf, cv::Point(10, 40));
                sprintf(buf, "Transmitted YAW=% 4.2fDEG PITCH=% 4.2fDEG", yaw, pitch);
                write(img, buf, cv::Point(10, 60));
            }
            imshow("constraint_set", img);
        }
        mtx_input.unlock();
        if(verbose > 0)
        {
            if(single_step)
            {
                int key = 0;
                do
                {
                    key = waitKey(0);
                } while(key != ' ' && key != 27);
                if(key == 27)
                    running = false;
            }
            else
            {
                if(waitKey(1) == 27)
                    running = false;
            }
        }
    }
}

int main(int argc, char *argv[])
{
    INPUT_TYPE input_type = CAMERA;
    string serial_device("/dev/ttyUSB0");
    serial_comm = true;
    int camera_id = 0, custom_width = 640, custom_height = 480;
    string video_file, record_file;
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
            else if(strcmp(argv[i], "--camera") == 0 && i+1 < argc)
            {
                camera_id = argv[++i][0] - '0';
            }
            else if(strcmp(argv[i], "--width") == 0 && i+1 < argc)
            {
                custom_width = atoi(argv[++i]);
            }
            else if(strcmp(argv[i], "--height") == 0 && i+1 < argc)
            {
                custom_height = atoi(argv[++i]);
            }
            else if(strcmp(argv[i], "--video") == 0 && i+1 < argc)
            {
                input_type = VIDEO;
                video_file = argv[++i];
            }
            else if(strcmp(argv[i], "--record") == 0 && i+1 < argc)
            {
                recording = true;
                record_file = argv[++i];
            }
            else if(strcmp(argv[i], "--dummy") == 0)
            {
                serial_comm = false;
            }
            else if(strcmp(argv[i], "--single-step") == 0)
            {
                single_step = true;
            }
            else
            {
                cout << "Unrecognized parameter " << argv[i] << endl;
                return 0;
            }
        }
    }
    Mat img, res;
    float intrinsic_matrix[] = { 1536.07f, 0.0f, 320.0f,
                                       0.0f, 1542.55f, 240.0f,
                                       0.0f, 0.0f, 1.0f };
    float distortion_coeffs[] = { 0.44686f, 15.5414f, -0.009048f, -0.009717f, -439.74f };
    constraint_set::InitializeConstraintSet(intrinsic_matrix, distortion_coeffs);
    switch(input_type)
    {
        case CAMERA:
            if(!cap.open(camera_id))
            {
                cerr << "Unable to open camera" << endl;
                return 1;
            }
            cap.set(cv::CAP_PROP_FRAME_WIDTH, custom_width);
            cap.set(cv::CAP_PROP_FRAME_HEIGHT, custom_height);
            break;
        case VIDEO:
            if(!cap.open(video_file.c_str()))
            {
                cerr << "Unable to open video file" << endl;
                return 1;
            }
            break;
    }
    if(!cap.read(img))
    {
        cerr << "Unable to read image from input" << endl;
        return 1;
    }
    if(serial_comm && !protocol::Connect(serial_device.c_str()))
    {
        cerr << "Unable to connect " << serial_device << endl;
        return 1;
    }
    if(recording)
    {
        writer.open(record_file.c_str(), VideoWriter::fourcc('M', 'J', 'P', 'G'), cap.get(cv::CAP_PROP_FPS), Size(custom_width, custom_height));
    }
    running = true;
    signal(SIGINT, sig_handler);
    signal(SIGKILL, sig_handler);
    signal(SIGTERM, sig_handler);
    switch(input_type)
    {
        case CAMERA:
            cout << "INPUT: CAMERA #" << camera_id << "@" << cap.get(cv::CAP_PROP_FRAME_WIDTH) << "x" << cap.get(cv::CAP_PROP_FRAME_HEIGHT) << "@" << cap.get(cv::CAP_PROP_FPS) << "FPS" << endl;
            break;
        case VIDEO:
            cout << "INPUT: VIDEO " << video_file << "@" << cap.get(cv::CAP_PROP_FRAME_WIDTH) << "x" << cap.get(cv::CAP_PROP_FRAME_HEIGHT) << "@" << cap.get(cv::CAP_PROP_FPS) << "FPS" << endl;
            break;
    }
    if(serial_comm)
    {
        cout << "OUTPUT: " << serial_device << endl;
    }
    if(recording)
    {
        cout << "RECORDING: " << record_file << endl;
    }
    thread Tin(VideoReader);
    Detector();
    Tin.join();
    clean_up();
    return 0;
}
