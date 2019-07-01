#include <bits/stdc++.h>
#include "ng.h"
using namespace std;
using namespace cv;

namespace DetectionNG
{
    vector<Point3f> ArmorVertex2D;
    Mat intrinsic, distortion;

    void InitDetector(float intrinsic_matrix[], float distortion_coeffs[])
    {
        ArmorVertex2D.emplace_back(cv::Point3f(-120.0f/2, 60.0f/2,  0.0));
        ArmorVertex2D.emplace_back(cv::Point3f(-120.0f/2, -60.0f/2, 0.0));
        ArmorVertex2D.emplace_back(cv::Point3f(120.0f/2,  -60.0f/2, 0.0));
        ArmorVertex2D.emplace_back(cv::Point3f(120.0f/2,  60.0f/2,  0.0));
        intrinsic = Mat(3, 3, CV_32F, intrinsic_matrix);
        distortion = Mat(1, 5, CV_32F, distortion_coeffs);
    }

    struct SingleLightbar
    {
        Point2f center;
        float angle, breadth, length;
        SingleLightbar() : center(Point2f(0.0f, 0.0f)), angle(0.0f), breadth(0.0f), length(0.0f) {}
        SingleLightbar(const RotatedRect &rect)
        {
            center = rect.center;
            if(rect.size.width < rect.size.height)
            {
                breadth = rect.size.width;
                length = rect.size.height;
                angle = 90.0f - rect.angle;
            }
            else
            {
                breadth = rect.size.height;
                length = rect.size.width;
                angle = -rect.angle;
            }
        }
        // 1 2
        // 0 3
        void points(Point2f vertex[]) const
        {
            const float theta = angle * CV_PI / 180;
            const float a = cos(theta) * 0.5f, b = sin(theta) * 0.5f;
            vertex[0].x = center.x - a * length - b * breadth;
            vertex[0].y = center.y + b * length - a * breadth;
            vertex[1].x = center.x + a * length - b * breadth;
            vertex[1].y = center.y - b * length - a * breadth;
            vertex[2].x = 2 * center.x - vertex[0].x;
            vertex[2].y = 2 * center.y - vertex[0].y;
            vertex[3].x = 2 * center.x - vertex[1].x;
            vertex[3].y = 2 * center.y - vertex[1].y;
        }
        void expansion()
        {
            breadth *= 3.0f;
            length *= 1.5f;
        }
    };

    struct ArmorPlate
    {
        Point2f vertex[4];
        double confidence;
    };

    float EvaluatePixels(cv::Mat img, const cv::Mat &mask)
    {
        int Rcount = 0, Bcount = 0;
        for(int i = 0; i < img.rows; i++)
        {
            for(int j = 0; j < img.cols; j++)
            {
                Vec3b &p = img.at<Vec3b>(i, j);
                uint8_t R = p[2], G = p[1], B = p[0];
                float RGB = R + G + B;
                float r = R / RGB, g = G / RGB, b = B / RGB;
                if(b > 0.75f)
                {
                    ++Bcount;
                }
                else if(r > 0.75f)
                {
                    ++Rcount;
                }
            }
        }
        return (Rcount - Bcount) * 1.0f / (Rcount + Bcount);
    }

    bool DetectArmor(Mat &img, Point3f &target)
    {
        Mat hsv(img.rows, img.cols, CV_8UC3);
        cvtColor(img, hsv, COLOR_BGR2HSV);
        inRange(hsv, Scalar(0, 0, 250), Scalar(255, 255, 255), hsv);
        vector<vector<Point>> contours;
        findContours(hsv, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        vector<SingleLightbar> lights;
        vector<ArmorPlate> armors;
        for(auto &ct : contours)
        {
            if(contourArea(ct) < 10 || contourArea(ct) > 2000) continue;
            SingleLightbar light(minAreaRect(ct));
            if(abs(light.angle - 90.0f) > 30.f) continue;
            lights.push_back(light);
        }
        for(int i = 0; i < lights.size(); i++)
        {
            for(int j = i+1; j < lights.size(); j++)
            {
                if(abs(lights[i].angle - lights[j].angle) > 15.0f) continue;
                float average_angle = (lights[i].angle + lights[j].angle) / 2;
                SingleLightbar left, right;
                if(lights[i].center.x < lights[j].center.x)
                {
                    left = lights[i];
                    right = lights[j];
                }
                else
                {
                    left = lights[j];
                    right = lights[i];
                }
                float dx = right.center.x - left.center.x;
                float dy = right.center.y - left.center.y;
                float slope = (abs(dx) < 2.0f ? -90.0f : atan(-dy/dx) * 180 / CV_PI);
                if(abs(average_angle - slope - 90) > 30.0f) continue;
                Point2f v_left[4], v_right[4];
                left.points(v_left);
                right.points(v_right);
                if(min(v_left[0].y, v_left[3].y) < max(v_right[1].y, v_right[2].y)) continue;
                if(max(v_left[1].y, v_left[2].y) > min(v_right[0].y, v_right[3].y)) continue;
                float distance = sqrt(dx*dx + dy*dy);
                if(distance > 4*min(left.length, right.length)) continue;
                ArmorPlate candidate;
                candidate.vertex[0] = v_left[3];
                candidate.vertex[1] = v_left[2];
                candidate.vertex[2] = v_right[1];
                candidate.vertex[3] = v_right[0];
                SingleLightbar eleft = left, eright = right;
                eleft.expansion(); eright.expansion();
                Point2f veleft[4], veright[4];
                eleft.points(veleft);
                eright.points(veright);
                Mat mask = Mat::zeros(img.size(), CV_8UC1);
                vector<Point2f> contour1(veleft, veleft + 4);
                vector<Point2f> contour2(veright, veright + 4);
                Point velefti[4], verighti[4];
                for(int i = 0; i < 4; i++) velefti[i] = veleft[i];
                for(int i = 0; i < 4; i++) verighti[i] = veright[i];
                fillConvexPoly(mask, velefti, 4, Scalar(255));
                fillConvexPoly(mask, verighti, 4, Scalar(255));
                candidate.confidence = EvaluatePixels(img, mask);
                armors.push_back(candidate);
            }
        }
        for(const ArmorPlate &armor : armors)
        {
            if(armor.confidence > 0.7f)
            {
                Mat rvec, tvec;
                vector<Point2f> ArmorVertex3D;
                for(int i = 0; i < 4; i++) ArmorVertex3D.push_back(armor.vertex[i]);
                solvePnP(ArmorVertex2D, ArmorVertex3D, intrinsic, distortion, rvec, tvec);
                target = Point3f(tvec);
                return true;
            }
        }
        return false;
    }
}
