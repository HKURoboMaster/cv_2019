#include <vector>
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
        // 1 2
        // 0 3
        Point2f center, vertex[4];
        float angle, breadth, length;
        void CalculateVertex()
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
        SingleLightbar() : center(Point2f(0.0f, 0.0f)), angle(0.0f), breadth(0.0f), length(0.0f) { CalculateVertex(); }
        SingleLightbar(const Point2f &_center, const float &_angle, const float &_breadth, const float &_length) :
            center(_center), angle(_angle), breadth(_breadth), length(_length) { CalculateVertex(); }
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
            CalculateVertex();
        }
        SingleLightbar Expand()
        {
            return SingleLightbar(center, angle, breadth * 3.0f, length * 1.5f);
        }
        RotatedRect toRotatedRect()
        {
            return RotatedRect(vertex[0], vertex[1], vertex[2]);
        }
    };

    struct ArmorPlate
    {
        Point2f vertex[4];
        bool operator < (const ArmorPlate &rhs) const
        {
            vector<Point2f> cl(vertex, vertex+4), cr(rhs.vertex, rhs.vertex+4);
            return contourArea(cl) > contourArea(cr);
        }
    };

    float EvaluateColor(cv::Mat roi)
    {
        int Rcount = 0, Bcount = 0;
        for(int i = 0; i < roi.rows; i++)
        {
            for(int j = 0; j < roi.cols; j++)
            {
                Vec3b &p = roi.at<Vec3b>(i, j);
                const uint8_t &R = p[2], &G = p[1], &B = p[0];
                const float RGB = R + G + B;
                if(B / RGB > 0.75f)
                    ++Bcount;
                else if(R / RGB > 0.75f)
                    ++Rcount;
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
                if(min(left.vertex[0].y, left.vertex[3].y) < max(right.vertex[1].y, right.vertex[2].y)) continue;
                if(max(left.vertex[1].y, left.vertex[2].y) > min(right.vertex[0].y, right.vertex[3].y)) continue;
                float distance = sqrt(dx*dx + dy*dy);
                if(distance > 4*min(left.length, right.length)) continue;
                if(distance < min(left.length, right.length)) continue;
                if((EvaluateColor(img(left.Expand().toRotatedRect().boundingRect() & Rect(0, 0, img.cols, img.rows))) +
                    EvaluateColor(img(right.Expand().toRotatedRect().boundingRect() & Rect(0, 0, img.cols, img.rows))) / 2.0f) < 0.9f) continue;
                ArmorPlate candidate;
                candidate.vertex[0] = left.vertex[3];
                candidate.vertex[1] = left.vertex[2];
                candidate.vertex[2] = right.vertex[1];
                candidate.vertex[3] = right.vertex[0];
                armors.push_back(candidate);
            }
        }
        if(armors.empty()) return false;
        sort(armors.begin(), armors.end());
        const ArmorPlate &armor = armors[0];
        Mat rvec, tvec;
        vector<Point2f> ArmorVertex3D;
        for(int i = 0; i < 4; i++) ArmorVertex3D.push_back(armor.vertex[i]);
        for(int i = 0; i < 4; i++) line(img, armor.vertex[i], armor.vertex[(i+1)%4], Scalar(255, 0, 0), 3);
        solvePnP(ArmorVertex2D, ArmorVertex3D, intrinsic, distortion, rvec, tvec);
        target = Point3f(tvec);
        return true;
    }
}
