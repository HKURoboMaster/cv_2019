#pragma once
#ifndef NG_H_
#define NG_H_

#include <opencv2/opencv.hpp>
namespace DetectionNG
{
    void InitDetector(float intrinsic_matrix[], float distortion_coeffs[]);
    bool DetectArmor(cv::Mat &img, cv::Point3f &target);
}

#endif
