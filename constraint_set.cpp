/****************************************************************************
 *  Copyright (C) 2019 RoboMaster
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
#include <vector>
#include <list>
#include <iostream>
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>
#include "constraint_set.h"
extern int verbose;

namespace constraint_set
{

struct LightInfo
{

    explicit LightInfo(cv::Point2f vertices[])
    {
        auto edge_1 = std::pow(vertices[0].x - vertices[1].x, 2) +
                      std::pow(vertices[0].y - vertices[1].y, 2);
        auto edge_2 = std::pow(vertices[1].x - vertices[2].x, 2) +
                      std::pow(vertices[1].y - vertices[2].y, 2);

        if (edge_1 > edge_2)
        {
            width_  = (float)std::sqrt(edge_1);
            height_ = (float)std::sqrt(edge_2);

            if (vertices[0].y < vertices[1].y)
            {
                angle_ = std::atan2(vertices[1].y - vertices[0].y, vertices[1].x - vertices[0].x);
            }
            else
            {
                angle_ = std::atan2(vertices[0].y - vertices[1].y, vertices[0].x - vertices[1].x);
            }

        }
        else
        {
            width_  = (float)std::sqrt(edge_2);
            height_ = (float)std::sqrt(edge_1);

            if (vertices[2].y < vertices[1].y)
            {
                angle_ = std::atan2(vertices[1].y - vertices[2].y, vertices[1].x - vertices[2].x);
            }
            else
            {
                angle_ = std::atan2(vertices[2].y - vertices[1].y, vertices[2].x - vertices[1].x);
            }

        }

        angle_ = (float)(angle_*180.0 / 3.1415926);
        area_ = width_ * height_;
        aspect_ratio_ = width_ / height_;
        center_.x = (vertices[1].x + vertices[3].x) / 2;
        center_.y = (vertices[1].y + vertices[3].y) / 2;
        vertices_.push_back(vertices[0]);
        vertices_.push_back(vertices[1]);
        vertices_.push_back(vertices[2]);
        vertices_.push_back(vertices[3]);
    }

public:
    //! Light area
    float area_;
    //! Light angle, come from the long edge's slope
    float angle_;
    //! Light center
    cv::Point2f center_;
    //! Light aspect ratio = width_/height_
    float aspect_ratio_;
    //! Light width
    float width_;
    //! Light height
    float height_;
    //! Light vertices
    std::vector<cv::Point2f> vertices_;
};

/**
 *  This class describes the armor information, including maximum bounding box, vertex, standard deviation.
 */
class ArmorInfo
{
public:
    ArmorInfo(cv::RotatedRect armor_rect, std::vector<cv::Point2f> armor_vertex, float armor_stddev = 0.0)
    {
        rect = armor_rect;
        vertex = armor_vertex;
        stddev = armor_stddev;
    }
public:
    cv::RotatedRect rect;
    std::vector<cv::Point2f> vertex;
    float stddev;
};

enum ENEMY_COLOR
{
    BLUE = 0,
    RED = 1
};

void DrawRotatedRect(const cv::Mat &img, const cv::RotatedRect &rect, const cv::Scalar &color, int thickness)
{
    cv::Point2f vertex[4];

    /*cv::Point2f center = rect.center;
    float angle = rect.angle;
    std::ostringstream ss;
    ss << angle;
    std::string text(ss.str());
    int font_face = cv::FONT_HERSHEY_COMPLEX;
    double font_scale = 0.5;
    cv::putText(img, text, center, font_face, font_scale, cv::Scalar(0, 255, 255), thickness, 8, 0);*/

    rect.points(vertex);
    cv::line(img, vertex[0], vertex[2], color, thickness);
    cv::line(img, vertex[1], vertex[3], color, thickness);
    for (int i = 0; i < 4; i++)
        cv::line(img, vertex[i], vertex[(i + 1) % 4], color, thickness);
}

cv::Mat DistillationColor(const cv::Mat &src_img, unsigned int color, bool using_hsv)
{
    if(using_hsv)
    {
        cv::Mat img_hsv;
        cv::cvtColor(src_img, img_hsv, cv::COLOR_BGR2HSV);
        if (color == 0)
        {
            cv::Mat img_hsv_blue, img_threshold_blue;
            img_hsv_blue = img_hsv.clone();
            cv::Mat blue_low(cv::Scalar(90, 150, 46));
            cv::Mat blue_higher(cv::Scalar(140, 255, 255));
            cv::inRange(img_hsv_blue, blue_low, blue_higher, img_threshold_blue);
            return img_threshold_blue;
        }
        else
        {
            cv::Mat img_hsv_red1, img_hsv_red2, img_threshold_red, img_threshold_red1, img_threshold_red2;
            img_hsv_red1 = img_hsv.clone();
            img_hsv_red2 = img_hsv.clone();
            cv::Mat red1_low(cv::Scalar(0, 43, 46));
            cv::Mat red1_higher(cv::Scalar(3, 255, 255));

            cv::Mat red2_low(cv::Scalar(170, 43, 46));
            cv::Mat red2_higher(cv::Scalar(180, 255, 255));
            cv::inRange(img_hsv_red1, red1_low, red1_higher, img_threshold_red1);
            cv::inRange(img_hsv_red2, red2_low, red2_higher, img_threshold_red2);
            img_threshold_red = img_threshold_red1 | img_threshold_red2;
            //cv::imshow("img_threshold_red", img_threshold_red);
            return img_threshold_red;
        }
    }
    else
    {
        std::vector<cv::Mat> bgr;
        cv::split(src_img, bgr);
        if (color == 1)
        {
            cv::Mat result_img;
            cv::subtract(bgr[2], bgr[1], result_img);
            return result_img;
        }
        else if (color == 0)
        {
            cv::Mat result_img;
            cv::subtract(bgr[0], bgr[2], result_img);
            return result_img;
        }
    }
}

std::vector<std::vector<cv::Point>> FindContours(const cv::Mat &binary_img)
{
    std::vector<std::vector<cv::Point>> contours;
    const auto mode = cv::RETR_EXTERNAL;
    const auto method = cv::CHAIN_APPROX_SIMPLE;
    cv::findContours(binary_img, contours, mode, method);
    return contours;
}

int filter_x_count_ = 0, filter_y_count_ = 0, filter_z_count_ = 0;
int filter_distance_count_ = 0, filter_pitch_count_ = 0, filter_yaw_count_ = 0;
ENEMY_COLOR enemy_color_ = BLUE;
bool using_hsv_ = true;
const float armor_width = 120.0f, armor_height = 60.0f;
const float light_max_aspect_ratio_ = 20.0f;
const float light_min_area_ = 1.0f;
const float light_max_angle_ = 20.0f;
const float light_max_angle_diff_ = 10.0f;
const float armor_max_angle_ = 15.0f;
const float armor_min_area_ = 100.0f;
const float armor_max_aspect_ratio_ = 4.0f;
const float armor_max_pixel_val_ = 150.0f;
const float armor_max_stddev_ = 90.0f;
const float armor_max_mean_   = 90.0f;
const float color_thread_ = 100.0f;
const float blue_thread_ = 90.0f;
const float red_thread_ = 50.0f;
cv::Mat intrinsic_matrix_, distortion_coeffs_;
cv::Mat gray_img_;
cv::Mat light_img, filter_light_img;
std::vector<cv::Point3f> armor_points_;

void SolveArmorCoordinate(const float width,
        const float height)
{
    armor_points_.emplace_back(cv::Point3f(-width/2, height/2,  0.0));
    armor_points_.emplace_back(cv::Point3f(width/2,  height/2,  0.0));
    armor_points_.emplace_back(cv::Point3f(width/2,  -height/2, 0.0));
    armor_points_.emplace_back(cv::Point3f(-width/2, -height/2, 0.0));
}

void InitializeConstraintSet(float intrinsic_matrix[], float distortion_coeffs[])
{
    SolveArmorCoordinate(armor_width, armor_height);
    intrinsic_matrix_ = cv::Mat(3, 3, CV_32F, intrinsic_matrix);
    distortion_coeffs_ = cv::Mat(1, 5, CV_32F, distortion_coeffs);
}

void FilterLights(std::vector<cv::RotatedRect> &lights)
{
    std::vector<cv::RotatedRect> rects;
    rects.reserve(lights.size());

    for (const auto &light : lights)
    {
        float angle;
        auto light_aspect_ratio =
            std::max(light.size.width, light.size.height) / std::min(light.size.width, light.size.height);
        //https://stackoverflow.com/questions/15956124/minarearect-angles-unsure-about-the-angle-returned/21427814#21427814
        if(light.size.width < light.size.height)
        {
            angle = light.angle; // -light.angle
        }
        else
            angle = light.angle; // light.angle + 90
        if (light_aspect_ratio < light_max_aspect_ratio_ &&
                light.size.area() >= light_min_area_)   //angle < light_max_angle_ &&
        {
            rects.push_back(light);
            DrawRotatedRect(filter_light_img, light, cv::Scalar(0, 255, 0), 2);
        }
    }
    lights = rects;
    if(verbose > 1)
    {
        cv::imshow("Filter Lights", filter_light_img);
    }
}

void CalcArmorInfo(std::vector<cv::Point2f> &armor_points,
                                  cv::RotatedRect left_light,
                                  cv::RotatedRect right_light)
{
    cv::Point2f left_points[4], right_points[4];
    left_light.points(left_points);
    right_light.points(right_points);

    cv::Point2f right_lu, right_ld, lift_ru, lift_rd;
    std::sort(left_points, left_points + 4, [](const cv::Point2f &p1, const cv::Point2f &p2)
    {
        return p1.x < p2.x;
    });
    std::sort(right_points, right_points + 4, [](const cv::Point2f &p1, const cv::Point2f &p2)
    {
        return p1.x < p2.x;
    });
    if (right_points[0].y < right_points[1].y)
    {
        right_lu = right_points[0];
        right_ld = right_points[1];
    }
    else
    {
        right_lu = right_points[1];
        right_ld = right_points[0];
    }

    if (left_points[2].y < left_points[3].y)
    {
        lift_ru = left_points[2];
        lift_rd = left_points[3];
    }
    else
    {
        lift_ru = left_points[3];
        lift_rd = left_points[2];
    }
    armor_points.push_back(lift_ru);
    armor_points.push_back(right_lu);
    armor_points.push_back(right_ld);
    armor_points.push_back(lift_rd);
}

void PossibleArmors(const std::vector<cv::RotatedRect> &lights, std::vector<ArmorInfo> &armors)
{
    for (unsigned int i = 0; i < lights.size(); i++)
    {
        for (unsigned int j = i + 1; j < lights.size(); j++)
        {
            cv::RotatedRect light1 = lights[i];
            cv::RotatedRect light2 = lights[j];
            auto edge1 = std::minmax(light1.size.width, light1.size.height);
            auto edge2 = std::minmax(light2.size.width, light2.size.height);
            auto lights_dis = std::sqrt((light1.center.x - light2.center.x) * (light1.center.x - light2.center.x) +
                                        (light1.center.y - light2.center.y) * (light1.center.y - light2.center.y));
            auto center_angle = std::atan(std::abs(light1.center.y - light2.center.y) / std::abs(light1.center.x - light2.center.x)) * 180 / CV_PI;
            center_angle = center_angle > 90 ? 180 - center_angle : center_angle;

            cv::RotatedRect rect;
            rect.angle = static_cast<float>(center_angle);
            rect.center.x = (light1.center.x + light2.center.x) / 2;
            rect.center.y = (light1.center.y + light2.center.y) / 2;
            float armor_width = std::abs(static_cast<float>(lights_dis) - std::max(edge1.first, edge2.first));
            float armor_height = std::max<float>(edge1.second, edge2.second);

            rect.size.width = std::max<float>(armor_width, armor_height);
            rect.size.height = std::min<float>(armor_width, armor_height);

            float light1_angle = light1.angle; //light1.size.width < light1.size.height ? -light1.angle : light1.angle + 90
            float light2_angle = light2.angle; //light2.size.width < light2.size.height ? -light2.angle : light2.angle + 90
            auto angle_diff = std::abs(light1_angle - light2_angle);
            // Avoid incorrect calculation at 180 and 0.
            if (angle_diff > 175)
            {
                angle_diff = 180 - angle_diff;
            }

            if (angle_diff < light_max_angle_diff_ &&
                    std::max<float>(edge1.second, edge2.second)/std::min<float>(edge1.second, edge2.second) < 2.0 &&
                    rect.size.width / (rect.size.height) < armor_max_aspect_ratio_ &&
                    std::abs(rect.size.area()) > armor_min_area_ &&
                    gray_img_.at<uchar>(static_cast<int>(rect.center.y), static_cast<int>(rect.center.x))
                    < armor_max_pixel_val_)   //std::abs(center_angle) < armor_max_angle_ &&
            {

                if (light1.center.x < light2.center.x)
                {
                    std::vector<cv::Point2f> armor_points;
                    CalcArmorInfo(armor_points, light1, light2);
                    armors.emplace_back(ArmorInfo(rect, armor_points));
                    armor_points.clear();
                }
                else
                {
                    std::vector<cv::Point2f> armor_points;
                    CalcArmorInfo(armor_points, light2, light1);
                    armors.emplace_back(ArmorInfo(rect, armor_points));
                    armor_points.clear();
                }
            }
        }
    }
}

void FilterArmors(std::vector<ArmorInfo> &armors)
{
    cv::Mat mask = cv::Mat::zeros(gray_img_.size(), CV_8UC1);
    for (auto armor_iter = armors.begin(); armor_iter != armors.end();)
    {
        cv::Point pts[4];
        for (unsigned int i = 0; i < 4; i++)
        {
            pts[i].x = (int) armor_iter->vertex[i].x;
            pts[i].y = (int) armor_iter->vertex[i].y;
        }
        cv::fillConvexPoly(mask, pts, 4, cv::Scalar(255), 8, 0);

        cv::Mat mat_mean;
        cv::Mat mat_stddev;
        cv::meanStdDev(gray_img_, mat_mean, mat_stddev, mask);

        auto stddev = mat_stddev.at<double>(0, 0);
        auto mean = mat_mean.at<double>(0, 0);

        if (stddev > armor_max_stddev_ || mean > armor_max_mean_)
        {
            armor_iter = armors.erase(armor_iter);
        }
        else
        {
            armor_iter++;
        }
    }

    // nms
    std::vector<bool> is_armor(armors.size(), true);
    for (int i = 0; i < armors.size() && is_armor[i] == true; i++)
    {
        for (int j = i + 1; j < armors.size() && is_armor[j]; j++)
        {
            float dx = armors[i].rect.center.x - armors[j].rect.center.x;
            float dy = armors[i].rect.center.y - armors[j].rect.center.y;
            float dis = std::sqrt(dx * dx + dy * dy);
            if (dis < armors[i].rect.size.width + armors[j].rect.size.width)
            {
                if (armors[i].rect.angle > armors[j].rect.angle)
                {
                    is_armor[i] = false;
                    //std::cout << "i: " << i << std::endl;
                }
                else
                {
                    is_armor[j] = false;
                    //std::cout << "j: " << j << std::endl;
                }
            }
        }
    }
    for (unsigned int i = 0; i < armors.size(); i++)
    {
        if (!is_armor[i])
        {
            armors.erase(armors.begin() + i);
            is_armor.erase(is_armor.begin() + i);
        }
    }
}

ArmorInfo SelectFinalArmor(std::vector<ArmorInfo> &armors)
{
    std::sort(armors.begin(),
              armors.end(),
              [](const ArmorInfo &p1, const ArmorInfo &p2)
    {
        return p1.rect.size.area() > p2.rect.size.area();
    });

    return armors[0];
}

void CalcControlInfo(const ArmorInfo & armor, cv::Point3f &target_3d)
{
    cv::Mat rvec;
    cv::Mat tvec;
    cv::solvePnP(armor_points_,
                 armor.vertex,
                 intrinsic_matrix_,
                 distortion_coeffs_,
                 rvec,
                 tvec);
    target_3d = cv::Point3f(tvec);
}

void SignalFilter(double &new_num, double &old_num, unsigned int &filter_count, double max_diff)
{
    if(fabs(new_num - old_num) > max_diff && filter_count < 2)
    {
        filter_count++;
        new_num += max_diff;
    }
    else
    {
        filter_count = 0;
        old_num = new_num;
    }
}

void DetectLights(const cv::Mat &src, std::vector<cv::RotatedRect> &lights)
{
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::dilate(src, src, element, cv::Point(-1, -1), 1);
    cv::Mat binary_brightness_img, binary_light_img, binary_color_img;
    if(using_hsv_)
    {
        binary_color_img = DistillationColor(src, enemy_color_, using_hsv_);
        cv::threshold(gray_img_, binary_brightness_img, color_thread_, 255, cv::THRESH_BINARY);
    }
    else
    {
        auto light = DistillationColor(src, enemy_color_, using_hsv_);
        cv::threshold(gray_img_, binary_brightness_img, color_thread_, 255, cv::THRESH_BINARY);
        float thresh;
        if (enemy_color_ == BLUE)
            thresh = blue_thread_;
        else
            thresh = red_thread_;
        cv::threshold(light, binary_color_img, thresh, 255, cv::THRESH_BINARY);
    }
    //binary_light_img = binary_color_img & binary_brightness_img;
    auto contours_light = FindContours(binary_color_img);
    auto contours_brightness = FindContours(binary_brightness_img);

    lights.reserve(contours_light.size());
    // TODO: To be optimized
    //std::vector<int> is_processes(contours_light.size());
    for (unsigned int i = 0; i < contours_brightness.size(); ++i)
    {
        for (unsigned int j = 0; j < contours_light.size(); ++j)
        {

            if (cv::pointPolygonTest(contours_light[j], contours_brightness[i][0], false) >= 0.0)
            {
                cv::RotatedRect single_light = cv::minAreaRect(contours_brightness[i]);
                cv::Point2f vertices_point[4];
                single_light.points(vertices_point);
                LightInfo light_info(vertices_point);
                single_light.angle = light_info.angle_;
                lights.push_back(single_light);
                DrawRotatedRect(light_img, single_light, cv::Scalar(0, 255, 0), 2);
                break;
            }
        }
    }
    if(verbose > 1)
    {
        cv::imshow("Detect Lights", light_img);
    }
}

bool DetectArmor(cv::Mat &img, cv::Point3f &target)
{
    light_img = img.clone();
    filter_light_img = img.clone();
    std::vector<cv::RotatedRect> lights;
    std::vector<ArmorInfo> armors;

    cv::cvtColor(img, gray_img_, cv::COLOR_BGR2GRAY);
    DetectLights(img, lights);
    FilterLights(lights);
    PossibleArmors(lights, armors);
    FilterArmors(armors);
    if(armors.empty())
        return false;
    ArmorInfo final_armor = SelectFinalArmor(armors);
    DrawRotatedRect(img, final_armor.rect, cv::Scalar(0, 255, 0), 2);
    CalcControlInfo(final_armor, target);
    return true;
}

}
