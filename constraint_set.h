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
#pragma once
#ifndef CONSTRAINT_SET_H
#define CONSTRAINT_SET_H

#include <opencv2/opencv.hpp>

namespace constraint_set
{
void InitializeConstraintSet(float intrinsic_matrix[], float distortion_coeffs[]);
bool DetectArmor(cv::Mat &img, cv::Point3f &target_3d);
}

#endif // CONSTRAINT_SET_H
