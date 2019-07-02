/*
 * Compatibility functions for older versions of dependent packages
 * Copyright (C) 2019 Copter Express Technologies
 *
 * Authors: Oleg Kalachev <okalachev@gmail.com>, Alexey Rogachevskiy <sfalexrog@gmail.com>
 *
 * Distributed under MIT License (available at https://opensource.org/licenses/MIT).
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 */

/*
 * Code is based on https://github.com/opencv/opencv_contrib, which is distributed
 * under the BSD license.
 */

#pragma once

#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace compat
{

/**
 * Given a board configuration and a set of detected markers, returns the corresponding image points and object points to call solvePnP.
 * 
 * Will call cv::aruco::getBoardObjectAndPoints if it's available. */
void getBoardObjectAndImagePoints(const cv::Ptr<cv::aruco::Board> &board, cv::InputArrayOfArrays detectedCorners,
                                  cv::InputArray detectedIds, cv::OutputArray objPoints, cv::OutputArray imgPoints);
}
