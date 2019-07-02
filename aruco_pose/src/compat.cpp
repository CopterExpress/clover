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

#include "compat.h"

#include <opencv2/core/version.hpp>

namespace compat
{

void getBoardObjectAndImagePoints(const cv::Ptr<cv::aruco::Board> &board, cv::InputArrayOfArrays detectedCorners,
                                  cv::InputArray detectedIds, cv::OutputArray objPoints, cv::OutputArray imgPoints)
{
#if ((CV_VERSION_MAJOR == 3) && (CV_VERSION_MINOR >= 3)) || (CV_VERSION_MAJOR > 3)
	cv::aruco::getBoardObjectAndImagePoints(board, detectedCorners, detectedIds, objPoints, imgPoints);
#else
	using std::vector;
	using cv::Point2f;
	using cv::Point3f;
	using cv::Mat;

	CV_Assert(board->ids.size() == board->objPoints.size());
	CV_Assert(detectedIds.total() == detectedCorners.total());

	size_t nDetectedMarkers = detectedIds.total();

	vector< Point3f > objPnts;
	objPnts.reserve(nDetectedMarkers);

	vector< Point2f > imgPnts;
	imgPnts.reserve(nDetectedMarkers);

	// look for detected markers that belong to the board and get their information
	for(unsigned int i = 0; i < nDetectedMarkers; i++) {
		int currentId = detectedIds.getMat().ptr< int >(0)[i];
		for(unsigned int j = 0; j < board->ids.size(); j++) {
			if(currentId == board->ids[j]) {
				for(int p = 0; p < 4; p++) {
					objPnts.push_back(board->objPoints[j][p]);
					imgPnts.push_back(detectedCorners.getMat(i).ptr< Point2f >(0)[p]);
				}
			}
		}
	}

	// create output
	Mat(objPnts).copyTo(objPoints);
	Mat(imgPnts).copyTo(imgPoints);
#endif
}

}