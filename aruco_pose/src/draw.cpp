// This code is basically taken from https://github.com/opencv/opencv_contrib/blob/master/modules/aruco/src/aruco.cpp
// with some improvements and fixes

#include "draw.h"

using namespace cv;
using namespace cv::aruco;

void drawPlanarBoard(Board *_board, Size outSize, OutputArray _img, int marginSize,
                     int borderBits) {

    CV_Assert(outSize.area() > 0);
    CV_Assert(marginSize >= 0);

    _img.create(outSize, CV_8UC1);
    Mat out = _img.getMat();
    out.setTo(Scalar::all(255));
    out.adjustROI(-marginSize, -marginSize, -marginSize, -marginSize);

    // calculate max and min values in XY plane
    CV_Assert(_board->objPoints.size() > 0);
    float minX, maxX, minY, maxY;
    minX = maxX = _board->objPoints[0][0].x;
    minY = maxY = _board->objPoints[0][0].y;

    for(unsigned int i = 0; i < _board->objPoints.size(); i++) {
        for(int j = 0; j < 4; j++) {
            minX = min(minX, _board->objPoints[i][j].x);
            maxX = max(maxX, _board->objPoints[i][j].x);
            minY = min(minY, _board->objPoints[i][j].y);
            maxY = max(maxY, _board->objPoints[i][j].y);
        }
    }

    float sizeX = maxX - minX;
    float sizeY = maxY - minY;

    // proportion transformations
    float xReduction = sizeX / float(out.cols);
    float yReduction = sizeY / float(out.rows);

    // determine the zone where the markers are placed
    if(xReduction > yReduction) {
        int nRows = int(sizeY / xReduction);
        int rowsMargins = (out.rows - nRows) / 2;
        out.adjustROI(-rowsMargins, -rowsMargins, 0, 0);
    } else {
        int nCols = int(sizeX / yReduction);
        int colsMargins = (out.cols - nCols) / 2;
        out.adjustROI(0, 0, -colsMargins, -colsMargins);
    }

    // now paint each marker
    Dictionary &dictionary = *(_board->dictionary);
    Mat marker;
    Point2f outCorners[3];
    Point2f inCorners[3];
    for(unsigned int m = 0; m < _board->objPoints.size(); m++) {
        // transform corners to markerZone coordinates
        for(int j = 0; j < 3; j++) {
            Point2f pf = Point2f(_board->objPoints[m][j].x, _board->objPoints[m][j].y);
            // move top left to 0, 0
            pf -= Point2f(minX, minY);
            pf.x = pf.x / sizeX * float(out.cols);
            pf.y = (1.0f - pf.y / sizeY) * float(out.rows);
            outCorners[j] = pf;
        }

        // get marker
        Size dst_sz(outCorners[2] - outCorners[0]); // assuming CCW order
        // dst_sz.width = dst_sz.height = std::min(dst_sz.width, dst_sz.height); //marker should be square
		double diag = std::round(std::hypot(dst_sz.width, dst_sz.height));
		int side = std::round(diag / std::sqrt(2));

		dictionary.drawMarker(_board->ids[m], side, marker, borderBits);

		try {
			if((outCorners[0].y == outCorners[1].y) && (outCorners[1].x == outCorners[2].x)) {
				// marker is aligned to image axes
				marker.copyTo(out(Rect(outCorners[0], dst_sz)));
				continue;
			}

			// interpolate tiny marker to marker position in markerZone
			inCorners[0] = Point2f(-0.5f, -0.5f);
			inCorners[1] = Point2f(marker.cols - 0.5f, -0.5f);
			inCorners[2] = Point2f(marker.cols - 0.5f, marker.rows - 0.5f);

			// remove perspective
			Mat transformation = getAffineTransform(inCorners, outCorners);
			warpAffine(marker, out, transformation, out.size(), INTER_LINEAR,
							BORDER_TRANSPARENT);
		} catch(cv::Exception& e) {
			ROS_INFO("Skip drawing marker %d", m);
		}
	}
}
