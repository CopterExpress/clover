// This code is basically taken from https://github.com/opencv/opencv_contrib/blob/master/modules/aruco/src/aruco.cpp
// with some improvements and fixes

#include "draw.h"
#include <math.h>
#include <vector>

using namespace cv;
using namespace cv::aruco;

void _drawPlanarBoard(Board *_board, Size outSize, OutputArray _img, int marginSize,
                      int borderBits, bool drawAxis) {

	CV_Assert(outSize.area() > 0);
	CV_Assert(marginSize >= 0);

	_img.create(outSize, drawAxis ? CV_8UC3 : CV_8UC1);
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
		side = std::max(side, 10);

		dictionary.drawMarker(_board->ids[m], side, marker, borderBits);
		if (drawAxis) {
			cvtColor(marker, marker, COLOR_GRAY2RGB);
		}

		// interpolate tiny marker to marker position in markerZone
		inCorners[0] = Point2f(-0.5f, -0.5f);
		inCorners[1] = Point2f(marker.cols - 0.5f, -0.5f);
		inCorners[2] = Point2f(marker.cols - 0.5f, marker.rows - 0.5f);

		// remove perspective
		Mat transformation = getAffineTransform(inCorners, outCorners);
		warpAffine(marker, out, transformation, out.size(), INTER_LINEAR,
						BORDER_TRANSPARENT);
	}

	// draw axis
	if (drawAxis) {
		Size wholeSize; Point ofs;
		out.locateROI(wholeSize, ofs);
		auto out_copy = _img.getMat();

		cv::Point center(ofs.x - minX / sizeX * float(out.cols), ofs.y + out.rows + minY / sizeY * float(out.rows));

		int axis_points[3][2] = {{300, 0}, {0, -300}, {-150, 150}};
		Point axis_names[3] = {Point(270, 50), Point(25, -270), Point(-160, 115)};
		Scalar colors[] = {Scalar(255, 0, 0), Scalar(0, 255, 0), Scalar(0, 0, 255)};
		String names[] = {"X", "Y", "Z"};

		int r_half = 14;
		int height = 55;

		for(int poly = 2; poly >= 0; poly--){
			double alpha = atan2(0 - axis_points[poly][0], 0 - axis_points[poly][1]);
			float x_delta = r_half * cos(alpha);
			float y_delta = r_half * sin(alpha);

			Point polygon_vertices[1][3];
			polygon_vertices[0][0] = center + Point(axis_points[poly][0] + x_delta, axis_points[poly][1] - y_delta);
			polygon_vertices[0][1] = center + Point(axis_points[poly][0] - x_delta, axis_points[poly][1] + y_delta);
			polygon_vertices[0][2] = center + Point(axis_points[poly][0] - sin(alpha) * height, axis_points[poly][1] - cos(alpha) * height);

			const Point* ppt[1] = {polygon_vertices[0]};
			int npt[] = {3};

			fillPoly(out_copy, ppt, npt, 1, colors[poly]);
			putText(out_copy, names[poly], center + axis_names[poly], FONT_HERSHEY_SIMPLEX, 1.2, colors[poly], 7);
			line(out_copy, center, center + Point(axis_points[poly][0], axis_points[poly][1]), colors[poly], 10);
		}
	}
}

/**
 * @brief Convert point coordinates from world space to camera space.
 *
 * @param points A vector of points in world space.
 * @param rvec Rotation matrix or Rodrigues rotation vector.
 * @param tvec Translation vector from world to camera space.
 *
 * @return A vector of points in camera space.
 */
template<typename CvPointType>
static std::vector<CvPointType> worldToCamera(const std::vector<CvPointType>& points,
                                              const cv::Mat& rvec, const cv::Mat& tvec)
{
	// We operate with CV_64F matrices internally to avoid precision loss
	cv::Mat rvec_64f;
	cv::Mat tvec_64f;
	rvec.convertTo(rvec_64f, CV_64F);
	tvec.convertTo(tvec_64f, CV_64F);

	// Convert Rodrigues vector to rotation matrix
	cv::Mat rmat;
	if ((rvec_64f.cols == 3 && rvec_64f.rows == 1) ||
	    (rvec_64f.cols == 1 && rvec_64f.rows == 3))
	{
		Rodrigues(rvec_64f, rmat);
	}
	else
	{
		rmat = rvec_64f.clone();
	}
	// Make sure tvec has a size of (3, 1)
	if (tvec_64f.rows == 1)
	{
		tvec_64f = tvec_64f.t();
	}
	std::vector<CvPointType> result;
	result.reserve(points.size());
	for(const auto& point : points)
	{
		// Calculate point coordinates in camera frame
		// static_casts are here to silence potential narrowing conversion warnings
		CvPointType camPoint{
			static_cast<decltype(CvPointType::x)>(point.x * rmat.at<double>(0,0) + point.y * rmat.at<double>(0,1) + point.z * rmat.at<double>(0,2) + tvec_64f.at<double>(0)),
			static_cast<decltype(CvPointType::y)>(point.x * rmat.at<double>(1,0) + point.y * rmat.at<double>(1,1) + point.z * rmat.at<double>(1,2) + tvec_64f.at<double>(1)),
			static_cast<decltype(CvPointType::z)>(point.x * rmat.at<double>(2,0) + point.y * rmat.at<double>(2,1) + point.z * rmat.at<double>(2,2) + tvec_64f.at<double>(2))
		};
		result.push_back(camPoint);
	}
	return result;
}

/**
 * @brief Project points from camera space to screen space, applying distortion in the process.
 *
 * @param points A vector of points in camera space.
 * @param cameraMatrix OpenCV intrinsic camera matrix.
 * @param distCoeffs OpenCV distortion model coefficients.
 *
 * @return A vector of points in screen space.
 */
template<typename CvPointType>
static std::vector<CvPointType> cameraToScreen(const std::vector<CvPointType>& points,
                                               const cv::Mat& cameraMatrix,
                                               const cv::Mat& distCoeffs)
{
	// We operate with CV_64F matrices internally to avoid precision loss
	cv::Mat cm_64f; // camera matrix, CV_64F
	cv::Mat dc_64f; // distortion coefficients, CV_64F
	cameraMatrix.convertTo(cm_64f, CV_64F);
	distCoeffs.convertTo(dc_64f, CV_64F);

	// Make sure distortion vector has a size of (N, 1)
	if (dc_64f.rows == 1)
	{
		dc_64f = dc_64f.t();
	}

	// We will always use 12 distortion coefficients,
	// and we can safely pad missing ones with zeroes
	dc_64f.resize(12, 0.0);

	std::vector<CvPointType> result;
	result.reserve(points.size());

	for(const auto& point : points)
	{
		// Apply perspective projection, preserving initial Z coordinate
		// Always use double-precision
		cv::Point3d camPoint{
			point.x / point.z,
			point.y / point.z,
			point.z
		};

		// Apply distortion
		// Note that we do not consider tilted sensor distortion
		// r^2 - distance from the image center squared
		double r2 = camPoint.x * camPoint.x + camPoint.y * camPoint.y;
		// r^4 - same, but to the 4th power
		double r4 = r2 * r2;
		// r^6 - same, but to the 6th power
		double r6 = r4 * r2;
		// tg1 - first tangential shift factor (2 * x * y)
		double tg1 = 2 * camPoint.x * camPoint.y;
		// tg2 - second tangential shift factor (r^2 + 2 * x^2)
		double tg2 = r2 + 2 * camPoint.x * camPoint.x;
		// tg3 - third tangential shift factor (r^2 + 2 * y^2)
		double tg3 = r2 + 2 * camPoint.y * camPoint.y;
		// polynomial distortion factor (numerator)
		double pndist = 1 + dc_64f.at<double>(0) * r2 + dc_64f.at<double>(1) * r4 + dc_64f.at<double>(4) * r6;
		// polynomial distortion factror (denominator)
		double pddist = 1.0 / (1 + dc_64f.at<double>(5) * r2 + dc_64f.at<double>(6) * r4 + dc_64f.at<double>(7) * r6);
		// Distorted point coordinates (always double-precision)
		cv::Point3d distortedPoint{
			camPoint.x * pndist * pddist + dc_64f.at<double>(2) * tg1 + dc_64f.at<double>(3) * tg2 + dc_64f.at<double>(8) * r2 + dc_64f.at<double>(9) * r4,
			camPoint.y * pndist * pddist + dc_64f.at<double>(2) * tg3 + dc_64f.at<double>(3) * tg1 + dc_64f.at<double>(10) * r2 + dc_64f.at<double>(11) * r4,
			camPoint.z
		};

		// Convert to screen space
		// We use static_cast here to silence potential warnings about narrowing conversions
		// (we expect that to be the case)
		CvPointType screenPoint{
			static_cast<decltype(CvPointType::x)>(distortedPoint.x * cm_64f.at<double>(0, 0) + cm_64f.at<double>(0, 2)),
			static_cast<decltype(CvPointType::y)>(distortedPoint.y * cm_64f.at<double>(1, 1) + cm_64f.at<double>(1, 2)),
			static_cast<decltype(CvPointType::z)>(distortedPoint.z)
		};

		result.push_back(screenPoint);
	}
	return result;
}

/**
 * @brief Clip a line against a clip plane.
 *
 * This function "clips" a line (described by two points in *camera space*)
 * against a clip plane that is `clipPlaneDistance` meters away from the
 * camera focal point. If both points are further away from the focal point
 * than `clipPlaneDistance`, they will be returned unmodified. If one of the
 * points is behind the clipping plane, a point *on* the clipping plane will
 * be computed and returned as one of the points.
 *
 * If none of the points are visible, an empty vector will be returned.
 *
 * @param p1 First point on the line, in camera space.
 * @param p2 Second point on the line, in camera space.
 * @param clipPlaneDistance Distance from the focal point to the clipping plane.
 * @return A vector of zero or two points on the clipped line, in camera space.
 */
static std::vector<Point3f> lineClip(Point3f p1, Point3f p2, float clipPlaneDistance = 0.1f)
{
	// We don't need to compute an intersection if both points are
	// behind us
	if (p1.z < clipPlaneDistance && p2.z < clipPlaneDistance)
	{
		return {};
	}
	// We don't need to compute an intersection if both points are
	// in front of us
	if (p1.z > clipPlaneDistance && p2.z > clipPlaneDistance)
	{
		return {p1, p2};
	}
	// We don't really want to compute an intersection if both Z coordinates
	// are sufficiently close to each other
	if (std::abs(p1.z - p2.z) < 0.0001) // The number here is chosen arbitrarily
	{
		return {p1, p2};
	}
	// We compute the intersection as such:
	// zi = (1 - alpha) * p1.z + alpha * p2.z = clipPlaneDistance
	// alpha = (p1.z - clipPlaneDistance) / (p1.z - p2.z)
	double alpha = (p1.z - clipPlaneDistance) / (p1.z - p2.z);
	Point3f clipPlanePoint{
		static_cast<float>((1 - alpha) * p1.x + alpha * p2.x),
		static_cast<float>((1 - alpha) * p1.y + alpha * p2.y),
		clipPlaneDistance
	};
	if (p1.z < clipPlaneDistance)
	{
		return {clipPlanePoint, p2};
	}
	else
	{
		return {p1, clipPlanePoint};
	}
	// Unreachable?
}

void _drawAxis(InputOutputArray _image, InputArray _cameraMatrix, InputArray _distCoeffs,
              InputArray _rvec, InputArray _tvec, float length) {

	CV_Assert(_image.getMat().total() != 0 &&
			  (_image.getMat().channels() == 1 || _image.getMat().channels() == 3));
	CV_Assert(length > 0);

	// project axis points
	std::vector<Point3f> axisPoints;
	axisPoints.push_back(Point3f(0, 0, 0));
	axisPoints.push_back(Point3f(length, 0, 0));
	axisPoints.push_back(Point3f(0, length, 0));
	axisPoints.push_back(Point3f(0, 0, length));
	auto camAxisPoints = worldToCamera(axisPoints, _rvec.getMat(), _tvec.getMat());
	auto axisX = cameraToScreen(lineClip(camAxisPoints[0], camAxisPoints[1]), _cameraMatrix.getMat(), _distCoeffs.getMat());
	auto axisY = cameraToScreen(lineClip(camAxisPoints[0], camAxisPoints[2]), _cameraMatrix.getMat(), _distCoeffs.getMat());
	auto axisZ = cameraToScreen(lineClip(camAxisPoints[0], camAxisPoints[3]), _cameraMatrix.getMat(), _distCoeffs.getMat());
	if (axisX.size() > 0)
	{
		line(_image, Point2f{axisX[0].x, axisX[0].y}, Point2f{axisX[1].x, axisX[1].y},
			Scalar(0, 0, 255), 3);
	}
	if (axisY.size() > 0)
	{
		line(_image, Point2f{axisY[0].x, axisY[0].y}, Point2f{axisY[1].x, axisY[1].y},
			Scalar(0, 255, 0), 3);
	}
	if (axisZ.size() > 0)
	{
		line(_image, Point2f{axisZ[0].x, axisZ[0].y}, Point2f{axisZ[1].x, axisZ[1].y},
			Scalar(255, 0, 0), 3);
	}
}
