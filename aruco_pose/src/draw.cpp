// This code is basically taken from https://github.com/opencv/opencv_contrib/blob/master/modules/aruco/src/aruco.cpp
// with some improvements and fixes

#include "draw.h"

using namespace cv;
using namespace cv::aruco;

static void _cvProjectPoints2( const CvMat* object_points, const CvMat* rotation_vector,
                       const CvMat* translation_vector, const CvMat* camera_matrix,
                       const CvMat* distortion_coeffs, CvMat* image_points,
                       CvMat* dpdrot CV_DEFAULT(NULL), CvMat* dpdt CV_DEFAULT(NULL),
                       CvMat* dpdf CV_DEFAULT(NULL), CvMat* dpdc CV_DEFAULT(NULL),
                       CvMat* dpddist CV_DEFAULT(NULL),
                       double aspect_ratio CV_DEFAULT(0));

static void _projectPoints( InputArray objectPoints,
                    InputArray rvec, InputArray tvec,
                    InputArray cameraMatrix, InputArray distCoeffs,
                    OutputArray imagePoints,
                    OutputArray jacobian = noArray(),
                    double aspectRatio = 0 );


void _drawPlanarBoard(Board *_board, Size outSize, OutputArray _img, int marginSize,
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
		side = std::max(side, 10);

		dictionary.drawMarker(_board->ids[m], side, marker, borderBits);

		// interpolate tiny marker to marker position in markerZone
		inCorners[0] = Point2f(-0.5f, -0.5f);
		inCorners[1] = Point2f(marker.cols - 0.5f, -0.5f);
		inCorners[2] = Point2f(marker.cols - 0.5f, marker.rows - 0.5f);

		// remove perspective
		Mat transformation = getAffineTransform(inCorners, outCorners);
		warpAffine(marker, out, transformation, out.size(), INTER_LINEAR,
						BORDER_TRANSPARENT);
	}
}

/* Draw a (potentially partially visible) line. */
static void linePartial(InputOutputArray image, Point3f p1, Point3f p2, const Scalar& color,
                        int thickness = 1, int lineType = LINE_8, int shift = 0)
{
	// If both points are behind the screen, don't draw anything
	if (p1.z <= 0 && p2.z <= 0) {
		return;
	}
	Point2f p1p{p1.x, p1.y};
	Point2f p2p{p2.x, p2.y};
	// If points are on the different sides of the plane, compute intersection point
	if (p1.z * p2.z < 0) {
		// Compute intersection point with the screen
		// We denote alpha as such:
		// xi = (1 - alpha) * x1 + alpha * x2
		// yi = (1 - alpha) * y1 + alpha * y2
		// zi = (1 - alpha) * z1 + alpha * z2 = 0
		// Thus, alpha can be expressed as
		// alpha = z1 / (z1 - z2)
		float alpha = p1.z / (p1.z - p2.z);
		Point2f pi{(1 - alpha) * p1.x + alpha * p2.x, (1 - alpha) * p1.y + alpha * p2.y};
		// Now, if z1 is negative, we draw the line from (xi, yi) to (x2, y2), else we draw from (x1, y1) to (xi, yi)
		if (p1.z < 0) {
			p1p = pi;
		} else {
			p2p = pi;
		}
	}
	line(image, p1p, p2p, color, thickness, lineType, shift);
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
	std::vector<Point3f> imagePointsZ;
	_projectPoints(axisPoints, _rvec, _tvec, _cameraMatrix, _distCoeffs, imagePointsZ);

	// draw axis lines
	linePartial(_image, imagePointsZ[0], imagePointsZ[1], Scalar(0, 0, 255), 3);
	linePartial(_image, imagePointsZ[0], imagePointsZ[2], Scalar(0, 255, 0), 3);
	linePartial(_image, imagePointsZ[0], imagePointsZ[3], Scalar(255, 0, 0), 3);
}

static CvMat _cvMat(const cv::Mat& m)
{
	CvMat self;
	CV_DbgAssert(m.dims <= 2);
	self = cvMat(m.rows, m.dims == 1 ? 1 : m.cols, m.type(), m.data);
	self.step = (int)m.step[0];
	self.type = (self.type & ~cv::Mat::CONTINUOUS_FLAG) | (m.flags & cv::Mat::CONTINUOUS_FLAG);
	return self;
}

static void _projectPoints( InputArray _opoints,
                        InputArray _rvec,
                        InputArray _tvec,
                        InputArray _cameraMatrix,
                        InputArray _distCoeffs,
                        OutputArray _ipoints,
                        OutputArray _jacobian,
                        double aspectRatio )
{
	Mat opoints = _opoints.getMat();
	int npoints = opoints.checkVector(3), depth = opoints.depth();
	CV_Assert(npoints >= 0 && (depth == CV_32F || depth == CV_64F));

	CvMat dpdrot, dpdt, dpdf, dpdc, dpddist;
	CvMat *pdpdrot = 0, *pdpdt = 0, *pdpdf = 0, *pdpdc = 0, *pdpddist = 0;

	CV_Assert(_ipoints.needed());

	_ipoints.create(npoints, 1, CV_MAKETYPE(depth, 3), -1, true);
	Mat imagePoints = _ipoints.getMat();
	CvMat c_imagePoints = _cvMat(imagePoints);
	CvMat c_objectPoints = _cvMat(opoints);
	Mat cameraMatrix = _cameraMatrix.getMat();

	Mat rvec = _rvec.getMat(), tvec = _tvec.getMat();
	CvMat c_cameraMatrix = _cvMat(cameraMatrix);
	CvMat c_rvec = _cvMat(rvec), c_tvec = _cvMat(tvec);

	double dc0buf[5] = {0};
	Mat dc0(5, 1, CV_64F, dc0buf);
	Mat distCoeffs = _distCoeffs.getMat();
	if (distCoeffs.empty())
		distCoeffs = dc0;
	CvMat c_distCoeffs = _cvMat(distCoeffs);
	int ndistCoeffs = distCoeffs.rows + distCoeffs.cols - 1;

	Mat jacobian;
	if (_jacobian.needed())
	{
		_jacobian.create(npoints * 2, 3 + 3 + 2 + 2 + ndistCoeffs, CV_64F);
		jacobian = _jacobian.getMat();
		pdpdrot = &(dpdrot = _cvMat(jacobian.colRange(0, 3)));
		pdpdt = &(dpdt = _cvMat(jacobian.colRange(3, 6)));
		pdpdf = &(dpdf = _cvMat(jacobian.colRange(6, 8)));
		pdpdc = &(dpdc = _cvMat(jacobian.colRange(8, 10)));
		pdpddist = &(dpddist = _cvMat(jacobian.colRange(10, 10 + ndistCoeffs)));
	}

	_cvProjectPoints2(&c_objectPoints, &c_rvec, &c_tvec, &c_cameraMatrix, &c_distCoeffs,
					  &c_imagePoints, pdpdrot, pdpdt, pdpdf, pdpdc, pdpddist, aspectRatio);
}

namespace _detail
{
    template <typename FLOAT>
    void computeTiltProjectionMatrix(FLOAT tauX,
                                     FLOAT tauY,
                                     Matx<FLOAT, 3, 3>* matTilt = 0,
                                     Matx<FLOAT, 3, 3>* dMatTiltdTauX = 0,
                                     Matx<FLOAT, 3, 3>* dMatTiltdTauY = 0,
                                     Matx<FLOAT, 3, 3>* invMatTilt = 0)
    {
        FLOAT cTauX = cos(tauX);
        FLOAT sTauX = sin(tauX);
        FLOAT cTauY = cos(tauY);
        FLOAT sTauY = sin(tauY);
        Matx<FLOAT, 3, 3> matRotX = Matx<FLOAT, 3, 3>(1,0,0,0,cTauX,sTauX,0,-sTauX,cTauX);
        Matx<FLOAT, 3, 3> matRotY = Matx<FLOAT, 3, 3>(cTauY,0,-sTauY,0,1,0,sTauY,0,cTauY);
        Matx<FLOAT, 3, 3> matRotXY = matRotY * matRotX;
        Matx<FLOAT, 3, 3> matProjZ = Matx<FLOAT, 3, 3>(matRotXY(2,2),0,-matRotXY(0,2),0,matRotXY(2,2),-matRotXY(1,2),0,0,1);
        if (matTilt)
        {
            // Matrix for trapezoidal distortion of tilted image sensor
            *matTilt = matProjZ * matRotXY;
        }
        if (dMatTiltdTauX)
        {
            // Derivative with respect to tauX
            Matx<FLOAT, 3, 3> dMatRotXYdTauX = matRotY * Matx<FLOAT, 3, 3>(0,0,0,0,-sTauX,cTauX,0,-cTauX,-sTauX);
            Matx<FLOAT, 3, 3> dMatProjZdTauX = Matx<FLOAT, 3, 3>(dMatRotXYdTauX(2,2),0,-dMatRotXYdTauX(0,2),
                                                                 0,dMatRotXYdTauX(2,2),-dMatRotXYdTauX(1,2),0,0,0);
            *dMatTiltdTauX = (matProjZ * dMatRotXYdTauX) + (dMatProjZdTauX * matRotXY);
        }
        if (dMatTiltdTauY)
        {
            // Derivative with respect to tauY
            Matx<FLOAT, 3, 3> dMatRotXYdTauY = Matx<FLOAT, 3, 3>(-sTauY,0,-cTauY,0,0,0,cTauY,0,-sTauY) * matRotX;
            Matx<FLOAT, 3, 3> dMatProjZdTauY = Matx<FLOAT, 3, 3>(dMatRotXYdTauY(2,2),0,-dMatRotXYdTauY(0,2),
                                                                 0,dMatRotXYdTauY(2,2),-dMatRotXYdTauY(1,2),0,0,0);
            *dMatTiltdTauY = (matProjZ * dMatRotXYdTauY) + (dMatProjZdTauY * matRotXY);
        }
        if (invMatTilt)
        {
            FLOAT inv = 1./matRotXY(2,2);
            Matx<FLOAT, 3, 3> invMatProjZ = Matx<FLOAT, 3, 3>(inv,0,inv*matRotXY(0,2),0,inv,inv*matRotXY(1,2),0,0,1);
            *invMatTilt = matRotXY.t()*invMatProjZ;
        }
    }
}

static const char* cvDistCoeffErr = "Distortion coefficients must be 1x4, 4x1, 1x5, 5x1, 1x8, 8x1, 1x12, 12x1, 1x14 or 14x1 floating-point vector";

static void _cvProjectPoints2Internal( const CvMat* objectPoints,
                                      const CvMat* r_vec,
                                      const CvMat* t_vec,
                                      const CvMat* A,
                                      const CvMat* distCoeffs,
                                      CvMat* imagePoints, CvMat* dpdr CV_DEFAULT(NULL),
                                      CvMat* dpdt CV_DEFAULT(NULL), CvMat* dpdf CV_DEFAULT(NULL),
                                      CvMat* dpdc CV_DEFAULT(NULL), CvMat* dpdk CV_DEFAULT(NULL),
                                      CvMat* dpdo CV_DEFAULT(NULL),
                                      double aspectRatio CV_DEFAULT(0) )
{
    Ptr<CvMat> matM, _m;
    Ptr<CvMat> _dpdr, _dpdt, _dpdc, _dpdf, _dpdk;
    Ptr<CvMat> _dpdo;

    int i, j, count;
    int calc_derivatives;
    const CvPoint3D64f* M;
    CvPoint3D64f* m;
    double r[3], R[9], dRdr[27], t[3], a[9], k[14] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0}, fx, fy, cx, cy;
    Matx33d matTilt = Matx33d::eye();
    Matx33d dMatTiltdTauX(0,0,0,0,0,0,0,-1,0);
    Matx33d dMatTiltdTauY(0,0,0,0,0,0,1,0,0);
    CvMat _r, _t, _a = cvMat( 3, 3, CV_64F, a ), _k;
    CvMat matR = cvMat( 3, 3, CV_64F, R ), _dRdr = cvMat( 3, 9, CV_64F, dRdr );
    double *dpdr_p = 0, *dpdt_p = 0, *dpdk_p = 0, *dpdf_p = 0, *dpdc_p = 0;
    double* dpdo_p = 0;
    int dpdr_step = 0, dpdt_step = 0, dpdk_step = 0, dpdf_step = 0, dpdc_step = 0;
    int dpdo_step = 0;
    bool fixedAspectRatio = aspectRatio > FLT_EPSILON;

    if( !CV_IS_MAT(objectPoints) || !CV_IS_MAT(r_vec) ||
        !CV_IS_MAT(t_vec) || !CV_IS_MAT(A) ||
        /*!CV_IS_MAT(distCoeffs) ||*/ !CV_IS_MAT(imagePoints) )
        CV_Error( CV_StsBadArg, "One of required arguments is not a valid matrix" );

    int total = objectPoints->rows * objectPoints->cols * CV_MAT_CN(objectPoints->type);
    if(total % 3 != 0)
    {
        //we have stopped support of homogeneous coordinates because it cause ambiguity in interpretation of the input data
        CV_Error( CV_StsBadArg, "Homogeneous coordinates are not supported" );
    }
    count = total / 3;

    if( CV_IS_CONT_MAT(objectPoints->type) &&
        (CV_MAT_DEPTH(objectPoints->type) == CV_32F || CV_MAT_DEPTH(objectPoints->type) == CV_64F)&&
        ((objectPoints->rows == 1 && CV_MAT_CN(objectPoints->type) == 3) ||
         (objectPoints->rows == count && CV_MAT_CN(objectPoints->type)*objectPoints->cols == 3) ||
         (objectPoints->rows == 3 && CV_MAT_CN(objectPoints->type) == 1 && objectPoints->cols == count)))
    {
        matM.reset(cvCreateMat( objectPoints->rows, objectPoints->cols, CV_MAKETYPE(CV_64F,CV_MAT_CN(objectPoints->type)) ));
        cvConvert(objectPoints, matM);
    }
    else
    {
//        matM = cvCreateMat( 1, count, CV_64FC3 );
//        cvConvertPointsHomogeneous( objectPoints, matM );
        CV_Error( CV_StsBadArg, "Homogeneous coordinates are not supported" );
    }

    if( CV_IS_CONT_MAT(imagePoints->type) &&
        (CV_MAT_DEPTH(imagePoints->type) == CV_32F || CV_MAT_DEPTH(imagePoints->type) == CV_64F) &&
        ((imagePoints->rows == 1 && CV_MAT_CN(imagePoints->type) == 3) ||
         (imagePoints->rows == count && CV_MAT_CN(imagePoints->type)*imagePoints->cols == 3) ||
         (imagePoints->rows == 3 && CV_MAT_CN(imagePoints->type) == 1 && imagePoints->cols == count)))
    {
        _m.reset(cvCreateMat( imagePoints->rows, imagePoints->cols, CV_MAKETYPE(CV_64F,CV_MAT_CN(imagePoints->type)) ));
        cvConvert(imagePoints, _m);
    }
    else
    {
//        _m = cvCreateMat( 1, count, CV_64FC2 );
        CV_Error( CV_StsBadArg, "Homogeneous coordinates are not supported" );
    }

    M = (CvPoint3D64f*)matM->data.db;
    m = (CvPoint3D64f*)_m->data.db;

    if( (CV_MAT_DEPTH(r_vec->type) != CV_64F && CV_MAT_DEPTH(r_vec->type) != CV_32F) ||
        (((r_vec->rows != 1 && r_vec->cols != 1) ||
          r_vec->rows*r_vec->cols*CV_MAT_CN(r_vec->type) != 3) &&
         ((r_vec->rows != 3 && r_vec->cols != 3) || CV_MAT_CN(r_vec->type) != 1)))
        CV_Error( CV_StsBadArg, "Rotation must be represented by 1x3 or 3x1 "
                                "floating-point rotation vector, or 3x3 rotation matrix" );

    if( r_vec->rows == 3 && r_vec->cols == 3 )
    {
        _r = cvMat( 3, 1, CV_64FC1, r );
        cvRodrigues2( r_vec, &_r );
        cvRodrigues2( &_r, &matR, &_dRdr );
        cvCopy( r_vec, &matR );
    }
    else
    {
        _r = cvMat( r_vec->rows, r_vec->cols, CV_MAKETYPE(CV_64F,CV_MAT_CN(r_vec->type)), r );
        cvConvert( r_vec, &_r );
        cvRodrigues2( &_r, &matR, &_dRdr );
    }

    if( (CV_MAT_DEPTH(t_vec->type) != CV_64F && CV_MAT_DEPTH(t_vec->type) != CV_32F) ||
        (t_vec->rows != 1 && t_vec->cols != 1) ||
        t_vec->rows*t_vec->cols*CV_MAT_CN(t_vec->type) != 3 )
        CV_Error( CV_StsBadArg,
                  "Translation vector must be 1x3 or 3x1 floating-point vector" );

    _t = cvMat( t_vec->rows, t_vec->cols, CV_MAKETYPE(CV_64F,CV_MAT_CN(t_vec->type)), t );
    cvConvert( t_vec, &_t );

    if( (CV_MAT_TYPE(A->type) != CV_64FC1 && CV_MAT_TYPE(A->type) != CV_32FC1) ||
        A->rows != 3 || A->cols != 3 )
        CV_Error( CV_StsBadArg, "Instrinsic parameters must be 3x3 floating-point matrix" );

    cvConvert( A, &_a );
    fx = a[0]; fy = a[4];
    cx = a[2]; cy = a[5];

    if( fixedAspectRatio )
        fx = fy*aspectRatio;

    if( distCoeffs )
    {
        if( !CV_IS_MAT(distCoeffs) ||
            (CV_MAT_DEPTH(distCoeffs->type) != CV_64F &&
             CV_MAT_DEPTH(distCoeffs->type) != CV_32F) ||
            (distCoeffs->rows != 1 && distCoeffs->cols != 1) ||
            (distCoeffs->rows*distCoeffs->cols*CV_MAT_CN(distCoeffs->type) != 4 &&
             distCoeffs->rows*distCoeffs->cols*CV_MAT_CN(distCoeffs->type) != 5 &&
             distCoeffs->rows*distCoeffs->cols*CV_MAT_CN(distCoeffs->type) != 8 &&
             distCoeffs->rows*distCoeffs->cols*CV_MAT_CN(distCoeffs->type) != 12 &&
             distCoeffs->rows*distCoeffs->cols*CV_MAT_CN(distCoeffs->type) != 14) )
            CV_Error( CV_StsBadArg, cvDistCoeffErr );

        _k = cvMat( distCoeffs->rows, distCoeffs->cols,
                    CV_MAKETYPE(CV_64F,CV_MAT_CN(distCoeffs->type)), k );
        cvConvert( distCoeffs, &_k );
        if(k[12] != 0 || k[13] != 0)
        {
            _detail::computeTiltProjectionMatrix(k[12], k[13],
                                                &matTilt, &dMatTiltdTauX, &dMatTiltdTauY);
        }
    }

    if( dpdr )
    {
        if( !CV_IS_MAT(dpdr) ||
            (CV_MAT_TYPE(dpdr->type) != CV_32FC1 &&
             CV_MAT_TYPE(dpdr->type) != CV_64FC1) ||
            dpdr->rows != count*2 || dpdr->cols != 3 )
            CV_Error( CV_StsBadArg, "dp/drot must be 2Nx3 floating-point matrix" );

        if( CV_MAT_TYPE(dpdr->type) == CV_64FC1 )
        {
            _dpdr.reset(cvCloneMat(dpdr));
        }
        else
            _dpdr.reset(cvCreateMat( 2*count, 3, CV_64FC1 ));
        dpdr_p = _dpdr->data.db;
        dpdr_step = _dpdr->step/sizeof(dpdr_p[0]);
    }

    if( dpdt )
    {
        if( !CV_IS_MAT(dpdt) ||
            (CV_MAT_TYPE(dpdt->type) != CV_32FC1 &&
             CV_MAT_TYPE(dpdt->type) != CV_64FC1) ||
            dpdt->rows != count*2 || dpdt->cols != 3 )
            CV_Error( CV_StsBadArg, "dp/dT must be 2Nx3 floating-point matrix" );

        if( CV_MAT_TYPE(dpdt->type) == CV_64FC1 )
        {
            _dpdt.reset(cvCloneMat(dpdt));
        }
        else
            _dpdt.reset(cvCreateMat( 2*count, 3, CV_64FC1 ));
        dpdt_p = _dpdt->data.db;
        dpdt_step = _dpdt->step/sizeof(dpdt_p[0]);
    }

    if( dpdf )
    {
        if( !CV_IS_MAT(dpdf) ||
            (CV_MAT_TYPE(dpdf->type) != CV_32FC1 && CV_MAT_TYPE(dpdf->type) != CV_64FC1) ||
            dpdf->rows != count*2 || dpdf->cols != 2 )
            CV_Error( CV_StsBadArg, "dp/df must be 2Nx2 floating-point matrix" );

        if( CV_MAT_TYPE(dpdf->type) == CV_64FC1 )
        {
            _dpdf.reset(cvCloneMat(dpdf));
        }
        else
            _dpdf.reset(cvCreateMat( 2*count, 2, CV_64FC1 ));
        dpdf_p = _dpdf->data.db;
        dpdf_step = _dpdf->step/sizeof(dpdf_p[0]);
    }

    if( dpdc )
    {
        if( !CV_IS_MAT(dpdc) ||
            (CV_MAT_TYPE(dpdc->type) != CV_32FC1 && CV_MAT_TYPE(dpdc->type) != CV_64FC1) ||
            dpdc->rows != count*2 || dpdc->cols != 2 )
            CV_Error( CV_StsBadArg, "dp/dc must be 2Nx2 floating-point matrix" );

        if( CV_MAT_TYPE(dpdc->type) == CV_64FC1 )
        {
            _dpdc.reset(cvCloneMat(dpdc));
        }
        else
            _dpdc.reset(cvCreateMat( 2*count, 2, CV_64FC1 ));
        dpdc_p = _dpdc->data.db;
        dpdc_step = _dpdc->step/sizeof(dpdc_p[0]);
    }

    if( dpdk )
    {
        if( !CV_IS_MAT(dpdk) ||
            (CV_MAT_TYPE(dpdk->type) != CV_32FC1 && CV_MAT_TYPE(dpdk->type) != CV_64FC1) ||
            dpdk->rows != count*2 || (dpdk->cols != 14 && dpdk->cols != 12 && dpdk->cols != 8 && dpdk->cols != 5 && dpdk->cols != 4 && dpdk->cols != 2) )
            CV_Error( CV_StsBadArg, "dp/df must be 2Nx14, 2Nx12, 2Nx8, 2Nx5, 2Nx4 or 2Nx2 floating-point matrix" );

        if( !distCoeffs )
            CV_Error( CV_StsNullPtr, "distCoeffs is NULL while dpdk is not" );

        if( CV_MAT_TYPE(dpdk->type) == CV_64FC1 )
        {
            _dpdk.reset(cvCloneMat(dpdk));
        }
        else
            _dpdk.reset(cvCreateMat( dpdk->rows, dpdk->cols, CV_64FC1 ));
        dpdk_p = _dpdk->data.db;
        dpdk_step = _dpdk->step/sizeof(dpdk_p[0]);
    }

    if( dpdo )
    {
        if( !CV_IS_MAT( dpdo ) || ( CV_MAT_TYPE( dpdo->type ) != CV_32FC1
                                    && CV_MAT_TYPE( dpdo->type ) != CV_64FC1 )
            || dpdo->rows != count * 2 || dpdo->cols != count * 3 )
            CV_Error( CV_StsBadArg, "dp/do must be 2Nx3N floating-point matrix" );

        if( CV_MAT_TYPE( dpdo->type ) == CV_64FC1 )
        {
            _dpdo.reset( cvCloneMat( dpdo ) );
        }
        else
            _dpdo.reset( cvCreateMat( 2 * count, 3 * count, CV_64FC1 ) );
        cvZero(_dpdo);
        dpdo_p = _dpdo->data.db;
        dpdo_step = _dpdo->step / sizeof( dpdo_p[0] );
    }

    calc_derivatives = dpdr || dpdt || dpdf || dpdc || dpdk || dpdo;

    for( i = 0; i < count; i++ )
    {
        double X = M[i].x, Y = M[i].y, Z = M[i].z;
        double x = R[0]*X + R[1]*Y + R[2]*Z + t[0];
        double y = R[3]*X + R[4]*Y + R[5]*Z + t[1];
        double z = R[6]*X + R[7]*Y + R[8]*Z + t[2];
        double r2, r4, r6, a1, a2, a3, cdist, icdist2;
        double xd, yd, xd0, yd0, invProj;
        Vec3d vecTilt;
        Vec3d dVecTilt;
        Matx22d dMatTilt;
        Vec2d dXdYd;

        double z0 = z;
        z = z ? 1./z : 1;
        x *= z; y *= z;

        r2 = x*x + y*y;
        r4 = r2*r2;
        r6 = r4*r2;
        a1 = 2*x*y;
        a2 = r2 + 2*x*x;
        a3 = r2 + 2*y*y;
        cdist = 1 + k[0]*r2 + k[1]*r4 + k[4]*r6;
        icdist2 = 1./(1 + k[5]*r2 + k[6]*r4 + k[7]*r6);
        xd0 = x*cdist*icdist2 + k[2]*a1 + k[3]*a2 + k[8]*r2+k[9]*r4;
        yd0 = y*cdist*icdist2 + k[2]*a3 + k[3]*a1 + k[10]*r2+k[11]*r4;

        // additional distortion by projecting onto a tilt plane
        vecTilt = matTilt*Vec3d(xd0, yd0, 1);
        invProj = vecTilt(2) ? 1./vecTilt(2) : 1;
        xd = invProj * vecTilt(0);
        yd = invProj * vecTilt(1);

        m[i].x = xd*fx + cx;
        m[i].y = yd*fy + cy;
        m[i].z = z; // Just put the projected Z coordinate here, we mainly care about the sign

        if( calc_derivatives )
        {
            if( dpdc_p )
            {
                dpdc_p[0] = 1; dpdc_p[1] = 0; // dp_xdc_x; dp_xdc_y
                dpdc_p[dpdc_step] = 0;
                dpdc_p[dpdc_step+1] = 1;
                dpdc_p += dpdc_step*2;
            }

            if( dpdf_p )
            {
                if( fixedAspectRatio )
                {
                    dpdf_p[0] = 0; dpdf_p[1] = xd*aspectRatio; // dp_xdf_x; dp_xdf_y
                    dpdf_p[dpdf_step] = 0;
                    dpdf_p[dpdf_step+1] = yd;
                }
                else
                {
                    dpdf_p[0] = xd; dpdf_p[1] = 0;
                    dpdf_p[dpdf_step] = 0;
                    dpdf_p[dpdf_step+1] = yd;
                }
                dpdf_p += dpdf_step*2;
            }
            for (int row = 0; row < 2; ++row)
                for (int col = 0; col < 2; ++col)
                    dMatTilt(row,col) = matTilt(row,col)*vecTilt(2)
                                        - matTilt(2,col)*vecTilt(row);
            double invProjSquare = (invProj*invProj);
            dMatTilt *= invProjSquare;
            if( dpdk_p )
            {
                dXdYd = dMatTilt*Vec2d(x*icdist2*r2, y*icdist2*r2);
                dpdk_p[0] = fx*dXdYd(0);
                dpdk_p[dpdk_step] = fy*dXdYd(1);
                dXdYd = dMatTilt*Vec2d(x*icdist2*r4, y*icdist2*r4);
                dpdk_p[1] = fx*dXdYd(0);
                dpdk_p[dpdk_step+1] = fy*dXdYd(1);
                if( _dpdk->cols > 2 )
                {
                    dXdYd = dMatTilt*Vec2d(a1, a3);
                    dpdk_p[2] = fx*dXdYd(0);
                    dpdk_p[dpdk_step+2] = fy*dXdYd(1);
                    dXdYd = dMatTilt*Vec2d(a2, a1);
                    dpdk_p[3] = fx*dXdYd(0);
                    dpdk_p[dpdk_step+3] = fy*dXdYd(1);
                    if( _dpdk->cols > 4 )
                    {
                        dXdYd = dMatTilt*Vec2d(x*icdist2*r6, y*icdist2*r6);
                        dpdk_p[4] = fx*dXdYd(0);
                        dpdk_p[dpdk_step+4] = fy*dXdYd(1);

                        if( _dpdk->cols > 5 )
                        {
                            dXdYd = dMatTilt*Vec2d(
                                    x*cdist*(-icdist2)*icdist2*r2, y*cdist*(-icdist2)*icdist2*r2);
                            dpdk_p[5] = fx*dXdYd(0);
                            dpdk_p[dpdk_step+5] = fy*dXdYd(1);
                            dXdYd = dMatTilt*Vec2d(
                                    x*cdist*(-icdist2)*icdist2*r4, y*cdist*(-icdist2)*icdist2*r4);
                            dpdk_p[6] = fx*dXdYd(0);
                            dpdk_p[dpdk_step+6] = fy*dXdYd(1);
                            dXdYd = dMatTilt*Vec2d(
                                    x*cdist*(-icdist2)*icdist2*r6, y*cdist*(-icdist2)*icdist2*r6);
                            dpdk_p[7] = fx*dXdYd(0);
                            dpdk_p[dpdk_step+7] = fy*dXdYd(1);
                            if( _dpdk->cols > 8 )
                            {
                                dXdYd = dMatTilt*Vec2d(r2, 0);
                                dpdk_p[8] = fx*dXdYd(0); //s1
                                dpdk_p[dpdk_step+8] = fy*dXdYd(1); //s1
                                dXdYd = dMatTilt*Vec2d(r4, 0);
                                dpdk_p[9] = fx*dXdYd(0); //s2
                                dpdk_p[dpdk_step+9] = fy*dXdYd(1); //s2
                                dXdYd = dMatTilt*Vec2d(0, r2);
                                dpdk_p[10] = fx*dXdYd(0);//s3
                                dpdk_p[dpdk_step+10] = fy*dXdYd(1); //s3
                                dXdYd = dMatTilt*Vec2d(0, r4);
                                dpdk_p[11] = fx*dXdYd(0);//s4
                                dpdk_p[dpdk_step+11] = fy*dXdYd(1); //s4
                                if( _dpdk->cols > 12 )
                                {
                                    dVecTilt = dMatTiltdTauX * Vec3d(xd0, yd0, 1);
                                    dpdk_p[12] = fx * invProjSquare * (
                                            dVecTilt(0) * vecTilt(2) - dVecTilt(2) * vecTilt(0));
                                    dpdk_p[dpdk_step+12] = fy*invProjSquare * (
                                            dVecTilt(1) * vecTilt(2) - dVecTilt(2) * vecTilt(1));
                                    dVecTilt = dMatTiltdTauY * Vec3d(xd0, yd0, 1);
                                    dpdk_p[13] = fx * invProjSquare * (
                                            dVecTilt(0) * vecTilt(2) - dVecTilt(2) * vecTilt(0));
                                    dpdk_p[dpdk_step+13] = fy * invProjSquare * (
                                            dVecTilt(1) * vecTilt(2) - dVecTilt(2) * vecTilt(1));
                                }
                            }
                        }
                    }
                }
                dpdk_p += dpdk_step*2;
            }

            if( dpdt_p )
            {
                double dxdt[] = { z, 0, -x*z }, dydt[] = { 0, z, -y*z };
                for( j = 0; j < 3; j++ )
                {
                    double dr2dt = 2*x*dxdt[j] + 2*y*dydt[j];
                    double dcdist_dt = k[0]*dr2dt + 2*k[1]*r2*dr2dt + 3*k[4]*r4*dr2dt;
                    double dicdist2_dt = -icdist2*icdist2*(k[5]*dr2dt + 2*k[6]*r2*dr2dt + 3*k[7]*r4*dr2dt);
                    double da1dt = 2*(x*dydt[j] + y*dxdt[j]);
                    double dmxdt = (dxdt[j]*cdist*icdist2 + x*dcdist_dt*icdist2 + x*cdist*dicdist2_dt +
                                    k[2]*da1dt + k[3]*(dr2dt + 4*x*dxdt[j]) + k[8]*dr2dt + 2*r2*k[9]*dr2dt);
                    double dmydt = (dydt[j]*cdist*icdist2 + y*dcdist_dt*icdist2 + y*cdist*dicdist2_dt +
                                    k[2]*(dr2dt + 4*y*dydt[j]) + k[3]*da1dt + k[10]*dr2dt + 2*r2*k[11]*dr2dt);
                    dXdYd = dMatTilt*Vec2d(dmxdt, dmydt);
                    dpdt_p[j] = fx*dXdYd(0);
                    dpdt_p[dpdt_step+j] = fy*dXdYd(1);
                }
                dpdt_p += dpdt_step*2;
            }

            if( dpdr_p )
            {
                double dx0dr[] =
                        {
                                X*dRdr[0] + Y*dRdr[1] + Z*dRdr[2],
                                X*dRdr[9] + Y*dRdr[10] + Z*dRdr[11],
                                X*dRdr[18] + Y*dRdr[19] + Z*dRdr[20]
                        };
                double dy0dr[] =
                        {
                                X*dRdr[3] + Y*dRdr[4] + Z*dRdr[5],
                                X*dRdr[12] + Y*dRdr[13] + Z*dRdr[14],
                                X*dRdr[21] + Y*dRdr[22] + Z*dRdr[23]
                        };
                double dz0dr[] =
                        {
                                X*dRdr[6] + Y*dRdr[7] + Z*dRdr[8],
                                X*dRdr[15] + Y*dRdr[16] + Z*dRdr[17],
                                X*dRdr[24] + Y*dRdr[25] + Z*dRdr[26]
                        };
                for( j = 0; j < 3; j++ )
                {
                    double dxdr = z*(dx0dr[j] - x*dz0dr[j]);
                    double dydr = z*(dy0dr[j] - y*dz0dr[j]);
                    double dr2dr = 2*x*dxdr + 2*y*dydr;
                    double dcdist_dr = (k[0] + 2*k[1]*r2 + 3*k[4]*r4)*dr2dr;
                    double dicdist2_dr = -icdist2*icdist2*(k[5] + 2*k[6]*r2 + 3*k[7]*r4)*dr2dr;
                    double da1dr = 2*(x*dydr + y*dxdr);
                    double dmxdr = (dxdr*cdist*icdist2 + x*dcdist_dr*icdist2 + x*cdist*dicdist2_dr +
                                    k[2]*da1dr + k[3]*(dr2dr + 4*x*dxdr) + (k[8] + 2*r2*k[9])*dr2dr);
                    double dmydr = (dydr*cdist*icdist2 + y*dcdist_dr*icdist2 + y*cdist*dicdist2_dr +
                                    k[2]*(dr2dr + 4*y*dydr) + k[3]*da1dr + (k[10] + 2*r2*k[11])*dr2dr);
                    dXdYd = dMatTilt*Vec2d(dmxdr, dmydr);
                    dpdr_p[j] = fx*dXdYd(0);
                    dpdr_p[dpdr_step+j] = fy*dXdYd(1);
                }
                dpdr_p += dpdr_step*2;
            }

            if( dpdo_p )
            {
                double dxdo[] = { z * ( R[0] - x * z * z0 * R[6] ),
                                  z * ( R[1] - x * z * z0 * R[7] ),
                                  z * ( R[2] - x * z * z0 * R[8] ) };
                double dydo[] = { z * ( R[3] - y * z * z0 * R[6] ),
                                  z * ( R[4] - y * z * z0 * R[7] ),
                                  z * ( R[5] - y * z * z0 * R[8] ) };
                for( j = 0; j < 3; j++ )
                {
                    double dr2do = 2 * x * dxdo[j] + 2 * y * dydo[j];
                    double dr4do = 2 * r2 * dr2do;
                    double dr6do = 3 * r4 * dr2do;
                    double da1do = 2 * y * dxdo[j] + 2 * x * dydo[j];
                    double da2do = dr2do + 4 * x * dxdo[j];
                    double da3do = dr2do + 4 * y * dydo[j];
                    double dcdist_do
                            = k[0] * dr2do + k[1] * dr4do + k[4] * dr6do;
                    double dicdist2_do = -icdist2 * icdist2
                                         * ( k[5] * dr2do + k[6] * dr4do + k[7] * dr6do );
                    double dxd0_do = cdist * icdist2 * dxdo[j]
                                     + x * icdist2 * dcdist_do + x * cdist * dicdist2_do
                                     + k[2] * da1do + k[3] * da2do + k[8] * dr2do
                                     + k[9] * dr4do;
                    double dyd0_do = cdist * icdist2 * dydo[j]
                                     + y * icdist2 * dcdist_do + y * cdist * dicdist2_do
                                     + k[2] * da3do + k[3] * da1do + k[10] * dr2do
                                     + k[11] * dr4do;
                    dXdYd = dMatTilt * Vec2d( dxd0_do, dyd0_do );
                    dpdo_p[i * 3 + j] = fx * dXdYd( 0 );
                    dpdo_p[dpdo_step + i * 3 + j] = fy * dXdYd( 1 );
                }
                dpdo_p += dpdo_step * 2;
            }
        }
    }

    if( _m != imagePoints )
        cvConvert( _m, imagePoints );

    if( _dpdr != dpdr )
        cvConvert( _dpdr, dpdr );

    if( _dpdt != dpdt )
        cvConvert( _dpdt, dpdt );

    if( _dpdf != dpdf )
        cvConvert( _dpdf, dpdf );

    if( _dpdc != dpdc )
        cvConvert( _dpdc, dpdc );

    if( _dpdk != dpdk )
        cvConvert( _dpdk, dpdk );

    if( _dpdo != dpdo )
        cvConvert( _dpdo, dpdo );
}

static void _cvProjectPoints2( const CvMat* objectPoints,
                        const CvMat* r_vec,
                        const CvMat* t_vec,
                        const CvMat* A,
                        const CvMat* distCoeffs,
                        CvMat* imagePoints, CvMat* dpdr,
                        CvMat* dpdt, CvMat* dpdf,
                        CvMat* dpdc, CvMat* dpdk,
                        double aspectRatio )
{
    _cvProjectPoints2Internal( objectPoints, r_vec, t_vec, A, distCoeffs, imagePoints, dpdr, dpdt,
                               dpdf, dpdc, dpdk, NULL, aspectRatio );
}
