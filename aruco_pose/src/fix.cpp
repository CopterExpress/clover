using namespace cv;
using namespace cv::aruco;

// Temporal fix!
// TODO: remove
// fix strange bug in our OpenCV version

void _getBoardObjectAndImagePoints(const Ptr<aruco::Board> &board, InputArrayOfArrays detectedCorners,
    InputArray detectedIds, OutputArray objPoints, OutputArray imgPoints) {

    CV_Assert(board->ids.size() == board->objPoints.size());
    CV_Assert(detectedIds.total() == detectedCorners.total());

    size_t nDetectedMarkers = detectedIds.total();

    std::vector< Point3f > objPnts;
    objPnts.reserve(nDetectedMarkers);

    std::vector< Point2f > imgPnts;
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
}

int _estimatePoseBoard(InputArrayOfArrays _corners, InputArray _ids, const Ptr<aruco::Board> &board,
                      InputArray _cameraMatrix, InputArray _distCoeffs, OutputArray _rvec,
                      OutputArray _tvec, bool useExtrinsicGuess, Mat &objPoints) {

    CV_Assert(_corners.total() == _ids.total());

    // get object and image points for the solvePnP function
    Mat /*objPoints, */imgPoints;
    _getBoardObjectAndImagePoints(board, _corners, _ids, objPoints, imgPoints);

    CV_Assert(imgPoints.total() == objPoints.total());

    if(objPoints.total() == 0) // 0 of the detected markers in board
        return 0;

//    std::cout << "objPoints: " << objPoints << std::endl;
//    std::cout << "imgPoints: " << imgPoints << std::endl;

    solvePnP(objPoints, imgPoints, _cameraMatrix, _distCoeffs, _rvec, _tvec, useExtrinsicGuess);

    // divide by four since all the four corners are concatenated in the array for each marker
    return (int)objPoints.total() / 4;
}

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
        dst_sz.width = dst_sz.height = std::min(dst_sz.width, dst_sz.height); //marker should be square
        dictionary.drawMarker(_board->ids[m], dst_sz.width, marker, borderBits);

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
    }
}
