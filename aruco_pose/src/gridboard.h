void createCustomGridBoard(cv::Ptr<cv::aruco::Board>& board, int markersX, int markersY, float markerLength,
                           float markerSeparationX, float markerSeparationY, std::vector<int> ids)
{
	size_t totalMarkers = (size_t) markersX * markersY;
	board->ids = ids;
	board->objPoints.reserve(totalMarkers);

	// calculate Board objPoints
	float maxY = (float)markersY * markerLength + (markersY - 1) * markerSeparationY;
	for(int y = 0; y < markersY; y++) {
		for(int x = 0; x < markersX; x++) {
			std::vector< cv::Point3f > corners;
			corners.resize(4);
			corners[0] = cv::Point3f(x * (markerLength + markerSeparationX),
								 maxY - y * (markerLength + markerSeparationY), 0);
			corners[1] = corners[0] + cv::Point3f(markerLength, 0, 0);
			corners[2] = corners[0] + cv::Point3f(markerLength, -markerLength, 0);
			corners[3] = corners[0] + cv::Point3f(0, -markerLength, 0);
			board->objPoints.push_back(corners);
		}
	}
}
