#include <cmath>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

using namespace cv;
using namespace cv::aruco;

void drawPlanarBoard(Board *_board, Size outSize, OutputArray _img, int marginSize, int borderBits);
