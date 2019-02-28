#include <cmath>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

void drawPlanarBoard(cv::aruco::Board *_board, cv::Size outSize, cv::OutputArray _img, int marginSize, int borderBits);
