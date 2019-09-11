#include <cmath>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

void _drawPlanarBoard(cv::aruco::Board *_board, cv::Size outSize, cv::OutputArray _img,
                      int marginSize, int borderBits, bool drawAxis); // editorconfig-checker-disable-line
void _drawAxis(cv::InputOutputArray image, cv::InputArray cameraMatrix, cv::InputArray distCoeffs,
               cv::InputArray rvec, cv::InputArray tvec, float length); // editorconfig-checker-disable-line
