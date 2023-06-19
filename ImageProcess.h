#ifndef IMAGEPROCESS_H
#define IMAGEPROCESS_H
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <iomanip>
#include <iostream>
#include <vector>
#include <opencv2\imgproc\types_c.h>

class ProcessTool
{
public:
	std::vector<cv::Point2d> AverageLine(cv::Mat img, cv::Point2d leftup, cv::Point2f rightdown);
	std::vector<cv::Point2d> StegerLine(cv::Mat img0, int col = 3, int row = 3, int sqrtx = 1, int sqrty = 1, int shreshold = 80, float distance = 0.5);
private:
	void ShowLine(std::vector<cv::Point2d> Points, cv::Mat image);
};

#endif
