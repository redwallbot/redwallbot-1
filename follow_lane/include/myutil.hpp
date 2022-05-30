#ifndef MYUTIL_HPP
#define MYUTIL_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>



// 黑白循迹图像处理
void processImage(cv::Mat& src_img, cv::Mat& dst_img) {

	cv::Mat blured_img, gray_img, mask;

	cv::GaussianBlur(src_img, blured_img, cv::Size(3, 3), 0);

	cv::Mat hsv_img;
	cv::cvtColor(blured_img, hsv_img, cv::COLOR_RGB2HSV);
	cv::Scalar lower_black(0, 0, 0);
	cv::Scalar uper_black(25, 75, 120);
	cv::inRange(hsv_img, lower_black, uper_black, mask);
	cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7,7), cv::Point(-1, -1));
	morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);

	cv::imshow("src", src_img);
	cv::imshow("gray", mask);

	cv::waitKey(30);

	int width = mask.cols;
	int height = mask.rows;

	int search_top = 3 * height / 4;
	int search_bottom = search_top + 20;

	for (int i = 0; i < height - 2; i++) {
		if (i < search_top || i > search_bottom) {
			for (int j = 0; j < width; j++) {
				mask.at<uchar>(i, j) = 0;

			}
		}
	}
	//cv::imshow("mask", mask);
	mask.copyTo(dst_img);
}

void followlane(cv::Mat src_img, double* speed) {
	cv::Mat dst_img;
	processImage(src_img, dst_img);
	cv::Moments moments = cv::moments(dst_img);
	
	if (moments.m00 > 0) {
		int cx = int(moments.m10 / moments.m00);
		int cy = int(moments.m01 / moments.m00);
		cv::circle(src_img, cv::Point(cx, cy), 20, CV_RGB(255, 0, 0), -1);
		
		int err = cx - dst_img.cols / 2;

		*speed = 0.3;
		*(speed+2) = -(double)err / 500;

	}
}

#endif