// detect_lanes.h

#ifndef DETEC_LANES_H
#define DETEC_LANES_H

#include <opencv2/opencv.hpp>
#include <vector>

cv::Mat applyGrayscale(cv::Mat source);
cv::Mat applyGuassianBlur(cv::Mat source);
cv::Mat applyThresholding(cv::Mat source);
cv::Mat getROI(cv::Mat source);
cv::Mat filterColors(cv::Mat source, bool isDayTime);
cv::Mat applyMorphologicalClosing(cv::Mat source);
cv::Vec4f getBestFitLine(std::vector<cv::Point>& points);
void drawBestFitLine(cv::Mat& img, cv::Vec4f lineParams);
cv::Point getBottomIntercept(float m, float b, int imgHeight);
cv::Point getBottomMidPoint(float m1, float b1, float m2, float b2, int imgHeight);
std::pair<float, float> getLineParams(cv::Vec4f lineParams);
std::vector<cv::Point> getAveragePoints(std::vector<cv::Point>& points, int threshold);
double getError(const cv::Point &midpoint, int img_width);
bool isTurn(float slope1, float slope2);
cv::Mat fillHoles(cv::Mat source);
std::vector<cv::Point> convertLinesToPoints(const std::vector<cv::Vec4i>& lines);
cv::Mat convertColorSpaceToGray(cv::Mat source);
cv::Mat convertColorSpace(cv::Mat source);
cv::Mat maskTriangleROI(cv::Mat src);
cv::Mat applyCanny(cv::Mat source, int edges =50, int threshold = 150);
std::pair<std::vector<cv::Vec4i>, std::vector<cv::Vec4i>> splitLanes(const std::vector<cv::Vec4i>& const lines, int img_width);
void drawAndLabelPoint(cv::Mat &image, const cv::Point &points);
void drawAndLabelPoints(cv::Mat &image, const std::vector<cv::Point> &points);
cv::Mat maskUpperHalfImage(cv::Mat source);
std::vector<cv::Vec4i> houghLines(cv::Mat canny, cv::Mat source, bool drawHough);

#endif //DETECT_LANES_Happly