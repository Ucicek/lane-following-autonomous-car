#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <opencv2/opencv.hpp>
#include "detect_lanes.h"

namespace py = pybind11;

std::pair<double, bool> process_frame(py::array_t<unsigned char> input_array) {
    py::buffer_info bufInfo = input_array.request();
    cv::Mat frame(bufInfo.shape[0], bufInfo.shape[1], CV_8UC3, (unsigned char*)bufInfo.ptr);

    cv::Mat small_frame1, small_frame2;
    cv::Point bottomMidPoint;
    std::vector<cv::Vec4i> lines;
    bool curve;
    double error;
    float m1;
    float m2;
    float b1;
    float b2;

    cv::Mat imgCopy = frame.clone();

    // Calculate the new size, which is 1/3 of the original size.
    cv::Size new_size(imgCopy.cols / 3, imgCopy.rows / 5);

    // Resize the images.
    cv::resize(imgCopy, small_frame1, new_size);
    cv::resize(imgCopy, small_frame2, new_size);

    small_frame1 = filterColors(small_frame1,true);
    small_frame1 = applyGrayscale(small_frame1);
    small_frame1 = applyGuassianBlur(small_frame1);
    small_frame1 = applyCanny(small_frame1);

    lines =  houghLines(small_frame1, small_frame2,  false);

    std::sort(lines.begin(), lines.end(), [](const cv::Vec4i &a, const cv::Vec4i &b){ 
        return std::max(a[1], a[3]) > std::max(b[1], b[3]); 
    });

    if (lines.size() > 10){
        lines.erase(lines.begin() + 10, lines.end());
    }

    std::vector<cv::Vec4i> rightLane;
    std::vector<cv::Vec4i> leftLane;
    std::vector<cv::Point> lastRightPoints, lastLeftPoints;
    std::tie(rightLane, leftLane) = splitLanes(lines, small_frame1.cols);

        // Sorting and trimming the right lane lines
    std::sort(rightLane.begin(), rightLane.end(), [](const cv::Vec4i &a, const cv::Vec4i &b){ 
        return std::max(a[1], a[3]) > std::max(b[1], b[3]); 
    });
    rightLane.resize(2); // Keep only the first line


        // Sorting and trimming the left lane lines
    std::sort(leftLane.begin(), leftLane.end(), [](const cv::Vec4i &a, const cv::Vec4i &b){ 
        return std::max(a[1], a[3]) > std::max(b[1], b[3]); 
    });
    leftLane.resize(2); // Keep only the first line

    std::vector<cv::Point> rightPoints = convertLinesToPoints(rightLane);
    std::vector<cv::Point> leftPoints = convertLinesToPoints(leftLane);

    if (rightPoints.empty() && !lastRightPoints.empty()) {
        rightPoints = lastRightPoints;
    }
    if (leftPoints.empty() && !lastLeftPoints.empty()) {
        leftPoints = lastLeftPoints;
    }

    // If the current set of points is not empty, update the last non-empty set.
    if (!rightPoints.empty()) {
        lastRightPoints = rightPoints;
    }
    if (!leftPoints.empty()) {
        lastLeftPoints = leftPoints;
    }

    cv::Vec4f lineParams1 = getBestFitLine(leftPoints);

    cv::Vec4f lineParams2 = getBestFitLine(rightPoints);
    std::tie(m1,b1) = getLineParams(lineParams1);
    std::tie(m2,b2) = getLineParams(lineParams2);

    bottomMidPoint = getBottomMidPoint(m1,b1,m2,b2, small_frame2.rows/2);

    error = getError(bottomMidPoint, small_frame2.cols);
    curve = isTurn(m1,m2);

    return std::make_pair(error, curve);
}

PYBIND11_MODULE(lane_detection, m) {
    m.def("process_frame", &process_frame, "A function that processes a frame and returns lane error and curve");
}
