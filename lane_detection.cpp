#include "detec_lanes.h"
#include <iostream>
#include <algorithm> 
#include <opencv2/opencv.hpp>
#include <cmath>
#include <tuple>
//#include <python.h>





int main(){

    cv::Mat frame1, frame2, frame3;
    cv::Point bottomMidPoint;
    std::vector<cv::Vec4i> lines;
    bool curve;
    double error;
    float m1;
    float m2;
    float b1;
    float b2;

    // Create a VideoCapture object and open the input file
    cv::VideoCapture cap("C:/Users/FERIT/Desktop/utku_website/IMG_5178.mov"); 

    // Check if camera opened successfully
    if(!cap.isOpened()){
        std::cout << "Error opening video stream or file" << std::endl;
        return -1;
    }

    // Create new matrices for the resized images.
    cv::Mat small_frame1, small_frame2, small_frame3;

    // Read and display each frame sequentially
    while(1){
        cv::Mat frame;
        // Capture frame-by-frame
        cap >> frame;

        // If the frame is empty, break immediately
        if (frame.empty())
            break;

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

        // cv::imshow("after canny", small_frame1);
        // cv::waitKey(0);

        lines =  houghLines(small_frame1, small_frame2,  false);

        
        std::sort(lines.begin(), lines.end(), [](const cv::Vec4i &a, const cv::Vec4i &b){ 
            return std::max(a[1], a[3]) > std::max(b[1], b[3]); 
        });

        if (lines.size() > 15){//15
            lines.erase(lines.begin() + 15, lines.end());//15
        }
        std::vector<cv::Vec4i> rightLane;
        std::vector<cv::Vec4i> leftLane;
        std::vector<cv::Point> lastRightPoints, lastLeftPoints;
        std::tie (rightLane, leftLane) = splitLanes(lines, small_frame1.cols);

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
        drawBestFitLine(small_frame2, lineParams1);

        cv::namedWindow("Image", cv::WINDOW_AUTOSIZE );
        cv::imshow("Image", small_frame2);
        cv::waitKey(0); // Wait for a keystroke in the window

        cv::Vec4f lineParams2 = getBestFitLine(rightPoints);
        std::tie(m1,b1) = getLineParams(lineParams1);
        std::tie(m2,b2) = getLineParams(lineParams2);

        bottomMidPoint = getBottomMidPoint(m1,b1,m2,b2, small_frame2.rows);

        error = getError(bottomMidPoint, small_frame2.cols);
        curve = isTurn(m1,m2);
        std::cout << "#######################################################" << "\n";
        std::cout << "The error is: " << error << " and the curve is: " << curve << "\n";
        std::cout << "#######################################################";
        drawBestFitLine(small_frame2, lineParams2);
        drawAndLabelPoint(small_frame2,bottomMidPoint);






        drawAndLabelPoints(small_frame2, leftPoints);
        drawAndLabelPoints(small_frame2, rightPoints);

        // Display the resulting frame
        cv::imshow("Frame", frame);

        // Press  'q' to quit the video
        if(cv::waitKey(25) == 'q')
            break;
    }
    cv::waitKey(0); // Wait for a keystroke in the window

    // When everything done, release the video capture and write object
    cap.release();

    // Closes all the frames
    cv::destroyAllWindows();

    return 0;
}