#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <algorithm> 
#include <numeric>
#include <opencv2/imgproc.hpp>
// #include <eigen3/Eigen/Dense>

cv::Mat applyGrayscale(cv::Mat source){
    cv::Mat destination;
    cv::cvtColor(source, destination, cv::COLOR_RGB2GRAY);
    return destination;
}

cv::Mat applyGuassianBlur(cv::Mat source){
    cv::Mat dst;
    cv::threshold(source, dst, 140, 255, cv::THRESH_BINARY);
    return dst;
}



cv::Mat applyMorphologicalClosing(cv::Mat source) {
    // Clone the source image to avoid modifying the original
    cv::Mat destination = source.clone();

    // Define the structuring element
    // Note: You might need to adjust the size of the structuring element
    // based on your specific use case.
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

    // Apply the closing operation
    cv::morphologyEx(destination, destination, cv::MORPH_CLOSE, element);

    return destination;
}



cv::Mat filterColors(cv::Mat source, bool isDayTime)
{
    cv::Mat hsv, whiteMask, whiteImage, yellowMask, yellowImage, whiteYellow;

    // White mask
    std::vector< int > lowerWhite = {130, 130, 130};
    std::vector< int > upperWhite = {255, 255, 255};
    cv::inRange(source, lowerWhite, upperWhite, whiteMask);
    cv::bitwise_and(source, source, whiteImage, whiteMask);

    // Yellow mask
    cv::cvtColor(source, hsv, cv::COLOR_BGR2HSV);
    std::vector< int > lowerYellow = {20, 100, 110}; 
    std::vector< int > upperYellow = {30, 180, 240};
    cv::inRange(hsv, lowerYellow, upperYellow, yellowMask);
    cv::bitwise_and(source, source, yellowImage, yellowMask);

    // Blend yellow and white together
    cv::addWeighted(whiteImage, 1., yellowImage, 1., 0., whiteYellow);

    // Add gray filter if image is not taken during the day
    if (isDayTime == false)
    {   
        // Gray mask
        cv::Mat grayMask, grayImage, grayAndWhite, dst;
        std::vector< int > lowerGray = {80, 80, 80};
        std::vector< int > upperGray = {130, 130, 130};
        cv::inRange(source, lowerGray, upperGray, grayMask);
        cv::bitwise_and(source, source, grayImage, grayMask);

        // Blend gray, yellow and white together and return the result
        cv::addWeighted(grayImage, 1., whiteYellow, 1., 0., dst);
        return dst;
    }
    
    // Return white and yellow mask if image is taken during the day
    return whiteYellow;
}

cv::Mat convertColorSpaceToGray(cv::Mat source) {
    // Create a destination Mat object
    cv::Mat hsv, grayscale;

    // Convert the source image from the BGR (OpenCV's default) color space to HSV
    cv::cvtColor(source, hsv, cv::COLOR_BGR2HSV);

    // Convert HSV to grayscale by taking the value channel
    std::vector<cv::Mat> channels(3);
    cv::split(hsv, channels);
    grayscale = channels[2];

    return grayscale;
}

cv::Mat fillHoles(cv::Mat source) {
    // Clone the source image to avoid modifying the original
    cv::Mat destination = source.clone();

    // Invert the image
    cv::bitwise_not(destination, destination);

    // Define the structuring element
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

    // Apply the closing operation
    cv::morphologyEx(destination, destination, cv::MORPH_CLOSE, element);

    // Invert the image back
    cv::bitwise_not(destination, destination);

    return destination;
}

cv::Mat convertColorSpace(cv::Mat source) {
    // Create a destination Mat object
    cv::Mat destination;

    // Convert the source image from the BGR (OpenCV's default) color space to HSV
    cv::cvtColor(source, destination, cv::COLOR_BGR2HSV);

    return destination;
}

cv::Mat applyThresholding(cv::Mat source){
    cv::Mat dst;
    cv::GaussianBlur(source, dst, cv::Size(4,4), 0);
    return dst;
}

cv::Mat applyCanny(cv::Mat source, int edges = 100, int threshold = 200){
    cv::Mat dst;
    cv::Canny(source, dst, edges, threshold);
    return dst;
}

std::vector<cv::Point> getAveragePoints(std::vector<cv::Point>& points){
    std::map<int, std::vector<int>> yCoordinates;

    // Group all y coordinates by x
    for(const auto& point: points){
        yCoordinates[point.x].push_back(point.y);
    }

    // Calculate average y for each x
    std::vector<cv::Point> averagePoints;
    for(const auto& [x, yCoords]: yCoordinates){
        int totalY = std::accumulate(yCoords.begin(), yCoords.end(), 0);
        int averageY = totalY / yCoords.size();
        averagePoints.emplace_back(x, averageY);
    }
    
    return averagePoints;
}

std::vector<cv::Point> getAveragePoints(std::vector<cv::Point>& points, int threshold){
    std::map<int, std::vector<int>> xCoordinates;
    
    // Group all x coordinates by y
    for(const auto& point : points) {
        xCoordinates[point.y].push_back(point.x);
    }

    std::vector<cv::Point> averagePoints;

    // For each group of points with the same y-coordinate
    for(auto& [y, xCoords] : xCoordinates){
        // Sort the x coordinates to find the range
        std::sort(xCoords.begin(), xCoords.end());

        // Check if the range of x is within the threshold
        if(xCoords.back() - xCoords.front() <= threshold) {
            // Calculate the average x for the group
            int totalX = std::accumulate(xCoords.begin(), xCoords.end(), 0);
            int averageX = totalX / xCoords.size();

            // Add the point with the average x and this y to the result
            averagePoints.emplace_back(averageX, y);
        }
    }

    return averagePoints;
}


std::pair<float, float> getLineParams(cv::Vec4f lineParams){
    float vx = lineParams[0];
    float vy = lineParams[1];
    float x = lineParams[2];
    float y = lineParams[3];

    float m = vy / vx;
    float b = y - m*x;

    return {m,b};
}

cv::Point getBottomIntercept(float m, float b, int imgHeight){
    int y = imgHeight;
    int x = static_cast<int>((y - b) / m); // x-coordinate of bottom intercept

    return cv::Point(x, y); // returns the bottom intercept
}

cv::Point getBottomMidPoint(float m1, float b1, float m2, float b2, int imgHeight){
    cv::Point leftIntercept = getBottomIntercept(m1, b1, imgHeight);
    cv::Point rightIntercept = getBottomIntercept(m2, b2, imgHeight);

    int midX = (leftIntercept.x + rightIntercept.x) / 2;
    int midY = imgHeight;

    return cv::Point(midX, midY); // returns the middle point of the lane at the bottom of the image
}

double getError(const cv::Point &midpoint, int img_width){
    double lane_center = midpoint.x;
    double error = lane_center - ((int)img_width/2);
    return error;
}

cv::Vec4f getBestFitLine(std::vector<cv::Point>& points){
    cv::Vec4f lineParams;
    cv::fitLine(points, lineParams, cv::DIST_L2, 0, 0.01, 0.01);

    return lineParams;
}

void drawBestFitLine(cv::Mat& img, cv::Vec4f lineParams){
    // lineParams contains (vx, vy, x0, y0). (vx, vy) is a normalized vector collinear to the line and (x0, y0) is a point on the line.
    float vx = lineParams[0];
    float vy = lineParams[1];
    float x0 = lineParams[2];
    float y0 = lineParams[3];

    // Now let's find two points on this line. You can change the factor to get further points.
    cv::Point pt1(x0 - 1000*vx, y0 - 1000*vy);
    cv::Point pt2(x0 + 1000*vx, y0 + 1000*vy);

    // Draw the line
    cv::line(img, pt1, pt2, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
}

cv::Mat maskUpperHalfImage(cv::Mat source) {
    // Clone the source image to avoid modifying the original
    cv::Mat destination = source.clone();

    // Create a region of interest (ROI) that covers the upper third of the image
    cv::Rect roi(0, 0, destination.cols, destination.rows / 3);

    // Set the pixel values in the ROI to black (0)
    destination(roi) = cv::Scalar(0, 0, 0);

    return destination;
}

void drawAndLabelPoints(cv::Mat &image, const std::vector<cv::Point> &points)
{
    for(int i = 0; i < points.size(); ++i)
    {
        // Draw a circle at each point
        cv::circle(image, points[i], 5, cv::Scalar(0, 0, 255), cv::FILLED);

        // Label the point with its coordinates
        std::string label = "(" + std::to_string(points[i].x) + ", " + std::to_string(points[i].y) + ")";
        cv::putText(image, label, points[i], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2);
    }
    cv::imshow("Labelled", image);
    cv::waitKey(0);
}

void drawAndLabelPoint(cv::Mat &image, const cv::Point &points){
        // Draw a circle at each point
    cv::circle(image, points, 5, cv::Scalar(0, 0, 255), cv::FILLED);

        // Label the point with its coordinates
    std::string label = "(" + std::to_string(points.x) + ", " + std::to_string(points.y) + ")";
    cv::putText(image, label, points, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2);
    cv::namedWindow("Singular point", cv::WINDOW_AUTOSIZE);
    cv::imshow("Singular point", image);
    cv::waitKey(25);
}


std::pair<std::vector<cv::Vec4i>, std::vector<cv::Vec4i>> splitLanes(const std::vector<cv::Vec4i>& lines, const int img_width) {
    std::vector <cv::Vec4i> right;
    std::vector <cv::Vec4i> left;

    for(const auto& line : lines){
        int x1 = line[0];
        int y1 = line[1]; 
        int x2 = line[2]; 
        int y2 = line[3];

        if (std::min(x1, x2) > img_width / 2) {
            right.push_back(line); // Right lane lines are in the right half of the image
        }
        else if (std::max(x1, x2) < img_width / 2) {
            left.push_back(line); // Left lane lines are in the left half of the image
        }
    }
    return {right,left};
}

std::vector<cv::Point> convertLinesToPoints(const std::vector<cv::Vec4i>& lines)
{
    std::vector<cv::Point> points;

    for (const auto& line : lines)
    {
        cv::Point pt1(line[0], line[1]);
        cv::Point pt2(line[2], line[3]);

        points.push_back(pt1);
        points.push_back(pt2);
    }

    return points;
}

bool isTurn(float slope1, float slope2){
    if (slope1 > 0 && slope2 > 0){
        return true;
    }
    else if (slope1 < 0 && slope2 < 0){
        return true;
    }
    else{
        return false;
    }
}

std::vector<cv::Vec4i> houghLines(cv::Mat canny, cv::Mat source, bool drawHough)
{
    const double pi = 3.14159265358979323846;
    double rho = 2; // Distance resolution in pixels of the Hough grid
    double theta = 1 * pi / 180; // Angular resolution in radians of the Hough grid
    int thresh = 65; // Minimum number of votes (intersections in Hough grid cell)
    double minLineLength = 20; // Minimum number of pixels making up a line
    double maxGapLength = 20; // Maximum gap in pixels between connectable line segments
   
    std::vector<cv::Vec4i> linesP; // Will hold the results of the detection
    cv::HoughLinesP(canny, linesP, rho, theta, thresh, minLineLength, maxGapLength);

    // If requested, draw lines and label points
    if (drawHough == true)
    {
        std::vector<cv::Point> endpoints;

        for( size_t i = 0; i < linesP.size(); i++ )
        {
            cv::Vec4i l = linesP[i];
            line( source, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 3, cv::LINE_AA);

            // Add the endpoints of the line to the endpoints vector
            endpoints.push_back(cv::Point(l[0], l[1]));
            endpoints.push_back(cv::Point(l[2], l[3]));
        } 

        // Draw and label points
        drawAndLabelPoints(source, endpoints);

        // cv::imshow("Hough Lines", source);
        // cv::waitKey(0);
    }

    return linesP;
}




cv::Mat maskTriangleROI(cv::Mat src) {
    cv::Mat mask = cv::Mat::zeros(src.size(), src.type());

    cv::Point pts[3] = {
        cv::Point(src.cols/2, src.rows/2),
        cv::Point(src.cols, src.rows),
        cv::Point(0, src.rows)
    };

    cv::fillConvexPoly(mask, pts, 3, cv::Scalar(255, 255, 255));

    cv::Mat maskedImage;
    src.copyTo(maskedImage, mask);

    return maskedImage;
}

