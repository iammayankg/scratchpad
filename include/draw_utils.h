// #ifndef DRAW_UTILS_H
// #define DRAW_UTILS_H
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/hal/interface.h>

cv::Scalar getColor(std::string sh)
{
    if (sh == "red")
    {
        return cv::Scalar(255, 0, 0);
    }

    if (sh == "green")
    {
        return cv::Scalar(0, 255, 0);
    }
    if (sh == "blue") {
        return cv::Scalar(0, 0, 255);
    }
    if (sh == "black") {
        return cv::Scalar(0, 0, 0);
    }
    return cv::Scalar(255, 255, 255);
}

void drawLine(cv::Mat &img, cv::Point2d &start, cv::Point2d &end, std::string color="red")
{
    int thickness = 2;
    int lineType = cv::LINE_8;
    cv::line(img,
             start,
             end,
             getColor(color),
             thickness,
             lineType);
}
void drawCircle(cv::Mat &img, cv::Point2d &center, std::string color="blue")
{
    cv::circle(img,
               center,
               10,
               getColor(color),
               cv::FILLED,
               cv::LINE_8);
}

// #endif