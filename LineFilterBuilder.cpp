//
// Created by yht on 04/10/15.
//

#include "LineFilterBuilder.h"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace std;
using namespace cv;

LineFilterBuilder::LineFilterBuilder(const cv::Mat &depthMatrix, const cv::Mat &imageMatrix)
        : PointCloudBuilder(depthMatrix, imageMatrix)
{
}

void LineFilterBuilder::buildPointCloud()
{
    removeLines();
    PointCloudBuilder::buildPointCloud();
}

void LineFilterBuilder::removeLines()
{
    Mat dst, cdst;
    Canny(imageMat, dst, 50, 200, 3);
    cvtColor(depthMat, cdst, COLOR_GRAY2BGR);

    vector<Vec4i> lines;
    HoughLinesP(dst, lines, 1, CV_PI / 180, 50, 50, 10);
    for (size_t i = 0; i < lines.size(); i++)
    {
        Vec4i l = lines[i];
        line(cdst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 0), 5, CV_AA);
#ifdef __DEBUG__
        line(imageMat, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255, 255, 255), 5, CV_AA);
#endif
    }
    cvtColor(cdst, depthMat, COLOR_RGB2GRAY);
}
