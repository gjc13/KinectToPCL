//
// Created by 郭嘉丞 on 15/9/25.
//

#include "ImageRebuild.h"
#include "KinectParameters.h"
#include <iostream>

using std::cerr;
using std::endl;
using std::cout;

struct PixelNumber
{
    int nx;
    int ny;
};

const int lowResWidth = 512;
const int lowResHeight = 424;
const int highResWidth = 1920;
const int highResHeight = 1080;

static PixelNumber getPixelNumber(const pcl::PointXYZRGB &point, double projectionMat[3][4]);

cv::Mat get2DImageFromPointCloud(PointCloudPtr cloud)
{
    cv::Mat rebuiltImage(lowResHeight, lowResWidth, CV_8UC3);
    for (int i = 0; i < rebuiltImage.rows; i++)
    {
        for (int j = 0; j < rebuiltImage.cols; j++)
        {
            rebuiltImage.at<cv::Vec3b>(i, j)[0] = 0;
            rebuiltImage.at<cv::Vec3b>(i, j)[1] = 0;
            rebuiltImage.at<cv::Vec3b>(i, j)[2] = 0;
        }
    }
    int minPixelX = INT32_MAX, maxPixelX = -1;
    int minPixelY = INT32_MAX, maxPixelY = -1;
    for (unsigned i=0; i<cloud->points.size(); i++)
    {
        const pcl::PointXYZRGB & point = cloud->points[i];
        PixelNumber pxNumber = getPixelNumber(point, projectionParameter);
        if(pxNumber.nx >= lowResWidth || pxNumber.nx < 0)
        {
//            cerr << pxNumber.nx << " " << pxNumber.ny << endl;
            continue;
        }
        if(pxNumber.ny >= lowResHeight || pxNumber.nx < 0)
        {
//            cerr << pxNumber.nx << " " << pxNumber.ny << endl;
            continue;
        }
        cv::Vec3b &pixel = rebuiltImage.at<cv::Vec3b>(pxNumber.ny, pxNumber.nx);
        pixel[0] = point.b;
        pixel[1] = point.g;
        pixel[2] = point.r;
        if (pxNumber.nx > maxPixelX) maxPixelX = pxNumber.nx;
        if (pxNumber.nx < minPixelX) minPixelX = pxNumber.nx;
        if (pxNumber.ny > maxPixelY) maxPixelY = pxNumber.ny;
        if (pxNumber.ny < minPixelY) minPixelY = pxNumber.ny;
    }
    if(maxPixelX < 0 || minPixelX < 0) return cv::Mat();
    return cv::Mat(rebuiltImage,
                   cv::Range(minPixelY, maxPixelY+1),
                   cv::Range(minPixelX, maxPixelX+1));
}

cv::Mat getHDImageFromPointCloud(PointCloudPtr cloud, cv::Mat &totalImage)
{
    int minPixelX = INT32_MAX, maxPixelX = -1;
    int minPixelY = INT32_MAX, maxPixelY = -1;
    for (unsigned i=0; i<cloud->points.size(); i++)
    {
        const pcl::PointXYZRGB & point = cloud->points[i];
        PixelNumber pxNumber = getPixelNumber(point, projectionParameter1080);
        if (pxNumber.nx > maxPixelX) maxPixelX = pxNumber.nx;
        if (pxNumber.nx < minPixelX) minPixelX = pxNumber.nx;
        if (pxNumber.ny > maxPixelY) maxPixelY = pxNumber.ny;
        if (pxNumber.ny < minPixelY) minPixelY = pxNumber.ny;
    }
    if(minPixelX < 0 || maxPixelX >= totalImage.cols || maxPixelX < 0 ||
            minPixelY < 0 || maxPixelY >= totalImage.rows || maxPixelY < 0)
    {
        return cv::Mat();
    }
    return cv::Mat(totalImage,
                   cv::Range(minPixelY, maxPixelY+1),
                   cv::Range(minPixelX, maxPixelX+1));
}

PixelNumber getPixelNumber(const pcl::PointXYZRGB &point, double projectionMat[3][4])
{
    double projectedPoint[3];   //2d point in the homogeneous coordinate
    for(int i=0; i<3; i++)
    {
        projectedPoint[i] = projectionMat[i][0] * point.x +
                            projectionMat[i][1] * point.y +
                            projectionMat[i][2] * point.z +
                            projectionMat[i][3];
    }
    PixelNumber pixelNumber;
    pixelNumber.nx = (int)(projectedPoint[0] / projectedPoint[2]);
    pixelNumber.ny = (int)(projectedPoint[1] / projectedPoint[2]);
    return pixelNumber;
}
