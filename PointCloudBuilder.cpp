//
// Created by 郭嘉丞 on 15/9/12.
//

#include "PointCloudBuilder.h"
#include "KinectParameters.h"
#include <iostream>

using std::cerr;
using std::cout;
using std::endl;

PointCloudBuilder::PointCloudBuilder(const cv::Mat &depthMatrix, const cv::Mat &imageMatrix)
        : depthMat(depthMatrix), imageMat(imageMatrix),
          pointCloud(new pcl::PointCloud<pcl::PointXYZRGB>())
{
    buildPointCloud();
}

void PointCloudBuilder::buildPointCloud()
{
    auto &p = projectionParameter;
    depthMat = depthMat * depthToZ[0] + depthToZ[1];
    for (int y = 0; y < imageMat.rows; y++)
    {
        for (int x = 0; x < imageMat.cols; x++)
        {
            double Z = depthMat.at<float>(y, x);
            if (Z < MIN_DEPTH_CM)
            {
                continue;
            }

            double matrixAval[2][2] = {
                    {p[0][0] - p[2][0] * x, p[0][1] - p[2][1] * x},
                    {p[1][0] - p[2][0] * y, p[1][1] - p[2][1] * y}
            };
            cv::Mat matrixA(2, 2, CV_64FC1, &matrixAval);
            double matrixbval[2] = {
                    p[2][2] * Z * x + p[2][3] * x - p[0][2] * Z - p[0][3],
                    p[2][2] * Z * y + p[2][3] * y - p[1][2] * Z - p[1][3]
            };
            cv::Mat matrixb(2, 1, CV_64FC1, &matrixbval);
            cv::Mat solvedXY;
            if (!cv::solve(matrixA, matrixb, solvedXY))
            {
                cerr << "singular matrix A at x,y" << x << " " << y << endl;
                continue;
            }
            double X = solvedXY.at<double>(0);
            double Y = solvedXY.at<double>(1);
            pcl::PointXYZRGB newPoint;
            newPoint.b = imageMat.at<cv::Vec3b>(y, x)[0];
            newPoint.g = imageMat.at<cv::Vec3b>(y, x)[1];
            newPoint.r = imageMat.at<cv::Vec3b>(y, x)[2];
            newPoint.x = (float) X;
            newPoint.y = (float) Y;
            newPoint.z = (float) Z;
            pointCloud->points.push_back(newPoint);
        }
    }
    pointCloud->width = (int) pointCloud->points.size();
    pointCloud->height = 1;
    cout << "Build success" << endl;
}
