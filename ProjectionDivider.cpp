//
// Created by 郭嘉丞 on 15/9/12.
//

#include "ProjectionDivider.h"
#include <iostream>
#include <fstream>

using namespace std;


ProjectionDivider::ProjectionDivider(PointCloudPtr pointCloud)
        : cloud(pointCloud),
          xDensity(cv::Mat(1, 1000, CV_32SC1)),
          yDensity(cv::Mat(1, 1000, CV_32SC1)),
          zDensity(cv::Mat(1, 1000, CV_32SC1))
{
    for (int i = 0; i < 1000; i++)
    {
        xDensity.at<int>(i) = 0;
        yDensity.at<int>(i) = 0;
        zDensity.at<int>(i) = 0;
    }
    cout << xDensity.depth() << endl;
    calculateDensity();
}

std::vector<PointCloudPtr> ProjectionDivider::getDividedPointClouds()
{
}

void ProjectionDivider::saveDensity()
{
    int xDensityVal[1000];
    int yDensityVal[1000];
    int zDensityVal[1000];
    for (int i = 0; i < 1000; i++)
    {
        xDensityVal[i] = xDensity.at<int>(i);
        yDensityVal[i] = yDensity.at<int>(i);
        zDensityVal[i] = zDensity.at<int>(i);
    }
    cout << zDensityVal[30] << endl;
    ofstream fout("/Users/gjc13/ClionProjects/KinectToPCL/xdensity.bin");
    fout.write((char *) &xDensityVal, 1000 * sizeof(int));
    fout.close();
    fout.open("/Users/gjc13/ClionProjects/KinectToPCL/ydensity.bin");
    fout.write((char *) &yDensityVal, 1000 * sizeof(int));
    fout.close();
    fout.open("/Users/gjc13/ClionProjects/KinectToPCL/zdensity.bin");
    fout.write((char *) &zDensityVal, 1000 * sizeof(int));
    fout.close();
}

void ProjectionDivider::calculateDensity()
{
    double xmin = INT32_MAX;
    double ymin = INT32_MAX;
    double zmin = INT32_MAX;
    for (pcl::PointXYZRGB point:cloud->points)
    {
        if (point.x < xmin) xmin = point.x;
        if (point.y < ymin) ymin = point.y;
        if (point.z < zmin) zmin = point.z;
    }
    cout << cloud->points.size() << endl;
    cout << xmin << " " << ymin << " " << zmin << endl;
    for (pcl::PointXYZRGB point:cloud->points)
    {
        int x = (int) (point.x - xmin);
        int y = (int) (point.y - ymin);
        int z = (int) (point.z - zmin);
        if (x > 1000 || y > 1000 || z > 1000) continue;
        xDensity.at<int>(x)++;
        yDensity.at<int>(y)++;
        zDensity.at<int>(z)++;
    }
}

void ProjectionDivider::filterDensity()
{

}

void ProjectionDivider::getDividePositions()
{

}
