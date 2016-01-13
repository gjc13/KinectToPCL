//
// Created by 郭嘉丞 on 15/9/21.
//

#ifndef POINTCLOUDUTILITIES_H
#define POINTCLOUDUTILITIES_H

#include "common.h"
#include <opencv2/opencv.hpp>

cv::Mat reload_32f_image(const char * filename);

PointCloudPtr getSubXCloud(PointCloudPtr cloud, double fromX, double toX);

PointCloudPtr getSubYCloud(PointCloudPtr cloud, double fromY, double toY);

PointCloudPtr getSubZCloud(PointCloudPtr cloud, double fromZ, double toZ);

PointCloudPtr getSubCloud(PointCloudPtr cloud, double fromX, double toX,
                          double fromY, double toY, double fromZ, double toZ);

void rgbVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);

#endif //OBJECTFINDER_POINTCLOUDUTILITIES_H
