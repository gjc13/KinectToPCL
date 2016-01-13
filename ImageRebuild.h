//
// Created by 郭嘉丞 on 15/9/25.
//

#ifndef OBJECTFINDER_IMAGEREBUILD_H
#define OBJECTFINDER_IMAGEREBUILD_H

#include <opencv2/opencv.hpp>
#include "common.h"

extern const int lowResWidth;
extern const int lowResHeight;
extern const int highResWidth;
extern const int highResHeight;

cv::Mat get2DImageFromPointCloud(PointCloudPtr cloud);

cv::Mat getHDImageFromPointCloud(PointCloudPtr cloud, cv::Mat & totalImage);

#endif //OBJECTFINDER_IMAGEREBUILD_H
