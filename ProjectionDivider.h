//
// Created by 郭嘉丞 on 15/9/12.
//

#ifndef KINECTTOPCL_PROJECTIONDIVIDER_H
#define KINECTTOPCL_PROJECTIONDIVIDER_H

#include <opencv2/core/core.hpp>
#include "IPointCloudDivider.h"

class ProjectionDivider:public IPointCloudDivider
{
public:
    ProjectionDivider(PointCloudPtr pointCloud);

    virtual std::vector<PointCloudPtr> getDividedPointClouds() override;

    void saveDensity();
private:
    void calculateDensity();
    void filterDensity();
    void getDividePositions();

    PointCloudPtr cloud;
    cv::Mat xDensity;
    cv::Mat yDensity;
    cv::Mat zDensity;
    std::vector<int> xDividePositions;
    std::vector<int> yDividePositions;
    std::vector<int> zDividePositions;
};


#endif //KINECTTOPCL_PROJECTIONDIVIDER_H
