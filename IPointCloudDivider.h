//
// Created by 郭嘉丞 on 15/9/12.
//

#ifndef KINECTTOPCL_IPOINTCLOUDDIVIDER_H
#define KINECTTOPCL_IPOINTCLOUDDIVIDER_H

#include <vector>
#include "common.h"

class IPointCloudDivider
{
public:
    virtual std::vector<PointCloudPtr> getDividedPointClouds() = 0;
};


#endif //KINECTTOPCL_IPOINTCLOUDDIVIDER_H
