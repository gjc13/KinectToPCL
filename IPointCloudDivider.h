//
// Created by 郭嘉丞 on 15/9/21.
//

#ifndef IPOINTCLOUDDIVIDER_H
#define IPOINTCLOUDDIVIDER_H

#include <vector>
#include "common.h"


class IPointCloudDivider
{
public:
    virtual std::vector<PointCloudPtr> getDividedPointClouds() = 0;
    virtual ~IPointCloudDivider() { }
};


#endif //OBJECTFINDER_IPOINTCLOUDDIVIDER_H
