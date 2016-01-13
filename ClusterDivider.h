#ifndef __CLUSTER_DIVIDER_H__
#define __CLUSTER_DIVIDER_H__

#include "IPointCloudDivider.h"

class ClusterDivider:public IPointCloudDivider
{
public:
    ClusterDivider(PointCloudPtr point_cloud);
    virtual std::vector<PointCloudPtr> getDividedPointClouds();
private:
    PointCloudPtr point_cloud_;
};

#endif

