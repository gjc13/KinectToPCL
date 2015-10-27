//
// Created by yht on 04/10/15.
//

#ifndef KINECTTOPCL_LINEFILTER_H
#define KINECTTOPCL_LINEFILTER_H


#include "PointCloudBuilder.h"

class LineFilterBuilder :public PointCloudBuilder{
public:
    LineFilterBuilder(const cv::Mat & depthMatrix, const cv::Mat & imageMatrix);

protected:

    void removeLines();

    void buildPointCloud();
};


#endif //KINECTTOPCL_LINEFILTER_H
