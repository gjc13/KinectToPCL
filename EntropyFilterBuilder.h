//
// Created by 郭嘉丞 on 15/10/10.
//

#ifndef KINECTTOPCL_ENTROPYFILTERBUILDER_H
#define KINECTTOPCL_ENTROPYFILTERBUILDER_H

#include "LineFilterBuilder.h"

class EntropyFilterBuilder : public LineFilterBuilder
{
public:

    EntropyFilterBuilder(const cv::Mat &depthMatrix, const cv::Mat &imageMatrix, int filtersize = 5, double threshold = 0.4)
            : LineFilterBuilder(depthMatrix, imageMatrix), entropyTable(new double[filtersize * filtersize + 1]),
              filterSize(filtersize), threshold_(threshold)
    {
        buildEntropyTable();
    }


    ~EntropyFilterBuilder()
    {
        delete[] entropyTable;
    }

protected:
    void buildPointCloud();

private:
    void saveFilter();

    cv::Mat getEntropyImage();

    void filterEntropy(double threshold, cv::Mat &entropyImage);

    void buildEntropyTable();

    void openDepthImage();

    double *entropyTable;
    int filterSize;
    double threshold_;
};

#endif
//KINECTTOPCL_ENTROPYFILTERBUILDER_H
