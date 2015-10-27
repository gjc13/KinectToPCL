//
// Created by 郭嘉丞 on 15/10/10.
//

#include "EntropyFilterBuilder.h"
#include <iostream>
#include <cmath>

using namespace std;

void EntropyFilterBuilder::buildPointCloud()
{
    LineFilterBuilder::removeLines();
    cv::Mat entropyImage = getEntropyImage();
    filterEntropy(0.3, entropyImage);
    openDepthImage();
#ifdef __DEBUG__
    saveFilter();
#endif
    PointCloudBuilder::buildPointCloud();
}

cv::Mat EntropyFilterBuilder::getEntropyImage()
{
    cv::Mat entropyImage(imageMat.rows, imageMat.cols, CV_64FC1);
    cv::Mat grayScaleImage;
    cv::cvtColor(imageMat, grayScaleImage, CV_BGR2GRAY);
    int greyLevels = filterSize * filterSize;
    double *grayScale = new double[greyLevels];
    int movStart = filterSize / 2;
    for (int i = 0; i < imageMat.rows; i++)
    {
        for (int j = 0; j < imageMat.cols; j++)
        {
            if (i - movStart < 0 || j - movStart < 0
                || i - movStart + filterSize >= imageMat.rows ||
                j - movStart + filterSize >= imageMat.cols)
            {
                entropyImage.at<double>(i, j) = 0;
            }
            else
            {
                int x = i - movStart;
                int y = j - movStart;
                for (int i = 0; i < greyLevels; i++)
                {
                    grayScale[i] = 0;
                }
                for (int movx = 0; movx < filterSize; movx++)
                    for (int movy = 0; movy < filterSize; movy++)
                    {
                        double gray = grayScaleImage.at<uchar>(movx + x, movy + y);
                        gray = gray * ((double) greyLevels) / 256;
                        grayScale[int(gray)]++;
                    }
                double entropy = 0;
                for (int i = 0; i < greyLevels; i++)
                {
                    entropy += grayScale[i] * entropyTable[(int) grayScale[i]] / (greyLevels);
                }
                entropyImage.at<double>(i, j) = entropy / entropyTable[1];
            }
        }
    }
    return entropyImage;
}

void EntropyFilterBuilder::filterEntropy(double threshold, cv::Mat &entropyImage)
{
    for (int i = 0; i < entropyImage.rows; i++)
    {
        for (int j = 0; j < entropyImage.cols; j++)
        {
            if (entropyImage.at<double>(i, j) < threshold)
            {
                depthMat.at<float>(i, j) = 0;
#ifdef __DEBUG__
                imageMat.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 255, 255);
#endif
            }
        }
    }
#ifdef __DEBUG__
    cv::imwrite("/Users/gjc13/KinectData/filtered.png", imageMat);
#endif
}

void EntropyFilterBuilder::buildEntropyTable()
{
    entropyTable[0] = 0;
    double greyLevels = filterSize * filterSize;
    for (int i = 1; i <= filterSize * filterSize; i++)
    {
        entropyTable[i] = log2((double) i / greyLevels);
    }
}

void EntropyFilterBuilder::openDepthImage()
{
    static const int kernelSize = 2;
    cv::Mat erodeKernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                                    cv::Size(2 * kernelSize + 1, 2 * kernelSize + 1),
                                                    cv::Point(kernelSize, kernelSize));
    cv::erode(depthMat, depthMat, erodeKernel);
    cv::dilate(depthMat, depthMat, erodeKernel);
}

void EntropyFilterBuilder::saveFilter()
{
    cv::Mat filterImage(imageMat.rows, imageMat.cols, CV_8UC1);
    for (int i = 0; i < imageMat.rows; i++)
        for (int j = 0; j < imageMat.cols; j++)
        {
            if (depthMat.at<float>(i, j) < 0.0001)
                filterImage.at<uchar>(i, j) = 255;
            else
            {
                filterImage.at<uchar>(i, j) = 0;
            }
        }
    cv::imwrite("/Users/gjc13/KinectData/filter.png", filterImage);
}
