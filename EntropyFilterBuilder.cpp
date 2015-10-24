//
// Created by 郭嘉丞 on 15/10/10.
//

#include "EntropyFilterBuilder.h"
#include <iostream>
#include <cmath>

using namespace std;

void EntropyFilterBuilder::buildPointCloud()
{
    cv::Mat entropyImage = getEntropyImage();
    filterEntropy(0.3, entropyImage);
    LineFilterBuilder::buildPointCloud();
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
                imageMat.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 255, 255);
            }
        }
    }
    cv::imwrite("/Users/gjc13/KinectData/filtered.png", imageMat);
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
