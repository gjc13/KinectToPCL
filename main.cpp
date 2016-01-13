#include <iostream>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include "PointCloudBuilder.h"
#include "PointCloudUtilities.h"
#include "ClusterDivider.h"
#include "KinectParameters.h"
#include "LineFilterBuilder.h"
#include "PointCloudBuilder.h"
#include "EntropyFilterBuilder.h"
#include "ImageRebuild.h"

using namespace std;

void print_depth(cv::Mat depthMat);

void printPojectedXY(double x, double y, double z, double projectionMatrix[3][4]);

void print_usage();

int main(int argc, const char * argv[])
{
    if(argc != 5)
    {
        print_usage();
        return -1;
    }
    cv::Mat depthMat = reload_32f_image(argv[1]);
    cv::Mat imageMat = cv::imread(argv[2]);
    cv::Mat highResImage = cv::imread(argv[3]);
    cv::Mat grayScaleImage;
    cv::cvtColor(imageMat, grayScaleImage, CV_BGR2GRAY);
    printf("type: %d\n", grayScaleImage.type());
	PointCloudBuilder * builder = new EntropyFilterBuilder(depthMat, imageMat, 5, 0.4);
    PointCloudPtr pointCloud = builder->getPointCloud();
    pcl::io::savePCDFile(argv[4], *pointCloud, true);
    IPointCloudDivider * divider = new ClusterDivider(pointCloud);
    vector<PointCloudPtr> dividedPointClouds = divider->getDividedPointClouds();
    cout << "Objects found:"  << dividedPointClouds.size() << endl;
    char buffer[20];
    for (int i = 0; i < (int) dividedPointClouds.size(); i++)
    {
        string pcdPrefix = "dividedCloud";
        string lowResPrefix = "lowRes";
        string highResPrefix = "hiRes";
        sprintf(buffer, "%d", i);
        pcl::io::savePCDFile(pcdPrefix + buffer + ".pcd", *dividedPointClouds[i], true);
        cv::imwrite(lowResPrefix + buffer + ".png", get2DImageFromPointCloud(dividedPointClouds[i]));
        cv::imwrite(highResPrefix + buffer + ".png",
                    getHDImageFromPointCloud(dividedPointClouds[i], highResImage));
    }
    delete divider;
	delete builder;
    return 0;
}

void print_depth(cv::Mat depthMat)
{
    int x1 = 205, x2 = 377, y1 = 125, y2 = 229;
    cv::Mat mean;
    cv::Mat stddev;
    cv::Mat boardDepth(depthMat, cv::Range(y1, y2), cv::Range(x1, x2));
    cv::meanStdDev(boardDepth, mean, stddev);
    cout << mean.at<double>(0) << endl;
    cout << stddev.at<double>(0) << endl;
    cv::imshow("depth", boardDepth);
}

void printPojectedXY(double x, double y, double z, double projectionMatrix[3][4])
{
    double position[4] = {x, y, z, 1};
    double projectedPositions[3] = {0, 0, 0};
    cout << x  << " " << y << " " << z << endl;
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            projectedPositions[i] += projectionMatrix[i][j] * position[j];
        }
    }
    cout << projectedPositions[0] / projectedPositions[2] << " "
    << projectedPositions[1] / projectedPositions[2] << endl;

}

void print_usage()
{
    cout << "KinectToPCL depthBinFile registeredImage rgbImage saveFile" << endl;
}
