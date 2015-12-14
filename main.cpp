#include <iostream>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "PointCloudBuilder.h"
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include "KinectParameters.h"
#include "LineFilterBuilder.h"
#include "PointCloudBuilder.h"
#include "EntropyFilterBuilder.h"

using namespace std;

cv::Mat reload_32f_image(string filename);

void print_depth(cv::Mat depthMat);

boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);

void printPojectedXY(double x, double y, double z, double projectionMatrix[3][4]);

void print_usage();

int main(int argc, const char * argv[])
{
    if(argc != 4)
    {
        print_usage();
        return -1;
    }
    cv::Mat depthMat = reload_32f_image(argv[1]);
    cv::Mat imageMat = cv::imread(argv[2]);
    cv::Mat grayScaleImage;
    cv::cvtColor(imageMat, grayScaleImage, CV_BGR2GRAY);
    printf("type: %d\n", grayScaleImage.type());
	PointCloudBuilder * builder = new PointCloudBuilder(depthMat, imageMat);
    PointCloudPtr pointCloud = builder->getPointCloud();
    pcl::io::savePCDFile(argv[3], *pointCloud, true);
	delete builder;

//    PointCloudPtr filteredCloud = removeBigPlanes(pointCloud, 0.2, 3, 5);
//    pcl::VoxelGrid<pcl::PointXYZRGB> grid;
//
//
//    ProjectionDivider divider(pointCloud);
//    double xMin = divider.getXMin();
//    double yMin = divider.getYMin();
//    double zMin = divider.getZMin();
//
//    PointCloudPtr subCloud = getSubCloud(pointCloud, xMin +55, xMin + 180,
//        yMin + 30, yMin + 42, zMin, zMin + 300);
//    ofstream fxOut("/Users/gjc13/KinectData/planeX.data");
//    ofstream fyOut("/Users/gjc13/KinectData/planeY.data");
//    ofstream fzOut("/Users/gjc13/KinectData/planeZ.data");
//    for(auto point: subCloud->points)
//    {
//        double x = point.x;
//        fxOut << x << endl;
//    }
//    for(auto point: subCloud->points)
//    {
//        double y = point.y;
//        fyOut << y << endl;
//    }
//    for(auto point: subCloud->points)
//    {
//        double z = point.z;
//        fzOut << z << endl;
//    }
//    auto viewer = rgbVis(pointCloud);
//    while (!viewer->wasStopped())
//    {
//        viewer->spinOnce(100);
//        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
//    }
    return 0;
}

//恢复CV::32fc1的深度图
cv::Mat reload_32f_image(string filename)
{
    ifstream fin(filename.c_str(), ifstream::binary);
    int num_rows, num_cols;
    fin.read((char *) &num_rows, sizeof(int));
    fin.read((char *) &num_cols, sizeof(int));
    cv::Mat mat = cv::Mat::zeros(num_rows, num_cols, CV_32FC1);
    fin.read((char *) mat.data, num_cols * num_rows * 4);
    return mat;
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

boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    return (viewer);
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
    cout << "KinectToPCL depthBinFile registeredImage saveFile" << endl;
}
