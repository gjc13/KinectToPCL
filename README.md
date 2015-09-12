#KinectToPCL
> A project for tinker2016

将Kinect2数据转换为PCL库格式的数据用于后续处理

##Kinect2数据来源
使用[libfreenect2](https://github.com/OpenKinect/libfreenect2 "libfreenect2")作为Kinect2的数据采集库。
现在需要保存的是其中的
depth image和registered image。
保存方法：
```C
void saveFrames(libfreenect2::Frame *rgb, libfreenect2::Frame *ir, libfreenect2::Frame *depth,libfreenect2::Frame *registered, const char * saveID)
{
    cv::Mat rgbMatrix(rgb->height, rgb->width, CV_8UC4, rgb->data);
    cv::imwrite(string("/Users/gjc13/KinectData/rgb") + saveID +".png", rgbMatrix);
    cv::Mat depthMatrix(depth->height, depth->width, CV_32FC1, depth->data);
    depthMatrix /= 4500.0f;
    cv::imshow("depth", depthMatrix);
    write_depth_data(depthMatrix, string("/Users/gjc13/KinectData/depth") + saveID + ".bin");
    cv::Mat registeredMatrix(registered->height, registered->width, CV_8UC4, registered->data);
    cv::imwrite(string("/Users/gjc13/KinectData/registered") + saveID + ".png", registeredMatrix);
}

void write_depth_data(const cv::Mat & depth_matrix, std::string filename)
{
  //depth: 32FC1
  std::ofstream fout(filename, std::ofstream::binary);
//  for(int i=0; i<depth_matrix.rows; i++)
//  {
//    for(int j=0; j<depth_matrix.cols; j++)
//    {
//      fout<<depth_matrix.at<float>(i,j)<<" ";
//    }
//    fout << std::endl;
//  }
  fout.write((char *)&depth_matrix.rows, sizeof(int));
  fout.write((char *)&depth_matrix.cols, sizeof(int));
  fout.write((char *)depth_matrix.data, depth_matrix.cols * depth_matrix.rows * 4);
}
```

##摄像头定参
需要确定的参数有两组：保存的depth到实际深度的映射和Kinect投影参数矩阵。
depth到实际深度的映射使用`print_depth`方法使用一块标准平面尽行线性拟合。
投影参数矩阵的拟合使用了一般教科书上的方法，参见[calibrationpdf](http://www.cim.mcgill.ca/~langer/558/19-cameracalibration.pdf "本讲义"),相关的代码在calibration.py中。
最后定出的参数写在了KinectParameters.cpp中。

