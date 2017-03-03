#include <iostream>
#include "tiff_to_pcd.h"

// function that returns a vector with rgb Mat files and a vector with xyzrgba pcl point clouds
// "counter" is the point cloud index
void loadDataset(std::string path, std::vector<PointCloud<PointXYZRGBA> > &pc_xyzrgba_vector, int counter, int number_frames_to_load);

void loadDataset(std::string path, std::vector<PointCloud<PointXYZRGBA> > &pc_xyzrgba_vector, int counter, int number_frames_to_load, cv::Mat &rgb_image);

void loadDataset(std::string path, PointCloud<PointXYZRGBA> &pc_xyzrgba, int counter, int number_frames_to_load, cv::Mat &rgb_image);