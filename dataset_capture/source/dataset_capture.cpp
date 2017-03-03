#include "dataset_capture.h"

void loadDataset(std::string path, std::vector<PointCloud<PointXYZRGBA> > &pc_xyzrgba_vector, int counter, int number_frames_to_load)
{
	cv::Mat dummy = cv::Mat();
	loadDataset(path, pc_xyzrgba_vector, counter, number_frames_to_load, dummy);	
}

void loadDataset(std::string path, std::vector<PointCloud<PointXYZRGBA> > &pc_xyzrgba_vector, int counter, int number_frames_to_load, cv::Mat &rgb_image)
{
  tiff_to_pcd(path, pc_xyzrgba_vector, counter, number_frames_to_load, rgb_image);
}

void loadDataset(std::string path, PointCloud<PointXYZRGBA> &pc_xyzrgba, int counter, int number_frames_to_load, cv::Mat &rgb_image)
{
	std::vector<PointCloud<PointXYZRGBA> > pc_xyzrgba_vector;
	loadDataset(path, pc_xyzrgba_vector, counter, number_frames_to_load, rgb_image);	
	pc_xyzrgba = pc_xyzrgba_vector[0];
}