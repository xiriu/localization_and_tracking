/**
 *  Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: $
 * @brief This file is an app to convert two folders with tiffs to pointclouds
 * @copyright Copyright (2012) KU Leuven
 * @authors Koen Buys
 **/

#include <iostream>
#include <boost/filesystem.hpp>
#include <vector>

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>

#include <pcl/io/vtk_lib_io.h>
#include <vtkSmartPointer.h>
#include <vtkImageViewer2.h>
#include <vtkTIFFReader.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>

#include <opencv2/opencv.hpp>

using namespace pcl;
//using namespace cv;

PointCloud<PointXYZRGBA> processAndSave( vtkSmartPointer<vtkImageData>  depth_data,
                     vtkSmartPointer<vtkImageData>  rgb_data,
                     std::string                    time,
                     float                          focal_length,
                     bool                           format,
                     bool                           color,
                     bool                           depth,
                     bool                           use_output_path,
                     std::string                    output_path);

// load tiff files and return pcd ones. 
// "counter" is the index of the first frame of the dataset one wants to load
// "number_frames_to_load" is how many frames one wants to load
void tiff_to_pcd(std::string path, std::vector<PointCloud<PointXYZRGBA> > &pc_xyzrgba_vector,int counter, int number_frames_to_load);

void tiff_to_pcd(std::string path, std::vector<PointCloud<PointXYZRGBA> > &pc_xyzrgba_vector,int counter, int number_frames_to_load, cv::Mat & rgb_image);

