/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012-, Open Perception, Inc.
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
 *   * Neither the name of the copyright holder(s) nor the names of its
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
 */

#include "tiff_to_pcd.h"
#include "template_alignment.h"
#include "openni_capture.h"

void
usage (char** argv)
{
  std::cout << "usage: " << argv[0] << " <device_id> [-C] [-g]\n\n";
  std::cout << "  -C:  initialize the pointcloud to track without plane segmentation"
            << std::endl;
  std::cout << "  -D: visualizing with non-downsampled pointclouds."
            << std::endl;
  std::cout << "  -P: not visualizing particle cloud."
            << std::endl;
  std::cout << "  -fixed: use the fixed number of the particles."
            << std::endl;
  std::cout << "  -d <value>: specify the grid size of downsampling (defaults to 0.01)."
            << std::endl;
}

int
main (int argc, char** argv)
{
  
  pcl::io::loadPLYFile("turtle_with_normals.ply", *model_3d); 
    
  loadMesh("turtle.ply");

  bool use_convex_hull = true;
  bool visualize_non_downsample = false;
  bool visualize_particles = true;
  bool use_fixed = false;

  double downsampling_grid_size = 0.01;
  
  if (pcl::console::find_argument (argc, argv, "-C") > 0)
    use_convex_hull = false;
  if (pcl::console::find_argument (argc, argv, "-D") > 0)
    visualize_non_downsample = true;
  if (pcl::console::find_argument (argc, argv, "-P") > 0)
    visualize_particles = false;
  if (pcl::console::find_argument (argc, argv, "-fixed") > 0)
    use_fixed = true;
  pcl::console::parse_argument (argc, argv, "-d", downsampling_grid_size);
  if (argc < 2)
  {
    usage (argv);
    exit (1);
  }
  
  std::string device_id = std::string (argv[1]);

  if (device_id == "--help" || device_id == "-h")
  {
    usage (argv);
    exit (1);
  }
  

  // open kinect
  OpenNISegmentTracking<pcl::PointXYZRGBA> v (device_id, 8, downsampling_grid_size,
                                              use_convex_hull,
                                              visualize_non_downsample, visualize_particles,
                                              use_fixed);
  v.run ();
}