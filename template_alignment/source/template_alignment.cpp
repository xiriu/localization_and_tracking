////////////////////////////// TEMPLATE MATCHING //////////////////////////////////////

#include "template_alignment.h"

// Align a collection of object templates to a sample point cloud
TemplateAlignment::Result
template_matching (pcl::PointCloud<pcl::PointXYZRGBA>::Ptr result_cloud, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr model_3d)
{
  FeatureCloud template_cloud;
  template_cloud.loadInputCloud (model_3d);

  // Load the target cloud PCD file
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  
  cloud->points.resize(result_cloud->size());
  for (size_t i = 0; i < result_cloud->points.size(); i++) {
      cloud->points[i].x = result_cloud->points[i].x;
      cloud->points[i].y = result_cloud->points[i].y;
      cloud->points[i].z = result_cloud->points[i].z;
  }

  // Preprocess the cloud by...
  // ...removing distant points
  const float depth_limit = 1.0;
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0, depth_limit);
  pass.filter (*cloud);

  // ... and downsampling the point cloud
  const float voxel_grid_size = 0.005f;
  pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
  vox_grid.setInputCloud (cloud);
  vox_grid.setLeafSize (voxel_grid_size, voxel_grid_size, voxel_grid_size);
  //vox_grid.filter (*cloud); // Please see this http://www.pcl-developers.org/Possible-problem-in-new-VoxelGrid-implementation-from-PCL-1-5-0-td5490361.html
  pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZ>); 
  vox_grid.filter (*tempCloud);
  cloud = tempCloud; 

  // Assign to the target FeatureCloud
  FeatureCloud target_cloud;
  target_cloud.setInputCloud (cloud);

  // Set the TemplateAlignment inputs
  TemplateAlignment template_align;
  template_align.addTemplateCloud (template_cloud);
  template_align.setTargetCloud (target_cloud);

  // Find the best template alignment
  TemplateAlignment::Result best_alignment;
  template_align.findBestAlignment (best_alignment);
  const FeatureCloud &best_template = target_cloud;

  // Print the alignment fitness score (values less than 0.00002 are good)
  printf ("Best fitness score: %f\n", best_alignment.fitness_score);

  // Save the aligned template for visualization
  pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
  //pcl::transformPointCloud (*best_template.getPointCloud (), transformed_cloud, best_alignment.final_transformation);
  //pcl::io::savePCDFileBinary ("output.pcd", transformed_cloud);

  return best_alignment;
}