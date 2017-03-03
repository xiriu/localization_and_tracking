#include <pcl/tracking/tracking.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl/tracking/particle_filter_omp.h>

#include <pcl/tracking/coherence.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/hsv_color_coherence.h>
#include <pcl/tracking/normal_coherence.h>

#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/console/parse.h>
#include <pcl/common/time.h>
#include <pcl/common/centroid.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/integral_image_normal.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/surface/convex_hull.h>

#include <pcl/search/pcl_search.h>
#include <pcl/common/transforms.h>

#include <boost/format.hpp>

#include <pcl/io/ply_io.h>

#define FPS_CALC_BEGIN                          \
    static double duration = 0;                 \
    double start_time = pcl::getTime ();        \

#define FPS_CALC_END(_WHAT_)                    \
  {                                             \
    double end_time = pcl::getTime ();          \
    static unsigned count = 0;                  \
    if (++count == 10)                          \
    {                                           \
      std::cout << "Average framerate("<< _WHAT_ << "): " << double(count)/double(duration) << " Hz" <<  std::endl; \
      count = 0;                                                        \
      duration = 0.0;                                                   \
    }                                           \
    else                                        \
    {                                           \
      duration += end_time - start_time;        \
    }                                           \
  }

using namespace pcl::tracking;

//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr model_3d (new pcl::PointCloud<pcl::PointXYZRGBA> );
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr model_3d(new pcl::PointCloud<pcl::PointXYZRGBNormal>); 

TemplateAlignment::Result best_alignment;
TextureMesh mesh;
pcl::PolygonMesh triangles;
cv::Mat rgb_image; 

template <typename PointType>
class OpenNISegmentTracking
{
public:
  typedef pcl::PointXYZRGBA RefPointType;
  typedef ParticleXYZRPY ParticleT;
  
  typedef pcl::PointCloud<PointType> Cloud;
  typedef pcl::PointCloud<RefPointType> RefCloud;
  typedef typename RefCloud::Ptr RefCloudPtr;
  typedef typename RefCloud::ConstPtr RefCloudConstPtr;
  typedef typename Cloud::Ptr CloudPtr;
  typedef typename Cloud::ConstPtr CloudConstPtr;
  typedef ParticleFilterTracker<RefPointType, ParticleT> ParticleFilter;
  typedef typename ParticleFilter::CoherencePtr CoherencePtr;
  typedef typename pcl::search::KdTree<PointType> KdTree;
  typedef typename KdTree::Ptr KdTreePtr;
  OpenNISegmentTracking (const std::string& device_id, int thread_nr, double downsampling_grid_size,
                         bool use_convex_hull,
                         bool visualize_non_downsample, bool visualize_particles,
                         bool use_fixed)
  : viewer_ ("PCL OpenNI Tracking Viewer")
  , device_id_ (device_id)
  , new_cloud_ (false)
  , template_alignment(false)
  , ne_ (thread_nr)
  , counter_ (0)
  , use_convex_hull_ (use_convex_hull)
  , visualize_non_downsample_ (visualize_non_downsample)
  , visualize_particles_ (visualize_particles)
  , downsampling_grid_size_ (downsampling_grid_size)
  {
    KdTreePtr tree (new KdTree (false));
    ne_.setSearchMethod (tree);
    ne_.setRadiusSearch (0.03);
    
    std::vector<double> default_step_covariance = std::vector<double> (6, 0.015 * 0.015);
    default_step_covariance[3] *= 40.0;
    default_step_covariance[4] *= 40.0;
    default_step_covariance[5] *= 40.0;
    
    std::vector<double> initial_noise_covariance = std::vector<double> (6, 0.00001);
    std::vector<double> default_initial_mean = std::vector<double> (6, 0.0);
    if (use_fixed)
    {
      boost::shared_ptr<ParticleFilterOMPTracker<RefPointType, ParticleT> > tracker
        (new ParticleFilterOMPTracker<RefPointType, ParticleT> (thread_nr));
      tracker_ = tracker;
    }
    else
    {
      boost::shared_ptr<KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT> > tracker
        (new KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT> (thread_nr));
      tracker->setMaximumParticleNum (500);
      tracker->setDelta (0.99);
      tracker->setEpsilon (0.2);
      ParticleT bin_size;
      bin_size.x = 0.1f;
      bin_size.y = 0.1f;
      bin_size.z = 0.1f;
      bin_size.roll = 0.1f;
      bin_size.pitch = 0.1f;
      bin_size.yaw = 0.1f;
      tracker->setBinSize (bin_size);
      tracker_ = tracker;
    }
    
    tracker_->setTrans (Eigen::Affine3f::Identity ());
    tracker_->setStepNoiseCovariance (default_step_covariance);
    tracker_->setInitialNoiseCovariance (initial_noise_covariance);
    tracker_->setInitialNoiseMean (default_initial_mean);
    tracker_->setIterationNum (1);
    
    tracker_->setParticleNum (400);
    tracker_->setResampleLikelihoodThr(0.00);
    tracker_->setUseNormal (false);
    
    // setup coherences
    ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr coherence = ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr
      (new ApproxNearestPairPointCloudCoherence<RefPointType> ());
    
    boost::shared_ptr<DistanceCoherence<RefPointType> > distance_coherence
      = boost::shared_ptr<DistanceCoherence<RefPointType> > (new DistanceCoherence<RefPointType> ());
    coherence->addPointCoherence (distance_coherence);
    
    boost::shared_ptr<HSVColorCoherence<RefPointType> > color_coherence
      = boost::shared_ptr<HSVColorCoherence<RefPointType> > (new HSVColorCoherence<RefPointType> ());
    color_coherence->setWeight (0.1);
    coherence->addPointCoherence (color_coherence);
    
    boost::shared_ptr<pcl::search::Octree<RefPointType> > search (new pcl::search::Octree<RefPointType> (0.01));

    coherence->setSearchMethod (search);
    coherence->setMaximumDistance (0.01);
    tracker_->setCloudCoherence (coherence);
  }

  bool
  drawParticles (pcl::visualization::PCLVisualizer& viz)
  {
    ParticleFilter::PointCloudStatePtr particles = tracker_->getParticles ();
    if (particles)
    {
      if (visualize_particles_)
      {
        pcl::PointCloud<pcl::PointXYZ>::Ptr particle_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
        for (size_t i = 0; i < particles->points.size (); i++)
        {
          pcl::PointXYZ point;
          
          point.x = particles->points[i].x;
          point.y = particles->points[i].y;
          point.z = particles->points[i].z;
          particle_cloud->points.push_back (point);
        }
        
        {
          //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue_color (particle_cloud, 250, 99, 71);
          /*if (!viz.updatePointCloud (particle_cloud, blue_color, "particle cloud"))
            viz.addPointCloud (particle_cloud, blue_color, "particle cloud"); */
          /*if (!viz.updatePointCloud (particle_cloud, "particle cloud"))
            viz.addPointCloud (particle_cloud, "particle cloud");*/ 
        }
      }
      return true;
    }
    else
    {
      PCL_WARN ("no particles\n");
      return false;
    }
  }
  
  void
  drawResult (pcl::visualization::PCLVisualizer& viz)
  {
    ParticleXYZRPY result = tracker_->getResult ();

    Eigen::Affine3f transformation = tracker_->toEigenMatrix (result);
    
    // move a little bit for better visualization
    //transformation.translation () += Eigen::Vector3f (0.0f, 0.0f, -0.005f);

    //Result cloud is the segmented point cloud corresponding to the object
    RefCloudPtr result_cloud (new RefCloud ());

    //pcl::visualization::PointCloudColorHandlerCustom<RefPointType> red_color (result_cloud, 0, 0, 255);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal> ());

    if (!visualize_non_downsample_)
    {
      pcl::transformPointCloud<RefPointType> (*(tracker_->getReferenceCloud ()), *result_cloud, transformation);
    }
    else
    {
      pcl::transformPointCloud<RefPointType> (*reference_, *result_cloud, transformation);
    }

    if(!template_alignment)
    {
      template_alignment = true;

      best_alignment = template_matching(result_cloud,model_3d);

      //we wont use the translation given by the template alignment, only the rotation
      /*best_alignment.final_transformation(0,3)-=transformation(0,3);
      best_alignment.final_transformation(1,3)-=transformation(1,3);
      best_alignment.final_transformation(2,3)-=transformation(2,3);*/


      for(int i=0;i<4;i++)
        for(int j=0;j<4;j++)
          cout<<"transformation_template_matching("<<i<<","<<j<<") = "<<best_alignment.final_transformation(i,j)<<";"<<endl;


    }
    if (!visualize_non_downsample_)
    {
      pcl::transformPointCloudWithNormals<PointXYZRGBNormal> (*model_3d, *transformed_cloud, best_alignment.final_transformation);
      //pcl::transformPointCloudWithNormals<PointXYZRGBNormal> (*transformed_cloud, *transformed_cloud, transformation);
    }
    else
    {
      pcl::transformPointCloudWithNormals<PointXYZRGBNormal> (*model_3d, *transformed_cloud, best_alignment.final_transformation);
      //pcl::transformPointCloudWithNormals<PointXYZRGBNormal> (*transformed_cloud, *transformed_cloud, transformation);
    }

    pcl::toPCLPointCloud2(*transformed_cloud, triangles.cloud );
    mesh.cloud = triangles.cloud;
    
    if (!viz.updatePointCloud (result_cloud,  "resultcloud"))
    {
        viz.addPointCloud (result_cloud,  "resultcloud");
    }

    if (!viz.updatePointCloud (result_cloud, "resultcloud"))
    {
        viz.addPointCloud (result_cloud, "resultcloud");
    }

    //uncomment to draw the 3d model
    /*if (!viz.updatePointCloud (transformed_cloud, "transformed_cloud"))
    {
        viz.addPointCloud (transformed_cloud, "transformed_cloud");
    }*/

  }
  
  void
  viz_cb (pcl::visualization::PCLVisualizer& viz)
  {
    boost::mutex::scoped_lock lock (mtx_);
    
    if (!cloud_pass_)
    {
      boost::this_thread::sleep (boost::posix_time::seconds (1));
      return;
    }
    
    if (new_cloud_ && cloud_pass_downsampled_)
    {
      CloudPtr cloud_pass;
      if (!visualize_non_downsample_)
        cloud_pass = cloud_pass_downsampled_;
      else
        cloud_pass = cloud_pass_;
      
      if (!viz.updatePointCloud (cloud_pass, "cloudpass"))
        {
          viz.addPointCloud (cloud_pass, "cloudpass");
          viz.resetCameraViewpoint ("cloudpass");
        }
    }

    if (new_cloud_ && reference_)
    {
      bool ret = drawParticles (viz);
      if (ret)
      {
        drawResult (viz);
        
        // draw some texts
        viz.removeShape ("N");
        viz.addText ((boost::format ("number of Reference PointClouds: %d") % tracker_->getReferenceCloud ()->points.size ()).str (),
                     10, 20, 20, 1.0, 1.0, 1.0, "N");
        
        viz.removeShape ("M");
        viz.addText ((boost::format ("number of Measured PointClouds:  %d") % cloud_pass_downsampled_->points.size ()).str (),
                     10, 40, 20, 1.0, 1.0, 1.0, "M");
        
        viz.removeShape ("tracking");
        viz.addText ((boost::format ("tracking:        %f fps") % (1.0 / tracking_time_)).str (),
                     10, 60, 20, 1.0, 1.0, 1.0, "tracking");
        
        viz.removeShape ("downsampling");
        viz.addText ((boost::format ("downsampling:    %f fps") % (1.0 / downsampling_time_)).str (),
                     10, 80, 20, 1.0, 1.0, 1.0, "downsampling");
        
        viz.removeShape ("computation");
        viz.addText ((boost::format ("computation:     %f fps") % (1.0 / computation_time_)).str (),
                     10, 100, 20, 1.0, 1.0, 1.0, "computation");

        viz.removeShape ("particles");
        viz.addText ((boost::format ("particles:     %d") % tracker_->getParticles ()->points.size ()).str (),
                     10, 120, 20, 1.0, 1.0, 1.0, "particles");
        
      }
    }
    new_cloud_ = false;
  }

  void filterPassThrough (const CloudConstPtr &cloud, Cloud &result)
  {
    FPS_CALC_BEGIN;
    pcl::PassThrough<PointType> pass;
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 10.0);
    //pass.setFilterLimits (0.0, 1.5);
    //pass.setFilterLimits (0.0, 0.6);
    pass.setKeepOrganized (false);
    pass.setInputCloud (cloud);
    pass.filter (result);
    FPS_CALC_END("filterPassThrough");
  }

  void euclideanSegment (const CloudConstPtr &cloud,
                         std::vector<pcl::PointIndices> &cluster_indices)
  {
    FPS_CALC_BEGIN;
    pcl::EuclideanClusterExtraction<PointType> ec;
    KdTreePtr tree (new KdTree ());
    
    ec.setClusterTolerance (0.05); // 2cm
    ec.setMinClusterSize (50);
    ec.setMaxClusterSize (25000);
    //ec.setMaxClusterSize (400);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);
    FPS_CALC_END("euclideanSegmentation");
  }
  
  void gridSample (const CloudConstPtr &cloud, Cloud &result, double leaf_size = 0.01)
  {
    FPS_CALC_BEGIN;
    double start = pcl::getTime ();
    pcl::VoxelGrid<PointType> grid;
    //pcl::ApproximateVoxelGrid<PointType> grid;
    grid.setLeafSize (float (leaf_size), float (leaf_size), float (leaf_size));
    grid.setInputCloud (cloud);
    grid.filter (result);
    //result = *cloud;
    double end = pcl::getTime ();
    downsampling_time_ = end - start;
    FPS_CALC_END("gridSample");
  }
  
  void gridSampleApprox (const CloudConstPtr &cloud, Cloud &result, double leaf_size = 0.01)
  {
    FPS_CALC_BEGIN;
    double start = pcl::getTime ();
    //pcl::VoxelGrid<PointType> grid;
    pcl::ApproximateVoxelGrid<PointType> grid;
    grid.setLeafSize (static_cast<float> (leaf_size), static_cast<float> (leaf_size), static_cast<float> (leaf_size));
    grid.setInputCloud (cloud);
    grid.filter (result);
    //result = *cloud;
    double end = pcl::getTime ();
    downsampling_time_ = end - start;
    FPS_CALC_END("gridSample");
  }
  
  void planeSegmentation (const CloudConstPtr &cloud,
                          pcl::ModelCoefficients &coefficients,
                          pcl::PointIndices &inliers)
  {
    FPS_CALC_BEGIN;
    pcl::SACSegmentation<PointType> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.03);
    seg.setInputCloud (cloud);
    seg.segment (inliers, coefficients);
    FPS_CALC_END("planeSegmentation");
  }

  void planeProjection (const CloudConstPtr &cloud,
                        Cloud &result,
                        const pcl::ModelCoefficients::ConstPtr &coefficients)
  {
    FPS_CALC_BEGIN;
    pcl::ProjectInliers<PointType> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (cloud);
    proj.setModelCoefficients (coefficients);
    proj.filter (result);
    FPS_CALC_END("planeProjection");
  }

  void convexHull (const CloudConstPtr &cloud,
                   Cloud &,
                   std::vector<pcl::Vertices> &hull_vertices)
  {
    FPS_CALC_BEGIN;
    pcl::ConvexHull<PointType> chull;
    chull.setInputCloud (cloud);
    chull.reconstruct (*cloud_hull_, hull_vertices);
    FPS_CALC_END("convexHull");
  }

  void normalEstimation (const CloudConstPtr &cloud,
                         pcl::PointCloud<pcl::Normal> &result)
  {
    FPS_CALC_BEGIN;
    ne_.setInputCloud (cloud);
    ne_.compute (result);
    FPS_CALC_END("normalEstimation");
  }
  
  void tracking (const RefCloudConstPtr &cloud)
  {
    double start = pcl::getTime ();
    FPS_CALC_BEGIN;
    tracker_->setInputCloud (cloud);
    tracker_->compute ();
    double end = pcl::getTime ();
    FPS_CALC_END("tracking");
    tracking_time_ = end - start;
  }

  void addNormalToCloud (const CloudConstPtr &cloud,
                         const pcl::PointCloud<pcl::Normal>::ConstPtr &,
                         RefCloud &result)
  {
    result.width = cloud->width;
    result.height = cloud->height;
    result.is_dense = cloud->is_dense;
    for (size_t i = 0; i < cloud->points.size (); i++)
    {
      RefPointType point;
      point.x = cloud->points[i].x;
      point.y = cloud->points[i].y;
      point.z = cloud->points[i].z;
      point.rgba = cloud->points[i].rgba;
      // point.normal[0] = normals->points[i].normal[0];
      // point.normal[1] = normals->points[i].normal[1];
      // point.normal[2] = normals->points[i].normal[2];
      result.points.push_back (point);
    }
  }

  void extractNonPlanePoints (const CloudConstPtr &cloud,
                              const CloudConstPtr &cloud_hull,
                              Cloud &result)
  {
    pcl::ExtractPolygonalPrismData<PointType> polygon_extract;
    pcl::PointIndices::Ptr inliers_polygon (new pcl::PointIndices ());
    polygon_extract.setHeightLimits (0.01, 10.0);
    polygon_extract.setInputPlanarHull (cloud_hull);
    polygon_extract.setInputCloud (cloud);
    polygon_extract.segment (*inliers_polygon);
    {
      pcl::ExtractIndices<PointType> extract_positive;
      extract_positive.setNegative (false);
      extract_positive.setInputCloud (cloud);
      extract_positive.setIndices (inliers_polygon);
      extract_positive.filter (result);
    }
  }

  void removeZeroPoints (const CloudConstPtr &cloud,
                         Cloud &result)
  {
    for (size_t i = 0; i < cloud->points.size (); i++)
    {
      PointType point = cloud->points[i];
      if (!(fabs(point.x) < 0.01 &&
            fabs(point.y) < 0.01 &&
            fabs(point.z) < 0.01) &&
          !pcl_isnan(point.x) &&
          !pcl_isnan(point.y) &&
          !pcl_isnan(point.z))
        result.points.push_back(point);
    }

    result.width = static_cast<pcl::uint32_t> (result.points.size ());
    result.height = 1;
    result.is_dense = true;
  }
  
  void extractSegmentCluster (const CloudConstPtr &cloud,
                              const std::vector<pcl::PointIndices> cluster_indices,
                              const int segment_index,
                              Cloud &result)
  {
    pcl::PointIndices segmented_indices = cluster_indices[segment_index];
    for (size_t i = 0; i < segmented_indices.indices.size (); i++)
    {
      PointType point = cloud->points[segmented_indices.indices[i]];
      result.points.push_back (point);
    }
    result.width = pcl::uint32_t (result.points.size ());
    result.height = 1;
    result.is_dense = true;
  }
  
  void
  cloud_cb (const CloudConstPtr &cloud)
  {
    boost::mutex::scoped_lock lock (mtx_);
    double start = pcl::getTime ();
    FPS_CALC_BEGIN;

    

    if (cloud->isOrganized()) {
        rgb_image = cv::Mat(cloud->height, cloud->width, CV_8UC3);

        if (!cloud->empty()) {

            for (int h=0; h<rgb_image.rows; h++) {
                for (int w=0; w<rgb_image.cols; w++) {
                    pcl::PointXYZRGBA point = cloud->at(w, h);

                    Eigen::Vector3i rgb = point.getRGBVector3i();

                    rgb_image.at<cv::Vec3b>(h,w)[0] = rgb[2];
                    rgb_image.at<cv::Vec3b>(h,w)[1] = rgb[1];
                    rgb_image.at<cv::Vec3b>(h,w)[2] = rgb[0];
                }
            }
        }
    }

    cv::imshow("res",rgb_image);
    cv::waitKey(3);

    cloud_pass_.reset (new Cloud);
    cloud_pass_downsampled_.reset (new Cloud);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    filterPassThrough (cloud, *cloud_pass_);
    if (counter_ < 10)
    {
      gridSample (cloud_pass_, *cloud_pass_downsampled_, downsampling_grid_size_);
    }
    else if (counter_ == 10)
    {
      //gridSample (cloud_pass_, *cloud_pass_downsampled_, 0.01);
      cloud_pass_downsampled_ = cloud_pass_;
      CloudPtr target_cloud;
      if (use_convex_hull_)
      {
        planeSegmentation (cloud_pass_downsampled_, *coefficients, *inliers);
        if (inliers->indices.size () > 3)
        {
          CloudPtr cloud_projected (new Cloud);
          cloud_hull_.reset (new Cloud);
          nonplane_cloud_.reset (new Cloud);
          
          planeProjection (cloud_pass_downsampled_, *cloud_projected, coefficients);
          convexHull (cloud_projected, *cloud_hull_, hull_vertices_);
          
          extractNonPlanePoints (cloud_pass_downsampled_, cloud_hull_, *nonplane_cloud_);
          target_cloud = nonplane_cloud_;
        }
        else
        {
          PCL_WARN ("cannot segment plane\n");
        }
      }
      else
      {
        PCL_WARN ("without plane segmentation\n");
        target_cloud = cloud_pass_downsampled_;
      }
      
      if (target_cloud != NULL)
      {
        PCL_INFO ("segmentation, please wait...\n");
        std::vector<pcl::PointIndices> cluster_indices;
        euclideanSegment (target_cloud, cluster_indices);
        if (cluster_indices.size () > 0)
        {
          // select the cluster to track
          CloudPtr temp_cloud (new Cloud);
          extractSegmentCluster (target_cloud, cluster_indices, 0, *temp_cloud);
          Eigen::Vector4f c;
          pcl::compute3DCentroid<RefPointType> (*temp_cloud, c);
          int segment_index = 0;
          double segment_distance = c[0] * c[0] + c[1] * c[1];
          for (size_t i = 1; i < cluster_indices.size (); i++)
          {
            temp_cloud.reset (new Cloud);
            extractSegmentCluster (target_cloud, cluster_indices, int (i), *temp_cloud);
            pcl::compute3DCentroid<RefPointType> (*temp_cloud, c);
            double distance = c[0] * c[0] + c[1] * c[1];
            if (distance < segment_distance)
            {
              segment_index = int (i);
              segment_distance = distance;
            }
          }
          
          segmented_cloud_.reset (new Cloud);
          extractSegmentCluster (target_cloud, cluster_indices, segment_index, *segmented_cloud_);
          //pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
          //normalEstimation (segmented_cloud_, *normals);
          RefCloudPtr ref_cloud (new RefCloud);
          //addNormalToCloud (segmented_cloud_, normals, *ref_cloud);
          ref_cloud = segmented_cloud_;
          RefCloudPtr nonzero_ref (new RefCloud);
          removeZeroPoints (ref_cloud, *nonzero_ref);
          
          PCL_INFO ("calculating cog\n");
          
          RefCloudPtr transed_ref (new RefCloud);
          pcl::compute3DCentroid<RefPointType> (*nonzero_ref, c);
          Eigen::Affine3f trans = Eigen::Affine3f::Identity ();
          trans.translation ().matrix () = Eigen::Vector3f (c[0], c[1], c[2]);
          //pcl::transformPointCloudWithNormals<RefPointType> (*ref_cloud, *transed_ref, trans.inverse());
          pcl::transformPointCloud<RefPointType> (*nonzero_ref, *transed_ref, trans.inverse());
          CloudPtr transed_ref_downsampled (new Cloud);
          gridSample (transed_ref, *transed_ref_downsampled, downsampling_grid_size_);
          tracker_->setReferenceCloud (transed_ref_downsampled);
          tracker_->setTrans (trans);
          reference_ = transed_ref;
          tracker_->setMinIndices (int (ref_cloud->points.size ()) / 2);
        }
        else
        {
          PCL_WARN ("euclidean segmentation failed\n");
        }
      }
    }
    else
    {
      //normals_.reset (new pcl::PointCloud<pcl::Normal>);
      //normalEstimation (cloud_pass_downsampled_, *normals_);
      //RefCloudPtr tracking_cloud (new RefCloud ());
      //addNormalToCloud (cloud_pass_downsampled_, normals_, *tracking_cloud);
      //tracking_cloud = cloud_pass_downsampled_;
      
      //*cloud_pass_downsampled_ = *cloud_pass_;
      //cloud_pass_downsampled_ = cloud_pass_;
      gridSampleApprox (cloud_pass_, *cloud_pass_downsampled_, downsampling_grid_size_);
      tracking (cloud_pass_downsampled_);
    }
    
    new_cloud_ = true;
    double end = pcl::getTime ();
    computation_time_ = end - start;
    FPS_CALC_END("computation");
    counter_++;
  }
      
  void
  run ()
  {
    pcl::Grabber* interface = new pcl::OpenNIGrabber (device_id_);
    boost::function<void (const CloudConstPtr&)> f =
      boost::bind (&OpenNISegmentTracking::cloud_cb, this, _1);
    interface->registerCallback (f);
    
    viewer_.runOnVisualizationThread (boost::bind(&OpenNISegmentTracking::viz_cb, this, _1), "viz_cb");
    
    interface->start ();
      
    while (!viewer_.wasStopped ())
      boost::this_thread::sleep(boost::posix_time::seconds(1));
    interface->stop ();
  }
  
  pcl::visualization::CloudViewer viewer_;
  pcl::PointCloud<pcl::Normal>::Ptr normals_;
  CloudPtr cloud_pass_;
  CloudPtr cloud_pass_downsampled_;
  CloudPtr plane_cloud_;
  CloudPtr nonplane_cloud_;
  CloudPtr cloud_hull_;
  CloudPtr segmented_cloud_;
  CloudPtr reference_;
  std::vector<pcl::Vertices> hull_vertices_;
  
  std::string device_id_;
  boost::mutex mtx_;
  bool new_cloud_;
  bool template_alignment;
  pcl::NormalEstimationOMP<PointType, pcl::Normal> ne_; // to store threadpool
  boost::shared_ptr<ParticleFilter> tracker_;
  int counter_;
  bool use_convex_hull_;
  bool visualize_non_downsample_;
  bool visualize_particles_;
  double tracking_time_;
  double computation_time_;
  double downsampling_time_;
  double downsampling_grid_size_;
  };

void loadMesh(std::string path)
{
  
  pcl::io::loadPolygonFile(path, triangles);
  // Create the texturemesh object that will contain our UV-mapped mesh
  
  mesh.cloud = triangles.cloud;
  std::vector< pcl::Vertices> polygon_1;
  
  // push faces into the texturemesh object
  polygon_1.resize (triangles.polygons.size ());
  for(size_t i =0; i < triangles.polygons.size (); ++i)
  {
    polygon_1[i] = triangles.polygons[i];
  }
  mesh.tex_polygons.push_back(polygon_1);

}