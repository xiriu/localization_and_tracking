#include "tiff_to_pcd.h"

PointCloud<PointXYZRGBA> processAndSave( vtkSmartPointer<vtkImageData>  depth_data,
                     vtkSmartPointer<vtkImageData>  rgb_data,
                     std::string                    time,
                     float                          focal_length,
                     bool                           format,
                     bool                           color,
                     bool                           depth,
                     bool                           use_output_path,
                     std::string                    output_path)
{
  // Retrieve the entries from the image data and copy them into the output RGB cloud
  int rgb_components = rgb_data->GetNumberOfScalarComponents();
  //std::cout << "RGB comp:" << rgb_components << std::endl;

  if(rgb_components != 3)
  {
    std::cout << "RGB image doesn't have 3 components, proceed with next image" << std::endl;
    //return;
  }

  int rgb_dimensions[3];
  rgb_data->GetDimensions (rgb_dimensions);
  //std::cout << "RGB dim1: " << rgb_dimensions[0] << " dim2: " << rgb_dimensions[1] << " dim3: " << rgb_dimensions[2] << std::endl;

  // Retrieve the entries from the image data and copy them into the output RGB cloud
  int depth_components = depth_data->GetNumberOfScalarComponents();
  //std::cout << "Depth comp:" << depth_components << std::endl;

  if(depth_components != 1)
  {
    std::cout << "Depth image doesn't have a single component, proceed with next image" << std::endl;
    //return;
  }

  int depth_dimensions[3];
  depth_data->GetDimensions (depth_dimensions);
  //std::cout << "Depth dim1: " << depth_dimensions[0] << " dim2: " << depth_dimensions[1] << " dim3: " << depth_dimensions[2] << std::endl;

  // Check if dimensions fit
  if(rgb_dimensions[0] != depth_dimensions[0] || rgb_dimensions[1] != depth_dimensions[1] || rgb_dimensions[2] != depth_dimensions[2])
  {
    std::cout << "RGB and Depth dimensions don't match, proceed with next image" << std::endl;
    //return;
  }

  // Now let's create the clouds
  PointCloud<RGB>          pc_image;
  PointCloud<Intensity>    pc_depth;
  PointCloud<PointXYZRGBA> pc_xyzrgba;

  pc_image.width = pc_depth.width = pc_xyzrgba.width = rgb_dimensions[0];
  pc_image.height = pc_depth.height = pc_xyzrgba.height = rgb_dimensions[1];

  float constant = 1.0f / focal_length;
  float bad_point = std::numeric_limits<float>::quiet_NaN ();

  bool insert_only_good_points = true;

  for(int v = 0; v < rgb_dimensions[1]; v++)
  {
    for(int u = 0; u < rgb_dimensions[0]; u++)
    {
      RGB           color_point;
      Intensity     depth_point;
      PointXYZRGBA  xyzrgba_point;

      color_point.r = xyzrgba_point.r = static_cast<uint8_t> (rgb_data->GetScalarComponentAsFloat(u,v,0,0));
      color_point.g = xyzrgba_point.g = static_cast<uint8_t> (rgb_data->GetScalarComponentAsFloat(u,v,0,1));
      color_point.b = xyzrgba_point.b = static_cast<uint8_t> (rgb_data->GetScalarComponentAsFloat(u,v,0,2));
      xyzrgba_point.a = 255;

      pc_image.points.push_back(color_point);
      pc_depth.points.push_back(depth_point);

      float d =  depth_data->GetScalarComponentAsFloat(u,v,0,0);
      depth_point.intensity = d;

      if(d != 0 && !pcl_isnan(d) && !pcl_isnan(d))
      {
        xyzrgba_point.z = d * 0.001f;
        xyzrgba_point.x = static_cast<float> (u - rgb_dimensions[0]/2) * d * 0.001f * constant;
        xyzrgba_point.y = static_cast<float> (v - rgb_dimensions[1]/2) * d * 0.001f * constant;
      }
      else
      {
        xyzrgba_point.z = xyzrgba_point.x = xyzrgba_point.y = bad_point;
      }

      
      if(insert_only_good_points)
      {
        if(d != 0 && !pcl_isnan(d) && !pcl_isnan(d))
        {
          //inserting only if it is not a bad point
          pc_xyzrgba.points.push_back(xyzrgba_point);
        }
      }
      else
        pc_xyzrgba.points.push_back(xyzrgba_point);
      

    } // for u
  } // for v

  std::stringstream ss;

  if(use_output_path)
  {
    if(insert_only_good_points)
    {
      // Fill in the cloud data
      pc_xyzrgba.width    = pc_xyzrgba.size();
      pc_xyzrgba.height   = 1;
      pc_xyzrgba.is_dense = false;
      pc_xyzrgba.points.resize (pc_xyzrgba.width * pc_xyzrgba.height);
    }

    if(depth)
    {
      if(use_output_path)
        ss << output_path << "/frame_" << time << "_depth.pcd";
      else
        ss << "frame_" << time << "_depth.pcd";
      pcl::io::savePCDFile (ss.str(), pc_depth, format);
      ss.str(""); //empty
    }

    if(color)
    {
      if(use_output_path)
        ss << output_path << "/frame_" << time << "_color.pcd";
      else
        ss << "frame_" << time << "_color.pcd";
      pcl::io::savePCDFile (ss.str(), pc_image, format);
      ss.str(""); //empty
    }

    //if(use_output_path)
      ss << output_path << "/frame_" << time << "_xyzrgba.pcd";
    //else
      //ss << "frame_" << time << "_xyzrgba.pcd";
    pcl::io::savePCDFile (ss.str(), pc_xyzrgba, format);

    std::cout << "Saved " << time << " to pcd" << std::endl;
  }

  return pc_xyzrgba;
}

void tiff_to_pcd(std::string path, std::vector<PointCloud<PointXYZRGBA> > &pc_xyzrgba_vector,int counter, int number_frames_to_load)
{
  cv::Mat dummy = cv::Mat();
  tiff_to_pcd(path, pc_xyzrgba_vector,counter, number_frames_to_load, dummy);
}


void tiff_to_pcd(std::string path, std::vector<PointCloud<PointXYZRGBA> > &pc_xyzrgba_vector,int counter, int number_frames_to_load, cv::Mat & rgb_image)
{
  std::string rgb_path_, depth_path_, output_path_;

  rgb_path_ = path+"/rgb";
  depth_path_ = path+"/depth";

  bool verbose = 0;
  bool format = 0;
  bool color = 0;
  bool depth = 0;
  bool use_output_path = false;
  float focal_length = 525.0;

  std::vector<std::string> tiff_rgb_files;
  std::vector<boost::filesystem::path> tiff_rgb_paths;
  boost::filesystem::directory_iterator end_itr;

  if(boost::filesystem::is_directory(rgb_path_))
  {
    for (boost::filesystem::directory_iterator itr(rgb_path_); itr != end_itr; ++itr)
    {
      std::string ext = itr->path().extension().string();
      if(ext.compare(".tiff") == 0)
      {
        tiff_rgb_files.push_back (itr->path ().string ());
        tiff_rgb_paths.push_back (itr->path ());
      }
      else
      {
        // Found non tiff file
      }

      if(verbose)
      {
        std::cout << "Extension" << itr->path().extension() << std::endl;
        std::cout << "Filename" << itr->path().filename() << std::endl;
      }
    }
  }
  else
  {
    PCL_ERROR("RGB path is not a directory\n");
    exit(-1);
  }

  sort (tiff_rgb_files.begin (), tiff_rgb_files.end ());
  sort (tiff_rgb_paths.begin (), tiff_rgb_paths.end ());

  if(verbose)
    PCL_INFO ("Creating Depth Tiff List\n");

  std::vector<std::string> tiff_depth_files;
  std::vector<boost::filesystem::path> tiff_depth_paths;

  if(boost::filesystem::is_directory(depth_path_))
  {
    for (boost::filesystem::directory_iterator itr(depth_path_); itr != end_itr; ++itr)
    {
      std::string ext = itr->path().extension().string();
      if(ext.compare(".tiff") == 0)
      {
        tiff_depth_files.push_back (itr->path ().string ());
        tiff_depth_paths.push_back (itr->path ());
      }
      else
      {
        // Found non tiff file
      }

      if(verbose)
      {
        std::cout << "Extension" << itr->path().extension() << std::endl;
        std::cout << "Filename" << itr->path().filename() << std::endl;
      }
    }
  }
  else
  {
    PCL_ERROR("Depth path is not a directory\n");
    exit(-1);
  }

  sort (tiff_depth_files.begin (), tiff_depth_files.end ());
  sort (tiff_depth_paths.begin (), tiff_depth_paths.end ());


  for(unsigned int i = counter; i<counter+number_frames_to_load && i<tiff_rgb_paths.size(); i++)
  {
    // Load the input file
    vtkSmartPointer<vtkImageData> rgb_data;
    vtkSmartPointer<vtkTIFFReader> reader = vtkSmartPointer<vtkTIFFReader>::New ();

    // Check if the file is correct
    int ret = reader->CanReadFile (tiff_rgb_files[i].c_str());
    // 0 can't read the file, 1 can't prove it
    if(ret == 0 || ret == 1)
    {
      std::cout << "We have a broken tiff file: " << tiff_rgb_files[i] << std::endl;
      //continue;
    }
    // 2 can read it, 3 I'm the correct reader
    if(ret == 2 || ret == 3)
    {
      rgb_image = cv::imread(tiff_rgb_files[i].c_str());
      cv::imshow("rgb_image_from_tiff",rgb_image);
      cv::waitKey(3);
      reader->SetFileName (tiff_rgb_files[i].c_str());
      reader->Update ();
      rgb_data = reader->GetOutput ();

      std::string rgb_filename = tiff_rgb_paths[i].filename().string();
      std::string rgb_time = rgb_filename.substr(6,22);

      //std::cout << "RGB Time: " << rgb_time << std::endl;

      // Try to read the depth file
      int found = 0; // indicates if a corresponding depth file was found
      // Find the correct file name
      for(size_t j = i; j < tiff_depth_paths.size(); j++)
      {
        std::string depth_filename = tiff_depth_paths[i].filename().string();
        std::string depth_time = depth_filename.substr(6,22);

        if(depth_time.compare(rgb_time) == 0) // found the correct depth
        {
          //std::cout << "found " << depth_time<< " "<<rgb_time << std::endl;
          found = 1;

          // Process here!

          vtkSmartPointer<vtkImageData> depth_data;
          vtkSmartPointer<vtkTIFFReader> depth_reader = vtkSmartPointer<vtkTIFFReader>::New ();

          // Check if the file is correct
          int read = depth_reader->CanReadFile (tiff_depth_files[j].c_str());
          // 0 can't read the file, 1 can't prove it
          if(read == 0 || read == 1)
          {
            std::cout << "We have a broken tiff file: " << tiff_depth_files[j] << std::endl;
            continue;
          }
          // 2 can read it, 3 I'm the correct reader
          if(read == 2 || read == 3)
          {
            depth_reader->SetFileName (tiff_depth_files[j].c_str());
            depth_reader->Update ();
            depth_data = depth_reader->GetOutput ();
            
            //std::cout<<"loading "<<100.*((double)i-counter/(double)number_frames_to_load)<<std::endl;
            
            pc_xyzrgba_vector.push_back(processAndSave(depth_data, rgb_data, depth_time, focal_length, format, color, depth, use_output_path, output_path_));
          }

          // TODO: remove this depth entry from vector before break > speed up search time
          break;
        }
        else
        {
          // Continue with the next depth entry
          continue;
        }
        if(found == 0)
        {
          std::cout << "We couldn't find a Depth file for this RGB image" << std::endl;
        }
      } //for depth_paths
    } //if ret = 2 or 3
  } //for rgb paths


}