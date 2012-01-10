#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"

#include "pcl/filters/passthrough.h"
#include "pcl/filters/extract_indices.h"

int  main (int argc, char** argv){

	pcl::PointCloud<pcl::PointXYZ>::Ptr
  	  cloud (new pcl::PointCloud<pcl::PointXYZ> ()),
  	  cloud_filtered (new pcl::PointCloud<pcl::PointXYZ> ()),
  	  cloud_removed (new pcl::PointCloud<pcl::PointXYZ> ());

  // Fill in the cloud data
  cloud->width  = 5;
  cloud->height = 1;
  cloud->points.resize (cloud->width * cloud->height);

  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0);
    cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0);
    cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0);
  }
  std::cerr << "Cloud before filtering: " << std::endl;
  for (size_t i = 0; i < cloud->points.size (); ++i)
    std::cerr << "    " << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << std::endl;

  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZ> pass(true);
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.0);
  //pass.setFilterLimitsNegative (true);

  pass.filter (*cloud_filtered);
  pcl::IndicesConstPtr indices = pass.getRemovedIndices();
//  std::cerr<<"n_indices: "<<indices->size()<<std::endl;
//  std::cerr<<"indices: ";
//  for(unsigned int i=0;i<indices->size();i++)
//	  std::cerr<<(*indices)[i]<<"\t";
//  std::cerr<<std::endl;

  pcl::PointIndices::Ptr removed_indices (new pcl::PointIndices());
  removed_indices->indices = *indices;

  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud (cloud);
  extract.setIndices (removed_indices);
  extract.setNegative (false);
  extract.filter (*cloud_removed);

  std::cerr <<std::endl<< "Cloud after filtering: " << std::endl;
  for (size_t i = 0; i < cloud_filtered->points.size (); ++i)
    std::cerr 	<< cloud_filtered->points[i].x << " "
    			<< cloud_filtered->points[i].y << " "
    			<< cloud_filtered->points[i].z << std::endl;

  std::cerr <<std::endl<< "Cloud being removed: " << std::endl;
  for (size_t i = 0; i < cloud_removed->points.size (); ++i)
    std::cerr 	<< cloud_removed->points[i].x << " "
    			<< cloud_removed->points[i].y << " "
    			<< cloud_removed->points[i].z << std::endl;
  return (0);
}
